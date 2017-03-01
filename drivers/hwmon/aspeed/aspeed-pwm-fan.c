/*
 *  drivers/hwmon/aspeed/aspeed-pwm-fan.c
 *
 *  ASPEED PWM & Fan Tacho controller driver
 *
 *  Copyright (C) 2012-2020  ASPEED Technology Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  History:
 *    2012.08.06: Initial version [Ryan Chen]
 *    2016.08.06: Porting to kernel 4.7 [Vadim Pasternak vadimp@mellanox.com]
 */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/timer.h>
#include <linux/hwmon-sysfs.h>
#include <linux/hwmon.h>
#include <linux/workqueue.h>
#include <linux/sysfs.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/thermal.h>
#include <asm/irq.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/reset.h>

/* CLK sysfs
 * 0 : enable
 * 1 : clk_source
 *
 * PWM sysfs A~H (0~7)
 * 0 - show/store enable
 * 1 - show/store type
 * 2 - show/store falling
 * 3 - show/store rising
 *
 * PWM M/N/O Type sysfs
 * 0 - show/store unit
 * 1 - show/store division_l
 * 2 - show/store division_h
 *
 * FAN sysfs (0~15)
 *  show/store enable
 *  show/store source
 *  show/store rpm
 *  show/store alarm
 *  show/store alarm_en
 *
 * Fan M/N/O Type sysfs
 *  0 - show/store enable
 *  1 - show/store mode
 *  2 - show/store unit
 *  3 - show/store division
 *  4 - show/store limit
 */

#define AST_SCU_FUN_PIN_CTRL1	0x80	/* Multi-function Pin Control#1 */
#define AST_SCU_FUN_PIN_CTRL2	0x84	/* Multi-function Pin Control#2 */
#define AST_SCU_FUN_PIN_CTRL3	0x88	/* Multi-function Pin Control#3 */
#define AST_SCU_FUN_PIN_CTRL4	0x8C	/* Multi-function Pin Control#4 */
#define AST_SCU_FUN_PIN_CTRL5	0x90	/* Multi-function Pin Control#5 */
#define AST_PWM_GROUP_NUM	(1 + PWM_CH_NUM + 2 * PWM_TYPE_NUM + TACHO_NUM)

/* Aspeed PWM & FAN Register Definition */
#define AST_PTCR_CTRL		0x00
#define AST_PTCR_CLK_CTRL	0x04
#define AST_PTCR_DUTY0_CTRL	0x08
#define AST_PTCR_DUTY1_CTRL	0x0c
#define AST_PTCR_TYPEM_CTRL0	0x10
#define AST_PTCR_TYPEM_CTRL1	0x14
#define AST_PTCR_TYPEN_CTRL0	0x18
#define AST_PTCR_TYPEN_CTRL1	0x1c
#define AST_PTCR_TACH_SOURCE	0x20
#define AST_PTCR_TRIGGER	0x28
#define AST_PTCR_RESULT		0x2c
#define AST_PTCR_INTR_CTRL	0x30
#define AST_PTCR_INTR_STS	0x34
#define AST_PTCR_TYPEM_LIMIT	0x38
#define AST_PTCR_TYPEN_LIMIT	0x3C
#define AST_PTCR_CTRL_EXT	0x40
#define AST_PTCR_CLK_EXT_CTRL	0x44
#define AST_PTCR_DUTY2_CTRL	0x48
#define AST_PTCR_DUTY3_CTRL	0x4c
#define AST_PTCR_TYPEO_CTRL0	0x50
#define AST_PTCR_TYPEO_CTRL1	0x54
#define AST_PTCR_TACH_SOURCE_EXT 0x60
#define AST_PTCR_TYPEO_LIMIT	0x78

/* Common definitions */
#define FALL_EDGE	(0)
#define RISE_EDGE	(0x1)
#define BOTH_EDGE	(0x2)
#define PWM_TYPE_NUM	3
#define PWM_TYPE_M	0x0
#define PWM_TYPE_N	0x1
#define PWM_TYPE_O	0x2
#define PWM_TYPE_MASK	0x3

#define TACHO_NUM	16
#define PWM_CH_NUM	8
#define PWMA		0x0
#define PWMB		0x1
#define PWMC		0x2
#define PWMD		0x3
#define PWME		0x4
#define PWMF		0x5
#define PWMG		0x6
#define PWMH		0x7


/* AST_PTCR_CTRL:0x00 - PWM-FAN General Control Register */
#define AST_PTCR_CTRL_SET_PWMD_TYPE(x) \
	((x & 0x1) << 15 | (x & 0x2) << 6)
#define AST_PTCR_CTRL_GET_PWMD_TYPE(x) \
	(((x & (0x1 << 7)) >> 6) | ((x & (0x1 << 15)) >> 15))
#define AST_PTCR_CTRL_SET_PWMD_TYPE_MASK \
	((0x1 << 7) | (0x1 << 15))

#define AST_PTCR_CTRL_SET_PWMC_TYPE(x) \
	((x & 0x1) << 14 | (x & 0x2) << 5)
#define AST_PTCR_CTRL_GET_PWMC_TYPE(x) \
	(((x & (0x1 << 6)) >> 5) | ((x & (0x1 << 14)) >> 14))
#define AST_PTCR_CTRL_SET_PWMC_TYPE_MASK \
	((0x1 << 6) | (0x1 << 14))

#define AST_PTCR_CTRL_SET_PWMB_TYPE(x) \
	((x & 0x1)<<13 | (x & 0x2) << 4)
#define AST_PTCR_CTRL_GET_PWMB_TYPE(x) \
	(((x & (0x1 << 5)) >> 4) | ((x & (0x1 << 13)) >> 13))
#define AST_PTCR_CTRL_SET_PWMB_TYPE_MASK \
	((0x1 << 5) | (0x1 << 13))

#define AST_PTCR_CTRL_SET_PWMA_TYPE(x) \
	((x & 0x1) << 12 | (x & 0x2) << 3)
#define AST_PTCR_CTRL_GET_PWMA_TYPE(x) \
	(((x & (0x1 << 4)) >> 3) | ((x & (0x1 << 12)) >> 12))
#define AST_PTCR_CTRL_SET_PWMA_TYPE_MASK \
	((0x1 << 4) | (0x1 << 12))

#define	AST_PTCR_CTRL_FAN_NUM_EN(x)		(0x1 << (16 + x))

#define	AST_PTCR_CTRL_PMWD		(11)
#define	AST_PTCR_CTRL_PMWD_EN		(0x1 << 11)
#define	AST_PTCR_CTRL_PMWC		(10)
#define	AST_PTCR_CTRL_PMWC_EN		(0x1 << 10)
#define	AST_PTCR_CTRL_PMWB		(9)
#define	AST_PTCR_CTRL_PMWB_EN		(0x1 << 9)
#define	AST_PTCR_CTRL_PMWA		(8)
#define	AST_PTCR_CTRL_PMWA_EN		(0x1 << 8)
#define	AST_PTCR_CTRL_CLK_MCLK		0x2 /* 0:24Mhz, 1:MCLK */
#define	AST_PTCR_CTRL_CLK_EN		0x1

/* AST_PTCR_CLK_CTRL:0x04 - PWM-FAN Clock Control Register */
/* TYPE N */
#define AST_PTCR_CLK_CTRL_TYPEN_UNIT		(24)
#define AST_PTCR_CLK_CTRL_TYPEN_UNIT_MASK	(0xff << 24)
#define AST_PTCR_CLK_CTRL_TYPEN_H		(20)
#define AST_PTCR_CLK_CTRL_TYPEN_H_MASK		(0xf << 20)
#define AST_PTCR_CLK_CTRL_TYPEN_L		(16)
#define AST_PTCR_CLK_CTRL_TYPEN_L_MASK		(0xf << 16)
/* TYPE M */
#define AST_PTCR_CLK_CTRL_TYPEM_UNIT		(8)
#define AST_PTCR_CLK_CTRL_TYPEM_UNIT_MASK	(0xff << 8)
#define AST_PTCR_CLK_CTRL_TYPEM_H		(4)
#define AST_PTCR_CLK_CTRL_TYPEM_H_MASK		(0xf << 4)
#define AST_PTCR_CLK_CTRL_TYPEM_L		(0)
#define AST_PTCR_CLK_CTRL_TYPEM_L_MASK		(0xf)


/* AST_PTCR_DUTY_CTRL0:0x08 - PWM-FAN duty control 0 register */
#define DUTY_CTRL0_PWMB_FALL_POINT		(24)
#define DUTY_CTRL0_PWMB_FALL_POINT_MASK		(0xff << 24)
#define DUTY_CTRL0_PWMB_RISE_POINT		(16)
#define DUTY_CTRL0_PWMB_RISE_POINT_MASK		(0xff << 16)
#define DUTY_CTRL0_PWMA_FALL_POINT		(8)
#define DUTY_CTRL0_PWMA_FALL_POINT_MASK		(0xff << 8)
#define DUTY_CTRL0_PWMA_RISE_POINT		(0)
#define DUTY_CTRL0_PWMA_RISE_POINT_MASK		(0xff)


/* AST_PTCR_DUTY_CTRL1 : 0x0c - PWM-FAN duty control 1 register */
#define DUTY_CTRL1_PWMD_FALL_POINT		(24)
#define DUTY_CTRL1_PWMD_FALL_POINT_MASK		(0xff << 24)
#define DUTY_CTRL1_PWMD_RISE_POINT		(16)
#define DUTY_CTRL1_PWMD_RISE_POINT_MASK		(0xff << 16)
#define DUTY_CTRL1_PWMC_FALL_POINT		(8)
#define DUTY_CTRL1_PWMC_FALL_POINT_MASK		(0xff << 8)
#define DUTY_CTRL1_PWMC_RISE_POINT		(0)
#define DUTY_CTRL1_PWMC_RISE_POINT_MASK		(0xff)


/* AST_PTCR_TYPEM_CTRL0 : 0x10/0x18/0x50 - Type M/N/O Ctrl 0 Register */
#define TYPE_CTRL0_FAN_PERIOD			(16)
#define TYPE_CTRL0_FAN_PERIOD_MASK		(0xffff << 16)
#define TYPE_CTRL0_FLAT_EN			(0x1 << 7)


/* 0 : FALL_EDGE,	0x1 : RISE_EDGE , 0x2 :BOTH_EDGE */
#define TYPE_CTRL0_FAN_MODE			(4)
#define TYPE_CTRL0_FAN_MODE_MASK		(0x3<<4)
#define TYPE_CTRL0_CLK_DIVISION			(1)
#define TYPE_CTRL0_CLK_DIVISION_MASK		(0x7<<1)
#define TYPE_CTRL0_FAN_TYPE_EN			(1)


/* AST_PTCR_TYPEM_CTRL1 : 0x14/0x1c/0x54 - Type M/N/O Ctrl 1 Register */
#define TYPE_CTRL1_FALL_POINT			(16)
#define TYPE_CTRL1_FALL_POINT_MASK		(0xff << 16)
#define TYPE_CTRL1_RISE_POINT			(0)
#define TYPE_CTRL1_RISE_POINT_MASK		(0xff)


/* AST_PTCR_TACH_SOURCE : 0x20/0x60 - Tach Source Register
 * bit [0,1] at 0x20, bit [2] at 0x60
 */
#define TACH_PWM_SOURCE_BIT01(x)		(x * 2)
#define TACH_PWM_SOURCE_BIT2(x)			(x * 2)
#define TACH_PWM_SOURCE_MASK_BIT01(x)		(0x3 << (x * 2))
#define TACH_PWM_SOURCE_MASK_BIT2(x)		(0x1 << (x * 2))

/* AST_PTCR_TRIGGER : 0x28 - Trigger Register */
#define TRIGGER_READ_FAN_NUM(x)			(0x1 << x)

/* AST_PTCR_RESULT : 0x2c - Result Register */
#define RESULT_STATUS				(31)
#define RESULT_VALUE_MASK			(0xfffff)

/* AST_PTCR_INTR_CTRL : 0x30 - Interrupt Ctrl Register */
#define INTR_CTRL_EN_NUM(x)			(0x1 << x)

/* AST_PTCR_INTR_STS : 0x34 - Interrupt Status Register	*/
#define INTR_CTRL_NUM(x)			(0x1 << x)

/* AST_PTCR_TYPEM_LIMIT, AST_PTCR_TYPEN_LIMIT,AST_PTCR_TYPEO_LIMIT:
 * 0x38/0x3C/0x78 - Type M / N / O Limit Register
 */
#define FAN_LIMIT_MASK				(0xfffff)

/* AST_PTCR_CTRL_EXT : 0x40 - General Ctrl Extension #1 */
#define AST_PTCR_CTRL_SET_PWMH_TYPE(x) \
	((x & 0x1) << 15 | (x & 0x2) << 6)
#define AST_PTCR_CTRL_GET_PWMH_TYPE(x) \
	(((x & (0x1 << 7))>>6) | ((x & (0x1 << 15)) >> 15))
#define AST_PTCR_CTRL_SET_PWMH_TYPE_MASK	((0x1 << 7) | (0x1 << 15))

#define AST_PTCR_CTRL_SET_PWMG_TYPE(x) \
	((x & 0x1)<<14 | (x & 0x2) << 5)
#define AST_PTCR_CTRL_GET_PWMG_TYPE(x) \
	(((x & (0x1 << 6))>>5) | ((x & (0x1 << 14)) >> 14))
#define AST_PTCR_CTRL_SET_PWMG_TYPE_MASK	((0x1 << 6) | (0x1 << 14))

#define AST_PTCR_CTRL_SET_PWMF_TYPE(x) \
		((x & 0x1) << 13 | (x & 0x2) << 4)
#define AST_PTCR_CTRL_GET_PWMF_TYPE(x) \
	(((x & (0x1 << 5)) >> 4) | ((x & (0x1 << 13)) >> 13))
#define AST_PTCR_CTRL_SET_PWMF_TYPE_MASK	((0x1 << 5) | (0x1 << 13))

#define AST_PTCR_CTRL_SET_PWME_TYPE(x) \
	((x & 0x1) << 12 | (x & 0x2) << 3)
#define AST_PTCR_CTRL_GET_PWME_TYPE(x) \
	(((x & (0x1 << 4)) >> 3) | ((x & (0x1 << 12)) >> 12))
#define AST_PTCR_CTRL_SET_PWME_TYPE_MASK	((0x1 << 4) | (0x1 << 12))

#define	AST_PTCR_CTRL_PMWH			(11)
#define	AST_PTCR_CTRL_PMWH_EN			(0x1 << 11)
#define	AST_PTCR_CTRL_PMWG			(10)
#define	AST_PTCR_CTRL_PMWG_EN			(0x1 << 10)
#define	AST_PTCR_CTRL_PMWF			(9)
#define	AST_PTCR_CTRL_PMWF_EN			(0x1 << 9)
#define	AST_PTCR_CTRL_PMWE			(8)
#define	AST_PTCR_CTRL_PMWE_EN			(0x1 << 8)

/* AST_PTCR_CLK_EXT_CTRL : 0x44 - Clock Control Extension #1 */
/* TYPE O */
#define AST_PTCR_CLK_CTRL_TYPEO_UNIT		(8)
#define AST_PTCR_CLK_CTRL_TYPEO_UNIT_MASK	(0xff << 8)
#define AST_PTCR_CLK_CTRL_TYPEO_H		(4)
#define AST_PTCR_CLK_CTRL_TYPEO_H_MASK		(0xf << 4)
#define AST_PTCR_CLK_CTRL_TYPEO_L		(0)
#define AST_PTCR_CLK_CTRL_TYPEO_L_MASK		(0xf)

/* AST_PTCR_DUTY2_CTRL : 0x48 - Duty Control 2 Register */
#define DUTY_CTRL2_PWMF_FALL_POINT		(24)
#define DUTY_CTRL2_PWMF_FALL_POINT_MASK		(0xff << 24)
#define DUTY_CTRL2_PWMF_RISE_POINT		(16)
#define DUTY_CTRL2_PWMF_RISE_POINT_MASK		(0xff << 16)
#define DUTY_CTRL2_PWME_FALL_POINT		(8)
#define DUTY_CTRL2_PWME_FALL_POINT_MASK		(0xff << 8)
#define DUTY_CTRL2_PWME_RISE_POINT		(0)
#define DUTY_CTRL2_PWME_RISE_POINT_MASK		(0xff)

/* AST_PTCR_DUTY3_CTRL : 0x4c - Duty Control 3 Register */
#define DUTY_CTRL3_PWMH_FALL_POINT		(24)
#define DUTY_CTRL3_PWMH_FALL_POINT_MASK		(0xff << 24)
#define DUTY_CTRL3_PWMH_RISE_POINT		(16)
#define DUTY_CTRL3_PWMH_RISE_POINT_MASK		(0xff << 16)
#define DUTY_CTRL3_PWMG_FALL_POINT		(8)
#define DUTY_CTRL3_PWMG_FALL_POINT_MASK		(0xff << 8)
#define DUTY_CTRL3_PWMG_RISE_POINT		(0)
#define DUTY_CTRL3_PWMG_RISE_POINT_MASK		(0xff)

struct aspeed_pwm_driver_data {
	u32 (*get_pwm_clock)(void);
};

struct aspeed_pwm_tacho_data;

struct aspeed_pwm_port_data {
	char name[16];
	struct thermal_cooling_device *tcdev;
	struct aspeed_pwm_tacho_data *priv;
	u8 pwm_port_type;
	u8 pwm_port_ctrl;
	u8 mode;
	u32 *cooling_levels;
	u8 cooling_level;
	u8 max_state;
	u8 channel;
};

struct aspeed_pwm_tacho_data {
	struct device			*hwmon_dev;
	void __iomem			*reg_base;
	int				irq;
	struct aspeed_pwm_driver_data	*aspeed_pwm_data;
	struct aspeed_pwm_port_data	*pwm_port[PWM_CH_NUM];
	const struct attribute_group	*groups[AST_PWM_GROUP_NUM + 1];
};

struct aspeed_pwm_tacho_data *aspeed_pwm_tacho;

static u8
aspeed_get_pwm_type(struct aspeed_pwm_tacho_data *aspeed_pwm_tacho, u8 pwm_ch);
static u8
aspeed_get_pwm_en(struct aspeed_pwm_tacho_data *aspeed_pwm_tacho, u8 pwm_ch);
static u8
aspeed_get_tacho_type_division(struct aspeed_pwm_tacho_data *aspeed_pwm_tacho,
			       u8 pwm_type);
static u16
aspeed_get_tacho_type_unit(struct aspeed_pwm_tacho_data *aspeed_pwm_tacho,
			   u8 pwm_type);
static u8
aspeed_get_pwm_clock_div_h(struct aspeed_pwm_tacho_data *aspeed_pwm_tacho,
			   u8 pwm_type);
static u8
aspeed_get_pwm_clock_div_l(struct aspeed_pwm_tacho_data *aspeed_pwm_tacho,
			   u8 pwm_type);
static u8
aspeed_get_pwm_clock_unit(struct aspeed_pwm_tacho_data *aspeed_pwm_tacho,
			  u8 pwm_type);

static inline void
aspeed_pwm_tacho_write(struct aspeed_pwm_tacho_data *aspeed_pwm_tacho, u32 val,
		       u32 reg)
{
	writel(val, aspeed_pwm_tacho->reg_base + reg);
}

static inline u32
aspeed_pwm_tacho_read(struct aspeed_pwm_tacho_data *aspeed_pwm_tacho, u32 reg)
{
	return readl(aspeed_pwm_tacho->reg_base + reg);
}

/*
 * (1) The PWM base clock = 24Mhz / (Clock_Division_H D[7:4] in PTCR04 *
 *	Clock_Division_L D[3:0] in PTCR04)
 * (2) The frequency of PWM = The PWM base clock / (PWM period D[15:8] in
 *	PTCR04 + 1)
 * (3) If a plan to output 25Khz PWM frequency and 10% step of duty cycle,
 *	suggested to set 0x943 in PTCR04 register.
 * The PWM frequency = 24Mhz / (16 * 6 * (9 + 1)) = 25Khz
 * duty cycle settings in the PTCR08 register:
 * 0x1e786008 D[15:0] = 0x0900, duty = 90%
 * 0x1e786008 D[15:0] = 0x0902, duty = 70%
 * ...
 * 0x1e786008 D[15:0] = 0x0908, duty = 10%
 * 0x1e786008 D[15:0] = 0x0909, duty = 100%
 * 0x1e786008 D[15:0] = 0x0000, duty = 100%
 * (falling) - (rising+1) /unit
 */
static void aspeed_pwm_tacho_init(void)
{
	/* Enable PWM TACH CLK
	 * Set M/N/O out is 25Khz
	 * The PWM frequency = 24Mhz / (16 * 6 * (9 + 1)) = 25Khz
	 */
	aspeed_pwm_tacho_write(aspeed_pwm_tacho, 0x09430943,
			       AST_PTCR_CLK_CTRL);
#ifdef PWM_TYPE_O
	aspeed_pwm_tacho_write(aspeed_pwm_tacho, 0x0943,
			       AST_PTCR_CLK_EXT_CTRL);
#endif
	/* FULL SPEED at initialize 100% pwm A~H */
	aspeed_pwm_tacho_write(aspeed_pwm_tacho, 0x0, AST_PTCR_DUTY0_CTRL);
	aspeed_pwm_tacho_write(aspeed_pwm_tacho, 0x0, AST_PTCR_DUTY1_CTRL);
	aspeed_pwm_tacho_write(aspeed_pwm_tacho, 0x0, AST_PTCR_DUTY2_CTRL);
	aspeed_pwm_tacho_write(aspeed_pwm_tacho, 0x0, AST_PTCR_DUTY3_CTRL);

	/* Set TACO M/N/O initial unit 0x1000, falling , divide 4 , Enable */
	aspeed_pwm_tacho_write(aspeed_pwm_tacho, 0x10000001,
			       AST_PTCR_TYPEM_CTRL0);
	aspeed_pwm_tacho_write(aspeed_pwm_tacho, 0x10000001,
			       AST_PTCR_TYPEN_CTRL0);
#ifdef PWM_TYPE_O
	aspeed_pwm_tacho_write(aspeed_pwm_tacho, 0x10000001,
			       AST_PTCR_TYPEO_CTRL0);
#endif

	/* TACO measure period = 24000000 / 2 / 2 / 256 / 4096 / 1 (only enable
	 * 1 TACHO) = 5.72Hz, it means that software needs to wait at least 0.2
	 * sec to get refreshed TACO value. If you will enable more TACO or
	 * require faster response, you have to control the clock divisor and
	 * the period to be smaller.
	 *
	 * Full Range to do measure unit 0x1000
	 * PTCRM/N/O[3:1] = 0, Type M/N/O fan tach clock is div 4. -->
	 * calculate RPM.
	 */
	aspeed_pwm_tacho_write(aspeed_pwm_tacho, 0x10000000,
			       AST_PTCR_TYPEM_CTRL1);
	aspeed_pwm_tacho_write(aspeed_pwm_tacho, 0x10000000,
			       AST_PTCR_TYPEN_CTRL1);
#ifdef PWM_TYPE_O
	aspeed_pwm_tacho_write(aspeed_pwm_tacho, 0x10000000,
			       AST_PTCR_TYPEO_CTRL1);
#endif

	/* TACO Source Selection, PWMA for fan0~15 */
	aspeed_pwm_tacho_write(aspeed_pwm_tacho, 0x0, AST_PTCR_TACH_SOURCE);
	aspeed_pwm_tacho_write(aspeed_pwm_tacho, 0x0,
			       AST_PTCR_TACH_SOURCE_EXT);

	/* PWM A~D -> Disable , type M, Tacho 0~15 Disable, CLK source 24Mhz */
#ifdef MCLK
	aspeed_pwm_tacho_write(aspeed_pwm_tacho, AST_PTCR_CTRL_CLK_MCLK |
			       AST_PTCR_CTRL_CLK_EN, AST_PTCR_CTRL);
#else
	aspeed_pwm_tacho_write(aspeed_pwm_tacho, AST_PTCR_CTRL_CLK_EN,
			       AST_PTCR_CTRL);
#endif
}

/* index 0 : clk_en , 1: clk_source */
static ssize_t aspeed_store_clk(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long input_val;
	struct sensor_device_attribute_2 *sensor_attr =
						to_sensor_dev_attr_2(attr);
	int ret;

	ret = kstrtoul(buf, 10, &input_val);
	if (ret)
		return ret;

	if ((input_val > 1) || (input_val < 0))
		return -EINVAL;

	switch (sensor_attr->nr) {
	case 0:	/* clk_en */
		if (input_val)
			aspeed_pwm_tacho_write(aspeed_pwm_tacho,
				aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					AST_PTCR_CTRL) | AST_PTCR_CTRL_CLK_EN,
					AST_PTCR_CTRL);
		else
			aspeed_pwm_tacho_write(aspeed_pwm_tacho,
					aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					AST_PTCR_CTRL) & ~AST_PTCR_CTRL_CLK_EN,
					AST_PTCR_CTRL);
		break;

	case 1: /* clk_source */
		if (input_val)
			aspeed_pwm_tacho_write(aspeed_pwm_tacho,
				aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					AST_PTCR_CTRL) |
					AST_PTCR_CTRL_CLK_MCLK,
					AST_PTCR_CTRL);
		else
			aspeed_pwm_tacho_write(aspeed_pwm_tacho,
					aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					AST_PTCR_CTRL) &
					~AST_PTCR_CTRL_CLK_MCLK,
					AST_PTCR_CTRL);
		break;

	default:
		return -EINVAL;
	}

	return count;

}


static ssize_t aspeed_show_clk(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct sensor_device_attribute_2 *sensor_attr =
						to_sensor_dev_attr_2(attr);

	switch (sensor_attr->nr)	 {
	case 0:	/* clk_en */
		if (AST_PTCR_CTRL_CLK_EN &
		    aspeed_pwm_tacho_read(aspeed_pwm_tacho, AST_PTCR_CTRL))
			return sprintf(buf, "1: Enable\n");
		else
			return sprintf(buf, "0: Disable\n");
		break;

	case 1: /* clk_source */
		if (AST_PTCR_CTRL_CLK_MCLK &
		    aspeed_pwm_tacho_read(aspeed_pwm_tacho, AST_PTCR_CTRL))
			return sprintf(buf, "1: MCLK\n");
		else
			return sprintf(buf, "0: 24Mhz\n");

		break;

	default:
		return sprintf(buf, "error CLK Index\n");
	}
}

static u32
aspeed_get_tacho_measure_period(struct aspeed_pwm_tacho_data *aspeed_pwm_tacho,
				u8 pwm_type)
{
	u32 clk, clk_unit, div_h, div_l, tacho_unit, tacho_div;

	if (AST_PTCR_CTRL_CLK_MCLK & aspeed_pwm_tacho_read(aspeed_pwm_tacho,
							   AST_PTCR_CTRL))
		clk = aspeed_pwm_tacho->aspeed_pwm_data->get_pwm_clock();
	else
		clk = 24 * 1000 * 1000;

	clk_unit = aspeed_get_pwm_clock_unit(aspeed_pwm_tacho, pwm_type);
	div_h = aspeed_get_pwm_clock_div_h(aspeed_pwm_tacho, pwm_type);
	div_h = 0x1 << div_h;
	div_l = aspeed_get_pwm_clock_div_l(aspeed_pwm_tacho, pwm_type);
	if (div_l == 0)
		div_l = 1;
	else
		div_l = div_l * 2;

	tacho_unit = aspeed_get_tacho_type_unit(aspeed_pwm_tacho, pwm_type);
	tacho_div = aspeed_get_tacho_type_division(aspeed_pwm_tacho, pwm_type);

	tacho_div = 0x4 << (tacho_div * 2);

	return clk/(clk_unit * div_h * div_l * tacho_div * tacho_unit);
}

static u8
aspeed_get_tacho_type_division(struct aspeed_pwm_tacho_data *aspeed_pwm_tacho,
			       u8 pwm_type)
{
	u32 tmp = 0;

	switch (pwm_type) {
	case PWM_TYPE_M:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_TYPEM_CTRL0);
		break;

	case PWM_TYPE_N:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_TYPEN_CTRL0);
		break;

#ifdef PWM_TYPE_O
	case PWM_TYPE_O:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_TYPEO_CTRL0);
		break;

#endif
	default:
		dev_err(aspeed_pwm_tacho->hwmon_dev, "error type\n");
		break;

	}

	return ((tmp & TYPE_CTRL0_CLK_DIVISION_MASK) >>
		TYPE_CTRL0_CLK_DIVISION);
}

static void
aspeed_set_tacho_type_division(struct aspeed_pwm_tacho_data *aspeed_pwm_tacho,
			       u8 pwm_type, u32 division)
{
	u32 tmp = 0;

	if (division > 0x7)
		return;

	switch (pwm_type) {
	case PWM_TYPE_M:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_TYPEM_CTRL0);
		break;

	case PWM_TYPE_N:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_TYPEN_CTRL0);
		break;

#ifdef PWM_TYPE_O
	case PWM_TYPE_O:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_TYPEO_CTRL0);
		break;
#endif
	default:
		dev_err(aspeed_pwm_tacho->hwmon_dev, "error type\n");
		break;
	}

	tmp &= ~TYPE_CTRL0_CLK_DIVISION_MASK;
	tmp |= (division << TYPE_CTRL0_CLK_DIVISION);

	switch (pwm_type) {
	case PWM_TYPE_M:
		aspeed_pwm_tacho_write(aspeed_pwm_tacho, tmp,
				       AST_PTCR_TYPEM_CTRL0);
		break;
	case PWM_TYPE_N:
		aspeed_pwm_tacho_write(aspeed_pwm_tacho, tmp,
				       AST_PTCR_TYPEN_CTRL0);
		break;
#ifdef PWM_TYPE_O
	case PWM_TYPE_O:
		aspeed_pwm_tacho_write(aspeed_pwm_tacho, tmp,
				       AST_PTCR_TYPEO_CTRL0);
		break;
#endif
	default:
		dev_err(aspeed_pwm_tacho->hwmon_dev, "error type\n");
		break;
	}
}

static u16
aspeed_get_tacho_type_unit(struct aspeed_pwm_tacho_data *aspeed_pwm_tacho,
			   u8 pwm_type)
{
	u32 tmp = 0;

	switch (pwm_type) {
	case PWM_TYPE_M:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_TYPEM_CTRL0);
		break;
	case PWM_TYPE_N:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_TYPEN_CTRL0);
		break;
#ifdef PWM_TYPE_O
	case PWM_TYPE_O:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_TYPEO_CTRL0);
		break;
#endif
	default:
		dev_err(aspeed_pwm_tacho->hwmon_dev, "error type\n");
		break;
	}

	return ((tmp & TYPE_CTRL0_FAN_PERIOD_MASK) >> TYPE_CTRL0_FAN_PERIOD);
}

static void
aspeed_set_tacho_type_unit(struct aspeed_pwm_tacho_data *aspeed_pwm_tacho,
			   u8 pwm_type, u32 unit)
{
	u32 tmp = 0;

	if (unit > 0xffff)
		return;

	switch (pwm_type) {
	case PWM_TYPE_M:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_TYPEM_CTRL0);
		break;
	case PWM_TYPE_N:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_TYPEN_CTRL0);
		break;
#ifdef PWM_TYPE_O
	case PWM_TYPE_O:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_TYPEO_CTRL0);
		break;
#endif
	default:
		dev_err(aspeed_pwm_tacho->hwmon_dev, "error type\n");
		break;
	}

	tmp &= ~TYPE_CTRL0_FAN_PERIOD_MASK;
	tmp |= (unit << TYPE_CTRL0_FAN_PERIOD);

	switch (pwm_type) {
	case PWM_TYPE_M:
		aspeed_pwm_tacho_write(aspeed_pwm_tacho, tmp,
				       AST_PTCR_TYPEM_CTRL0);
		break;

	case PWM_TYPE_N:
		aspeed_pwm_tacho_write(aspeed_pwm_tacho, tmp,
				       AST_PTCR_TYPEN_CTRL0);
		break;

#ifdef PWM_TYPE_O
	case PWM_TYPE_O:
		aspeed_pwm_tacho_write(aspeed_pwm_tacho, tmp,
				       AST_PTCR_TYPEO_CTRL0);
		break;
#endif
	default:
		dev_err(aspeed_pwm_tacho->hwmon_dev, "error type\n");
		break;
	}
}

static u32
aspeed_get_tacho_type_mode(struct aspeed_pwm_tacho_data *aspeed_pwm_tacho,
			   u8 pwm_type)
{
	u32 tmp = 0;

	switch (pwm_type) {
	case PWM_TYPE_M:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_TYPEM_CTRL0);
		break;
	case PWM_TYPE_N:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_TYPEN_CTRL0);
		break;
#ifdef PWM_TYPE_O
	case PWM_TYPE_O:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_TYPEO_CTRL0);
		break;
#endif
	default:
		dev_err(aspeed_pwm_tacho->hwmon_dev, "error type\n");
		break;
	}

	return ((tmp & TYPE_CTRL0_FAN_MODE_MASK) >> TYPE_CTRL0_FAN_MODE);
}

static void
aspeed_set_tacho_type_mode(struct aspeed_pwm_tacho_data *aspeed_pwm_tacho,
			   u8 pwm_type, u32 mode)
{
	u32 tmp = 0;

	if (mode > 0x2)
		return;

	switch (pwm_type) {
	case PWM_TYPE_M:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_TYPEM_CTRL0);
		break;

	case PWM_TYPE_N:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_TYPEN_CTRL0);
		break;

#ifdef PWM_TYPE_O
	case PWM_TYPE_O:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_TYPEO_CTRL0);
		break;

#endif
	default:
		dev_err(aspeed_pwm_tacho->hwmon_dev, "error type\n");
		break;
	}

	tmp &= ~TYPE_CTRL0_FAN_MODE_MASK;
	tmp |= (mode << TYPE_CTRL0_FAN_MODE);

	switch (pwm_type) {
	case PWM_TYPE_M:
		aspeed_pwm_tacho_write(aspeed_pwm_tacho, tmp,
				       AST_PTCR_TYPEM_CTRL0);
		break;

	case PWM_TYPE_N:
		aspeed_pwm_tacho_write(aspeed_pwm_tacho, tmp,
				       AST_PTCR_TYPEN_CTRL0);
		break;

#ifdef PWM_TYPE_O
	case PWM_TYPE_O:
		aspeed_pwm_tacho_write(aspeed_pwm_tacho, tmp,
				       AST_PTCR_TYPEO_CTRL0);
		break;
#endif
	default:
		dev_err(aspeed_pwm_tacho->hwmon_dev, "error type\n");
		break;
	}

}

static u8
aspeed_get_tacho_type_en(struct aspeed_pwm_tacho_data *aspeed_pwm_tacho,
			 u8 pwm_type)
{
	u8 tmp = 0;

	switch (pwm_type) {
	case PWM_TYPE_M:
		tmp = (TYPE_CTRL0_FAN_TYPE_EN &
		       aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					     AST_PTCR_TYPEM_CTRL0));
		break;

	case PWM_TYPE_N:
		tmp = (TYPE_CTRL0_FAN_TYPE_EN &
		      aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_TYPEN_CTRL0));
		break;

#ifdef PWM_TYPE_O
	case PWM_TYPE_O:
		tmp = (TYPE_CTRL0_FAN_TYPE_EN &
		      aspeed_pwm_tacho_read(aspeed_pwm_tacho,
		      AST_PTCR_TYPEO_CTRL0));
		break;
#endif
	default:
		dev_err(aspeed_pwm_tacho->hwmon_dev, "error type\n");
		break;
	}

	return tmp;
}

static void
aspeed_set_tacho_type_en(struct aspeed_pwm_tacho_data *aspeed_pwm_tacho,
			 u8 pwm_type, u32 enable)
{
	switch (pwm_type) {
	case PWM_TYPE_M:
		aspeed_pwm_tacho_write(aspeed_pwm_tacho,
				       aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					      AST_PTCR_TYPEM_CTRL0) | enable,
			AST_PTCR_TYPEM_CTRL0);

		break;
	case PWM_TYPE_N:
		aspeed_pwm_tacho_write(aspeed_pwm_tacho,
		aspeed_pwm_tacho_read(aspeed_pwm_tacho, AST_PTCR_TYPEN_CTRL0)
		| enable, AST_PTCR_TYPEN_CTRL0);

		break;
#ifdef PWM_TYPE_O
	case PWM_TYPE_O:
		aspeed_pwm_tacho_write(aspeed_pwm_tacho,
		aspeed_pwm_tacho_read(aspeed_pwm_tacho, AST_PTCR_TYPEO_CTRL0) |
				      enable, AST_PTCR_TYPEO_CTRL0);

		break;
#endif
	default:
		dev_err(aspeed_pwm_tacho->hwmon_dev, "error type\n");
		break;
	}
}

static u32
aspeed_get_tacho_type_limit(struct aspeed_pwm_tacho_data *aspeed_pwm_tacho,
			    u8 pwm_type)
{
	switch (pwm_type) {
	case PWM_TYPE_M:
		return (FAN_LIMIT_MASK &
			aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					      AST_PTCR_TYPEM_LIMIT));

	case PWM_TYPE_N:
		return (FAN_LIMIT_MASK &
			aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					      AST_PTCR_TYPEN_LIMIT));

#ifdef PWM_TYPE_O
	case PWM_TYPE_O:
		return (FAN_LIMIT_MASK &
			aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					      AST_PTCR_TYPEO_LIMIT));

#endif
	default:
		dev_err(aspeed_pwm_tacho->hwmon_dev, "error type\n");
		break;
	}
	return 0;
}

static void
aspeed_set_tacho_type_limit(struct aspeed_pwm_tacho_data *aspeed_pwm_tacho,
			    u8 pwm_type, u32 limit)
{
	if (limit > FAN_LIMIT_MASK)
		return;

	switch (pwm_type) {
	case PWM_TYPE_M:
		aspeed_pwm_tacho_write(aspeed_pwm_tacho, limit,
				       AST_PTCR_TYPEM_LIMIT);
		break;
	case PWM_TYPE_N:
		aspeed_pwm_tacho_write(aspeed_pwm_tacho, limit,
				       AST_PTCR_TYPEN_LIMIT);
		break;
#ifdef PWM_TYPE_O
	case PWM_TYPE_O:
		aspeed_pwm_tacho_write(aspeed_pwm_tacho, limit,
				       AST_PTCR_TYPEO_LIMIT);
		break;
#endif
	default:
		dev_err(aspeed_pwm_tacho->hwmon_dev, "error type\n");
		break;
	}
}

static u8
aspeed_get_tacho_alarm_en(struct aspeed_pwm_tacho_data *aspeed_pwm_tacho,
			  u8 tacho_ch)
{
	/* tacho source */
	if (aspeed_pwm_tacho_read(aspeed_pwm_tacho, AST_PTCR_INTR_CTRL) &
				  INTR_CTRL_EN_NUM(tacho_ch))
		return 1;

	return 0;
}

static void
aspeed_set_tacho_alarm_en(struct aspeed_pwm_tacho_data *aspeed_pwm_tacho,
			  u8 tacho_ch, u8 enable)
{
	/* tacho source  */
	if (enable == 1)
		aspeed_pwm_tacho_write(aspeed_pwm_tacho,
			aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					      AST_PTCR_INTR_CTRL) |
			INTR_CTRL_EN_NUM(tacho_ch), AST_PTCR_INTR_CTRL);
	else
		aspeed_pwm_tacho_write(aspeed_pwm_tacho,
			aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					      AST_PTCR_INTR_CTRL) &
			~(INTR_CTRL_EN_NUM(tacho_ch)), AST_PTCR_INTR_CTRL);
}

static u8
aspeed_get_tacho_alarm(struct aspeed_pwm_tacho_data *aspeed_pwm_tacho,
		       u8 tacho_ch)
{
	/* tacho source */
	if (aspeed_pwm_tacho_read(aspeed_pwm_tacho, AST_PTCR_INTR_STS) &
				  INTR_CTRL_NUM(tacho_ch))
		return 1;

	return 0;
}

static u8
aspeed_get_tacho_en(struct aspeed_pwm_tacho_data *aspeed_pwm_tacho, u8 tacho_ch)
{
	if (aspeed_pwm_tacho_read(aspeed_pwm_tacho, AST_PTCR_CTRL) &
				  AST_PTCR_CTRL_FAN_NUM_EN(tacho_ch))
		return 1;

	return 0;
}

static void
aspeed_set_tacho_en(struct aspeed_pwm_tacho_data *aspeed_pwm_tacho,
		    u8 tacho_ch, u8 enable)
{
	/* tacho number enable */
	if (enable)
		aspeed_pwm_tacho_write(aspeed_pwm_tacho,
			aspeed_pwm_tacho_read(aspeed_pwm_tacho, AST_PTCR_CTRL) |
			AST_PTCR_CTRL_FAN_NUM_EN(tacho_ch), AST_PTCR_CTRL);
	else
		aspeed_pwm_tacho_write(aspeed_pwm_tacho,
			aspeed_pwm_tacho_read(aspeed_pwm_tacho, AST_PTCR_CTRL) &
			~(AST_PTCR_CTRL_FAN_NUM_EN(tacho_ch)), AST_PTCR_CTRL);
}

static u8
aspeed_get_tacho_source(struct aspeed_pwm_tacho_data *aspeed_pwm_tacho,
			u8 tacho_ch)
{
	u32 tmp1, tmp2;

	/* tacho source */
	tmp1 = aspeed_pwm_tacho_read(aspeed_pwm_tacho, AST_PTCR_TACH_SOURCE);
	tmp1 &= TACH_PWM_SOURCE_MASK_BIT01(tacho_ch);
	tmp1 = tmp1 >> (TACH_PWM_SOURCE_BIT01(tacho_ch));
	tmp2 = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
				     AST_PTCR_TACH_SOURCE_EXT);
	tmp2 &= TACH_PWM_SOURCE_MASK_BIT2(tacho_ch);
	tmp2 = tmp2 >> (TACH_PWM_SOURCE_BIT2(tacho_ch));
	tmp2 = tmp2 << 2;

	return (tmp2 | tmp1);
}

static void
aspeed_set_tacho_source(struct aspeed_pwm_tacho_data *aspeed_pwm_tacho,
			u8 tacho_ch, u8 tacho_source)
{
	u32 tmp1, tmp2;

	if (tacho_source > 7)
		return;

	/* tacho source */
	tmp1 = aspeed_pwm_tacho_read(aspeed_pwm_tacho, AST_PTCR_TACH_SOURCE);
	tmp1 &= ~(TACH_PWM_SOURCE_MASK_BIT01(tacho_ch));
	tmp1 |= ((tacho_source & 0x3) << (TACH_PWM_SOURCE_BIT01(tacho_ch)));

	tmp2 = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
				     AST_PTCR_TACH_SOURCE_EXT);
	tmp2 &= ~(TACH_PWM_SOURCE_MASK_BIT2(tacho_ch));
	tmp2 |= (((tacho_source & 0x4) >> 2) <<
		(TACH_PWM_SOURCE_BIT2(tacho_ch)));

	aspeed_pwm_tacho_write(aspeed_pwm_tacho, tmp1, AST_PTCR_TACH_SOURCE);
	aspeed_pwm_tacho_write(aspeed_pwm_tacho, tmp2,
			       AST_PTCR_TACH_SOURCE_EXT);

}

static s32
aspeed_get_tacho_rpm(struct aspeed_pwm_tacho_data *aspeed_pwm_tacho,
		     u8 tacho_ch)
{
	u32 raw_data, rpm, tacho_clk_div, clk_source, timeout = 0;
	u8 tacho_source, pwm_type, tacho_type_en;

	if (!(aspeed_get_tacho_en(aspeed_pwm_tacho, tacho_ch)))
		return -EPERM;

	aspeed_pwm_tacho_write(aspeed_pwm_tacho, 0, AST_PTCR_TRIGGER);
	aspeed_pwm_tacho_write(aspeed_pwm_tacho, 0x1 << tacho_ch,
			       AST_PTCR_TRIGGER);

	tacho_source = aspeed_get_tacho_source(aspeed_pwm_tacho, tacho_ch);
	pwm_type = aspeed_get_pwm_type(aspeed_pwm_tacho, tacho_source);
	tacho_type_en = aspeed_get_tacho_type_en(aspeed_pwm_tacho, pwm_type);

	/* check pwm_type and get clock division */
	if (!tacho_type_en)
		return -EPERM;

	/* Wait ready */
	while (!(aspeed_pwm_tacho_read(aspeed_pwm_tacho, AST_PTCR_RESULT) &
		(0x1 << RESULT_STATUS))) {
		timeout++;
		if (timeout > 25)
			return -ETIMEDOUT;
	};

	raw_data = aspeed_pwm_tacho_read(aspeed_pwm_tacho, AST_PTCR_RESULT) &
		   RESULT_VALUE_MASK;
	tacho_clk_div = aspeed_get_tacho_type_division(aspeed_pwm_tacho,
						       pwm_type);

	tacho_clk_div = 0x4 << (tacho_clk_div*2);

	if (AST_PTCR_CTRL_CLK_MCLK & aspeed_pwm_tacho_read(aspeed_pwm_tacho,
							   AST_PTCR_CTRL))
		clk_source = 166 * 1000 * 1000;
	else
		clk_source = 24 * 1000 * 1000;

	rpm = (clk_source * 60) / (2 * raw_data * tacho_clk_div);

	return rpm;
}

static u8
aspeed_get_pwm_clock_div_h(struct aspeed_pwm_tacho_data *aspeed_pwm_tacho,
			   u8 pwm_type)
{
	u8 tmp = 0;

	switch (pwm_type) {
	case PWM_TYPE_M:
		tmp = (aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					     AST_PTCR_CLK_CTRL) &
		      AST_PTCR_CLK_CTRL_TYPEM_H_MASK) >>
		      AST_PTCR_CLK_CTRL_TYPEM_H;
		break;

	case PWM_TYPE_N:
		tmp = (aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					     AST_PTCR_CLK_CTRL) &
		       AST_PTCR_CLK_CTRL_TYPEN_H_MASK) >>
		       AST_PTCR_CLK_CTRL_TYPEN_H;
		break;

#ifdef PWM_TYPE_O
	case PWM_TYPE_O:
		tmp = (aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					     AST_PTCR_CLK_EXT_CTRL) &
		       AST_PTCR_CLK_CTRL_TYPEO_H_MASK) >>
		       AST_PTCR_CLK_CTRL_TYPEO_H;
		break;
#endif

	default:
		dev_err(aspeed_pwm_tacho->hwmon_dev, "error channel aspeed_get_pwm_clock_div_h %d\n",
			pwm_type);
		break;
	}
	return tmp;
}

static void
aspeed_set_pwm_clock_division_h(struct aspeed_pwm_tacho_data *aspeed_pwm_tacho,
				u8 pwm_type, u8 div_high)
{
	if (div_high > 0xf)
		return;

	switch (pwm_type) {
	case PWM_TYPE_M:
		aspeed_pwm_tacho_write(aspeed_pwm_tacho,
		(aspeed_pwm_tacho_read(aspeed_pwm_tacho, AST_PTCR_CLK_CTRL) &
		~AST_PTCR_CLK_CTRL_TYPEM_H_MASK) |
		(div_high << AST_PTCR_CLK_CTRL_TYPEM_H), AST_PTCR_CLK_CTRL);
		break;

	case PWM_TYPE_N:
		aspeed_pwm_tacho_write(aspeed_pwm_tacho,
		(aspeed_pwm_tacho_read(aspeed_pwm_tacho, AST_PTCR_CLK_CTRL) &
		~AST_PTCR_CLK_CTRL_TYPEN_H_MASK) |
		(div_high << AST_PTCR_CLK_CTRL_TYPEN_H), AST_PTCR_CLK_CTRL);
		break;

#ifdef PWM_TYPE_O
	case PWM_TYPE_O:
		aspeed_pwm_tacho_write(aspeed_pwm_tacho,
		(aspeed_pwm_tacho_read(aspeed_pwm_tacho,
				       AST_PTCR_CLK_EXT_CTRL) &
		~AST_PTCR_CLK_CTRL_TYPEO_H_MASK) |
		(div_high << AST_PTCR_CLK_CTRL_TYPEO_H),
		AST_PTCR_CLK_EXT_CTRL);
		break;
#endif
	default:
		dev_err(aspeed_pwm_tacho->hwmon_dev, "error channel aspeed_get_pwm_type %d\n",
			pwm_type);
		break;
	}

}

static u8
aspeed_get_pwm_clock_div_l(struct aspeed_pwm_tacho_data *aspeed_pwm_tacho,
			   u8 pwm_type)
{
	u8 tmp = 0;

	switch (pwm_type) {
	case PWM_TYPE_M:
		tmp = (aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					     AST_PTCR_CLK_CTRL) &
		      AST_PTCR_CLK_CTRL_TYPEM_L_MASK) >>
		      AST_PTCR_CLK_CTRL_TYPEM_L;
		break;

	case PWM_TYPE_N:
		tmp = (aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					     AST_PTCR_CLK_CTRL) &
		      AST_PTCR_CLK_CTRL_TYPEN_L_MASK) >>
		      AST_PTCR_CLK_CTRL_TYPEN_L;
		break;

#ifdef PWM_TYPE_O
	case PWM_TYPE_O:
		tmp = (aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					     AST_PTCR_CLK_EXT_CTRL) &
		       AST_PTCR_CLK_CTRL_TYPEO_L_MASK) >>
		       AST_PTCR_CLK_CTRL_TYPEO_L;
		break;

#endif
	default:
		dev_err(aspeed_pwm_tacho->hwmon_dev, "error channel aspeed_get_pwm_clock_div_l %d\n",
			pwm_type);
		break;
	}
	return tmp;
}

static void
aspeed_set_pwm_clock_division_l(struct aspeed_pwm_tacho_data *aspeed_pwm_tacho,
				u8 pwm_type, u8 div_low)
{
	if (div_low > 0xf)
		return;

	switch (pwm_type) {
	case PWM_TYPE_M:
		aspeed_pwm_tacho_write(aspeed_pwm_tacho,
		(aspeed_pwm_tacho_read(aspeed_pwm_tacho, AST_PTCR_CLK_CTRL) &
		~AST_PTCR_CLK_CTRL_TYPEM_L_MASK) |
		(div_low << AST_PTCR_CLK_CTRL_TYPEM_L), AST_PTCR_CLK_CTRL);
		break;

	case PWM_TYPE_N:
		aspeed_pwm_tacho_write(aspeed_pwm_tacho,
		(aspeed_pwm_tacho_read(aspeed_pwm_tacho, AST_PTCR_CLK_CTRL) &
		~AST_PTCR_CLK_CTRL_TYPEN_L_MASK) |
		(div_low << AST_PTCR_CLK_CTRL_TYPEN_L), AST_PTCR_CLK_CTRL);
		break;

#ifdef PWM_TYPE_O
	case PWM_TYPE_O:
		aspeed_pwm_tacho_write(aspeed_pwm_tacho,
		(aspeed_pwm_tacho_read(aspeed_pwm_tacho,
				       AST_PTCR_CLK_EXT_CTRL) &
		~AST_PTCR_CLK_CTRL_TYPEO_L_MASK) |
		(div_low << AST_PTCR_CLK_CTRL_TYPEO_L), AST_PTCR_CLK_EXT_CTRL);
		break;
#endif
	default:
		dev_err(aspeed_pwm_tacho->hwmon_dev, "error channel aspeed_get_pwm_type %d\n",
			pwm_type);
		break;
	}
}

static u8
aspeed_get_pwm_clock_unit(struct aspeed_pwm_tacho_data *aspeed_pwm_tacho,
			  u8 pwm_type)
{
	u8 tmp = 0;

	switch (pwm_type) {
	case PWM_TYPE_M:
		tmp = (aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					     AST_PTCR_CLK_CTRL) &
		      AST_PTCR_CLK_CTRL_TYPEM_UNIT_MASK) >>
		      AST_PTCR_CLK_CTRL_TYPEM_UNIT;
		break;

	case PWM_TYPE_N:
		tmp = (aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					     AST_PTCR_CLK_CTRL) &
		      AST_PTCR_CLK_CTRL_TYPEN_UNIT_MASK) >>
		      AST_PTCR_CLK_CTRL_TYPEN_UNIT;
		break;

#ifdef PWM_TYPE_O
	case PWM_TYPE_O:
		tmp = (aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					     AST_PTCR_CLK_EXT_CTRL) &
		       AST_PTCR_CLK_CTRL_TYPEO_UNIT_MASK) >>
		       AST_PTCR_CLK_CTRL_TYPEO_UNIT;
		break;

#endif
	default:
		dev_err(aspeed_pwm_tacho->hwmon_dev, "error channel aspeed_get_pwm_clock_unit %d\n",
			pwm_type);
		break;
	}

	return tmp;
}

static void
aspeed_set_pwm_clock_unit(struct aspeed_pwm_tacho_data *aspeed_pwm_tacho,
			  u8 pwm_type, u8 unit)
{
	if (unit > 0xff)
		return;

	switch (pwm_type) {
	case PWM_TYPE_M:
		aspeed_pwm_tacho_write(aspeed_pwm_tacho,
		(aspeed_pwm_tacho_read(aspeed_pwm_tacho, AST_PTCR_CLK_CTRL) &
		~AST_PTCR_CLK_CTRL_TYPEM_UNIT_MASK) |
		(unit << AST_PTCR_CLK_CTRL_TYPEM_UNIT),
		AST_PTCR_CLK_CTRL);
		break;

	case PWM_TYPE_N:
		aspeed_pwm_tacho_write(aspeed_pwm_tacho,
		(aspeed_pwm_tacho_read(aspeed_pwm_tacho, AST_PTCR_CLK_CTRL) &
		~AST_PTCR_CLK_CTRL_TYPEN_UNIT_MASK) |
		(unit << AST_PTCR_CLK_CTRL_TYPEN_UNIT),
		AST_PTCR_CLK_CTRL);
		break;

#ifdef PWM_TYPE_O
	case PWM_TYPE_O:
		aspeed_pwm_tacho_write(aspeed_pwm_tacho,
		(aspeed_pwm_tacho_read(aspeed_pwm_tacho,
		AST_PTCR_CLK_EXT_CTRL) &
		~AST_PTCR_CLK_CTRL_TYPEO_UNIT_MASK) |
		(unit << AST_PTCR_CLK_CTRL_TYPEO_UNIT),
		AST_PTCR_CLK_EXT_CTRL);
		break;

#endif
	default:
		dev_err(aspeed_pwm_tacho->hwmon_dev, "error channel aspeed_get_pwm_type %d\n",
			pwm_type);
		break;
	}
}

static u32
aspeed_get_pwm_clock(struct aspeed_pwm_tacho_data *aspeed_pwm_tacho,
		     u8 pwm_type)
{
	u32 unit, div_low, div_high, clk_source;

	unit = aspeed_get_pwm_clock_unit(aspeed_pwm_tacho, pwm_type);

	div_high = aspeed_get_pwm_clock_div_h(aspeed_pwm_tacho, pwm_type);
	div_high = (0x1 << div_high);

	div_low = aspeed_get_pwm_clock_div_l(aspeed_pwm_tacho, pwm_type);
	if (div_low == 0)
		div_low = 1;
	else
		div_low = div_low*2;

	if (AST_PTCR_CTRL_CLK_MCLK & aspeed_pwm_tacho_read(aspeed_pwm_tacho,
	    AST_PTCR_CTRL))
		clk_source = aspeed_pwm_tacho->aspeed_pwm_data->get_pwm_clock();
	else
		clk_source = 24 * 1000 * 1000;

	return (clk_source / (div_high * div_low * (unit + 1)));
}

static u8
aspeed_get_pwm_en(struct aspeed_pwm_tacho_data *aspeed_pwm_tacho, u8 pwm_ch)
{
	u8 tmp = 0;

	switch (pwm_ch) {
	case PWMA:
		tmp = (aspeed_pwm_tacho_read(aspeed_pwm_tacho, AST_PTCR_CTRL) &
		      AST_PTCR_CTRL_PMWA_EN) >> AST_PTCR_CTRL_PMWA;
		break;

	case PWMB:
		tmp = (aspeed_pwm_tacho_read(aspeed_pwm_tacho, AST_PTCR_CTRL) &
		      AST_PTCR_CTRL_PMWB_EN) >> AST_PTCR_CTRL_PMWB;
		break;

	case PWMC:
		tmp = (aspeed_pwm_tacho_read(aspeed_pwm_tacho, AST_PTCR_CTRL) &
		      AST_PTCR_CTRL_PMWC_EN) >> AST_PTCR_CTRL_PMWC;
		break;

	case PWMD:
		tmp = (aspeed_pwm_tacho_read(aspeed_pwm_tacho, AST_PTCR_CTRL) &
		      AST_PTCR_CTRL_PMWD_EN) >> AST_PTCR_CTRL_PMWD;
		break;

	case PWME:
		tmp = (aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					     AST_PTCR_CTRL_EXT) &
		      AST_PTCR_CTRL_PMWE_EN) >> AST_PTCR_CTRL_PMWE;
		break;

	case PWMF:
		tmp = (aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					     AST_PTCR_CTRL_EXT) &
		      AST_PTCR_CTRL_PMWF_EN) >> AST_PTCR_CTRL_PMWF;
		break;

	case PWMG:
		tmp = (aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					     AST_PTCR_CTRL_EXT) &
		      AST_PTCR_CTRL_PMWG_EN) >> AST_PTCR_CTRL_PMWG;
		break;

	case PWMH:
		tmp = (aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					     AST_PTCR_CTRL_EXT) &
		      AST_PTCR_CTRL_PMWH_EN) >> AST_PTCR_CTRL_PMWH;
		break;

	default:
		dev_err(aspeed_pwm_tacho->hwmon_dev, "error channel aspeed_get_pwm_type %d\n",
			pwm_ch);
		break;
	}

	return tmp;

}

static void
aspeed_set_pwm_en(struct aspeed_pwm_tacho_data *aspeed_pwm_tacho, u8 pwm_ch,
		  u8 enable)
{
	switch (pwm_ch) {
	case PWMA:
		if (enable)
			aspeed_pwm_tacho_write(aspeed_pwm_tacho,
				aspeed_pwm_tacho_read(aspeed_pwm_tacho,
						     AST_PTCR_CTRL) |
				AST_PTCR_CTRL_PMWA_EN, AST_PTCR_CTRL);
		else
			aspeed_pwm_tacho_write(aspeed_pwm_tacho,
				aspeed_pwm_tacho_read(aspeed_pwm_tacho,
						      AST_PTCR_CTRL) &
				~AST_PTCR_CTRL_PMWA_EN, AST_PTCR_CTRL);

		break;

	case PWMB:
		if (enable)
			aspeed_pwm_tacho_write(aspeed_pwm_tacho,
				(aspeed_pwm_tacho_read(aspeed_pwm_tacho,
						       AST_PTCR_CTRL) |
				AST_PTCR_CTRL_PMWB_EN), AST_PTCR_CTRL);
		else
			aspeed_pwm_tacho_write(aspeed_pwm_tacho,
				(aspeed_pwm_tacho_read(aspeed_pwm_tacho,
						       AST_PTCR_CTRL) &
				~AST_PTCR_CTRL_PMWB_EN), AST_PTCR_CTRL);
		break;

	case PWMC:
		if (enable)
			aspeed_pwm_tacho_write(aspeed_pwm_tacho,
				(aspeed_pwm_tacho_read(aspeed_pwm_tacho,
						       AST_PTCR_CTRL) |
				AST_PTCR_CTRL_PMWC_EN), AST_PTCR_CTRL);
		else
			aspeed_pwm_tacho_write(aspeed_pwm_tacho,
				(aspeed_pwm_tacho_read(aspeed_pwm_tacho,
						       AST_PTCR_CTRL) &
				~AST_PTCR_CTRL_PMWC_EN), AST_PTCR_CTRL);

		break;

	case PWMD:
		if (enable)
			aspeed_pwm_tacho_write(aspeed_pwm_tacho,
				(aspeed_pwm_tacho_read(aspeed_pwm_tacho,
						       AST_PTCR_CTRL) |
				AST_PTCR_CTRL_PMWD_EN), AST_PTCR_CTRL);
		else
			aspeed_pwm_tacho_write(aspeed_pwm_tacho,
				(aspeed_pwm_tacho_read(aspeed_pwm_tacho,
						       AST_PTCR_CTRL) &
				~AST_PTCR_CTRL_PMWD_EN), AST_PTCR_CTRL);

		break;

	case PWME:
		if (enable)
			aspeed_pwm_tacho_write(aspeed_pwm_tacho,
				(aspeed_pwm_tacho_read(aspeed_pwm_tacho,
						       AST_PTCR_CTRL_EXT) |
				AST_PTCR_CTRL_PMWE_EN), AST_PTCR_CTRL_EXT);
		else
			aspeed_pwm_tacho_write(aspeed_pwm_tacho,
				(aspeed_pwm_tacho_read(aspeed_pwm_tacho,
						       AST_PTCR_CTRL_EXT) &
				~AST_PTCR_CTRL_PMWE_EN), AST_PTCR_CTRL_EXT);

		break;

	case PWMF:
		if (enable)
			aspeed_pwm_tacho_write(aspeed_pwm_tacho,
				(aspeed_pwm_tacho_read(aspeed_pwm_tacho,
						       AST_PTCR_CTRL_EXT) |
				AST_PTCR_CTRL_PMWF_EN), AST_PTCR_CTRL_EXT);
		else
			aspeed_pwm_tacho_write(aspeed_pwm_tacho,
				(aspeed_pwm_tacho_read(aspeed_pwm_tacho,
						       AST_PTCR_CTRL_EXT) &
				~AST_PTCR_CTRL_PMWF_EN), AST_PTCR_CTRL_EXT);

		break;

	case PWMG:
		if (enable)
			aspeed_pwm_tacho_write(aspeed_pwm_tacho,
				(aspeed_pwm_tacho_read(aspeed_pwm_tacho,
						       AST_PTCR_CTRL_EXT) |
				AST_PTCR_CTRL_PMWG_EN), AST_PTCR_CTRL_EXT);
		else
			aspeed_pwm_tacho_write(aspeed_pwm_tacho,
				(aspeed_pwm_tacho_read(aspeed_pwm_tacho,
						       AST_PTCR_CTRL_EXT) &
				~AST_PTCR_CTRL_PMWG_EN), AST_PTCR_CTRL_EXT);

		break;

	case PWMH:
		if (enable)
			aspeed_pwm_tacho_write(aspeed_pwm_tacho,
				(aspeed_pwm_tacho_read(aspeed_pwm_tacho,
						       AST_PTCR_CTRL_EXT) |
				AST_PTCR_CTRL_PMWH_EN), AST_PTCR_CTRL_EXT);
		else
			aspeed_pwm_tacho_write(aspeed_pwm_tacho,
				(aspeed_pwm_tacho_read(aspeed_pwm_tacho,
						       AST_PTCR_CTRL_EXT) &
				~AST_PTCR_CTRL_PMWH_EN), AST_PTCR_CTRL_EXT);

		break;

	default:
		dev_err(aspeed_pwm_tacho->hwmon_dev, "error channel aspeed_get_pwm_type %d\n",
			pwm_ch);
		break;
	}
}

static u8
aspeed_get_pwm_type(struct aspeed_pwm_tacho_data *aspeed_pwm_tacho, u8 pwm_ch)
{
	u8 tmp = 0;

	switch (pwm_ch) {
	case PWMA:
		tmp = AST_PTCR_CTRL_GET_PWMA_TYPE(
			aspeed_pwm_tacho_read(aspeed_pwm_tacho,
			AST_PTCR_CTRL));
		break;

	case PWMB:
		tmp = AST_PTCR_CTRL_GET_PWMB_TYPE(
			aspeed_pwm_tacho_read(aspeed_pwm_tacho,
			AST_PTCR_CTRL));
		break;

	case PWMC:
		tmp = AST_PTCR_CTRL_GET_PWMC_TYPE(
			aspeed_pwm_tacho_read(aspeed_pwm_tacho,
			AST_PTCR_CTRL));
		break;

	case PWMD:
		tmp = AST_PTCR_CTRL_GET_PWMD_TYPE(
			aspeed_pwm_tacho_read(aspeed_pwm_tacho,
			AST_PTCR_CTRL));
		break;

	case PWME:
		tmp = AST_PTCR_CTRL_GET_PWME_TYPE(
			aspeed_pwm_tacho_read(aspeed_pwm_tacho,
			AST_PTCR_CTRL_EXT));
		break;

	case PWMF:
		tmp = AST_PTCR_CTRL_GET_PWMF_TYPE(
			aspeed_pwm_tacho_read(aspeed_pwm_tacho,
			AST_PTCR_CTRL_EXT));
		break;

	case PWMG:
		tmp = AST_PTCR_CTRL_GET_PWMG_TYPE(
			aspeed_pwm_tacho_read(aspeed_pwm_tacho,
			AST_PTCR_CTRL_EXT));
		break;

	case PWMH:
		tmp = AST_PTCR_CTRL_GET_PWMH_TYPE(
			aspeed_pwm_tacho_read(aspeed_pwm_tacho,
			AST_PTCR_CTRL_EXT));
		break;

	default:
		dev_err(aspeed_pwm_tacho->hwmon_dev, "error channel aspeed_get_pwm_type %d\n",
			pwm_ch);
		break;
	}

	return tmp;
}

static void
aspeed_set_pwm_type(struct aspeed_pwm_tacho_data *aspeed_pwm_tacho,
		    u8 pwm_ch, u8 type)
{
	u32 tmp1, tmp2;

	if (type > 0x2)
		return;

	tmp1 = aspeed_pwm_tacho_read(aspeed_pwm_tacho, AST_PTCR_CTRL);
	tmp2 = aspeed_pwm_tacho_read(aspeed_pwm_tacho, AST_PTCR_CTRL_EXT);

	switch (pwm_ch) {
	case PWMA:
		tmp1 &= ~AST_PTCR_CTRL_SET_PWMA_TYPE_MASK;
		tmp1 |= AST_PTCR_CTRL_SET_PWMA_TYPE(type);
		aspeed_pwm_tacho_write(aspeed_pwm_tacho, tmp1, AST_PTCR_CTRL);
		break;

	case PWMB:
		tmp1 &= ~AST_PTCR_CTRL_SET_PWMB_TYPE_MASK;
		tmp1 |= AST_PTCR_CTRL_SET_PWMB_TYPE(type);
		aspeed_pwm_tacho_write(aspeed_pwm_tacho, tmp1, AST_PTCR_CTRL);
		break;

	case PWMC:
		tmp1 &= ~AST_PTCR_CTRL_SET_PWMC_TYPE_MASK;
		tmp1 |= AST_PTCR_CTRL_SET_PWMC_TYPE(type);
		aspeed_pwm_tacho_write(aspeed_pwm_tacho, tmp1, AST_PTCR_CTRL);
		break;
	case PWMD:
		tmp1 &= ~AST_PTCR_CTRL_SET_PWMD_TYPE_MASK;
		tmp1 |= AST_PTCR_CTRL_SET_PWMD_TYPE(type);
		aspeed_pwm_tacho_write(aspeed_pwm_tacho, tmp1, AST_PTCR_CTRL);
		break;

	case PWME:
		tmp2 &= ~AST_PTCR_CTRL_SET_PWME_TYPE_MASK;
		tmp2 |= AST_PTCR_CTRL_SET_PWME_TYPE(type);
		aspeed_pwm_tacho_write(aspeed_pwm_tacho, tmp2,
				       AST_PTCR_CTRL_EXT);
		break;

	case PWMF:
		tmp2 &= ~AST_PTCR_CTRL_SET_PWMF_TYPE_MASK;
		tmp2 |= AST_PTCR_CTRL_SET_PWMF_TYPE(type);
		aspeed_pwm_tacho_write(aspeed_pwm_tacho, tmp2,
				       AST_PTCR_CTRL_EXT);
		break;

	case PWMG:
		tmp2 &= ~AST_PTCR_CTRL_SET_PWMG_TYPE_MASK;
		tmp2 |= AST_PTCR_CTRL_SET_PWMG_TYPE(type);
		aspeed_pwm_tacho_write(aspeed_pwm_tacho, tmp2,
				       AST_PTCR_CTRL_EXT);
		break;

	case PWMH:
		tmp2 &= ~AST_PTCR_CTRL_SET_PWMH_TYPE_MASK;
		tmp2 |= AST_PTCR_CTRL_SET_PWMH_TYPE(type);
		aspeed_pwm_tacho_write(aspeed_pwm_tacho, tmp2,
				       AST_PTCR_CTRL_EXT);
		break;

	default:
		dev_err(aspeed_pwm_tacho->hwmon_dev, "error channel %d\n",
			pwm_ch);
		break;
	}
}

static u8
aspeed_get_pwm_duty_rising(struct aspeed_pwm_tacho_data *aspeed_pwm_tacho,
			   u8 pwm_ch)
{
	u32 tmp = 0;

	switch (pwm_ch) {
	case PWMA:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_DUTY0_CTRL);
		tmp &= DUTY_CTRL0_PWMA_RISE_POINT_MASK;
		break;

	case PWMB:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_DUTY0_CTRL);
		tmp &= DUTY_CTRL0_PWMB_RISE_POINT_MASK;
		tmp = (tmp >> DUTY_CTRL0_PWMB_RISE_POINT);
		break;

	case PWMC:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_DUTY1_CTRL);
		tmp &= DUTY_CTRL1_PWMC_RISE_POINT_MASK;
		break;
	case PWMD:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_DUTY1_CTRL);
		tmp &= DUTY_CTRL1_PWMD_RISE_POINT_MASK;
		tmp = (tmp >> DUTY_CTRL1_PWMD_RISE_POINT);
		break;

	case PWME:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_DUTY2_CTRL);
		tmp &= DUTY_CTRL2_PWME_RISE_POINT_MASK;
		break;

	case PWMF:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_DUTY2_CTRL);
		tmp &= DUTY_CTRL2_PWMF_RISE_POINT_MASK;
		tmp = (tmp >> DUTY_CTRL2_PWMF_RISE_POINT);
		break;

	case PWMG:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_DUTY3_CTRL);
		tmp &= DUTY_CTRL3_PWMG_RISE_POINT_MASK;
		break;

	case PWMH:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_DUTY3_CTRL);
		tmp &= DUTY_CTRL3_PWMH_RISE_POINT_MASK;
		tmp = (tmp >> DUTY_CTRL3_PWMH_RISE_POINT);
		break;

	default:
		dev_err(aspeed_pwm_tacho->hwmon_dev, "error pwm channel %d with duty R\n",
			pwm_ch);
		break;
	}

	return tmp;
}

static void
aspeed_set_pwm_duty_rising(struct aspeed_pwm_tacho_data *aspeed_pwm_tacho,
			   u8 pwm_ch, u8 rising)
{
	u32 tmp = 0;
	u32 pwm_type = aspeed_get_pwm_type(aspeed_pwm_tacho, pwm_ch);

	if ((rising > 0xff) || (rising >
	    aspeed_get_pwm_clock_unit(aspeed_pwm_tacho, pwm_type)))
		return;

	switch (pwm_ch) {
	case PWMA:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_DUTY0_CTRL);
		tmp &= ~DUTY_CTRL0_PWMA_RISE_POINT_MASK;
		tmp |= rising;
		aspeed_pwm_tacho_write(aspeed_pwm_tacho, tmp,
				       AST_PTCR_DUTY0_CTRL);
		break;

	case PWMB:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_DUTY0_CTRL);
		tmp &= ~DUTY_CTRL0_PWMB_RISE_POINT_MASK;
		tmp |= (rising << DUTY_CTRL0_PWMB_RISE_POINT);
		aspeed_pwm_tacho_write(aspeed_pwm_tacho, tmp,
				       AST_PTCR_DUTY0_CTRL);
		break;

	case PWMC:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_DUTY1_CTRL);
		tmp &= ~DUTY_CTRL1_PWMC_RISE_POINT_MASK;
		tmp |= rising;
		aspeed_pwm_tacho_write(aspeed_pwm_tacho, tmp,
				       AST_PTCR_DUTY1_CTRL);
		break;

	case PWMD:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_DUTY1_CTRL);
		tmp &= ~DUTY_CTRL1_PWMD_RISE_POINT_MASK;
		tmp |= (rising << DUTY_CTRL1_PWMD_RISE_POINT);
		aspeed_pwm_tacho_write(aspeed_pwm_tacho, tmp,
				       AST_PTCR_DUTY1_CTRL);
		break;

	case PWME:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_DUTY2_CTRL);
		tmp &= ~DUTY_CTRL2_PWME_RISE_POINT_MASK;
		tmp |= rising;
		aspeed_pwm_tacho_write(aspeed_pwm_tacho, tmp,
				       AST_PTCR_DUTY2_CTRL);
		break;

	case PWMF:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_DUTY2_CTRL);
		tmp &= ~DUTY_CTRL2_PWMF_RISE_POINT_MASK;
		tmp |= (rising << DUTY_CTRL2_PWMF_RISE_POINT);
		aspeed_pwm_tacho_write(aspeed_pwm_tacho, tmp,
				       AST_PTCR_DUTY2_CTRL);
		break;

	case PWMG:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_DUTY3_CTRL);
		tmp &= ~DUTY_CTRL3_PWMG_RISE_POINT_MASK;
		tmp |= rising;
		aspeed_pwm_tacho_write(aspeed_pwm_tacho, tmp,
				       AST_PTCR_DUTY3_CTRL);
		break;

	case PWMH:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_DUTY3_CTRL);
		tmp &= ~DUTY_CTRL3_PWMH_RISE_POINT_MASK;
		tmp |= (rising << DUTY_CTRL3_PWMH_RISE_POINT);
		aspeed_pwm_tacho_write(aspeed_pwm_tacho, tmp,
				       AST_PTCR_DUTY3_CTRL);
		break;

	default:
		dev_err(aspeed_pwm_tacho->hwmon_dev, "error pwm channel %d with duty\n",
			pwm_ch);
		break;
	}
}

static u8
aspeed_get_pwm_duty_falling(struct aspeed_pwm_tacho_data *aspeed_pwm_tacho,
			    u8 pwm_ch)
{
	u32 tmp = 0;

	switch (pwm_ch) {
	case PWMA:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_DUTY0_CTRL);
		tmp &= DUTY_CTRL0_PWMA_FALL_POINT_MASK;
		tmp = (tmp >> DUTY_CTRL0_PWMA_FALL_POINT);
		break;

	case PWMB:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_DUTY0_CTRL);
		tmp &= DUTY_CTRL0_PWMB_FALL_POINT_MASK;
		tmp = (tmp >> DUTY_CTRL0_PWMB_FALL_POINT);
		break;

	case PWMC:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_DUTY1_CTRL);
		tmp &= DUTY_CTRL1_PWMC_FALL_POINT_MASK;
		tmp = (tmp >> DUTY_CTRL1_PWMC_FALL_POINT);
		break;

	case PWMD:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_DUTY1_CTRL);
		tmp &= DUTY_CTRL1_PWMD_FALL_POINT_MASK;
		tmp = (tmp >> DUTY_CTRL1_PWMD_FALL_POINT);
		break;
	case PWME:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_DUTY2_CTRL);
		tmp &= DUTY_CTRL2_PWME_FALL_POINT_MASK;
		tmp = (tmp >> DUTY_CTRL2_PWME_FALL_POINT);
		break;

	case PWMF:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_DUTY2_CTRL);
		tmp &= DUTY_CTRL2_PWMF_FALL_POINT_MASK;
		tmp = (tmp >> DUTY_CTRL2_PWMF_FALL_POINT);
		break;

	case PWMG:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_DUTY3_CTRL);
		tmp &= DUTY_CTRL3_PWMG_FALL_POINT_MASK;
		tmp = (tmp >> DUTY_CTRL3_PWMG_FALL_POINT);
		break;

	case PWMH:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_DUTY3_CTRL);
		tmp &= DUTY_CTRL3_PWMH_FALL_POINT_MASK;
		tmp = (tmp >> DUTY_CTRL3_PWMH_FALL_POINT);
		break;

	default:
		dev_err(aspeed_pwm_tacho->hwmon_dev, "error pwm channel %d with duty F\n",
			pwm_ch);
		break;
	}

	return tmp;
}

static void
aspeed_set_pwm_duty_falling(struct aspeed_pwm_tacho_data *aspeed_pwm_tacho,
			    u8 pwm_ch, u8 falling)
{
	u32 tmp = 0;
	u32 pwm_type = aspeed_get_pwm_type(aspeed_pwm_tacho, pwm_ch);

	if ((falling > 0xff) || (falling >
	    aspeed_get_pwm_clock_unit(aspeed_pwm_tacho, pwm_type)))
		return;

	switch (pwm_ch) {
	case PWMA:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_DUTY0_CTRL);
		tmp &= ~DUTY_CTRL0_PWMA_FALL_POINT_MASK;
		tmp |= (falling << DUTY_CTRL0_PWMA_FALL_POINT);
		aspeed_pwm_tacho_write(aspeed_pwm_tacho, tmp,
				       AST_PTCR_DUTY0_CTRL);
		break;

	case PWMB:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_DUTY0_CTRL);
		tmp &= ~DUTY_CTRL0_PWMB_FALL_POINT_MASK;
		tmp |= (falling << DUTY_CTRL0_PWMB_FALL_POINT);
		aspeed_pwm_tacho_write(aspeed_pwm_tacho, tmp,
				       AST_PTCR_DUTY0_CTRL);
		break;

	case PWMC:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_DUTY1_CTRL);
		tmp &= ~DUTY_CTRL1_PWMC_FALL_POINT_MASK;
		tmp |= (falling << DUTY_CTRL1_PWMC_FALL_POINT);
		aspeed_pwm_tacho_write(aspeed_pwm_tacho, tmp,
				       AST_PTCR_DUTY1_CTRL);
		break;

	case PWMD:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_DUTY1_CTRL);
		tmp &= ~DUTY_CTRL1_PWMD_FALL_POINT_MASK;
		tmp |= (falling << DUTY_CTRL1_PWMD_FALL_POINT);
		aspeed_pwm_tacho_write(aspeed_pwm_tacho, tmp,
				       AST_PTCR_DUTY1_CTRL);
		break;

	case PWME:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_DUTY2_CTRL);
		tmp &= ~DUTY_CTRL2_PWME_FALL_POINT_MASK;
		tmp |= (falling << DUTY_CTRL2_PWME_FALL_POINT);
		aspeed_pwm_tacho_write(aspeed_pwm_tacho, tmp,
				       AST_PTCR_DUTY2_CTRL);
		break;

	case PWMF:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_DUTY2_CTRL);
		tmp &= ~DUTY_CTRL2_PWMF_FALL_POINT_MASK;
		tmp |= (falling << DUTY_CTRL2_PWMF_FALL_POINT);
		aspeed_pwm_tacho_write(aspeed_pwm_tacho, tmp,
				       AST_PTCR_DUTY2_CTRL);
		break;

	case PWMG:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_DUTY3_CTRL);
		tmp &= ~DUTY_CTRL3_PWMG_FALL_POINT_MASK;
		tmp |= (falling << DUTY_CTRL3_PWMG_FALL_POINT);
		aspeed_pwm_tacho_write(aspeed_pwm_tacho, tmp,
				       AST_PTCR_DUTY3_CTRL);
		break;

	case PWMH:
		tmp = aspeed_pwm_tacho_read(aspeed_pwm_tacho,
					    AST_PTCR_DUTY3_CTRL);
		tmp &= ~DUTY_CTRL3_PWMH_FALL_POINT_MASK;
		tmp |= (falling << DUTY_CTRL3_PWMH_FALL_POINT);
		aspeed_pwm_tacho_write(aspeed_pwm_tacho, tmp,
				       AST_PTCR_DUTY3_CTRL);
		break;

	default:
		dev_err(aspeed_pwm_tacho->hwmon_dev, "error pwm channel %d with duty\n",
			pwm_ch);
		break;
	}

}

/* PWM M/N/O Type sysfs
 *
 * Macro defining SENSOR_DEVICE_ATTR for a pwm sysfs entries.
 * 0 - show/store unit
 * 1 - show/store division_l
 * 2 - show/store division_h
 */
static ssize_t
aspeed_show_pwm_type_clock(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct sensor_device_attribute_2 *sensor_attr =
						to_sensor_dev_attr_2(attr);

	switch (sensor_attr->nr) {
	case 0: /* unit : 0~256 */
		return sprintf(buf, "%d (0~255)\n",
			       aspeed_get_pwm_clock_unit(aspeed_pwm_tacho,
						sensor_attr->index));

	case 1: /* division_l */
		return sprintf(buf, "%d (0~15)\n",
			       aspeed_get_pwm_clock_div_l(aspeed_pwm_tacho,
							  sensor_attr->index));

	case 2: /* division_h */
		return sprintf(buf, "%d (0~15)\n",
			       aspeed_get_pwm_clock_div_h(aspeed_pwm_tacho,
							  sensor_attr->index));

	case 3: /* expect clock */
		return sprintf(buf, "%d \n",
			       aspeed_get_pwm_clock(aspeed_pwm_tacho,
						    sensor_attr->index));

	default:
		return -EINVAL;
	}

	return sprintf(buf, "%d : %d\n", sensor_attr->nr, sensor_attr->index);
}

static ssize_t
aspeed_store_pwm_type_clock(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	unsigned long input_val;
	struct sensor_device_attribute_2 *sensor_attr =
						to_sensor_dev_attr_2(attr);
	int ret;

	ret = kstrtoul(buf, 10, &input_val);
	if (ret)
		return ret;

	switch (sensor_attr->nr) {
	case 0: /* unit : 0~256 */
		aspeed_set_pwm_clock_unit(aspeed_pwm_tacho, sensor_attr->index,
					  input_val);
		break;

	case 1: /* division_l */
		aspeed_set_pwm_clock_division_l(aspeed_pwm_tacho,
						sensor_attr->index, input_val);
		break;

	case 2: /* division_h */
		aspeed_set_pwm_clock_division_h(aspeed_pwm_tacho,
						sensor_attr->index, input_val);
		break;

	default:
		return -EINVAL;
	}

	return count;
}

/* attr
 * 0 - show/store enable
 * 1 - show/store type
 * 2 - show/store falling
 * 3 - show/store rising
 */
static ssize_t
aspeed_show_pwm_speed(struct device *dev, struct device_attribute *attr,
		      char *buf)
{
	struct sensor_device_attribute_2 *sensor_attr =
						to_sensor_dev_attr_2(attr);

	switch (sensor_attr->nr) {
	case 0: /* enable, disable */
		return sprintf(buf, "%d : %s\n",
			       aspeed_get_pwm_en(aspeed_pwm_tacho,
						 sensor_attr->index),
			       aspeed_get_pwm_en(aspeed_pwm_tacho,
						 sensor_attr->index) ?
			       "Enable":"Disable");

	case 1: /* pwm type M/N/O */
		return sprintf(buf, "%d (0:M/1:N/2:O)\n",
			       aspeed_get_pwm_type(aspeed_pwm_tacho,
						   sensor_attr->index));

	case 2: /* rising */
		return sprintf(buf, "%x : unit limit (0~%d)\n",
			       aspeed_get_pwm_duty_rising(aspeed_pwm_tacho,
							  sensor_attr->index),
			       aspeed_get_pwm_clock_unit(aspeed_pwm_tacho,
					aspeed_get_pwm_type(aspeed_pwm_tacho,
						sensor_attr->index)));

	case 3: /* falling */
		return sprintf(buf, "%x : unit limit (0~%d)\n",
			       aspeed_get_pwm_duty_falling(aspeed_pwm_tacho,
							   sensor_attr->index),
			       aspeed_get_pwm_clock_unit(aspeed_pwm_tacho,
					aspeed_get_pwm_type(aspeed_pwm_tacho,
						sensor_attr->index)));

	default:
		return -EINVAL;
	}
}

static ssize_t
aspeed_store_pwm_speed(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	unsigned long input_val;
	struct sensor_device_attribute_2 *sensor_attr =
						to_sensor_dev_attr_2(attr);
	int ret;

	ret = kstrtoul(buf, 10, &input_val);
	if (ret)
		return ret;

	switch (sensor_attr->nr) {
	case 0: /* enable, disable */
		aspeed_set_pwm_en(aspeed_pwm_tacho, sensor_attr->index,
				  input_val);
		break;
	case 1: /* pwm type M/N/O */
		aspeed_set_pwm_type(aspeed_pwm_tacho, sensor_attr->index,
				    input_val);
		break;
	case 2: /* rising */
		aspeed_set_pwm_duty_rising(aspeed_pwm_tacho,
					   sensor_attr->index, input_val);
		break;
	case 3: /* falling */
		aspeed_set_pwm_duty_falling(aspeed_pwm_tacho,
					    sensor_attr->index, input_val);
		break;
	default:
		return -EINVAL;
	}

	return count;
}

/* Fan Type  */
/* Fan M/N/O Type sysfs
 * Macro defining SENSOR_DEVICE_ATTR for a pwm sysfs entries.
 * 0 - show/store enable
 * 1 - show/store mode
 * 2 - show/store unit
 * 3 - show/store division
 * 4 - show/store limit
 */
static ssize_t
aspeed_show_tacho_type(struct device *dev, struct device_attribute *attr,
		       char *buf)
{
	struct sensor_device_attribute_2 *sensor_attr =
						to_sensor_dev_attr_2(attr);

	switch (sensor_attr->nr) {
	case 0: /* enable, disable */
		return sprintf(buf, "%d : %s\n",
			aspeed_get_tacho_type_en(aspeed_pwm_tacho,
						 sensor_attr->index),
			aspeed_get_tacho_type_en(aspeed_pwm_tacho,
						 sensor_attr->index) ?
			"Enable":"Disable");

	case 1: /* fan tacho mode */
		switch (aspeed_get_tacho_type_mode(
			aspeed_pwm_tacho, sensor_attr->index)) {
		case FALL_EDGE:
			return sprintf(buf, "0: falling\n");
		case RISE_EDGE:
			return sprintf(buf, "1: rising\n");
		case BOTH_EDGE:
			return sprintf(buf, "2: both\n");
		default:
			return sprintf(buf, "3: unknown\n");
		}
		break;

	case 2: /* unit */
		return sprintf(buf, "%d (0~65535)\n",
			aspeed_get_tacho_type_unit(aspeed_pwm_tacho,
						   sensor_attr->index));

	case 3: /* division */
		return sprintf(buf, "%d (0~7)\n",
			aspeed_get_tacho_type_division(aspeed_pwm_tacho,
						       sensor_attr->index));

	case 4: /* limit */
		return sprintf(buf, "%d (0~1048575)\n",
			aspeed_get_tacho_type_limit(aspeed_pwm_tacho,
						    sensor_attr->index));

	case 5: /* measure period */
		return sprintf(buf, "%d\n",
			aspeed_get_tacho_measure_period(aspeed_pwm_tacho,
							sensor_attr->index));

	default:
		return -EINVAL;
	}
}

static ssize_t
aspeed_store_tacho_type(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	unsigned long input_val;
	struct sensor_device_attribute_2 *sensor_attr =
						to_sensor_dev_attr_2(attr);
	int ret;

	ret = kstrtoul(buf, 10, &input_val);
	if (ret)
		return ret;

	switch (sensor_attr->nr) {
	case 0: /* enable, disable */
		aspeed_set_tacho_type_en(aspeed_pwm_tacho, sensor_attr->index,
					 input_val);
		break;
	case 1: /* fan tacho mode */
		aspeed_set_tacho_type_mode(aspeed_pwm_tacho,
					   sensor_attr->index, input_val);
		break;

	case 2: /* unit */
		aspeed_set_tacho_type_unit(aspeed_pwm_tacho,
					   sensor_attr->index, input_val);
		break;

	case 3: /* division */
		aspeed_set_tacho_type_division(aspeed_pwm_tacho,
					       sensor_attr->index, input_val);
		break;

	case 4: /* limit */
		aspeed_set_tacho_type_limit(aspeed_pwm_tacho,
					    sensor_attr->index, input_val);
		break;

	default:
		return -EINVAL;
	}

	return count;

}

/* fan detect */
/* FAN sysfs
 * Macro defining SENSOR_DEVICE_ATTR for a tacho sysfs entries.
 *  - show/store enable
 *  - show/store source
 *  - show/store rpm
 *  - show/store alarm
 *  - show/store alarm_en
*/
static ssize_t
aspeed_show_tacho_speed(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct sensor_device_attribute_2 *sensor_attr =
						to_sensor_dev_attr_2(attr);

	switch (sensor_attr->nr)  {
	case 0: /* enable, disable */
		return sprintf(buf, "%d : %s\n",
			       aspeed_get_tacho_en(aspeed_pwm_tacho,
						   sensor_attr->index),
			       aspeed_get_tacho_en(aspeed_pwm_tacho,
						   sensor_attr->index) ?
			       "Enable":"Disable");

	case 1: /* tacho source PWMA~H - 0~7 */
		return sprintf(buf, "PWM%d (0~7)\n",
			       aspeed_get_tacho_source(aspeed_pwm_tacho,
						       sensor_attr->index));

	case 2: /* rpm */
		{
			const s32 rpm = aspeed_get_tacho_rpm(aspeed_pwm_tacho,
				sensor_attr->index);

			if (rpm < 0)
				return rpm;
			return sprintf(buf, "%d\n", rpm);
		}

	case 3: /* alarm */
		return sprintf(buf, "%d \n",
			       aspeed_get_tacho_alarm(aspeed_pwm_tacho,
						      sensor_attr->index));

	case 4: /* alarm_en */
		return sprintf(buf, "%d : %s\n",
			       aspeed_get_tacho_alarm_en(aspeed_pwm_tacho,
							 sensor_attr->index),
			       aspeed_get_tacho_alarm_en(aspeed_pwm_tacho,
							 sensor_attr->index) ?
			       "Enable":"Disable");

	default:
		return -EINVAL;
	}

}

static ssize_t
aspeed_store_tacho_speed(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	unsigned long input_val;
	struct sensor_device_attribute_2 *sensor_attr =
						to_sensor_dev_attr_2(attr);
	int ret;

	ret = kstrtoul(buf, 10, &input_val);
	if (ret)
		return ret;

	switch (sensor_attr->nr)  {
	case 0: /* enable, disable */
		aspeed_set_tacho_en(aspeed_pwm_tacho, sensor_attr->index,
				    input_val);
		break;

	case 1: /* tacho source PWMA~H - 0~7 */
		aspeed_set_tacho_source(aspeed_pwm_tacho, sensor_attr->index,
					input_val);
		break;

	case 4: /* alarm_en */
		aspeed_set_tacho_alarm_en(aspeed_pwm_tacho, sensor_attr->index,
					  input_val);
		break;

	case 2: /* rpm */
	case 3: /* alarm */
	default:
		return -EINVAL;
	}

	return count;
}

/* sysfs attributes */

/* CLK sysfs */
static SENSOR_DEVICE_ATTR_2
(clk_en, S_IRUGO | S_IWUSR, aspeed_show_clk, aspeed_store_clk, 0, 0);
static SENSOR_DEVICE_ATTR_2
(clk_source, S_IRUGO | S_IWUSR, aspeed_show_clk, aspeed_store_clk, 1, 0);

static struct attribute *clk_attributes[] = {
	&sensor_dev_attr_clk_source.dev_attr.attr,
	&sensor_dev_attr_clk_en.dev_attr.attr,
	NULL
};

static const struct attribute_group clk_attribute_groups = {
	.attrs = clk_attributes,
};

/* PWM M/N/O Type sysfs
 * Macro defining SENSOR_DEVICE_ATTR for a pwm sysfs entries.
 * 0 - show/store unit
 * 1 - show/store division_l
 * 2 - show/store division_h
 */
#define sysfs_pwm_type(type,  index) \
static SENSOR_DEVICE_ATTR_2(pwm_type_##type##_unit, S_IRUGO | S_IWUSR, \
	aspeed_show_pwm_type_clock, aspeed_store_pwm_type_clock, 0, index); \
static SENSOR_DEVICE_ATTR_2(pwm_type_##type##_division_l, S_IRUGO | S_IWUSR, \
	aspeed_show_pwm_type_clock, aspeed_store_pwm_type_clock, 1, index); \
static SENSOR_DEVICE_ATTR_2(pwm_type_##type##_division_h, S_IRUGO | S_IWUSR, \
	aspeed_show_pwm_type_clock, aspeed_store_pwm_type_clock, 2, index); \
static SENSOR_DEVICE_ATTR_2(pwm_type_##type##_clk, S_IRUGO, \
	aspeed_show_pwm_type_clock, NULL, 3, index); \
static struct attribute *pwm_type_##type##_attributes[] = { \
	&sensor_dev_attr_pwm_type_##type##_unit.dev_attr.attr, \
	&sensor_dev_attr_pwm_type_##type##_division_l.dev_attr.attr, \
	&sensor_dev_attr_pwm_type_##type##_division_h.dev_attr.attr, \
	&sensor_dev_attr_pwm_type_##type##_clk.dev_attr.attr, \
	NULL \
}

/*
 * Create the needed functions for each pwm using the macro defined above
 * (4 pwms are supported)
 */
sysfs_pwm_type(m, 0);
sysfs_pwm_type(n, 1);
#ifdef PWM_TYPE_O
sysfs_pwm_type(o, 2);
#endif

static const struct attribute_group pwm_type_attribute_groups[] = {
	{ .attrs = pwm_type_m_attributes },
	{ .attrs = pwm_type_n_attributes },
#ifdef PWM_TYPE_O
	{ .attrs = pwm_type_o_attributes },
#endif
};

/* PWM sysfs
 * Macro defining SENSOR_DEVICE_ATTR for a pwm sysfs entries.
 * 0 - show/store enable
 * 1 - show/store type
 * 2 - show/store rising
 * 3 - show/store falling
 */
#define sysfs_pwm_speeds_num(index) \
static SENSOR_DEVICE_ATTR_2(pwm##index##_en, S_IRUGO | S_IWUSR, \
	aspeed_show_pwm_speed, aspeed_store_pwm_speed, 0, index); \
static SENSOR_DEVICE_ATTR_2(pwm##index##_type, S_IRUGO | S_IWUSR, \
	aspeed_show_pwm_speed, aspeed_store_pwm_speed, 1, index); \
static SENSOR_DEVICE_ATTR_2(pwm##index##_rising, S_IRUGO | S_IWUSR, \
	aspeed_show_pwm_speed, aspeed_store_pwm_speed, 2, index); \
static SENSOR_DEVICE_ATTR_2(pwm##index##_falling, S_IRUGO | S_IWUSR, \
	aspeed_show_pwm_speed, aspeed_store_pwm_speed, 3, index); \
static struct attribute *pwm##index##_attributes[] = { \
	&sensor_dev_attr_pwm##index##_en.dev_attr.attr, \
	&sensor_dev_attr_pwm##index##_type.dev_attr.attr, \
	&sensor_dev_attr_pwm##index##_rising.dev_attr.attr, \
	&sensor_dev_attr_pwm##index##_falling.dev_attr.attr, \
	NULL \
}

/*
 * Create the needed functions for each pwm using the macro defined above
 * (4 pwms are supported)
 */
sysfs_pwm_speeds_num(0);
sysfs_pwm_speeds_num(1);
sysfs_pwm_speeds_num(2);
sysfs_pwm_speeds_num(3);
sysfs_pwm_speeds_num(4);
sysfs_pwm_speeds_num(5);
sysfs_pwm_speeds_num(6);
sysfs_pwm_speeds_num(7);

static const struct attribute_group pwm_attribute_groups[] = {
	{ .attrs = pwm0_attributes },
	{ .attrs = pwm1_attributes },
	{ .attrs = pwm2_attributes },
	{ .attrs = pwm3_attributes },
	{ .attrs = pwm4_attributes },
	{ .attrs = pwm5_attributes },
	{ .attrs = pwm6_attributes },
	{ .attrs = pwm7_attributes },
};

/* Fan M/N/O Type sysfs
 * Macro defining SENSOR_DEVICE_ATTR for a pwm sysfs entries.
 *  0 - show/store enable
 *  1 - show/store mode
 *  2 - show/store unit
 *  3 - show/store division
 *  4 - show/store limit
 */
#define sysfs_tacho_type(type, index) \
static SENSOR_DEVICE_ATTR_2(tacho_type_##type##_en, S_IRUGO | S_IWUSR, \
	aspeed_show_tacho_type, aspeed_store_tacho_type, 0, index); \
static SENSOR_DEVICE_ATTR_2(tacho_type_##type##_mode, S_IRUGO | S_IWUSR, \
	aspeed_show_tacho_type, aspeed_store_tacho_type, 1, index); \
static SENSOR_DEVICE_ATTR_2(tacho_type_##type##_unit, S_IRUGO | S_IWUSR, \
	aspeed_show_tacho_type, aspeed_store_tacho_type, 2, index); \
static SENSOR_DEVICE_ATTR_2(tacho_type_##type##_division, S_IRUGO | S_IWUSR, \
	aspeed_show_tacho_type, aspeed_store_tacho_type, 3, index); \
static SENSOR_DEVICE_ATTR_2(tacho_type_##type##_limit, S_IRUGO | S_IWUSR, \
	aspeed_show_tacho_type, aspeed_store_tacho_type, 4, index); \
static SENSOR_DEVICE_ATTR_2(tacho_type_##type##_measure_period, \
	S_IRUGO | S_IWUSR, \
	aspeed_show_tacho_type, aspeed_store_tacho_type, 5, index); \
static struct attribute *tacho_type_##type##_attributes[] = { \
	&sensor_dev_attr_tacho_type_##type##_en.dev_attr.attr, \
	&sensor_dev_attr_tacho_type_##type##_mode.dev_attr.attr, \
	&sensor_dev_attr_tacho_type_##type##_unit.dev_attr.attr, \
	&sensor_dev_attr_tacho_type_##type##_division.dev_attr.attr, \
	&sensor_dev_attr_tacho_type_##type##_limit.dev_attr.attr, \
	&sensor_dev_attr_tacho_type_##type##_measure_period.dev_attr.attr, \
	NULL \
}

/*
 * Create the needed functions for each pwm using the macro defined above
 * (4 pwms are supported)
 */
sysfs_tacho_type(m, 0);
sysfs_tacho_type(n, 1);
#ifdef PWM_TYPE_O
sysfs_tacho_type(o, 2);
#endif

static const struct attribute_group tacho_type_attribute_groups[] = {
	{ .attrs = tacho_type_m_attributes },
	{ .attrs = tacho_type_n_attributes },
#ifdef PWM_TYPE_O
	{ .attrs = tacho_type_o_attributes },
#endif
};

/* FAN sysfs
 * Macro defining SENSOR_DEVICE_ATTR for a tacho sysfs entries.
 *  - show/store enable
 *  - show/store source
 *  - show/store rpm
 *  - show/store alarm
 *  - show/store alarm_en
 */
#define sysfs_tacho_speeds_num(index) \
static SENSOR_DEVICE_ATTR_2(tacho##index##_en, S_IRUGO | S_IWUSR, \
	aspeed_show_tacho_speed, aspeed_store_tacho_speed, 0, index); \
static SENSOR_DEVICE_ATTR_2(tacho##index##_source, S_IRUGO | S_IWUSR, \
	aspeed_show_tacho_speed, aspeed_store_tacho_speed, 1, index); \
static SENSOR_DEVICE_ATTR_2(tacho##index##_rpm, S_IRUGO, \
	aspeed_show_tacho_speed, NULL, 2, index); \
static SENSOR_DEVICE_ATTR_2(tacho##index##_alarm, S_IRUGO, \
	aspeed_show_tacho_speed, aspeed_store_tacho_speed, 3, index); \
static SENSOR_DEVICE_ATTR_2(tacho##index##_alarm_en, S_IRUGO | S_IWUSR, \
	aspeed_show_tacho_speed, aspeed_store_tacho_speed, 4, index); \
static struct attribute *tacho##index##_attributes[] = { \
	&sensor_dev_attr_tacho##index##_en.dev_attr.attr, \
	&sensor_dev_attr_tacho##index##_source.dev_attr.attr, \
	&sensor_dev_attr_tacho##index##_rpm.dev_attr.attr, \
	&sensor_dev_attr_tacho##index##_alarm.dev_attr.attr, \
	&sensor_dev_attr_tacho##index##_alarm_en.dev_attr.attr, \
	NULL \
}

/*
 * Create the needed functions for each tacho using the macro defined above
 * (4 tachos are supported)
 */
sysfs_tacho_speeds_num(0);
sysfs_tacho_speeds_num(1);
sysfs_tacho_speeds_num(2);
sysfs_tacho_speeds_num(3);
sysfs_tacho_speeds_num(4);
sysfs_tacho_speeds_num(5);
sysfs_tacho_speeds_num(6);
sysfs_tacho_speeds_num(7);
sysfs_tacho_speeds_num(8);
sysfs_tacho_speeds_num(9);
sysfs_tacho_speeds_num(10);
sysfs_tacho_speeds_num(11);
sysfs_tacho_speeds_num(12);
sysfs_tacho_speeds_num(13);
sysfs_tacho_speeds_num(14);
sysfs_tacho_speeds_num(15);

static const struct attribute_group tacho_attribute_groups[] = {
	{ .attrs = tacho0_attributes },
	{ .attrs = tacho1_attributes },
	{ .attrs = tacho2_attributes },
	{ .attrs = tacho3_attributes },
	{ .attrs = tacho4_attributes },
	{ .attrs = tacho5_attributes },
	{ .attrs = tacho6_attributes },
	{ .attrs = tacho7_attributes },
	{ .attrs = tacho8_attributes },
	{ .attrs = tacho9_attributes },
	{ .attrs = tacho10_attributes },
	{ .attrs = tacho11_attributes },
	{ .attrs = tacho12_attributes },
	{ .attrs = tacho13_attributes },
	{ .attrs = tacho14_attributes },
	{ .attrs = tacho15_attributes },
};

static int
aspeed_pwm_cz_get_max_state(struct thermal_cooling_device *tcdev,
			    unsigned long *state)
{
	struct aspeed_pwm_port_data *pwm_port =
				(struct aspeed_pwm_port_data *)tcdev->devdata;

	*state = pwm_port->max_state;

	return 0;
}

static int
aspeed_pwm_cz_get_cur_state(struct thermal_cooling_device *tcdev,
			    unsigned long *state)
{
	struct aspeed_pwm_port_data *pwm_port =
				(struct aspeed_pwm_port_data *)tcdev->devdata;

	u8 cooling_level;

	switch (pwm_port->pwm_port_ctrl) {
	case FALL_EDGE:
		cooling_level = aspeed_get_pwm_duty_falling(pwm_port->priv,
							    pwm_port->channel);
		break;

	case RISE_EDGE:
		cooling_level = aspeed_get_pwm_duty_rising(pwm_port->priv,
							   pwm_port->channel);
		break;

	case BOTH_EDGE:
	default:
		return -EEXIST;
	}

	if (pwm_port->cooling_level != cooling_level)
		pwm_port->cooling_level = cooling_level;

	*state = pwm_port->cooling_level;

	return 0;
}

static int
aspeed_pwm_cz_set_cur_state(struct thermal_cooling_device *tcdev,
			    unsigned long state)
{
	struct aspeed_pwm_port_data *pwm_port =
				(struct aspeed_pwm_port_data *)tcdev->devdata;

	pwm_port->cooling_level = (state >= pwm_port->max_state) ?
		pwm_port->cooling_level : state;

	switch (pwm_port->pwm_port_ctrl) {
	case FALL_EDGE:
		aspeed_set_pwm_duty_falling(pwm_port->priv, pwm_port->channel,
					    pwm_port->cooling_level);
		break;

	case RISE_EDGE:
		aspeed_set_pwm_duty_rising(pwm_port->priv, pwm_port->channel,
					   pwm_port->cooling_level);
		break;

	case BOTH_EDGE:
	default:
		return -EEXIST;
	}

	return 0;
}

static const struct thermal_cooling_device_ops aspeed_pwm_cool_ops = {
	.get_max_state = aspeed_pwm_cz_get_max_state,
	.get_cur_state = aspeed_pwm_cz_get_cur_state,
	.set_cur_state = aspeed_pwm_cz_set_cur_state,
};

#define PWM priv->pwm_port[index]
static int aspeed_create_pwm_ports(struct platform_device *pdev,
				   struct device_node *parent,
				   struct aspeed_pwm_tacho_data *priv)
{
	struct device_node *child;
	u8 val, index;
	int num, i;
	int err;

	for_each_child_of_node(parent, child) {
		err = of_property_read_u8(child, "pwm-port", &index);
		if (err)
			return err;

		PWM = devm_kzalloc(&pdev->dev, sizeof(*PWM), GFP_KERNEL);
		if (!PWM) {
			err = -ENOMEM;
			goto exit;
		}

		PWM->channel = index;
		aspeed_set_pwm_en(aspeed_pwm_tacho, index, 1);

		if (!of_property_read_u8(child, "type", &val)) {
			aspeed_set_pwm_type(aspeed_pwm_tacho, index, val);
			PWM->pwm_port_type = val;
		}


		err = of_property_count_u32_elems(child, "cooling-levels");
		if (err <= 0) {
			dev_err(&pdev->dev, "Wrong cooling-levels data.\n");
			err = -EINVAL;
			goto exit;
		}

		num = err;
		PWM->cooling_levels = devm_kzalloc(&pdev->dev, num *
						sizeof(u32), GFP_KERNEL);
		if (!PWM->cooling_levels) {
			err = -ENOMEM;
			goto exit;
		}

		err = of_property_read_u32_array(child, "cooling-levels",
						 PWM->cooling_levels, num);
		if (err) {
			dev_err(&pdev->dev, "Property 'cooling-levels' cannot be read.\n");
			goto exit;
		}

		for (i = 0; i < num; i++) {
			if (PWM->cooling_levels[i] > PWM_CH_NUM) {
				dev_err(&pdev->dev, "PWM state[%d]:%d > %d\n",
					i, PWM->cooling_levels[i], PWM_CH_NUM);
				err = -EINVAL;
				goto exit;
			}
		}

		PWM->max_state = num - 1;

		if (!of_property_read_u8(child, "cooling-level", &val))
			PWM->cooling_level = val;

		err = of_property_read_u8(child, "mode-selection",
					  &val);
		if (!err) {
			switch (val) {
			case FALL_EDGE:
				if ((PWM->cooling_level >
				     PWM->cooling_levels[0]) ||
				    (PWM->cooling_level <
				     PWM->cooling_levels[PWM->max_state])) {
					dev_err(&pdev->dev, "PWM cooling-level property is not in range %d:%d > %d\n",
					PWM->cooling_levels[0],
					PWM->cooling_levels[PWM->max_state],
					PWM->cooling_level);
					err = -EINVAL;
					goto exit;
				}
				aspeed_set_pwm_duty_falling(aspeed_pwm_tacho,
					index,
					PWM->cooling_level);
			break;

			case RISE_EDGE:
				if ((PWM->cooling_level <
				     PWM->cooling_levels[0]) ||
				    (PWM->cooling_level >
				     PWM->cooling_levels[PWM->max_state])) {
					dev_err(&pdev->dev, "PWM cooling-level property is not in range %d:%d > %d\n",
					PWM->cooling_levels[0],
					PWM->cooling_levels[PWM->max_state],
					PWM->cooling_level);
					err = -EINVAL;
					goto exit;
				}
				aspeed_set_pwm_duty_rising(aspeed_pwm_tacho,
					index,
					PWM->cooling_level);
			break;

			case BOTH_EDGE:
			default:
				err = -EEXIST;
				goto exit;
			}
		}

		sprintf(PWM->name, "%sd%d", "aspeed-pwm", index);
		PWM->tcdev = thermal_cooling_device_register(PWM->name, PWM,
							&aspeed_pwm_cool_ops);
		if (PTR_ERR_OR_ZERO(PWM->tcdev)) {
			err = PTR_ERR(PWM->tcdev);
			goto exit;
		}

		PWM->priv = priv;
	}

	return 0;

exit:
	for (index = 0; index < PWM_CH_NUM; index++) {
		if (PWM && PWM->tcdev)
			thermal_cooling_device_unregister(PWM->tcdev);
	}

	return err;
}

static int
aspeed_pwm_tacho_probe(struct platform_device *pdev)
{
	struct device_node *child;
	struct resource *res;
	int i, j = 0;
	int err = 0;

	if (!of_device_is_compatible(pdev->dev.of_node,
				     "aspeed,aspeed2500-pwm-fan"))
		return -ENODEV;

	aspeed_pwm_tacho = devm_kzalloc(&pdev->dev, sizeof(*aspeed_pwm_tacho),
					GFP_KERNEL);
	if (!aspeed_pwm_tacho) {
		err = -ENOMEM;
		goto out;
	}

	aspeed_pwm_tacho->aspeed_pwm_data = dev_get_platdata(&pdev->dev);
	platform_set_drvdata(pdev, aspeed_pwm_tacho);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		err = -ENOENT;
		goto out;
	}

	if (!request_mem_region(res->start, resource_size(res), res->name)) {
		dev_err(&pdev->dev, "cannot reserved region\n");
		err = -ENXIO;
		goto out;
	}

	aspeed_pwm_tacho->reg_base = ioremap(res->start, resource_size(res));
	if (!aspeed_pwm_tacho->reg_base) {
		err = -EIO;
		goto out_region;
	}

	aspeed_pwm_tacho->irq = platform_get_irq(pdev, 0);
	if (aspeed_pwm_tacho->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		err = -ENOENT;
		goto out_region;
	}

	aspeed_pwm_tacho->groups[j++] = &clk_attribute_groups;

	for (i = 0; i < PWM_CH_NUM; i++)
		aspeed_pwm_tacho->groups[j++] = &pwm_attribute_groups[i];

	for_each_child_of_node(pdev->dev.of_node, child) {
		if (!of_node_cmp(child->name, "pwm-port"))
			err = aspeed_create_pwm_ports(pdev, child,
						      aspeed_pwm_tacho);
			if (err)
				goto out_region;
	}

	for (i = 0; i < PWM_TYPE_NUM; i++)
		aspeed_pwm_tacho->groups[j++] = &pwm_type_attribute_groups[i];

	for (i = 0; i < TACHO_NUM; i++)
		aspeed_pwm_tacho->groups[j++] =  &tacho_attribute_groups[i];

	for (i = 0; i < PWM_TYPE_NUM; i++)
		aspeed_pwm_tacho->groups[j++] =
					&tacho_type_attribute_groups[i];

	aspeed_pwm_tacho->groups[j] = NULL;

	aspeed_pwm_tacho->hwmon_dev = devm_hwmon_device_register_with_groups(
						&pdev->dev, "aspeed_pwm_tacho",
						aspeed_pwm_tacho,
						aspeed_pwm_tacho->groups);

	if (PTR_ERR_OR_ZERO(aspeed_pwm_tacho->hwmon_dev)) {
		dev_err(&pdev->dev, "cannot register hwmon\n");
		err = PTR_ERR(aspeed_pwm_tacho->hwmon_dev);
		goto out_region;
	}

	/* SCU Pin-MUX PWM & TACHO */
	aspeed_scu_multi_func_reset(AST_SCU_FUN_PIN_CTRL3, 0xcfffff, 0xc000ff);

	/* SCU PWM CTRL Reset */
	aspeed_toggle_scu_reset(SCU_RESET_PWM, 3);

	aspeed_pwm_tacho_init();

	dev_info(&pdev->dev, "aspeed pwm tacho: driver successfully loaded.\n");

	return 0;

out_region:
	release_mem_region(res->start, res->end - res->start + 1);
out:
	dev_err(&pdev->dev, "aspeed pwm tacho: driver init failed (err=%d)!\n",
		err);
	return err;
}

static int aspeed_pwm_tacho_remove(struct platform_device *pdev)
{
	struct aspeed_pwm_tacho_data *aspeed_pwm_tacho =
						platform_get_drvdata(pdev);
	struct resource *res;

	platform_set_drvdata(pdev, NULL);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	iounmap(aspeed_pwm_tacho->reg_base);
	release_mem_region(res->start, res->end - res->start + 1);
	dev_info(&pdev->dev, "aspeed_pwm_tacho: driver unloaded.\n");

	return 0;
}

#ifdef CONFIG_PM
static int
aspeed_pwm_tacho_suspend(struct platform_device *pdev, pm_message_t state)
{
	dev_info(&pdev->dev, "aspeed_pwm_tacho_suspend\n");

	return 0;
}

static int
aspeed_pwm_tacho_resume(struct platform_device *pdev)
{
	aspeed_pwm_tacho_init();
	return 0;
}

#else
#define aspeed_pwm_tacho_suspend        NULL
#define aspeed_pwm_tacho_resume         NULL
#endif

static const struct of_device_id aspeed_pwm_fan_of_table[] = {
	{ .compatible = "aspeed,aspeed2500-pwm-fan", },
	{ },
};
MODULE_DEVICE_TABLE(of, aspeed_pwm_fan_of_table);

static struct platform_driver aspeed_pwm_tacho_driver = {
	.remove		= aspeed_pwm_tacho_remove,
	.suspend	= aspeed_pwm_tacho_suspend,
	.resume		= aspeed_pwm_tacho_resume,
	.driver		= {
		.name   = KBUILD_MODNAME,
		.owner  = THIS_MODULE,
		.of_match_table = aspeed_pwm_fan_of_table,
	},
};
module_platform_driver_probe(aspeed_pwm_tacho_driver, aspeed_pwm_tacho_probe);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED PWM TACHO driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:aspeed-pwm-fan");
