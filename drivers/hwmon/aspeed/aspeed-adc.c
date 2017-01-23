/*
 * Copyright (c) 2017 Mellanox Technologies. All rights reserved.
 * Copyright (c) 2017 Vadim Pasternak <vadimp@mellanox.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the names of the copyright holders nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *  History:
 *  2012.11.26: Initial version Ryan Chen <ryan_chen@aspeedtech.com>
 *  2017.01.20: Porting to kernel 4.11 Vadim Pasternak <vadimp@mellanox.com>
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <asm/irq.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/timer.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/reset.h>

#define ASPEED_ADC_CTRL			0x00
#define ASPEED_ADC_IER			0x04
#define ASPEED_ADC_CLK			0x0c
#define ASPEED_ADC_CH0_1		0x10
#define ASPEED_ADC_CH2_3		0x14
#define ASPEED_ADC_CH4_5		0x18
#define ASPEED_ADC_CH6_7		0x1c
#define ASPEED_ADC_CH8_9		0x20
#define ASPEED_ADC_CH10_11		0x24
#define ASPEED_ADC_CH12_13		0x28
#define ASPEED_ADC_CH14_15		0x2c
#define ASPEED_ADC_BOUND0		0x30
#define ASPEED_ADC_HYSTER0		0x70
#define ASPEED_ADC_CH16			0xD0
#define ASPEED_ADC_CH17			0xD4
#define ASPEED_ADC_COMP_TRIM		0xC4
#define ASPEED_ADC_CTRL_CH15_EN		(0x1 << 31)
#define ASPEED_ADC_CTRL_CH14_EN		(0x1 << 30)
#define ASPEED_ADC_CTRL_CH13_EN		(0x1 << 29)
#define ASPEED_ADC_CTRL_CH12_EN		(0x1 << 28)
#define ASPEED_ADC_CTRL_CH11_EN		(0x1 << 27)
#define ASPEED_ADC_CTRL_CH10_EN		(0x1 << 26)
#define ASPEED_ADC_CTRL_CH9_EN		(0x1 << 25)
#define ASPEED_ADC_CTRL_CH8_EN		(0x1 << 24)
#define ASPEED_ADC_CTRL_CH7_EN		(0x1 << 23)
#define ASPEED_ADC_CTRL_CH6_EN		(0x1 << 22)
#define ASPEED_ADC_CTRL_CH5_EN		(0x1 << 21)
#define ASPEED_ADC_CTRL_CH4_EN		(0x1 << 20)
#define ASPEED_ADC_CTRL_CH3_EN		(0x1 << 19)
#define ASPEED_ADC_CTRL_CH2_EN		(0x1 << 18)
#define ASPEED_ADC_CTRL_CH1_EN		(0x1 << 17)
#define ASPEED_ADC_CTRL_CH0_EN		(0x1 << 16)
#define ASPEED_ADC_CTRL_INIT_RDY	(0x1 << 8)
#define ASPEED_ADC_CTRL_COMPEN		(0x1 << 5)
#define ASPEED_ADC_TEMP_CH_RDY		(0x1 << 31)
#define ASPEED_ADC_GET_TEMP_A_MASK(x)	((x >> 16) & 0xfff)
#define ASPEED_ADC_TEMP_CH_EN		(0x1 << 15)
#define ASPEED_ADC_GET_TEMP_B_MASK(x)	(x & 0xfff)
#define ASPEED_ADC_CTRL_NORMAL		(0x7 << 1)
#define ASPEED_ADC_CTRL_EN		(0x1)
#define ASPEED_ADC_H_CH_MASK		(0x3ff << 16)
#define ASPEED_ADC_L_CH_MASK		(0x3ff)
#define ASPEED_ADC_H_BOUND		(0x3ff << 16)
#define ASPEED_ADC_L_BOUND		(0x3ff)
#define ASPEED_ADC_HYSTER_EN		(0x1 << 31)
#define ASPEED_ADC_MAX_CH_NO		16
#define ASPEED_ADC_TEMP_CH_NO		2

#define ASPEED_ADC_TEMP_CALC(t)		(((t - 247) * 100) / (320 - 247))

struct aspeed_adc_data {
	struct device	*dev;
	void __iomem	*reg_base;	/* Virtual IO */
	int		irq;		/* ADC IRQ number */
	struct clk	*clk;		/* Clock */
	int		compen_value;	/* Compensating value */
};

static inline void
aspeed_adc_write(struct aspeed_adc_data *aspeed_adc, u32 val, u32 reg)
{
	writel(val, aspeed_adc->reg_base + reg);
}

static inline u32
aspeed_adc_read(struct aspeed_adc_data *aspeed_adc, u32 reg)
{
	return readl(aspeed_adc->reg_base + reg);
}

static void aspeed_adc_ctrl_init(struct aspeed_adc_data *aspeed_adc)
{
	u8 trim = 0;
	/*
	 * Set wait a sensing cycle t (s) = 1000 * 12 * (1/PCLK) * 2 *
	 * (ADC0c[31:17] + 1) * (ADC0c[9:0] +1)
	 * ex : pclk = 48Mhz , ADC0c[31:17] = 0,  ADC0c[9:0] = 0x40 : 64,
	 * ADC0c[31:17] = 0x3e7 : 999
	 * --> 0.0325s	= 12 * 2 * (0x3e7 + 1) *(64+1) / 48000000
	 * --> 0.0005s	= 12 * 2 * (0x3e7 + 1) / 48000000
	 * Note:
	   trim = ast_scu_read(AST_SCU_OTP1) >> 28
	 */

	if (!trim)
		trim = 0x8;

	aspeed_adc_write(aspeed_adc, trim, ASPEED_ADC_COMP_TRIM);
	aspeed_adc_write(aspeed_adc, 0x40, ASPEED_ADC_CLK);
	aspeed_adc_write(aspeed_adc, ASPEED_ADC_CTRL_NORMAL |
			 ASPEED_ADC_CTRL_EN, ASPEED_ADC_CTRL);

	while (!(aspeed_adc_read(aspeed_adc, ASPEED_ADC_CTRL) &
				 ASPEED_ADC_CTRL_INIT_RDY))
	;

	aspeed_adc_write(aspeed_adc, ASPEED_ADC_CTRL_COMPEN |
			 ASPEED_ADC_CTRL_NORMAL | ASPEED_ADC_CTRL_EN,
			 ASPEED_ADC_CTRL);

	while (aspeed_adc_read(aspeed_adc, ASPEED_ADC_CTRL) &
			       ASPEED_ADC_CTRL_COMPEN)
	;

	/* compensating value = 0x200 - ADC10[9:0] */
	aspeed_adc->compen_value = 0x200 -
		((aspeed_adc_read(aspeed_adc, ASPEED_ADC_COMP_TRIM) >> 16) &
				  0x3ff);
}

static u16
aspeed_adc_get_hyst_lo(struct aspeed_adc_data *aspeed_adc, u8 ch)
{
	return aspeed_adc_read(aspeed_adc, ASPEED_ADC_HYSTER0 + (ch * 4)) &
			       ASPEED_ADC_L_BOUND;
}

static void
aspeed_adc_set_hyst_lo(struct aspeed_adc_data *aspeed_adc, u8 ch,
		       u16 value)
{
	aspeed_adc_write(aspeed_adc,
			(aspeed_adc_read(aspeed_adc, ASPEED_ADC_HYSTER0 +
					 (ch * 4)) & ~ASPEED_ADC_L_BOUND) |
					 value, ASPEED_ADC_HYSTER0 + (ch * 4));
}

static u16
aspeed_adc_get_hyst_up(struct aspeed_adc_data *aspeed_adc, u8 ch)
{
	return ((aspeed_adc_read(aspeed_adc, ASPEED_ADC_HYSTER0 +
				 (ch * 4)) & ASPEED_ADC_H_BOUND) >> 16);
}

static void
aspeed_adc_set_hyst_up(struct aspeed_adc_data *aspeed_adc, u8 ch, u32 value)
{
	aspeed_adc_write(aspeed_adc,
			(aspeed_adc_read(aspeed_adc, ASPEED_ADC_HYSTER0 +
			(ch * 4)) & ~ASPEED_ADC_H_BOUND) | (value << 16),
			ASPEED_ADC_HYSTER0 + (ch * 4));
}

static u8
aspeed_adc_get_hyster_en(struct aspeed_adc_data *aspeed_adc, u8 ch)
{
	if (aspeed_adc_read(aspeed_adc, ASPEED_ADC_HYSTER0 + (ch * 4)) &
			   ASPEED_ADC_HYSTER_EN)
		return 1;

	return 0;
}

static void
aspeed_adc_set_hyst_en(struct aspeed_adc_data *aspeed_adc, u8 ch, u8 enable)
{
	if (enable)
		aspeed_adc_write(aspeed_adc,
			aspeed_adc_read(aspeed_adc, ASPEED_ADC_HYSTER0 +
					(ch * 4)) | ASPEED_ADC_HYSTER_EN,
					ASPEED_ADC_HYSTER0 + (ch * 4));
	else
		aspeed_adc_write(aspeed_adc,
			aspeed_adc_read(aspeed_adc, ASPEED_ADC_HYSTER0 +
					(ch * 4)) & ~ASPEED_ADC_HYSTER_EN,
					ASPEED_ADC_HYSTER0 + (ch * 4));
}

static u16
aspeed_adc_get_lower(struct aspeed_adc_data *aspeed_adc, u8 ch)
{
	return aspeed_adc_read(aspeed_adc, ASPEED_ADC_BOUND0 + (ch * 4)) &
			       ASPEED_ADC_L_BOUND;
}

static void
aspeed_adc_set_lo(struct aspeed_adc_data *aspeed_adc, u8 ch, u16 value)
{
	aspeed_adc_write(aspeed_adc,
			(aspeed_adc_read(aspeed_adc, ASPEED_ADC_BOUND0 +
					 (ch * 4)) & ~ASPEED_ADC_L_BOUND) |
					 value, ASPEED_ADC_BOUND0 + (ch * 4));
}

static u16
aspeed_adc_get_upper(struct aspeed_adc_data *aspeed_adc, u8 ch)
{
	return ((aspeed_adc_read(aspeed_adc, ASPEED_ADC_BOUND0 +
				 (ch * 4)) & ASPEED_ADC_H_BOUND) >> 16);
}

static void
aspeed_adc_set_up(struct aspeed_adc_data *aspeed_adc, u8 ch, u32 value)
{
	aspeed_adc_write(aspeed_adc,
			 (aspeed_adc_read(aspeed_adc, ASPEED_ADC_BOUND0 +
					  (ch * 4)) & ~ASPEED_ADC_H_BOUND) |
					  (value << 16), ASPEED_ADC_BOUND0 +
					  (ch * 4));
}

static u8
aspeed_adc_get_alarm(struct aspeed_adc_data *aspeed_adc, u8 ch)
{
	if (aspeed_adc_read(aspeed_adc, ASPEED_ADC_IER) & (0x1 << ch))
		return 1;

	return 0;
}

static u16
aspeed_adc_get_value(struct aspeed_adc_data *aspeed_adc, u8 ch)
{
	int tmp;

	switch (ch) {
	case 0:
		tmp = aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH0_1) &
				      ASPEED_ADC_L_CH_MASK;
		break;
	case 1:
		tmp = (aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH0_1) &
				       ASPEED_ADC_H_CH_MASK) >> 16;
		break;
	case 2:
		tmp = aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH2_3) &
				      ASPEED_ADC_L_CH_MASK;
		break;
	case 3:
		tmp = (aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH2_3) &
				       ASPEED_ADC_H_CH_MASK) >> 16;
		break;
	case 4:
		tmp = aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH4_5) &
				      ASPEED_ADC_L_CH_MASK;
		break;
	case 5:
		tmp = (aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH4_5) &
				       ASPEED_ADC_H_CH_MASK) >> 16;
		break;
	case 6:
		tmp = aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH6_7) &
				      ASPEED_ADC_L_CH_MASK;
		break;
	case 7:
		tmp = (aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH6_7) &
				       ASPEED_ADC_H_CH_MASK) >> 16;
		break;
	case 8:
		tmp = aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH8_9) &
				      ASPEED_ADC_L_CH_MASK;
		break;
	case 9:
		tmp = (aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH8_9) &
				       ASPEED_ADC_H_CH_MASK) >> 16;
		break;
	case 10:
		tmp = aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH10_11) &
				      ASPEED_ADC_L_CH_MASK;
		break;
	case 11:
		tmp = (aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH10_11) &
				       ASPEED_ADC_H_CH_MASK) >> 16;
		break;
	case 12:
		tmp = aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH12_13) &
					      ASPEED_ADC_L_CH_MASK;
		break;
	case 13:
		tmp = (aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH12_13) &
				       ASPEED_ADC_H_CH_MASK) >> 16;
		break;
	case 14:
		tmp = aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH14_15) &
				      ASPEED_ADC_L_CH_MASK;
		break;
	case 15:
		tmp = (aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH14_15) &
				       ASPEED_ADC_H_CH_MASK) >> 16;
		break;
	}

	tmp += aspeed_adc->compen_value;

	return (tmp < 0) ? 0 : tmp;
}

static u8
aspeed_adc_get_en(struct aspeed_adc_data *aspeed_adc, u8 ch)
{
	if (aspeed_adc_read(aspeed_adc, ASPEED_ADC_CTRL) & (0x1 << (16 + ch)))
		return 1;

	return 0;

}

static void
aspeed_adc_set_en(struct aspeed_adc_data *aspeed_adc, u8 ch, u8 enable)
{
	if (enable)
		aspeed_adc_write(aspeed_adc,
				 aspeed_adc_read(aspeed_adc, ASPEED_ADC_CTRL) |
						 (0x1 << (16 + ch)),
						 ASPEED_ADC_CTRL);
	else
		aspeed_adc_write(aspeed_adc,
				 aspeed_adc_read(aspeed_adc, ASPEED_ADC_CTRL) &
						 ~(0x1 << (16 + ch)),
						 ASPEED_ADC_CTRL);
}

/*
 * attr ADC sysfs 0~max adc channel
 * 0 - show/store channel enable
 * 1 - show value
 * 2 - show alarm get statuse
 * 3 - show/store upper
 * 4 - show/store lower
 * 5 - show/store hystersis enable
 * 6 - show/store hystersis upper
 * 7 - show/store hystersis low
 */
static ssize_t
aspeed_adc_show_adc(struct device *dev, struct device_attribute *attr,
		    char *buf)
{
	struct aspeed_adc_data *aspeed_adc = dev_get_drvdata(dev);
	struct sensor_device_attribute_2 *attr2 = to_sensor_dev_attr_2(attr);
	u8 en;

	switch (attr2->nr) {
	case 0:
		en = aspeed_adc_get_en(aspeed_adc, attr2->index);
		return sprintf(buf, "%d: %s\n", en, en ? "Enable" : "Disable");
	case 1:
		return sprintf(buf, "%d\n", aspeed_adc_get_value(aspeed_adc,
								attr2->index));
	case 2:
		return sprintf(buf, "%d\n", aspeed_adc_get_alarm(aspeed_adc,
								attr2->index));
	case 3:
		return sprintf(buf, "%d\n", aspeed_adc_get_upper(aspeed_adc,
								attr2->index));
	case 4:
		return sprintf(buf, "%d\n", aspeed_adc_get_lower(aspeed_adc,
								attr2->index));
	case 5:
		en = aspeed_adc_get_hyster_en(aspeed_adc, attr2->index);
		return sprintf(buf, "%d: %s\n", en, en ? "Enable" : "Disable");
	case 6:
		return sprintf(buf, "%d\n", aspeed_adc_get_hyst_up(aspeed_adc,
								attr2->index));
	case 7:
		return sprintf(buf, "%d\n", aspeed_adc_get_hyst_lo(aspeed_adc,
								attr2->index));
	default:
		return -EINVAL;
	}
}

static ssize_t
aspeed_adc_store_adc(struct device *dev, struct device_attribute *attr,
		     const char *buf, size_t count)
{
	struct aspeed_adc_data *aspeed_adc = dev_get_drvdata(dev);
	struct sensor_device_attribute_2 *attr2 = to_sensor_dev_attr_2(attr);
	unsigned long val;
	int err;

	err = kstrtoul(buf, 10, &val);
	if (err)
		return err;

	switch (attr2->nr) {
	case 0:
		aspeed_adc_set_en(aspeed_adc, attr2->index, val);
		break;
	case 1:
	case 2:
		break;
	case 3:
		aspeed_adc_set_up(aspeed_adc, attr2->index, val);
		break;
	case 4:
		aspeed_adc_set_lo(aspeed_adc, attr2->index, val);
		break;
	case 5:
		aspeed_adc_set_hyst_en(aspeed_adc, attr2->index, val);
		break;
	case 6:
		aspeed_adc_set_hyst_up(aspeed_adc, attr2->index, val);
		break;
	case 7:
		aspeed_adc_set_hyst_lo(aspeed_adc, attr2->index, val);
		break;
	default:
		return -EINVAL;
	}

	return count;
}

static u16
aspeed_adc_get_temp(struct aspeed_adc_data *aspeed_adc, u8 ch)
{
	u16 temp;

	switch (ch) {
	case 0:
		aspeed_adc_write(aspeed_adc,
				 aspeed_adc_read(aspeed_adc,
						 ASPEED_ADC_CH16) |
						 ASPEED_ADC_TEMP_CH_EN,
						 ASPEED_ADC_CH16);

		while (!(aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH16) &
					 ASPEED_ADC_TEMP_CH_RDY))
		;

		temp = ASPEED_ADC_GET_TEMP_A_MASK(aspeed_adc_read(aspeed_adc,
						  ASPEED_ADC_CH16)) -
		       ASPEED_ADC_GET_TEMP_B_MASK(aspeed_adc_read(aspeed_adc,
						  ASPEED_ADC_CH16));
		break;

	case 1:
		aspeed_adc_write(aspeed_adc,
				 aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH17) |
						 ASPEED_ADC_TEMP_CH_EN,
						 ASPEED_ADC_CH17);

		while (!(aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH17) &
					 ASPEED_ADC_TEMP_CH_RDY))
		;

		temp = ASPEED_ADC_GET_TEMP_A_MASK(aspeed_adc_read(aspeed_adc,
						  ASPEED_ADC_CH17)) -
		       ASPEED_ADC_GET_TEMP_B_MASK(aspeed_adc_read(aspeed_adc,
						  ASPEED_ADC_CH17));
		break;
	}

	return ASPEED_ADC_TEMP_CALC(temp);

}

static ssize_t
aspeed_adc_show_temp(struct device *dev, struct device_attribute *attr,
		     char *buf)
{
	struct aspeed_adc_data *aspeed_adc = dev_get_drvdata(dev);
	struct sensor_device_attribute_2 *attr2 = to_sensor_dev_attr_2(attr);

	switch (attr2->nr) {
	case 0:
		return sprintf(buf, "%d\n", aspeed_adc_get_temp(aspeed_adc,
								attr2->index));

	default:
		return -EINVAL;
	}
}

/*
 * attr temperature sysfs 0~max t channel
 * 0 - show value
 */
#define sysfs_temp_ch(index) \
static SENSOR_DEVICE_ATTR_2(temp##index##_value, S_IRUGO | S_IWUSR, \
	aspeed_adc_show_temp, NULL, 0, index); \
static struct attribute *temp##index##_attributes[] = { \
	&sensor_dev_attr_temp##index##_value.dev_attr.attr, \
	NULL \
}

sysfs_temp_ch(0);
sysfs_temp_ch(1);

static const struct attribute_group aspeed_temp_attribute_groups[] = {
	{ .attrs = temp0_attributes },
	{ .attrs = temp1_attributes },
};

/*
 * attr ADC sysfs 0~max adc channel
 * 0 - show/store channel enable
 * 1 - show value
 * 2 - show alarm get statuse
 * 3 - show/store upper
 * 4 - show/store lower
 * 5 - show/store hystersis enable
 * 6 - show/store hystersis upper
 * 7 - show/store hystersis low
 */
#define sysfs_adc_ch(index) \
static SENSOR_DEVICE_ATTR_2(adc##index##_en, S_IRUGO | S_IWUSR, \
	aspeed_adc_show_adc, aspeed_adc_store_adc, 0, index); \
static SENSOR_DEVICE_ATTR_2(adc##index##_value, S_IRUGO | S_IWUSR, \
	aspeed_adc_show_adc, NULL, 1, index); \
static SENSOR_DEVICE_ATTR_2(adc##index##_alarm, S_IRUGO | S_IWUSR, \
	aspeed_adc_show_adc, NULL, 2, index); \
static SENSOR_DEVICE_ATTR_2(adc##index##_upper, S_IRUGO | S_IWUSR, \
	aspeed_adc_show_adc, aspeed_adc_store_adc, 3, index); \
static SENSOR_DEVICE_ATTR_2(adc##index##_lower, S_IRUGO | S_IWUSR, \
	aspeed_adc_show_adc, aspeed_adc_store_adc, 4, index); \
static SENSOR_DEVICE_ATTR_2(adc##index##_hyster_en, S_IRUGO | S_IWUSR, \
	aspeed_adc_show_adc, aspeed_adc_store_adc, 5, index); \
static SENSOR_DEVICE_ATTR_2(adc##index##_hyst_up, S_IRUGO | S_IWUSR, \
	aspeed_adc_show_adc, aspeed_adc_store_adc, 6, index); \
static SENSOR_DEVICE_ATTR_2(adc##index##_hyst_lo, S_IRUGO | S_IWUSR, \
	aspeed_adc_show_adc, aspeed_adc_store_adc, 7, index); \
static struct attribute *adc##index##_attributes[] = { \
	&sensor_dev_attr_adc##index##_en.dev_attr.attr, \
	&sensor_dev_attr_adc##index##_value.dev_attr.attr, \
	&sensor_dev_attr_adc##index##_alarm.dev_attr.attr, \
	&sensor_dev_attr_adc##index##_upper.dev_attr.attr, \
	&sensor_dev_attr_adc##index##_lower.dev_attr.attr, \
	&sensor_dev_attr_adc##index##_hyster_en.dev_attr.attr, \
	&sensor_dev_attr_adc##index##_hyst_up.dev_attr.attr, \
	&sensor_dev_attr_adc##index##_hyst_lo.dev_attr.attr, \
	NULL \
}

/*
 * Create the needed functions for each pwm using the macro defined above
 * (4 pwms are supported)
 */
sysfs_adc_ch(0);
sysfs_adc_ch(1);
sysfs_adc_ch(2);
sysfs_adc_ch(3);
sysfs_adc_ch(4);
sysfs_adc_ch(5);
sysfs_adc_ch(6);
sysfs_adc_ch(7);
sysfs_adc_ch(8);
sysfs_adc_ch(9);
sysfs_adc_ch(10);
sysfs_adc_ch(11);
sysfs_adc_ch(12);
sysfs_adc_ch(13);
sysfs_adc_ch(14);
sysfs_adc_ch(15);

static const struct attribute_group aspeed_adc_attribute_groups[] = {
	{ .attrs = adc0_attributes },
	{ .attrs = adc1_attributes },
	{ .attrs = adc2_attributes },
	{ .attrs = adc3_attributes },
	{ .attrs = adc4_attributes },
	{ .attrs = adc5_attributes },
	{ .attrs = adc6_attributes },
	{ .attrs = adc7_attributes },
	{ .attrs = adc8_attributes },
	{ .attrs = adc9_attributes },
	{ .attrs = adc10_attributes },
	{ .attrs = adc11_attributes },
	{ .attrs = adc12_attributes },
	{ .attrs = adc13_attributes },
	{ .attrs = adc14_attributes },
	{ .attrs = adc15_attributes },
};

static int
aspeed_adc_probe(struct platform_device *pdev)
{
	struct aspeed_adc_data *aspeed_adc;
	struct resource *res;
	int i;
	int err;

	if (!of_device_is_compatible(pdev->dev.of_node,
				     "aspeed,aspeed2500-adc"))
		return -ENODEV;

	/* SCU ADC CTRL Reset */
	aspeed_toggle_scu_reset(SCU_RESET_ADC, 3);

	aspeed_adc = devm_kzalloc(&pdev->dev, sizeof(struct aspeed_adc_data),
				  GFP_KERNEL);
	if (!aspeed_adc)
		return -ENOMEM;
	platform_set_drvdata(pdev, aspeed_adc);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		return -ENOENT;
	}

	aspeed_adc->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(aspeed_adc->reg_base))
		return PTR_ERR(aspeed_adc->reg_base);

	aspeed_adc->irq = platform_get_irq(pdev, 0);
	if (aspeed_adc->irq < 0)
		return -ENOENT;

	aspeed_adc->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(aspeed_adc->clk))
		return PTR_ERR(aspeed_adc->clk);

	/* Register sysfs hooks */
	aspeed_adc->dev = hwmon_device_register(&pdev->dev);
	if (IS_ERR(aspeed_adc->dev))
		return PTR_ERR(aspeed_adc->dev);

	for (i = 0; i < ASPEED_ADC_MAX_CH_NO; i++) {
		err = sysfs_create_group(&pdev->dev.kobj,
					 &aspeed_adc_attribute_groups[i]);
		if (err)
			return err;
	}

	for (i = 0; i < ASPEED_ADC_TEMP_CH_NO; i++) {
		err = sysfs_create_group(&pdev->dev.kobj,
					 &aspeed_temp_attribute_groups[i]);
		if (err)
			return err;
	}

	aspeed_adc_ctrl_init(aspeed_adc);

	return 0;
}

static int
aspeed_adc_remove(struct platform_device *pdev)
{
	struct aspeed_adc_data *aspeed_adc = platform_get_drvdata(pdev);
	int i = 0;

	hwmon_device_unregister(aspeed_adc->dev);

	for (i = 0; i < ASPEED_ADC_MAX_CH_NO; i++)
		sysfs_remove_group(&pdev->dev.kobj,
				   &aspeed_adc_attribute_groups[i]);

	for (i = 0; i < ASPEED_ADC_TEMP_CH_NO; i++)
		sysfs_remove_group(&pdev->dev.kobj,
				   &aspeed_temp_attribute_groups[i]);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM
static int
aspeed_adc_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int
aspeed_adc_resume(struct platform_device *pdev)
{
	struct aspeed_adc_data *aspeed_adc = platform_get_drvdata(pdev);

	aspeed_adc_ctrl_init(aspeed_adc);

	return 0;
}

#else
#define aspeed_adc_suspend        NULL
#define aspeed_adc_resume         NULL
#endif

static const struct of_device_id aspeed_adc_of_table[] = {
	{ .compatible = "aspeed,aspeed2500-adc", },
	{ },
};
MODULE_DEVICE_TABLE(of, aspeed_pwm_fan_of_table);

static struct platform_driver aspeed_adc_driver = {
	.remove		= aspeed_adc_remove,
	.suspend        = aspeed_adc_suspend,
	.resume         = aspeed_adc_resume,
	.driver         = {
		.name   = "aspeed_adc",
		.owner  = THIS_MODULE,
		.of_match_table = aspeed_adc_of_table,
	},
};

module_platform_driver_probe(aspeed_adc_driver, aspeed_adc_probe);

MODULE_AUTHOR("Vadim Pasternak <vadimp@mellanox.com>");
MODULE_DESCRIPTION("Aspeed ADC controller platform driver");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_ALIAS("platform:aspeed-adc");
