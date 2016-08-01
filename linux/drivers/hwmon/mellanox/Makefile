#
# Makefile for Mellanox chip drivers.
#

obj-$(CONFIG_MELLANOX_I2C_BACKEND)		+= mlnx-asic-i2c.o
obj-$(CONFIG_MELLANOX_ASIC_CORE)                += sx_core.o
sx_core-y                      			:= sx_core_main.o cq.o intf.o dq.o eq.o \
                                   		alloc.o cmd.o reset.o \
                                   		fw.o icm.o sx_dpt.o sx_proc.o
obj-$(CONFIG_MELLANOX_ASIC_HWMON)		+= mlnx-asic-drv.o

