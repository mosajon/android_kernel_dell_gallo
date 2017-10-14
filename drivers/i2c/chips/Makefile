#
# Makefile for miscellaneous I2C chip drivers.
#
# Do not add new drivers to this directory! It is DEPRECATED.
#
# Device drivers are better grouped according to the functionality they
# implement rather than to the bus they are connected to. In particular:
# * Hardware monitoring chip drivers go to drivers/hwmon
# * RTC chip drivers go to drivers/rtc
# * I/O expander drivers go to drivers/gpio
#

obj-$(CONFIG_DS1682)           += ds1682.o
obj-$(CONFIG_SENSORS_PCA963X)  += pca963x.o
obj-$(CONFIG_SENSORS_TSL2550)  += tsl2550.o
obj-$(CONFIG_SENSORS_HAPTIC_ISA1200_CLASS) += haptic-class.o
obj-$(CONFIG_SENSORS_HAPTIC_ISA1200) += isa1200.o
obj-$(CONFIG_CG7216AM) += cg7216am.o
obj-$(CONFIG_LDS6202) += lds6202.o

ifeq ($(CONFIG_I2C_DEBUG_CHIP), y)
EXTRA_CFLAGS += -DDEBUG
endif

