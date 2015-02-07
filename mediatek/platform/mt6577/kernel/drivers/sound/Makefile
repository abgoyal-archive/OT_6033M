obj-$(CONFIG_MTK_SOUND) += auddrv_kernel_driver.o auddrv_register.o auddrv_sine_tone.o

ifeq ($(PROJECT),telsa)
ifeq ($(JRD_2160KEYPAD),no)
else
ifeq ($(JRD_IPHONE_HDST_COMPAT_CONFIG), yes)
    ccflags-y += -DJRD_IPHONE_HDST_COMPATIBILITY
endif
endif
endif
