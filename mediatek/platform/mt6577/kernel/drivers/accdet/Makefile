obj-$(CONFIG_MTK_ACCDET) := accdet.o

ccflags-y += -I$(MTK_PATH_PLATFORM)/drivers/power\
             -I$(MTK_PATH_SOURCE)/drivers/power

ifeq ($(PROJECT),telsa)
ifeq ($(JRD_2160KEYPAD),no)
else
ifeq ($(JRD_IPHONE_HDST_COMPAT_CONFIG), yes)
    ccflags-y += -DJRD_IPHONE_HDST_COMPATIBILITY
endif
endif
endif
