PROJECT += diablohd

ifeq ($(PROJECT),telsa)
EXTRA_CFLAGS += -DTELSA_FAN5405_GPIO -DTELSA_FUEL_GAUGE
endif

ifeq ($(PROJECT),diablohd)
EXTRA_CFLAGS += -DDIABLOHD_FUEL_GAUGE
EXTRA_CFLAGS += -DDIABLOHD_CHARGING_POLICY_SUPPORT
EXTRA_CFLAGS += -DBATT_ID_CHECK_SUPPORT
EXTRA_CFLAGS += -DDIABLOHD_BAT_ID_CHECK
endif

ifeq ($(PROJECT),jrdsh77_cu_jb)
EXTRA_CFLAGS += -DHIGH_BATTERY_VOLTAGE_SUPPORT
EXTRA_CFLAGS += -DBATT_ID_CHECK_SUPPORT
EXTRA_CFLAGS += -DDIABLO_BAT_ID_CHECK
EXTRA_CFLAGS += -DDIABLO_FUEL_GAUGE
endif

obj-$(CONFIG_MTK_PMIC)          += pmic_mt6329.o pmic_mt6329_bank1.o upmu_common.o

ifeq ($(MTK_NCP1851_SUPPORT),yes) 
    obj-$(CONFIG_MTK_SMART_BATTERY) += mt_battery_tbl.o mt_fuel_gauge_tbl.o ncp1851.o
else
    ifeq ($(MTK_FAN5405_SUPPORT),yes)
        #obj-$(CONFIG_MTK_SMART_BATTERY) += mt_battery_fan5405.o mt_fuel_gauge.o fan5405.o
        obj-$(CONFIG_MTK_SMART_BATTERY) += mt_battery_fan5405.o mt_fuel_gauge.o fan5405_jrd.o
    else
        obj-$(CONFIG_MTK_SMART_BATTERY) += mt_battery.o mt_fuel_gauge.o
    endif
endif
