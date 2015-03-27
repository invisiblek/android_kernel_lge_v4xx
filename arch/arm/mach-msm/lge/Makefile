obj-$(CONFIG_MACH_LGE) += devices_lge.o

obj-$(CONFIG_MACH_MSM8226_E7WIFI) += board-8226-e7wifi.o board-8226-e7wifi-gpiomux.o
obj-$(CONFIG_MACH_MSM8926_E7LTE_ATT_US) += board-8926-e7lte-att-us.o board-8926-e7lte-att-us-gpiomux.o

obj-$(CONFIG_LGE_BOOTLOADER_LOG) += lge_bootloader_log.o
obj-$(CONFIG_LGE_BOOTLOADER_TIME_CHECKER) += lge_bootloader_time_checker.o
obj-$(CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO) += lge_charging_scenario.o

obj-$(CONFIG_MACH_LGE) += lge_kcal_ctrl.o

obj-$(CONFIG_LGE_QFPROM_INTERFACE) += lge_qfprom_access.o

ifdef CONFIG_LGE_PM_BATTERY_PROFILE_DATA
obj-$(CONFIG_LGE_PM_BATTERY_CAPACITY_1540mAh) += bms-batterydata-LGC_BL44JR_1540.o
obj-$(CONFIG_LGE_PM_BATTERY_CAPACITY_1700mAh) += bms-batterydata-LGC_BL44JH_1700.o bms-batterydata-LGC_BL44JH_1700_Dallas.o bms-batterydata-Tocad_BL44JH_1700_Intersil.o
obj-$(CONFIG_LGE_PM_BATTERY_CAPACITY_4000mAh) += bms-batterydata_LGE_BL_T12_4000mAh_TOCAD_Final.o bms-batterydata_LGE_BL_T12_4000mAh_LG_Chem_Final.o
endif

CFLAGS_devices_lge.o += -Idrivers/staging/android
