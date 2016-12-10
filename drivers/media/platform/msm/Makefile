obj-$(CONFIG_MSM_VIDC_V4L2) += vidc/
obj-$(CONFIG_MSM_WFD) += wfd/
obj-$(CONFIG_DVB_MPQ) += dvb/

ifeq ($(CONFIG_MACH_MSM8226_E7WIFI),y)
obj-$(CONFIG_MSMB_CAMERA) += camera_v2_v4xx/
else
ifeq ($(CONFIG_MACH_MSM8926_E7LTE_ATT_US),y)
obj-$(CONFIG_MSMB_CAMERA) += camera_v2_v4xx/
else
ifeq ($(CONFIG_MACH_MSM8226_E8WIFI),y)
obj-$(CONFIG_MSMB_CAMERA) += camera_v2_v4xx/
else
obj-$(CONFIG_MSMB_CAMERA) += camera_v2/
endif
endif
endif

obj-y += vcap/
