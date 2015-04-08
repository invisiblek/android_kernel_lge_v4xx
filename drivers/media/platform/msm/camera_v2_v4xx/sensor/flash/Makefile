ccflags-y += -Idrivers/media/platform/msm/camera_v2_v4xx
ccflags-y += -Idrivers/media/platform/msm/camera_v2_v4xx/sensor/io
obj-$(CONFIG_MSMB_CAMERA) += msm_led_flash.o
obj-$(CONFIG_MSMB_CAMERA) += msm_led_trigger.o
ifndef CONFIG_MACH_LGE
obj-$(CONFIG_MSMB_CAMERA) += msm_led_i2c_trigger.o
obj-$(CONFIG_MSMB_CAMERA) += adp1660.o
endif
obj-$(CONFIG_MSMB_CAMERA) += msm_led_torch.o
