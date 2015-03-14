/*
 * include/linux/input/sx9500_platform_data.h
 *
 * SX9500 Platform Data
 * 4 cap single  
 *
 * Copyright 2013 Semtech Corp.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef _SX9500_PLATFORM_DATA_H_
#define _SX9500_PLATFORM_DATA_H_



struct smtc_reg_data {
    unsigned char reg;
    unsigned char val;
};

struct _buttonInfo {
    /*! The Key to send to the input */
    int keycode;
    /*! Mask to look for on Touch Status */
    int mask;
    /*! Current state of button. */
    int state;
};

struct _totalButtonInformation {
    struct _buttonInfo *buttons;
    int buttonSize;
    struct input_dev *input;
};

struct _touchCheckParameters {
    u8 defaultStartupMainSensor;
#if defined(CONFIG_MACH_MSM8926_E8LTE)
    u8 defaultSecondStartupMainSensor;
#endif
    u8 defaultStartupRefSensor;  
    s32 main_csx;
    s32 ref_csx; // should be a lot less than 16 bits but just in case
    s32 minimum_threshold;
    s32 calibratoin_margin;
    u8 duringTouch_AvgThresh;
    u8 duringRelease_AvgThresh;
};

struct sx9500_platform_data {
    int i2c_reg_num;
    struct smtc_reg_data *pi2c_reg;

    struct _totalButtonInformation *pbuttonInformation;

    struct _touchCheckParameters   *ptouchCheckParameters;

    int (*get_is_nirq_low)(void);

    int     (*init_platform_hw)(void);
    /*int     (*init_platform_hw)(struct i2c_client *client);*/
    void    (*exit_platform_hw)(void);

    bool i2c_pull_up;
    bool digital_pwr_regulator;
    unsigned int irq_gpio;

    u32 vdd_ana_supply_min;
    u32 vdd_ana_supply_max;
    u32 vdd_ana_load_ua;

    u32 vddio_dig_supply_min;
    u32 vddio_dig_supply_max;
    u32 vddio_dig_load_ua;

    u32 vddio_i2c_supply_min;
    u32 vddio_i2c_supply_max;
    u32 vddio_i2c_load_ua;
};

/*! \struct sx9500
 * Specialized struct containing input event data, platform data, and
 * last cap state read if needed.
 */
struct sx9500
{
    struct _totalButtonInformation *pbuttonInformation;
    struct _touchCheckParameters *ptouchCheckParameters;
    struct sx9500_platform_data *hw; /* specific platform data settings */
};

//static int initialize_device(struct sx86XX *this);
void touchCheckWithReferenceSensor(struct sx86XX *this,
                                          unsigned char mainSensor,
                                          unsigned char refSensor);

#endif
