/*
 * include/linux/input/sx9500_platform_data.h
 *
 * SX9500 Platform Data
 * 2 cap differential  
 *
 * Copyright 2012 Semtech Corp.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef _SX9500_PLATFORM_DATA_H_
#define _SX9500_PLATFORM_DATA_H_
#define DIFFERENTIAL



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

struct sx9500_platform_data {
    int i2c_reg_num;
    struct smtc_reg_data *pi2c_reg;

    struct _totalButtonInformation *pbuttonInformation;

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

#endif
