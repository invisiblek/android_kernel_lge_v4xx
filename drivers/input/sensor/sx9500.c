/*! \file sx9500.c
 * \brief  SX9500 Driver
 *
 * Driver for the SX9500 
 * Copyright (c) 2011 Semtech Corp
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#define DEBUG
#define DRIVER_NAME "sx9500"

#define MAX_WRITE_ARRAY_SIZE 32
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/input.h>

#include <sx86xx.h> /* main struct, interrupt,init,pointers */
#include <sx9500_i2c_reg.h>
#include <sx9500_platform_data.h>  /* platform data */
//#include <sx9500-specifics.h>

#ifdef CONFIG_OF
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#endif

#define ACTIVE 0
#define IDLE   1
#define PROX_STATUS_NEAR 0 /*ACTIVE*/
#define PROX_STATUS_FAR  1 /*IDLE*/

/* IO Used for NIRQ */
#define GPIO_SX9500_NIRQ 54//114

#define CS2_DISABLE 0x00
#define CS2_ENABLE  0x04

#ifdef CONFIG_OF
enum sensor_dt_entry_status {
    DT_REQUIRED,
    DT_SUGGESTED,
    DT_OPTIONAL,
};

enum sensor_dt_entry_type {
    DT_U32,
    DT_GPIO,
    DT_BOOL,
};

struct sensor_dt_to_pdata_map {
    const char                  *dt_name;
    void                        *ptr_data;
    enum sensor_dt_entry_status status;
    enum sensor_dt_entry_type   type;
    int                          default_val;
};
#endif

/* 
 * Define Registers that need to be initialized to values different than default
 * Addr Name            Variable[Bits] - value
 * --------------------------------------------------------------
 * 0x03 RegIrqMsk      CLOSEIRQEN[6] - 1(enable), FARIRQEN[5] - 1(enable)
 * 0x06 RegProxCtrl0   SCANPERIOD[6:4] - 101(200ms), SENSOREN[3:0] - 0100(CS2 pin enable)
 * 0x07 RegProxCtrl1   SHIELEN[7:6] - 00(off), RANGE[1:0] - 11(small)
 * 0x08 RegProxCtrl2
 * 0x09 RegProxCtrl3
 * 0x0A RegProxCtrl4   AVGTHRESH[7:0] -  +/-24576 (24576/128=192=0xC0)
 * 0x0B RegProxCtrl5   AVGDEB[7:6] - 00(Off), AVGNEGFILT[5:3] - 001(Lowest), AVGPOSFILT[2:0] - 111(Highest)
 * 0x0C RegProxCtrl6   PROXTHRESH[4:0] - 00110(120)
 * 0x0D RegProxCtrl7   AVGCOMPDIS[7] - 0(Disable), COMPMETHOD[6] - 0(compensate each CSx pin), HYST[5:4] -00(32), CLOSEDEB[3:2] - 00(Off)
 * 0x0E RegProxCtrl8
 * 0x20 RegSensorSel  SENSORSEL[1:0] - 10(CS2)
 */
static struct smtc_reg_data sx9500_i2c_reg_setup[] = {
    { .reg = SX9500_IRQ_ENABLE_REG,   .val = 0x60, },
    { .reg = SX9500_CPS_CTRL0_REG,    .val = 0x54, },
    { .reg = SX9500_CPS_CTRL1_REG,    .val = 0x03, },
    { .reg = SX9500_CPS_CTRL2_REG,    .val = 0x77, },
    { .reg = SX9500_CPS_CTRL3_REG,    .val = 0x01, },
    { .reg = SX9500_CPS_CTRL4_REG,    .val = 0xC0, },
    { .reg = SX9500_CPS_CTRL5_REG,    .val = 0x0F, },
    { .reg = SX9500_CPS_CTRL6_REG,    .val = 0x06, },
    { .reg = SX9500_CPS_CTRL7_REG,    .val = 0x00, },
    { .reg = SX9500_CPS_CTRL8_REG,    .val = 0x00, },
    { .reg = SX9500_CPSRD_REG,        .val = 0x02, },
};

static struct _buttonInfo psmtcButtons[] = {
    { .keycode = KEY_0,    .mask = SX9500_TCHCMPSTAT_TCHSTAT0_FLAG, },
    { .keycode = KEY_1,    .mask = SX9500_TCHCMPSTAT_TCHSTAT1_FLAG, },
    { .keycode = KEY_2,    .mask = SX9500_TCHCMPSTAT_TCHSTAT2_FLAG, },
    { .keycode = KEY_3,    .mask = SX9500_TCHCMPSTAT_TCHSTAT3_FLAG, },
};

/*! \struct sx9500
 * Specialized struct containing input event data, platform data, and
 * last cap state read if needed.
 */
struct sx9500
{
  struct _totalButtonInformation *pbuttonInformation;
  struct sx9500_platform_data *hw; /* specific platform data settings */
} sx9500_t, *psx9500_t;


/*! \fn static int write_register(struct sx86XX *this, u8 address, u8 value)
 * \brief Sends a write register to the device
 * \param this Pointer to main parent struct 
 * \param address 8-bit register address
 * \param value   8-bit register value to write to address
 * \return Value from i2c_master_send
 */
static int write_register(struct sx86XX *this, u8 address, u8 value)
{
    struct i2c_client *i2c = 0;
    /*char*/u8 buffer[2];
    int returnValue = 0;

    buffer[0] = address;
    buffer[1] = value;
    returnValue = -ENOMEM;
    if (this && this->bus) {
        i2c = this->bus;

        returnValue = i2c_master_send(i2c,buffer,2);
        dev_dbg(&i2c->dev,"write_register Address: 0x%x Value: 0x%x Return: %d\n",address,value,returnValue);
    }

    return returnValue;
}

/*! \fn static int read_register(struct sx86XX *this, u8 address, u8 *value) 
* \brief Reads a register's value from the device
* \param this Pointer to main parent struct 
* \param address 8-Bit address to read from
* \param value Pointer to 8-bit value to save register value to 
* \return Value from i2c_smbus_read_byte_data if < 0. else 0
*/
static int read_register(struct sx86XX *this, u8 address, u8 *value)
{
    struct i2c_client *i2c = 0;
    s32 returnValue = 0;

    if (this && value && this->bus) {
        i2c = this->bus;
        returnValue = i2c_smbus_read_byte_data(i2c, address);

        if(address != SX9500_IRQSTAT_REG) //temp 2014.03.24 gooni.shim
            dev_dbg(&i2c->dev, "read_register Address: 0x%02x Return: 0x%02x\n", address, returnValue);

        if (returnValue >= 0) {
            *value = returnValue;
            return 0;
        } 
        else {
            return returnValue;
        }
    }

    return -ENOMEM;
}

/*! \brief Sends a write register range to the device
 * \param this Pointer to main parent struct 
 * \param reg 8-bit register address (base address)
 * \param data pointer to 8-bit register values
 * \param size size of the data pointer
 * \return Value from i2c_master_send
 */
/*static int write_registerEx(struct sx86XX *this, unsigned char reg,
                unsigned char *data, int size)
{
  struct i2c_client *i2c = 0;
  u8 tx[MAX_WRITE_ARRAY_SIZE];
  int ret = 0;

  if (this && (i2c = this->bus) && data && (size <= MAX_WRITE_ARRAY_SIZE))
  {
    dev_dbg(this->pdev, "inside write_registerEx()\n");
    tx[0] = reg;
    dev_dbg(this->pdev, "going to call i2c_master_send(0x%p, 0x%x ",
            (void *)i2c,tx[0]);
    for (ret = 0; ret < size; ret++)
    {
      tx[ret+1] = data[ret];
      dev_dbg(this->pdev, "0x%x, ",tx[ret+1]);
    }
    dev_dbg(this->pdev, "\n");

    ret = i2c_master_send(i2c, tx, size+1 );
      if (ret < 0)
        dev_err(this->pdev, "I2C write error\n");
  }
  dev_dbg(this->pdev, "leaving write_registerEx()\n");


  return ret;
}*/

/*! \brief Reads a group of registers from the device
* \param this Pointer to main parent struct 
* \param reg 8-Bit address to read from (base address)
* \param data Pointer to 8-bit value array to save registers to 
* \param size size of array
* \return Value from i2c_smbus_read_byte_data if < 0. else 0
*/
/*static int read_registerEx(struct sx86XX *this, unsigned char reg,
                unsigned char *data, int size)
{
  struct i2c_client *i2c = 0;
  int ret = 0;
  u8 tx[] = {
        reg
    };
  if (this && (i2c = this->bus) && data && (size <= MAX_WRITE_ARRAY_SIZE))
  {
    dev_dbg(this->pdev, "inside read_registerEx()\n");
    dev_dbg(this->pdev,
        "going to call i2c_master_send(0x%p,0x%p,1) Reg: 0x%x\n",
                                                               (void *)i2c,(void *)tx,tx[0]);
    ret = i2c_master_send(i2c,tx,1);
    if (ret >= 0) {
      dev_dbg(this->pdev, "going to call i2c_master_recv(0x%p,0x%p,%x)\n",
                                                              (void *)i2c,(void *)data,size);
        ret = i2c_master_recv(i2c, data, size);
    }
  }
  if (unlikely(ret < 0))
    dev_err(this->pdev, "I2C read error\n");
  dev_dbg(this->pdev, "leaving read_registerEx()\n");
  return ret;
}*/

/*********************************************************************/
/*! \brief Perform a manual offset calibration
* \param this Pointer to main parent struct 
* \return Value return value from the write register
 */
static int manual_offset_calibration(struct sx86XX *this)
{
    s32 returnValue = 0;

    returnValue = write_register(this, SX9500_IRQSTAT_REG, 0xFF);

    return returnValue;
}

/*! \brief sysfs show function for manual calibration which currently just
 * returns register value.
 */
static ssize_t manual_offset_calibration_show(struct device *dev,
                     struct device_attribute *attr, char *buf)
{
    u8 reg_value = 0;
    struct sx86XX *this = dev_get_drvdata(dev);

    dev_dbg(this->pdev, "Reading IRQSTAT_REG\n");
    read_register(this, SX9500_IRQSTAT_REG, &reg_value);

    return sprintf(buf, "0x%2x\n", reg_value);
}

/*! \brief sysfs store function for manual calibration
 */
static ssize_t manual_offset_calibration_store(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t count)
{
    struct sx86XX *this = dev_get_drvdata(dev);
    unsigned long val;

    if (strict_strtoul(buf, 0, &val))
        return -EINVAL;

    dev_info( this->pdev, "Performing manual_offset_calibration()\n");
    manual_offset_calibration(this);

    return count;
}

static ssize_t sx9500_show_proxstatus(struct device *dev,
                     struct device_attribute *attr, char *buf)
{
    int prox_status = -1;\

    struct sx9500 *pDevice = NULL;
    struct sx86XX *this = dev_get_drvdata(dev);

    struct _buttonInfo *buttons = NULL;
    struct _buttonInfo *pCurrentButton  = NULL;

    if (this && (pDevice = this->pDevice)){
        buttons = pDevice->pbuttonInformation->buttons;
        pCurrentButton = &buttons[2]; // only CS2
        prox_status = pCurrentButton->state;
    }

    return sprintf(buf, "%d\n", prox_status);
}

static ssize_t sx9500_show_regsensorsel(struct device *dev,
                     struct device_attribute *attr, char *buf)
{
    u8 reg_value = 0;
    struct sx86XX *this = dev_get_drvdata(dev);

    read_register(this, SX9500_CPSRD_REG, &reg_value);

    return sprintf(buf, "0x%2x\n", reg_value);
}

static ssize_t sx9500_store_regsensorsel(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t count)
{
    struct sx86XX *this = dev_get_drvdata(dev);
    unsigned long val;

    if (strict_strtoul(buf, 0, &val))
        return -EINVAL;

    write_register(this, SX9500_CPSRD_REG, val);

    return count;
}

static ssize_t sx9500_show_regproxdata(struct device *dev,
                     struct device_attribute *attr, char *buf)
{
    u8 reg_value_msb = 0;
    u8 reg_value_lsb = 0;

    s16 reg_value_prox_useful = 0;
    s16 reg_value_prox_avg = 0;
    s16 reg_value_prox_diff = 0;
    u16 reg_value_prox_offset = 0;

    struct sx86XX *this = dev_get_drvdata(dev);

    //write_register(this, SX9500_CPSRD_REG, 0x02);//select only CS2
    read_register(this, SX9500_USEMSB_REG, &reg_value_msb);
    read_register(this, SX9500_USELSB_REG, &reg_value_lsb);
    reg_value_prox_useful = (s16)((reg_value_msb << 8) | reg_value_lsb);

    reg_value_msb = 0; reg_value_lsb = 0;
    read_register(this, SX9500_AVGMSB_REG, &reg_value_msb);
    read_register(this, SX9500_AVGLSB_REG, &reg_value_lsb);
    reg_value_prox_avg = (s16)((reg_value_msb << 8) | reg_value_lsb);

    reg_value_msb = 0; reg_value_lsb = 0;
    read_register(this, SX9500_DIFFMSB_REG, &reg_value_msb);
    read_register(this, SX9500_DIFFLSB_REG, &reg_value_lsb);
    reg_value_prox_diff = (s16)((reg_value_msb << 8) | reg_value_lsb);

    read_register(this, SX9500_OFFSETMSB_REG, &reg_value_msb);
    read_register(this, SX9500_OFFSETLSB_REG, &reg_value_lsb);
    reg_value_prox_offset = (u16)((reg_value_msb << 8) | reg_value_lsb);

    return sprintf(buf, "%5d %5d %5d %5d\n", 
                reg_value_prox_useful, reg_value_prox_avg, reg_value_prox_diff, reg_value_prox_offset);
}

static ssize_t sx9500_store_regproxoffset_msb(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t count)
{
    struct sx86XX *this = dev_get_drvdata(dev);
    unsigned long val;

    if (strict_strtoul(buf, 0, &val))
        return -EINVAL;

    write_register(this, SX9500_OFFSETMSB_REG, val);

    return count;
}

static ssize_t sx9500_store_regproxoffset_lsb(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t count)
{
    struct sx86XX *this = dev_get_drvdata(dev);
    unsigned long val;

    if (strict_strtoul(buf, 0, &val))
        return -EINVAL;

    write_register(this, SX9500_OFFSETLSB_REG, val);

    return count;
}

static ssize_t sx9500_show_regproxctrol0(struct device *dev,
                     struct device_attribute *attr, char *buf)
{
    u8 reg_value = 0;
    struct sx86XX *this = dev_get_drvdata(dev);

    read_register(this, SX9500_CPS_CTRL0_REG, &reg_value);

    return sprintf(buf, "0x%2x\n", reg_value);
}

static ssize_t sx9500_store_regproxctrol0(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t count)
{
    struct sx9500 *pDevice = NULL;
    struct sx86XX *this = dev_get_drvdata(dev);

    struct _buttonInfo *buttons = NULL;
    struct _buttonInfo *pCurrentButton  = NULL;

    struct input_dev *input = NULL;

    unsigned long val;
    unsigned long val_check;

    int ret = 0;

    if (strict_strtoul(buf, 0, &val))
        return -EINVAL;

    ret = write_register(this, SX9500_CPS_CTRL0_REG, val);
    if (ret > 0) {
        val_check = val & CS2_ENABLE;
        if (val_check == CS2_DISABLE) {
            /* Initialize prox status : default FAR */
            if (this && (pDevice = this->pDevice)) {
                buttons = pDevice->pbuttonInformation->buttons;
                pCurrentButton = &buttons[2]; // only CS2
                pCurrentButton->state = IDLE;

                input = pDevice->pbuttonInformation->input;
                input_report_abs(input, ABS_DISTANCE, PROX_STATUS_FAR);
                input_sync(input);
            }
        }
    }

    return count;
}

static ssize_t sx9500_show_regproxctrol1(struct device *dev,
                     struct device_attribute *attr, char *buf)
{
    u8 reg_value = 0;
    struct sx86XX *this = dev_get_drvdata(dev);

    read_register(this, SX9500_CPS_CTRL1_REG, &reg_value);

    return sprintf(buf, "0x%2x\n", reg_value);
}

static ssize_t sx9500_store_regproxctrol1(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t count)
{
    struct sx86XX *this = dev_get_drvdata(dev);
    unsigned long val;

    if (strict_strtoul(buf, 0, &val))
        return -EINVAL;

    write_register(this, SX9500_CPS_CTRL1_REG, val);

    return count;
}

static ssize_t sx9500_show_regproxctrol2(struct device *dev,
                     struct device_attribute *attr, char *buf)
{
    u8 reg_value = 0;
    struct sx86XX *this = dev_get_drvdata(dev);

    read_register(this, SX9500_CPS_CTRL2_REG, &reg_value);

    return sprintf(buf, "0x%2x\n", reg_value);
}

static ssize_t sx9500_store_regproxctrol2(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t count)
{
    struct sx86XX *this = dev_get_drvdata(dev);
    unsigned long val;

    if (strict_strtoul(buf, 0, &val))
        return -EINVAL;

    write_register(this, SX9500_CPS_CTRL2_REG, val);

    return count;
}

static ssize_t sx9500_show_regproxctrol3(struct device *dev,
                     struct device_attribute *attr, char *buf)
{
    u8 reg_value = 0;
    struct sx86XX *this = dev_get_drvdata(dev);

    read_register(this, SX9500_CPS_CTRL3_REG, &reg_value);

    return sprintf(buf, "0x%2x\n", reg_value);
}

static ssize_t sx9500_store_regproxctrol3(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t count)
{
    struct sx86XX *this = dev_get_drvdata(dev);
    unsigned long val;

    if (strict_strtoul(buf, 0, &val))
        return -EINVAL;

    write_register(this, SX9500_CPS_CTRL3_REG, val);

    return count;
}

static ssize_t sx9500_show_regproxctrol4(struct device *dev,
                     struct device_attribute *attr, char *buf)
{
    u8 reg_value = 0;
    struct sx86XX *this = dev_get_drvdata(dev);

    read_register(this, SX9500_CPS_CTRL4_REG, &reg_value);

    return sprintf(buf, "0x%2x\n", reg_value);
}

static ssize_t sx9500_store_regproxctrol4(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t count)
{
    struct sx86XX *this = dev_get_drvdata(dev);
    unsigned long val;

    if (strict_strtoul(buf, 0, &val))
        return -EINVAL;

    write_register(this, SX9500_CPS_CTRL4_REG, val);

    return count;
}

static ssize_t sx9500_show_regproxctrol5(struct device *dev,
                     struct device_attribute *attr, char *buf)
{
    u8 reg_value = 0;
    struct sx86XX *this = dev_get_drvdata(dev);

    read_register(this, SX9500_CPS_CTRL5_REG, &reg_value);

    return sprintf(buf, "0x%2x\n", reg_value);
}

static ssize_t sx9500_store_regproxctrol5(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t count)
{
    struct sx86XX *this = dev_get_drvdata(dev);
    unsigned long val;

    if (strict_strtoul(buf, 0, &val))
        return -EINVAL;

    write_register(this, SX9500_CPS_CTRL5_REG, val);

    return count;
}

static ssize_t sx9500_show_regproxctrol6(struct device *dev,
                     struct device_attribute *attr, char *buf)
{
    u8 reg_value = 0;
    struct sx86XX *this = dev_get_drvdata(dev);

    read_register(this, SX9500_CPS_CTRL6_REG, &reg_value);

    return sprintf(buf, "0x%2x\n", reg_value);
}

static ssize_t sx9500_store_regproxctrol6(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t count)
{
    struct sx86XX *this = dev_get_drvdata(dev);
    unsigned long val;

    if (strict_strtoul(buf, 0, &val))
        return -EINVAL;

    write_register(this, SX9500_CPS_CTRL6_REG, val);

    return count;
}

static ssize_t sx9500_show_regproxctrol7(struct device *dev,
                     struct device_attribute *attr, char *buf)
{
    u8 reg_value = 0;
    struct sx86XX *this = dev_get_drvdata(dev);

    read_register(this, SX9500_CPS_CTRL7_REG, &reg_value);

    return sprintf(buf, "0x%2x\n", reg_value);
}

static ssize_t sx9500_store_regproxctrol7(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t count)
{
    struct sx86XX *this = dev_get_drvdata(dev);
    unsigned long val;

    if (strict_strtoul(buf, 0, &val))
        return -EINVAL;

    write_register(this, SX9500_CPS_CTRL7_REG, val);

    return count;
}

static ssize_t sx9500_show_regproxctrol8(struct device *dev,
                     struct device_attribute *attr, char *buf)
{
    u8 reg_value = 0;
    struct sx86XX *this = dev_get_drvdata(dev);

    read_register(this, SX9500_CPS_CTRL8_REG, &reg_value);

    return sprintf(buf, "0x%2x\n", reg_value);
}

static ssize_t sx9500_store_regproxctrol8(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t count)
{
    struct sx86XX *this = dev_get_drvdata(dev);
    unsigned long val;

    if (strict_strtoul(buf, 0, &val))
        return -EINVAL;

    write_register(this, SX9500_CPS_CTRL8_REG, val);

    return count;
}

static ssize_t sx9500_show_regirqmask(struct device *dev,
                     struct device_attribute *attr, char *buf)
{
    u8 reg_value = 0;
    struct sx86XX *this = dev_get_drvdata(dev);

    read_register(this, SX9500_IRQ_ENABLE_REG, &reg_value);

    return sprintf(buf, "0x%2x\n", reg_value);
}

static ssize_t sx9500_store_regirqmask(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t count)
{
    struct sx86XX *this = dev_get_drvdata(dev);
    unsigned long val;

    if (strict_strtoul(buf, 0, &val))
        return -EINVAL;

    write_register(this, SX9500_IRQ_ENABLE_REG, val);

    return count;
}

static ssize_t sx9500_store_regreset(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t count)
{
    struct sx86XX *this = dev_get_drvdata(dev);
    unsigned long val;

    if (strict_strtoul(buf, 0, &val))
        return -EINVAL;

    if (val == SX9500_SOFTRESET && this) {
        sx86XX_suspend(this);
        sx86XX_resume(this);
    }

    return count;
}

static DEVICE_ATTR(calibrate, 0664, manual_offset_calibration_show, manual_offset_calibration_store);
static DEVICE_ATTR(proxstatus, 0664, sx9500_show_proxstatus, NULL);
static DEVICE_ATTR(regsensorsel, 0664, sx9500_show_regsensorsel, sx9500_store_regsensorsel);
static DEVICE_ATTR(regproxdata, 0664, sx9500_show_regproxdata, NULL);
static DEVICE_ATTR(regoffsetmsb, 0664, NULL, sx9500_store_regproxoffset_msb);
static DEVICE_ATTR(regoffsetlsb, 0664, NULL, sx9500_store_regproxoffset_lsb);
static DEVICE_ATTR(regproxctrl0, 0664, sx9500_show_regproxctrol0, sx9500_store_regproxctrol0);
static DEVICE_ATTR(regproxctrl1, 0664, sx9500_show_regproxctrol1, sx9500_store_regproxctrol1);
static DEVICE_ATTR(regproxctrl2, 0664, sx9500_show_regproxctrol2, sx9500_store_regproxctrol2);
static DEVICE_ATTR(regproxctrl3, 0664, sx9500_show_regproxctrol3, sx9500_store_regproxctrol3);
static DEVICE_ATTR(regproxctrl4, 0664, sx9500_show_regproxctrol4, sx9500_store_regproxctrol4);
static DEVICE_ATTR(regproxctrl5, 0664, sx9500_show_regproxctrol5, sx9500_store_regproxctrol5);
static DEVICE_ATTR(regproxctrl6, 0664, sx9500_show_regproxctrol6, sx9500_store_regproxctrol6);
static DEVICE_ATTR(regproxctrl7, 0664, sx9500_show_regproxctrol7, sx9500_store_regproxctrol7);
static DEVICE_ATTR(regproxctrl8, 0664, sx9500_show_regproxctrol8, sx9500_store_regproxctrol8);
static DEVICE_ATTR(regirqmask, 0664, sx9500_show_regirqmask, sx9500_store_regirqmask);
static DEVICE_ATTR(regreset, 0664, NULL, sx9500_store_regreset);

static struct attribute *sx9500_attributes[] = {
    &dev_attr_calibrate.attr,
    &dev_attr_proxstatus.attr,
    &dev_attr_regsensorsel.attr,
    &dev_attr_regproxdata.attr,
    &dev_attr_regoffsetmsb.attr,
    &dev_attr_regoffsetlsb.attr,
    &dev_attr_regproxctrl0.attr,
    &dev_attr_regproxctrl1.attr,
    &dev_attr_regproxctrl2.attr,
    &dev_attr_regproxctrl3.attr,
    &dev_attr_regproxctrl4.attr,
    &dev_attr_regproxctrl5.attr,
    &dev_attr_regproxctrl6.attr,
    &dev_attr_regproxctrl7.attr,
    &dev_attr_regproxctrl8.attr,
    &dev_attr_regirqmask.attr,
    &dev_attr_regreset.attr,
    NULL,
};

static struct attribute_group sx9500_attr_group = {
    .attrs = sx9500_attributes,
};
/*********************************************************************/

/*! \fn static int read_regIrqStat(struct sx86XX *this)
 * \brief Shortcut to read what caused interrupt.
 * \details This is to keep the drivers a unified
 * function that will read whatever register(s) 
 * provide information on why the interrupt was caused.
 * \param this Pointer to main parent struct 
 * \return If successful, Value of bit(s) that cause interrupt, else 0
 */
static int read_regIrqStat(struct sx86XX *this)
{
    u8 data = 0;

    if (this) {
        if (read_register(this, SX9500_IRQSTAT_REG, &data) == 0){
            dev_dbg(this->pdev, "SX9500_IRQSTAT_REG = 0x%x\n", data);
            return (data & 0x00FF);
        }
    }

    return 0;
}

/*! \brief  Initialize I2C config from platform data
 * \param this Pointer to main parent struct 
 */
static void hw_init(struct sx86XX *this)
{
    struct sx9500 *pDevice = 0;
    struct sx9500_platform_data *pdata = 0;
    int i = 0;

    /* configure device */
    dev_dbg(this->pdev, "Going to Setup I2C Registers\n");
    if (this && (pDevice = this->pDevice) && (pdata = pDevice->hw))
    {
        while ( i < pdata->i2c_reg_num) {
            /* Write all registers/values contained in i2c_reg */
            dev_dbg(this->pdev, "Going to Write Reg: 0x%x Value: 0x%x\n", pdata->pi2c_reg[i].reg,pdata->pi2c_reg[i].val);
            //msleep(3);        
            write_register(this, pdata->pi2c_reg[i].reg,pdata->pi2c_reg[i].val);
            i++;
        }
    }
    else {
        dev_err(this->pdev, "ERROR! platform data 0x%p\n",pDevice->hw);
    }
}
/*********************************************************************/




/*! \fn static int initialize(struct sx86XX *this)
 * \brief Performs all initialization needed to configure the device
 * \param this Pointer to main parent struct 
 * \return Last used command's return value (negative if error)
 */
static int initialize(struct sx86XX *this)
{
    if (this) {
        /* prepare reset by disabling any irq handling */
        this->irq_disabled = 1;
        disable_irq(this->irq);
        /* perform a reset */
        write_register(this, SX9500_SOFTRESET_REG, SX9500_SOFTRESET);
        /* wait until the reset has finished by monitoring NIRQ */
        dev_dbg(this->pdev, "Sent Software Reset. Waiting until device is back from reset to continue.\n");
        /* just sleep for awhile instead of using a loop with reading irq status */

        msleep(300);

        //while(this->get_nirq_low && this->get_nirq_low()) { read_regIrqStat(this); }
        dev_dbg(this->pdev, "Device is back from the reset, continuing. NIRQ = %d\n", this->get_nirq_low());

        hw_init(this);

        msleep(100); /* make sure everything is running */
    
        manual_offset_calibration(this);

        /* re-enable interrupt handling */
        enable_irq(this->irq);
        this->irq_disabled = 0;

        /* make sure no interrupts are pending since enabling irq will only
         * work on next falling edge */
        read_regIrqStat(this);
        dev_dbg(this->pdev, "Exiting initialize(). NIRQ = %d\n",this->get_nirq_low());

        return 0;
    }

    return -ENOMEM;
}

/*! 
 * \brief Handle what to do when a Close/Far occurs
 * \param this Pointer to main parent struct 
 */
static void Irq_Process_Close_Far(struct sx86XX *this)
{
    int counter = 0;
    u8 regstat = 0;
    int numberOfButtons = 0;
    struct sx9500 *pDevice = NULL;
    struct _buttonInfo *buttons = NULL;
    struct input_dev *input = NULL;
  
    struct _buttonInfo *pCurrentButton  = NULL;

    if (this && (pDevice = this->pDevice))
    {
        dev_dbg(this->pdev, "Inside Irq_Process_Close_Far()\n");
        read_register(this, SX9500_TCHCMPSTAT_REG, &regstat);

        buttons = pDevice->pbuttonInformation->buttons;
        input = pDevice->pbuttonInformation->input;
        numberOfButtons = pDevice->pbuttonInformation->buttonSize;
    
        if (unlikely( (buttons==NULL) || (input==NULL) )) {
            dev_err(this->pdev, "ERROR!! buttons or input NULL!!!\n");
            return;
        }

        for (counter = 0; counter < numberOfButtons; counter++) {
            pCurrentButton = &buttons[counter];
            if (pCurrentButton==NULL) {
                dev_err(this->pdev, "ERROR!! current button at index: %d NULL!!!\n", counter);
                return; // ERRORR!!!!
            }

            switch (pCurrentButton->state) {
                case IDLE: /* Button is not being touched! */
                    if (((regstat & pCurrentButton->mask) == pCurrentButton->mask)) {
                        /* User pressed button */
                        dev_info(this->pdev, "cap button %d touched\n", counter);
                        /*input_report_key(input, pCurrentButton->keycode, 1);*/
                        input_report_abs(input, ABS_DISTANCE, PROX_STATUS_NEAR);
                        pCurrentButton->state = ACTIVE;
                    }
                    else {
                        dev_dbg(this->pdev, "Button %d already released.\n",counter);
                    }
                    break;
                case ACTIVE: /* Button is being touched! */ 
                    if (((regstat & pCurrentButton->mask) != pCurrentButton->mask)) {
                        /* User released button */
                        dev_info(this->pdev, "cap button %d released\n",counter);
                        /*input_report_key(input, pCurrentButton->keycode, 0);*/
                        input_report_abs(input, ABS_DISTANCE, PROX_STATUS_FAR);
                        pCurrentButton->state = IDLE;
                    }
                    else {
                        dev_dbg(this->pdev, "Button %d still touched.\n",counter);
                    }
                    break;
                default: /* Shouldn't be here, device only allowed ACTIVE or IDLE */
                    break;
            };
        }

        input_sync(input);

        dev_dbg(this->pdev, "Leaving Irq_Process_Close_Far()\n");
    }
}

#ifdef CONFIG_OF
static int sensor_parse_dt(struct device *dev, struct sx9500_platform_data *pdata)
{
    struct device_node *np = dev->of_node;

    int ret, err =0;    
    struct sensor_dt_to_pdata_map *itr;
    struct sensor_dt_to_pdata_map map[] = {
        {"Semtech,i2c-pull-up",           &pdata->i2c_pull_up,          DT_REQUIRED,    DT_BOOL,    0},
        {"Semtech,dig-reg-support",       &pdata->digital_pwr_regulator,DT_REQUIRED,    DT_BOOL,    0},
        {"Semtech,irq-gpio",              &pdata->irq_gpio,             DT_REQUIRED,    DT_GPIO,    0},
        {"Semtech,vdd_ana_supply_min",    &pdata->vdd_ana_supply_min,   DT_SUGGESTED,   DT_U32,     0},
        {"Semtech,vdd_ana_supply_max",    &pdata->vdd_ana_supply_max,   DT_SUGGESTED,   DT_U32,     0},
        {"Semtech,vdd_ana_load_ua",       &pdata->vdd_ana_load_ua,      DT_SUGGESTED,   DT_U32,     0},
        {"Semtech,vddio_dig_supply_min",  &pdata->vddio_dig_supply_min, DT_SUGGESTED,   DT_U32,     0},
        {"Semtech,vddio_dig_supply_max",  &pdata->vddio_dig_supply_max, DT_SUGGESTED,   DT_U32,     0},
        {"Semtech,vddio_dig_load_ua",     &pdata->vddio_dig_load_ua,    DT_SUGGESTED,   DT_U32,     0},
        {"Semtech,vddio_i2c_supply_min",  &pdata->vddio_i2c_supply_min, DT_SUGGESTED,   DT_U32,     0},
        {"Semtech,vddio_i2c_supply_max",  &pdata->vddio_i2c_supply_max, DT_SUGGESTED,   DT_U32,     0},
        {"Semtech,vddio_i2c_load_ua",     &pdata->vddio_i2c_load_ua,    DT_SUGGESTED,   DT_U32,     0},
        {NULL,                            NULL,                         0,              0,        0} ,     
    };

    for (itr = map; itr->dt_name ; ++itr) {
        switch (itr->type) {
            case DT_GPIO:
                ret = of_get_named_gpio(np, itr->dt_name, 0);
                if (ret >= 0) {
                    *((int *) itr->ptr_data) = ret;
                    ret = 0;
                }
                break;
            case DT_U32:
                ret = of_property_read_u32(np, itr->dt_name, (u32 *) itr->ptr_data);
                break;
            case DT_BOOL:
                *((bool *) itr->ptr_data) =    of_property_read_bool(np, itr->dt_name);
                ret = 0;
                break;
            default:
                printk(KERN_INFO "[%s] %d is an unknown DT entry type\n",__func__, itr->type);
                ret = -EBADE;
        }

        printk(KERN_INFO "[%s] DT entry ret:%d name:%s val:%d\n",__func__, ret, itr->dt_name, *((int *)itr->ptr_data));

        if (ret) {
            *((int *)itr->ptr_data) = itr->default_val;

            if (itr->status < DT_OPTIONAL) {
                printk(KERN_INFO "[%s] Missing '%s' DT entry\n",__func__, itr->dt_name);

                /* cont on err to dump all missing entries */
                if (itr->status == DT_REQUIRED && !err)
                    err = ret;
        }
    }
  }

  /* set functions of platform data */
//  pdata->init = sensor_platform_hw_init;
//  pdata->exit = sensor_platform_hw_exit;
//  pdata->power_on = sensor_platform_hw_power_on;

    return err;

}
#endif

/* get the NIRQ state (1->NIRQ-low, 0->NIRQ-high) */
static int sx9500_get_nirq_state(void)
{
    return !gpio_get_value(GPIO_SX9500_NIRQ);
}

/*! \fn static int sx9500_probe(struct i2c_client *client, const struct i2c_device_id *id)
 * \brief Probe function
 * \param client pointer to i2c_client
 * \param id pointer to i2c_device_id
 * \return Whether probe was successful
 */
static int sx9500_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int i = 0;
    int err = 0;

    struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);

    struct sx86XX *this = NULL;
    struct sx9500 *pDevice = NULL;
    struct sx9500_platform_data *pplatData = NULL;
    struct _totalButtonInformation *pButtonInformationData = NULL;
    /*struct _buttonInfo *pbuttonsData;*/

    struct input_dev *input = NULL;

    static struct regulator *vdd_l15;
    static struct regulator *vdd_l23;

    int error, rc;

    dev_info(&client->dev, "sx9500_probe()\n");

    if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_READ_WORD_DATA)){
        dev_err(&client->dev, "Check i2c functionality.Fail!\n");
        err = -EIO;
        goto exit;
    }

    /*pbuttonsData = devm_kzalloc(&client->dev, sizeof(struct _buttonInfo), GFP_KERNEL);
    if (!pbuttonsData) {
        dev_err(&client->dev, "Failed to allocate memory(_buttonInfo)\n");
        err = -ENOMEM;
        goto exit;
    }*/

    pButtonInformationData = devm_kzalloc(&client->dev, sizeof(struct _totalButtonInformation), GFP_KERNEL);
    if (!pButtonInformationData) {
        dev_err(&client->dev, "Failed to allocate memory(_totalButtonInformation)\n");
        err = -ENOMEM;
        goto exit;
    }

    pButtonInformationData->buttons = psmtcButtons;
    pButtonInformationData->buttonSize = ARRAY_SIZE(psmtcButtons),

    pplatData = devm_kzalloc(&client->dev, sizeof(struct sx9500_platform_data), GFP_KERNEL);
    if (!pplatData) {
        dev_err(&client->dev, "Failed to allocate memory(sx9500_platform_data)\n");
        err = -ENOMEM;
        goto exit;
    }

    pplatData->pi2c_reg = sx9500_i2c_reg_setup;
    pplatData->i2c_reg_num = ARRAY_SIZE(sx9500_i2c_reg_setup);
    pplatData->pbuttonInformation = pButtonInformationData;
    pplatData->get_is_nirq_low = sx9500_get_nirq_state;
    pplatData->init_platform_hw = NULL; 
    pplatData->exit_platform_hw = NULL;

    /*pDevice = devm_kzalloc(&client->dev, sizeof(struct sx9500), GFP_KERNEL);
    if (!pDevice) {
        dev_err(&client->dev, "Failed to allocate memory(sx9500)\n");
        err = -ENOMEM;
        goto exit;
    }

    pDevice->hw = pplatData;*/
    client->dev.platform_data = pplatData;

    /*pplatData = client->dev.platform_data;
    if (!pplatData) {
        dev_err(&client->dev, "platform data is required!\n");
        return -EINVAL;
    }

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_READ_WORD_DATA))
        return -EIO;*/

#ifdef CONFIG_OF
    err = sensor_parse_dt(&client->dev, pplatData);
    if(err){
        dev_err(&client->dev, "Failed to parse device tree\n");
        goto exit;
    }
#endif

    this = devm_kzalloc(&client->dev, sizeof(struct sx86XX), GFP_KERNEL); /* create memory for main struct */
    if (!this) {
        dev_err(&client->dev, "Failed to allocate memory(sx86XX)\n");
        err = -ENOMEM;
        goto exit;
    }

    vdd_l15 = regulator_get(&client->dev, "Semtech,vdd_ana");
    if (IS_ERR(vdd_l15)){
        rc = PTR_ERR(vdd_l15);
        printk(KERN_INFO "Regulator get failed vcc_ana rc=%d\n", rc);
    }

    rc = regulator_set_voltage(vdd_l15, 2800000, 2800000);
    if (rc < 0)
        printk(KERN_INFO "regulator set_vtg failed rc=%d\n", rc);

    vdd_l23 = regulator_get(&client->dev, "Semtech,vddio_i2c");
    if (IS_ERR(vdd_l23)){
        rc = PTR_ERR(vdd_l23);
        printk(KERN_INFO "Regulator get failed vcc_ana rc=%d\n", rc);
    }

    rc = regulator_set_voltage(vdd_l23, 1800000, 1800000);
    if (rc < 0)
        printk(KERN_INFO "regulator set_vtg failed rc=%d\n", rc);

    dev_info(&client->dev, "ready gpio_is_valid \n");
    if (gpio_is_valid(54)) {
        /* configure touchscreen irq gpio */
        dev_info(&client->dev, "gpio_is_valid(54) \n");
        error = gpio_request(54, "sx9500_irq_gpio");
        if (error)
            dev_info(&client->dev, "unable to request gpio 54\n");

        error = gpio_direction_input(54);
        if (error)
            dev_info(&client->dev, "unable to set direction for gpio 54\n");

        client->irq = gpio_to_irq(54);
        dev_info(&client->dev, "gpio_to_irq(54) = %d\n", gpio_to_irq(54));
    }
    else {
        dev_info(&client->dev, "irq gpio not provided\n");
    }

    rc = regulator_enable(vdd_l15);
    if (rc)
        printk(KERN_INFO "Regulator vcc_ana enable failed rc=%d\n", rc);

    rc = regulator_enable(vdd_l23);
    if (rc)
        printk(KERN_INFO "Regulator vcc_ana enable failed rc=%d\n", rc);

    if (this)
    {
        /* In case we need to reinitialize data 
        * (e.q. if suspend reset device) */
        this->init = initialize;
        /* shortcut to read status of interrupt */
        this->refreshStatus = read_regIrqStat;
        /* pointer to function from platform data to get pendown 
        * (1->NIRQ=0, 0->NIRQ=1) */
        this->get_nirq_low = pplatData->get_is_nirq_low;
        /* save irq in case we need to reference it */
        this->irq = client->irq;
        /* do we need to create an irq timer after interrupt ? */
        this->useIrqTimer = 0;

        /* Setup function to call on corresponding reg irq source bit */
        if (MAX_NUM_STATUS_BITS >= 8)
        {
            this->statusFunc[0] = 0; /* TXEN_STAT */
            this->statusFunc[1] = 0; /* UNUSED */
            this->statusFunc[2] = 0; /* UNUSED */
            this->statusFunc[3] = 0; /* CONVERSION_IRQ(CONV_STAT) */
            this->statusFunc[4] = 0; /* COMPENSATION_IRQ(COMP_STAT) */
            this->statusFunc[5] = Irq_Process_Close_Far; /* FAR_IRQ(RELEASE_STAT) */
            this->statusFunc[6] = Irq_Process_Close_Far; /* CLOSE_IRQ(TOUCH_STAT) */
            this->statusFunc[7] = 0; /* RESET_STAT */
        }

        /* setup i2c communication */
        this->bus = client;
        i2c_set_clientdata(client, this);

        /* record device struct */
        this->pdev = &client->dev;

        /* create memory for device specific struct */
        pDevice = devm_kzalloc(&client->dev, sizeof(struct sx9500), GFP_KERNEL);
        if (!pDevice) {
            dev_dbg(&client->dev, "Failed to allocate memory(sx9500)\n");
            err = -ENOMEM;
            goto exit;
        }

        this->pDevice = pDevice;

        if (pDevice)
        {
            /* for accessing items in user data (e.g. calibrate) */
            err = sysfs_create_group(&client->dev.kobj, &sx9500_attr_group);
            if (err)
                printk(KERN_INFO"sysfs create fail!");

            /* Check if we hava a platform initialization function to call*/
            if (pplatData->init_platform_hw)
                pplatData->init_platform_hw();

            /* Add Pointer to main platform data struct */
            pDevice->hw = pplatData;
      
            /* Initialize the button information initialized with keycodes */
            pDevice->pbuttonInformation = pplatData->pbuttonInformation;

            /* Create the input device */
            input = input_allocate_device();
            if (!input) {
                return -ENOMEM;
            }

            /* Set all the keycodes */
            /*__set_bit(EV_KEY, input->evbit);*/
            __set_bit(EV_ABS, input->evbit);
            input_set_abs_params(input, ABS_DISTANCE, 0, 1, 0, 0);
            for (i = 0; i < pDevice->pbuttonInformation->buttonSize; i++) {
                /*__set_bit(pDevice->pbuttonInformation->buttons[i].keycode, input->keybit);*/
                pDevice->pbuttonInformation->buttons[i].state = IDLE;
            }

            /* save the input pointer and finish initialization */
            pDevice->pbuttonInformation->input = input;
            input->name = "SX9500 Cap Touch";
            input->id.bustype = BUS_I2C;
            //input->id.product = sx863x->product;
            //input->id.version = sx863x->version;
            if(input_register_device(input))
                return -ENOMEM;
        }

        sx86XX_init(this);

        return  0;
    }

    return -1;

exit:
    return err;
}

/*! \fn static int sx9500_remove(struct i2c_client *client)
 * \brief Called when device is to be removed
 * \param client Pointer to i2c_client struct
 * \return Value from sx86XX_remove()
 */
static int sx9500_remove(struct i2c_client *client)
{
    struct sx9500_platform_data *pplatData =0;
    struct sx9500 *pDevice = 0;
    struct sx86XX *this = i2c_get_clientdata(client);

    if (this && (pDevice = this->pDevice))
    {
        input_unregister_device(pDevice->pbuttonInformation->input);

        sysfs_remove_group(&client->dev.kobj, &sx9500_attr_group);
        pplatData = client->dev.platform_data;
        if (pplatData && pplatData->exit_platform_hw)
            pplatData->exit_platform_hw();
        kfree(this->pDevice);
    }

    return sx86XX_remove(this);
}

#ifdef CONFIG_PM
/*====================================================*/
/***** Kernel Suspend *****/
static int sx9500_suspend(struct i2c_client *client, pm_message_t mesg)
{
    struct sx86XX *this = i2c_get_clientdata(client);
    sx86XX_suspend(this);

    return 0;
}

/***** Kernel Resume *****/
static int sx9500_resume(struct i2c_client *client)
{
    struct sx86XX *this = i2c_get_clientdata(client);
    sx86XX_resume(this);

    return 0;
}
/*====================================================*/
#else
#define sx9500_suspend    NULL
#define sx9500_resume    NULL
#endif /* CONFIG_PM */

static struct i2c_device_id sx9500_id[] = {
    { "sx9500", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, sx9500_id);

#ifdef CONFIG_OF
static struct of_device_id sx9500_match_table[] = {
    { .compatible = "Semtech,sx9500",},
    { },
};
#else
#define sx9500_match_table NULL
#endif

static struct i2c_driver sx9500_driver = {
    .driver = {
        .owner  = THIS_MODULE,
        .name   = "sx9500",
        .of_match_table = sx9500_match_table,
    },
    .id_table = sx9500_id,
    .probe      = sx9500_probe,
    .remove      = __devexit_p(sx9500_remove),
    .suspend = sx9500_suspend,
    .resume    = sx9500_resume,
};

static int __init sx9500_init(void)
{
    printk(KERN_INFO "sx9500 driver: initialize.");
    return i2c_add_driver(&sx9500_driver);
}

static void __exit sx9500_exit(void)
{
    printk(KERN_INFO "sx9500 driver: release.");
    i2c_del_driver(&sx9500_driver);
}

module_init(sx9500_init);
module_exit(sx9500_exit);

MODULE_AUTHOR("Semtech Corp. (http://www.semtech.com/)");
MODULE_DESCRIPTION("SX9500 Capacitive Touch Controller Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");