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

//#define DEBUG /* for dev_dbg function */

#define DRIVER_NAME "sx9500"

#define MAX_WRITE_ARRAY_SIZE 32
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/input.h>

#include <sx86xx.h> /* main struct, interrupt,init,pointers */
#include <sx9500_i2c_reg.h>
#include <sx9500_platform_data.h> /* platform data */
//#include <sx9500-specifics.h>

#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <linux/sort.h>

#ifdef CONFIG_OF
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#endif

#define ACTIVE              0
#define IDLE                1
#define PROX_STATUS_NEAR    0 /*ACTIVE*/
#define PROX_STATUS_FAR     1 /*IDLE*/

#define ENABLE_IRQ_MASK     0x10
#define DISABLE_IRQ_MASK    0x20
#define ENABLE_SENSOR_PINS  0x01
#define DISABLE_SENSOR_PINS 0x02

/* IO Used for NIRQ */
#define GPIO_SX9500_NIRQ    54//114

#define CS2_DISABLE         0x00
#define CS2_ENABLE          0x05

/* For Calibration & Startup function */
#define ON_SENSOR           1
#define OFF_SENSOR          2
#define COLLECT_NUM         20
#define FILTER_NUM          2
#define PATH_CAPSENSOR_CAL     "/sns/capsensor_cal.dat"
#if defined(CONFIG_MACH_MSM8926_E8LTE)
#define PATH_CAPSENSOR_CAL2    "/sns/capsensor_cal2.dat"
#endif
#define SCANPERIOD_CAL      0x0
#define PATH_XO_THERM       "/sys/class/hwmon/hwmon0/device/xo_therm_pu2"

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
    int                         default_val;
};
#endif

/* 
 * Define Registers that need to be initialized to values different than default
 * Addr Name           Variable[Bits] - value
 * ---------------------------------------------------------------------------
 * 0x03 RegIrqMsk      CLOSEIRQEN[6] - 1(enable)
 *                     FARIRQEN[5] - 1(enable)
 *                     COMPDONEIRQEN[4] - 1(enable)
 * 0x06 RegProxCtrl0   SCANPERIOD[6:4] - 101(200ms)
 *                     SENSOREN[3:0] - 1100(CS3/CS2 pin enable)
 * 0x07 RegProxCtrl1   SHIELEN[7:6] - 00(off)
 *                     RANGE[1:0] - 10(Medium Small, +/-3pF FS)
 * 0x08 RegProxCtrl2   GAIN[6:5] - 11(x8)
 *                     FREQ[4:3] - 10(167kHz)
 *                     RESOLUTION[2:0] -111(Finest)
 * 0x09 RegProxCtrl3   DOZEEN[6] - 0(disable)
 *                     DOZEPERIOD[5:4] - 00(2x)
 *                     RAWFILT[1:0] - 01(Low)
 * 0x0A RegProxCtrl4   AVGTHRESH[7:0] -  +/-8192 (24576/128=64=0x40)
 * 0x0B RegProxCtrl5   AVGDEB[7:6] - 00(Off)
 *                     AVGNEGFILT[5:3] - 001(Lowest)
 *                     AVGPOSFILT[2:0] - 101(Middle High)
 * 0x0C RegProxCtrl6   PROXTHRESH[4:0] - 01001(180)
 * 0x0D RegProxCtrl7   AVGCOMPDIS[7] - 0(Disable)
 *                     COMPMETHOD[6] - 0(compensate each CSx pin)
 *                     HYST[5:4] -00(32)
 *                     CLOSEDEB[3:2] - 00(Off)
 * 0x0E RegProxCtrl8   STUCK[7:4] - 0000(Off)
 *                     COMPPRD[3:0] - 0000(Off)
 * 0x20 RegSensorSel   SENSORSEL[1:0] - 10(CS2)
 */
static struct smtc_reg_data sx9500_i2c_reg_setup[] = {
    { .reg = SX9500_IRQ_ENABLE_REG,   .val = 0x70, },
#if defined(CONFIG_MACH_MSM8926_E8LTE)
    { .reg = SX9500_CPS_CTRL0_REG,    .val = 0x5D, },
#else
    { .reg = SX9500_CPS_CTRL0_REG,    .val = 0x5C, },
#endif
    { .reg = SX9500_CPS_CTRL1_REG,    .val = 0x02, },
    { .reg = SX9500_CPS_CTRL2_REG,    .val = 0x77, },
    { .reg = SX9500_CPS_CTRL3_REG,    .val = 0x01, },
    { .reg = SX9500_CPS_CTRL4_REG,    .val = 0x40, },
    { .reg = SX9500_CPS_CTRL5_REG,    .val = 0x0D, },
    { .reg = SX9500_CPS_CTRL6_REG,    .val = 0x09, },
    { .reg = SX9500_CPS_CTRL7_REG,    .val = 0x00, },
    { .reg = SX9500_CPS_CTRL8_REG,    .val = 0x00, },
    { .reg = SX9500_SENSORSEL_REG,    .val = 0x02, },
};

static struct _buttonInfo psmtcButtons[] = {
    { .keycode = KEY_0,    .mask = SX9500_TCHCMPSTAT_TCHSTAT0_FLAG, },
    { .keycode = KEY_1,    .mask = SX9500_TCHCMPSTAT_TCHSTAT1_FLAG, },
    { .keycode = KEY_2,    .mask = SX9500_TCHCMPSTAT_TCHSTAT2_FLAG, },
    { .keycode = KEY_3,    .mask = SX9500_TCHCMPSTAT_TCHSTAT3_FLAG, },
};

static struct _touchCheckParameters smtcTouchCheckParameters = {
    .defaultStartupMainSensor = SENSORSEL_CS2,/* Main */
#if defined(CONFIG_MACH_MSM8926_E8LTE)
    .defaultStartupMainSensor = SENSORSEL_CS0,/* Main */
    .defaultSecondStartupMainSensor = SENSORSEL_CS2, /* Main2 */
#endif
    .defaultStartupRefSensor  = SENSORSEL_CS3,/* Ref */
#if defined(CONFIG_MACH_MSM8926_E8LTE)
    .main_csx = 82000,
    .ref_csx  = 6, // Setting this to 0 gives us a static threshold defined by capMain
    .minimum_threshold = 800,
    .calibratoin_margin = 0,
#else
    .main_csx = 43000,
    .ref_csx  = 4, // Setting this to 0 gives us a static threshold defined by capMain
    .minimum_threshold = 600,
    .calibratoin_margin = 0,
#endif
#if defined(CONFIG_MACH_MSM8926_E8LTE)
    .duringTouch_AvgThresh = 0x36, // +/-7000/128=54=0x36
#else
    .duringTouch_AvgThresh = 0x29, // +/-5250/128= 41=0x29
#endif
    .duringRelease_AvgThresh = 0x40, // Same value (RegProxCtrl4)
};

static bool running_cal_or_reset = false;
static bool skip_startup = false;
static bool check_allnear = false;
static bool on_sensor = false;

static int initialize_device(struct sx86XX *this);

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
        dev_dbg(&i2c->dev, "write reg Add=0x%02x Val=0x%02x Ret=%d\n", address, value, returnValue);
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

        if (address != SX9500_IRQSTAT_REG) //temp 2014.03.24 gooni.shim
            dev_dbg(&i2c->dev, "read reg Add=0x%02x Ret=0x%02x\n", address, returnValue);

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

static void onoff_sensor(struct sx86XX *this, int onoff_mode)
{
    struct sx9500 *pDevice = 0;
    struct sx9500_platform_data *pdata = 0;

    unsigned char val_regproxctrl0;
    unsigned char val_regirqmask;

    int nparse_mode;
    int i = 0;

    if (this && (pDevice = this->pDevice) && (pdata = pDevice->hw)) {
        while ( i < pdata->i2c_reg_num) {
            if (pdata->pi2c_reg[i].reg == SX9500_IRQ_ENABLE_REG)
                val_regirqmask = pdata->pi2c_reg[i].val;
            else if (pdata->pi2c_reg[i].reg == SX9500_CPS_CTRL0_REG)
                val_regproxctrl0 = pdata->pi2c_reg[i].val;
            i++;
        }

        nparse_mode = onoff_mode & ENABLE_SENSOR_PINS;
        dev_dbg(this->pdev, "nparse_mode=0x%02x, on_sensor=%d\n", nparse_mode, (int)on_sensor);
        if (nparse_mode == ENABLE_SENSOR_PINS) {
            write_register(this, SX9500_CPS_CTRL0_REG, val_regproxctrl0);

            if (!on_sensor)
                enable_irq_wake(this->irq);

            on_sensor = true;
        }

        nparse_mode = onoff_mode & DISABLE_SENSOR_PINS;
        if (nparse_mode == DISABLE_SENSOR_PINS) {
            for (i = 0; i < pDevice->pbuttonInformation->buttonSize; i++) {
                pDevice->pbuttonInformation->buttons[i].state = IDLE;
            }

            write_register(this, SX9500_CPS_CTRL0_REG, ((val_regproxctrl0 >> 4) << 4) | 0x00);

            if (on_sensor)
                disable_irq_wake(this->irq);

            on_sensor = false;
        }

        nparse_mode = onoff_mode & ENABLE_IRQ_MASK;
        if (nparse_mode == ENABLE_IRQ_MASK)
            write_register(this, SX9500_IRQ_ENABLE_REG, val_regirqmask);

        nparse_mode = onoff_mode & DISABLE_IRQ_MASK;
        if (nparse_mode == DISABLE_IRQ_MASK)
            write_register(this, SX9500_IRQ_ENABLE_REG, 0x00);

        msleep(100); /* make sure everything is running */

    }
}

static int write_calibration_data(struct sx86XX *this, s32 val, u8 msByte, u8 lsByte)
{
    int fd;
    int ret = 0;
    char buf[50];
    mm_segment_t old_fs = get_fs();

    memset(buf, 0, sizeof(buf));

    sprintf(buf, "%d %02x %02x", val, msByte, lsByte);

    dev_info(this->pdev, "buf = %s\n", buf);

    set_fs(KERNEL_DS);
    fd = sys_open(PATH_CAPSENSOR_CAL, O_WRONLY|O_CREAT, 0664);

    if (fd >= 0) {
        sys_write(fd, buf, sizeof(buf));
        sys_fsync(fd); //ensure calibration data write to file system 
        sys_close(fd);
        sys_chmod(PATH_CAPSENSOR_CAL, 0664);
        set_fs(old_fs);
    }
    else {
        ret++;
        sys_close(fd);
        set_fs(old_fs);
        return ret;
    }

    return ret;
}

#if defined(CONFIG_MACH_MSM8926_E8LTE)
static int write_calibration2_data(struct sx86XX *this, s32 val, u8 msByte, u8 lsByte)
{
    int fd;
    int ret = 0;
    char buf[50];
    mm_segment_t old_fs = get_fs();

    memset(buf, 0, sizeof(buf));

    sprintf(buf, "%d %02x %02x", val, msByte, lsByte);

    dev_info(this->pdev, "buf = %s\n", buf);

    set_fs(KERNEL_DS);
    fd = sys_open(PATH_CAPSENSOR_CAL2, O_WRONLY|O_CREAT, 0664);

    if (fd >= 0) {
        sys_write(fd, buf, sizeof(buf));
        sys_fsync(fd); //ensure calibration data write to file system
        sys_close(fd);
        sys_chmod(PATH_CAPSENSOR_CAL2, 0664);
        set_fs(old_fs);
    }
    else {
        ret++;
        sys_close(fd);
        set_fs(old_fs);
        return ret;
    }

    return ret;
}
#endif

static void read_calibration_data(struct sx86XX *this, s32 *nMargin, u8 *msByte, u8 *lsByte)
{
    int fd;
    //int ret = 0;
    int len = 0;
    char read_buf[50];
    mm_segment_t old_fs = get_fs();

    s32 nCalMargin = -1;
    u32 msByte_Offset = 0;
    u32 lsByte_Offset = 0;

    *nMargin = (s32) nCalMargin;
    *msByte = (u8) msByte_Offset;
    *lsByte = (u8) lsByte_Offset;

    memset(read_buf, 0, sizeof(read_buf));
    set_fs(KERNEL_DS);

    fd = sys_open(PATH_CAPSENSOR_CAL, O_RDONLY, 0);
    if(fd >= 0) {
        len = sys_read(fd, read_buf, sizeof(read_buf));
        dev_dbg(this->pdev, "cap sensor calibration file size is = %d\n", len);
        if(len <= 0)
        {
            //ret = -1;
            sys_close(fd);
            set_fs(old_fs);
            //return ret;
            return;
        }

        sscanf(read_buf, "%d %02x %02x", &nCalMargin, &msByte_Offset, &lsByte_Offset);

        *nMargin = (s32) nCalMargin;
        *msByte = (u8) msByte_Offset;
        *lsByte = (u8) lsByte_Offset;

        sys_close(fd);
        set_fs(old_fs);
    }
    else
    {
        dev_err(this->pdev, "Read cap sensor cal data. Error[%d]!!!\n",  fd);
        //ret = -1;
        sys_close(fd);
        set_fs(old_fs);
        //return ret;
        return;
    }
}

static void read_xo_therm_data(struct sx86XX *this, int *xo_therm)
{
    int fd;
    //int ret = 0;
    int len = 0;
    char read_buf[32];
    mm_segment_t old_fs = get_fs();

    int nresult = 0;
    int nraw = 0;

    *xo_therm = (int) nresult;

    memset(read_buf, 0, sizeof(read_buf));
    set_fs(KERNEL_DS);

    fd = sys_open(PATH_XO_THERM, O_RDONLY, 0);
    if(fd >= 0) {
        len = sys_read(fd, read_buf, sizeof(read_buf));
        dev_dbg(this->pdev, "current xo_thermal data file size is = %d\n", len);
        if(len <= 0)
        {
            //ret = -1;
            sys_close(fd);
            set_fs(old_fs);
            //return ret;
            return;
        }

        sscanf(read_buf, "Result:%d Raw:%d\n", &nresult, &nraw);

        *xo_therm = (int) nresult;

        sys_close(fd);
        set_fs(old_fs);
    }
    else
    {
        dev_err(this->pdev, "Read xo_therm data. Error[%d]!!!\n",  fd);
        //ret = -1;
        sys_close(fd);
        set_fs(old_fs);
        //return ret;
        return;
    }
}

#if defined(CONFIG_MACH_MSM8926_E8LTE)
static void read_calibration2_data(struct sx86XX *this, s32 *nMargin, u8 *msByte, u8 *lsByte)
{
    int fd;
    //int ret = 0;
    int len = 0;
    char read_buf[50];
    mm_segment_t old_fs = get_fs();

    s32 nCalMargin = -1;
    u32 msByte_Offset = 0;
    u32 lsByte_Offset = 0;

    *nMargin = (s32) nCalMargin;
    *msByte = (u8) msByte_Offset;
    *lsByte = (u8) lsByte_Offset;

    memset(read_buf, 0, sizeof(read_buf));
    set_fs(KERNEL_DS);

    fd = sys_open(PATH_CAPSENSOR_CAL2, O_RDONLY, 0);
    if(fd >= 0) {
        len = sys_read(fd, read_buf, sizeof(read_buf));
        dev_dbg(this->pdev, "cap sensor calibration file size is = %d\n", len);
        if(len <= 0)
        {
            //ret = -1;
            sys_close(fd);
            set_fs(old_fs);
            //return ret;
            return;
        }

        sscanf(read_buf, "%d %02x %02x", &nCalMargin, &msByte_Offset, &lsByte_Offset);

        *nMargin = (s32) nCalMargin;
        *msByte = (u8) msByte_Offset;
        *lsByte = (u8) lsByte_Offset;

        sys_close(fd);
        set_fs(old_fs);
    }
    else
    {
        dev_dbg(this->pdev, "Return error code : %d\n",  fd);
        //ret = -1;
        sys_close(fd);
        set_fs(old_fs);
        //return ret;
        return;
    }

    //return (simple_strtol(read_buf, NULL, 10));
}
#endif

static void read_sensor_regdata(struct sx86XX *this, unsigned char nSensorSel,
                             s16 *pUseful_CSx, s16 *pAvg_CSx, s16 *pDiff_CSx, u16 *pOffset_CSx)
{
    u8 msByte = 0;
    u8 lsByte = 0;

    s16 Useful_CSx = 0;
    s16 Avg_CSx = 0;
    s16 Diff_CSx = 0;
    u16 Offset_CSx = 0;

    if (this) {
        write_register(this, SX9500_SENSORSEL_REG, nSensorSel);

        read_register(this, SX9500_USEMSB_REG, &msByte);
        read_register(this, SX9500_USELSB_REG, &lsByte);
        Useful_CSx = (s16)((msByte << 8) | lsByte);

        msByte = 0; lsByte = 0;
        read_register(this, SX9500_AVGMSB_REG, &msByte);
        read_register(this, SX9500_AVGLSB_REG, &lsByte);
        Avg_CSx = (s16)((msByte << 8) | lsByte);

        msByte = 0; lsByte = 0;
        read_register(this, SX9500_DIFFMSB_REG, &msByte);
        read_register(this, SX9500_DIFFLSB_REG, &lsByte);
        Diff_CSx = (s16)((msByte << 8) | lsByte);
        if (Diff_CSx > 4095)
            Diff_CSx -= 8192;

        msByte = 0; lsByte = 0;
        read_register(this, SX9500_OFFSETMSB_REG, &msByte);
        read_register(this, SX9500_OFFSETLSB_REG, &lsByte);
        Offset_CSx = (u16)((msByte << 8) | lsByte);

        *pUseful_CSx = (s16) Useful_CSx;
        *pAvg_CSx = (s16) Avg_CSx;
        *pDiff_CSx = (s16) Diff_CSx;
        *pOffset_CSx = (u16) Offset_CSx;
    }

    return;
}

static s32 calculate_CSx_rawdata(struct sx86XX *this, unsigned char channel_num)
{
    u8 msByte = 0;
    u8 lsByte = 0;

    s16 Useful_CSx = 0;
    u16 Offset_CSx = 0; /* fullbyte */
    s32 Calculate_CSx = 0;

    // Calculate out the CSx Cap information //
    write_register(this, SX9500_SENSORSEL_REG, channel_num);
    read_register(this, SX9500_USEMSB_REG, &msByte);
    read_register(this, SX9500_USELSB_REG, &lsByte);
    Useful_CSx = (s16) ((msByte << 8) | lsByte);

    msByte = 0;  lsByte = 0;
    read_register(this, SX9500_OFFSETMSB_REG, &msByte);
    read_register(this, SX9500_OFFSETLSB_REG, &lsByte);
    Offset_CSx = (u16)((msByte << 8) | lsByte);

    msByte = 0; lsByte = 0;
    msByte = (u8)(Offset_CSx >> 6);
    lsByte = (u8)(Offset_CSx - (((u16) msByte) << 6));

    /*~+/-3.0 *20000  - medium smallm, digital gain factor - 8x */
    Calculate_CSx = 2 * (((s32) msByte * 3600) + ((s32) lsByte * 225)) +
                                (((s32) Useful_CSx * 60000) / (8 * 65536));

    dev_info(this->pdev, "CS[%02x] Useful = %6d, Offset = %6d, Calculate = %6d\n", 
                            channel_num, Useful_CSx, Offset_CSx, Calculate_CSx);

    return Calculate_CSx;
}

#if 0
static int compensate_offset_by_temperature(struct sx86XX *this, int nCelsius)
{
    if (nCelsius >= 60)
        return 3;
    else if (nCelsius >= 50 && nCelsius < 60)
        return 3;
    else if (nCelsius >= 40 && nCelsius < 50)
        return 2;
    else if (nCelsius >= 30 && nCelsius < 40)
        return 1;
    else if (nCelsius < 30)
        return 0;

    return 0;
}
#endif

/***********************************************************/
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

    dev_dbg( this->pdev, "Performing manual_offset_calibration()\n");
    manual_offset_calibration(this);

    return count;
}

static unsigned char conv_sensorsel_to_sensoren(unsigned char csx_pin)
{
    unsigned char sensoren_csx;

    switch(csx_pin) {
        case SENSORSEL_CS0: sensoren_csx = SENSOREN_CS0; break;
        case SENSORSEL_CS1: sensoren_csx = SENSOREN_CS1; break;
        case SENSORSEL_CS2: sensoren_csx = SENSOREN_CS2; break;
        case SENSORSEL_CS3: sensoren_csx = SENSOREN_CS3; break;
    }

    return sensoren_csx;
}

static int cmp_rawdata(const void *a, const void *b)
{
    return (*(s32 *) a) - (*(s32 *) b);
}

static ssize_t sx9500_show_skipstartup(struct device *dev,
                     struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", (int) skip_startup);
}

static ssize_t sx9500_store_skipstartup(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t count)
{
    struct sx86XX *this = dev_get_drvdata(dev);
    unsigned long val;

    if (strict_strtoul(buf, 0, &val))
        return -EINVAL;

    dev_dbg(this->pdev,"skipstartup set val = %d\n", (int) val);
    if (val == 0)
        skip_startup = false;
    else if (val == 1)
        skip_startup = true;

    return count;
}

static ssize_t sx9500_store_onoffsensor(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t count)
{
    struct sx86XX *this = dev_get_drvdata(dev);
    unsigned long val;

    if (strict_strtoul(buf, 0, &val))
        return -EINVAL;

    dev_info(this->pdev,"request onoff val = %d\n", (int) val);
    if (val == ON_SENSOR)
        onoff_sensor(this, ENABLE_IRQ_MASK | ENABLE_SENSOR_PINS);
    else if (val == OFF_SENSOR)
        onoff_sensor(this, DISABLE_IRQ_MASK | DISABLE_SENSOR_PINS);

    return count;
}

static ssize_t sx9500_store_docalibration(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t count)
{
    struct sx86XX *this = dev_get_drvdata(dev);
    struct sx9500 *pDevice = NULL;
    unsigned char mainSensor;
#if defined(CONFIG_MACH_MSM8926_E8LTE)
    unsigned char mainSensor2;
    u8 capMain2_Offset_msByte = 0;
    u8 capMain2_Offset_lsByte = 0;
    s32 capMain2[COLLECT_NUM];
    s32 avg_capMain2 = 0;
    s32 capMargin2 = 0;
#endif
    unsigned char refSensor;
    unsigned char cal_scanperiod = SCANPERIOD_CAL;// Set SCANPERIOD[6:4] - 000(30ms).
    unsigned char write_RegProxCtrl0 = 0x00;

    u8 capMain_Offset_msByte = 0;
    u8 capMain_Offset_lsByte = 0;
    s32 capMain[COLLECT_NUM];
    s32 capRef[COLLECT_NUM];
    s32 avg_capMain = 0;
    s32 avg_capRef = 0;
    s32 min_threshold = 0;
    s32 capMargin = 0;

    int ret = 0;

    if (this && (pDevice = this->pDevice) && !running_cal_or_reset) {
        int i = 0;
        u8 old_RegIrqMsk_val = 0;
        u8 old_RegCtrl0_val = 0;
        unsigned char conv_mainSensor;
#if defined(CONFIG_MACH_MSM8926_E8LTE)
        unsigned char conv_mainSensor2;
#endif
        unsigned char conv_refSensor;

        running_cal_or_reset = true;

        dev_info(this->pdev, "calibration start!!!\n");

        mainSensor = (unsigned char)pDevice->ptouchCheckParameters->defaultStartupMainSensor;
#if defined(CONFIG_MACH_MSM8926_E8LTE)
        mainSensor2 = (unsigned char)pDevice->ptouchCheckParameters->defaultSecondStartupMainSensor;
#endif
        refSensor = (unsigned char)pDevice->ptouchCheckParameters->defaultStartupRefSensor;

        conv_mainSensor = conv_sensorsel_to_sensoren(mainSensor);
#if defined(CONFIG_MACH_MSM8926_E8LTE)
        conv_mainSensor2 = conv_sensorsel_to_sensoren(mainSensor2);
#endif
        conv_refSensor = conv_sensorsel_to_sensoren(refSensor);
#if defined(CONFIG_MACH_MSM8926_E8LTE)
        write_RegProxCtrl0 = (unsigned char) ((cal_scanperiod << 4) | (conv_mainSensor | conv_mainSensor2 | conv_refSensor));
#else
        write_RegProxCtrl0 = (unsigned char) ((cal_scanperiod << 4) | (conv_mainSensor | conv_refSensor));
#endif

        read_register(this, SX9500_IRQ_ENABLE_REG, &old_RegIrqMsk_val);
        read_register(this, SX9500_CPS_CTRL0_REG, &old_RegCtrl0_val);

        write_register(this, SX9500_IRQ_ENABLE_REG, 0x70);
        write_register(this, SX9500_CPS_CTRL0_REG, write_RegProxCtrl0);
        msleep(100); /* make sure everything is running */
        manual_offset_calibration(this);

        // Calculate out the Main Cap information.
        // To sort the collected value. 2 max & 2 min values are excluded.
        for (i = 0; i < COLLECT_NUM; i++) {
            capMain[i] = calculate_CSx_rawdata(this, mainSensor);
            dev_dbg(this->pdev,"capMain[%d] = %d\n", i, capMain[i]);
            msleep(30);
        }
        read_register(this, SX9500_OFFSETMSB_REG, &capMain_Offset_msByte);
        read_register(this, SX9500_OFFSETLSB_REG, &capMain_Offset_lsByte);

        sort(capMain, COLLECT_NUM, sizeof(s32), cmp_rawdata, NULL);
        for (i = FILTER_NUM; i < COLLECT_NUM - FILTER_NUM; i++)
            avg_capMain += capMain[i];
        avg_capMain = avg_capMain / (COLLECT_NUM - (FILTER_NUM * 2));

#if defined(CONFIG_MACH_MSM8926_E8LTE)
        // Calculate out the Main2 Cap information.
        // To sort the collected value. 2 max & 2 min values are excluded.
        for (i = 0; i < COLLECT_NUM; i++) {
            capMain2[i] = calculate_CSx_rawdata(this, mainSensor2);
            dev_dbg(this->pdev,"capMain2[%d] = %d\n", i, capMain2[i]);
            msleep(30);
        }
        read_register(this, SX9500_OFFSETMSB_REG, &capMain2_Offset_msByte);
        read_register(this, SX9500_OFFSETLSB_REG, &capMain2_Offset_lsByte);

        sort(capMain2, COLLECT_NUM, sizeof(s32), cmp_rawdata, NULL);
        for (i = FILTER_NUM; i < COLLECT_NUM - FILTER_NUM; i++)
            avg_capMain2 += capMain2[i];
        avg_capMain2 = avg_capMain2 / (COLLECT_NUM - (FILTER_NUM * 2));
        dev_dbg(this->pdev,"avg_capMain2 = %d\n", avg_capMain2);
#endif

        // Calculate out the Reference Cap information + Apply temperature Compensation
        // To sort the collected value. 2 max & 2 min values are excluded.
        for (i = 0; i < COLLECT_NUM; i++) {
            capRef[i] = calculate_CSx_rawdata(this, refSensor);
            dev_dbg(this->pdev, "capRef[%d] = %d\n", i, capRef[i]);
            msleep(30);
        }        

        sort(capRef, COLLECT_NUM, sizeof(s32), cmp_rawdata, NULL);
        for (i = FILTER_NUM; i < COLLECT_NUM - FILTER_NUM; i++)
            avg_capRef += capRef[i];
        avg_capRef = avg_capRef / (COLLECT_NUM - (FILTER_NUM * 2));

        avg_capRef = pDevice->ptouchCheckParameters->main_csx + 
                        (pDevice->ptouchCheckParameters->ref_csx * avg_capRef);

        min_threshold = pDevice->ptouchCheckParameters->minimum_threshold;

        write_register(this, SX9500_IRQ_ENABLE_REG, old_RegIrqMsk_val);
        write_register(this, SX9500_CPS_CTRL0_REG, old_RegCtrl0_val);

        // Must be sure to set the Main CS pin.
        write_register(this, SX9500_SENSORSEL_REG, mainSensor);

        capMargin = avg_capMain - avg_capRef + min_threshold;
#if defined(CONFIG_MACH_MSM8926_E8LTE)
        capMargin2 = avg_capMain2 - avg_capRef + min_threshold;
#endif
        ret = write_calibration_data(this, capMargin, capMain_Offset_msByte, capMain_Offset_lsByte);
#if defined(CONFIG_MACH_MSM8926_E8LTE)
        ret = write_calibration2_data(this, capMargin2, capMain2_Offset_msByte, capMain2_Offset_lsByte);
#endif

        dev_info(this->pdev, "calibration end!!!\n");

        running_cal_or_reset = false;

    }

    return count;
}

static ssize_t sx9500_store_checkallnear(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t count)
{
    struct sx86XX *this = dev_get_drvdata(dev);
    unsigned long val;

    if (strict_strtoul(buf, 0, &val))
        return -EINVAL;

    dev_dbg(this->pdev,"checkallnear set val = %d\n", (int) val);
    if (val == 0)
        check_allnear = false;
    else if (val == 1)
        check_allnear = true;

    return count;
}

static ssize_t sx9500_show_count_inputpins(struct device *dev,
                     struct device_attribute *attr, char *buf)
{
    int count_inputpins = 0;

    struct sx86XX *this = dev_get_drvdata(dev);
    struct sx9500 *pDevice = NULL;
    struct sx9500_platform_data *pdata = NULL;

    if (this && (pDevice = this->pDevice) && (pdata = pDevice->hw)) {
        int i = 0;

        unsigned char val_regproxctrl0;
        unsigned char refSensor;
        unsigned char conv_refSensor;

        unsigned char val_mainpins;

        unsigned char mask_sensoren;

        while (i < pdata->i2c_reg_num) {
            if (pdata->pi2c_reg[i].reg == SX9500_CPS_CTRL0_REG) {
                val_regproxctrl0 = pdata->pi2c_reg[i].val;
                break;
            }
            i++;
        }

        if (val_regproxctrl0) {      
            refSensor = (unsigned char)pDevice->ptouchCheckParameters->defaultStartupRefSensor;
            conv_refSensor = conv_sensorsel_to_sensoren(refSensor);

            val_mainpins = val_regproxctrl0 - conv_refSensor;
            if (val_mainpins > 0) {
                for (i=0; i < 4; i++) {
                    mask_sensoren = (SENSOREN_CS0 << i);
                    if ((val_mainpins & mask_sensoren) == mask_sensoren) {
                        count_inputpins++;
                    }
                }
            }

        }
    }

    dev_info(this->pdev,"count input pins = %d\n", count_inputpins);

    return sprintf(buf, "%d\n", count_inputpins);
}

static ssize_t sx9500_show_proxstatus(struct device *dev,
                     struct device_attribute *attr, char *buf)
{
    int prox_status = -1;

    struct sx9500 *pDevice = NULL;
    struct sx86XX *this = dev_get_drvdata(dev);

    struct _buttonInfo *buttons = NULL;
#if !defined(CONFIG_MACH_MSM8926_E8LTE)
    struct _buttonInfo *pCurrentButton  = NULL;
#endif

    if (this && (pDevice = this->pDevice)) {
        buttons = pDevice->pbuttonInformation->buttons;
#if defined(CONFIG_MACH_MSM8926_E8LTE)
        if (check_allnear) {
            if(buttons[0].state == IDLE || buttons[2].state == IDLE) {
               prox_status = 1;
            }

            if(buttons[0].state == ACTIVE && buttons[2].state == ACTIVE) {
                prox_status = 0;
            }
        }
        else {
            if(buttons[0].state == IDLE && buttons[2].state == IDLE) {
                prox_status = 1;
            }

            if(buttons[0].state == ACTIVE || buttons[2].state == ACTIVE) {
                prox_status = 0;
            }
        }
#else
        pCurrentButton = &buttons[2]; // only CS2
        prox_status = pCurrentButton->state;
#endif
    }

    return sprintf(buf, "%d\n", prox_status);
}

static ssize_t sx9500_show_regproxdata(struct device *dev,
                     struct device_attribute *attr, char *buf)
{
    s16 Useful_CSx = 0, Avg_CSx = 0, Diff_CSx = 0;
    s16 ref_Useful_CSx = 0, ref_Avg_CSx = 0, ref_Diff_CSx = 0;
    u16 Offset_CSx = 0, ref_Offset_CSx = 0;

    unsigned char mainSensor;
    unsigned char refSensor;

#if defined(CONFIG_MACH_MSM8926_E8LTE)
    s16 Useful_CSx2 = 0, Avg_CSx2 = 0, Diff_CSx2 = 0;
    u16 Offset_CSx2 = 0;
    unsigned char mainSensor2;
#endif

    int current_xo_therm = 0;

    struct sx9500 *pDevice = NULL;
    struct sx86XX *this = dev_get_drvdata(dev);

    if (this && (pDevice = this->pDevice) && !running_cal_or_reset) {
        mainSensor = (unsigned char)pDevice->ptouchCheckParameters->defaultStartupMainSensor;
#if defined(CONFIG_MACH_MSM8926_E8LTE)
        mainSensor2 = (unsigned char)pDevice->ptouchCheckParameters->defaultSecondStartupMainSensor;
#endif
        refSensor = (unsigned char)pDevice->ptouchCheckParameters->defaultStartupRefSensor;

        // Select Main Sensor, Readback Reg Sensor Data
        read_sensor_regdata(this, mainSensor, &Useful_CSx, &Avg_CSx, &Diff_CSx, &Offset_CSx);
        dev_info(this->pdev,"capMain = %6d %6d %6d %6d\n", Useful_CSx, Avg_CSx, Diff_CSx, Offset_CSx);

#if defined(CONFIG_MACH_MSM8926_E8LTE)
        // Select Main2 Sensor, Readback Reg Sensor Data
        read_sensor_regdata(this, mainSensor2, &Useful_CSx2, &Avg_CSx2, &Diff_CSx2, &Offset_CSx2);
        dev_info(this->pdev,"capMain2 = %6d %6d %6d %6d\n", Useful_CSx2, Avg_CSx2, Diff_CSx2, Offset_CSx2);
#endif

        // Select Reference Sensor, Readback Reg Sensor Data
        read_sensor_regdata(this, refSensor, &ref_Useful_CSx, &ref_Avg_CSx, &ref_Diff_CSx, &ref_Offset_CSx);
        dev_info(this->pdev,"capRef = %6d %6d %6d %6d\n", ref_Useful_CSx, ref_Avg_CSx, ref_Diff_CSx, ref_Offset_CSx);

        // Please Select Main Sensor again.
        write_register(this, SX9500_SENSORSEL_REG, mainSensor);

        read_xo_therm_data(this, &current_xo_therm);
    }

#if defined(CONFIG_MACH_MSM8926_E8LTE)
    return sprintf(buf, "%6d %6d %6d %6d %3d\n%6d %6d %6d %6d\n%6d %6d %6d %6d\n",
                        Useful_CSx, Avg_CSx, Diff_CSx, Offset_CSx, current_xo_therm,
                        Useful_CSx2, Avg_CSx2, Diff_CSx2, Offset_CSx2,
                        ref_Useful_CSx, ref_Avg_CSx, ref_Diff_CSx, ref_Offset_CSx);
#else
    return sprintf(buf, "%6d %6d %6d %6d %3d\n%6d %6d %6d %6d\n",
                        Useful_CSx, Avg_CSx, Diff_CSx, Offset_CSx, current_xo_therm,
                        ref_Useful_CSx, ref_Avg_CSx, ref_Diff_CSx, ref_Offset_CSx);
#endif

}

static ssize_t sx9500_show_regproxctrl0(struct device *dev,
                     struct device_attribute *attr, char *buf)
{
    u8 reg_value = 0;
    struct sx86XX *this = dev_get_drvdata(dev);

    read_register(this, SX9500_CPS_CTRL0_REG, &reg_value);

    return sprintf(buf, "0x%02x\n", reg_value);
}

static ssize_t sx9500_store_regproxctrl0(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t count)
{
    struct sx9500 *pDevice = NULL;
    struct sx86XX *this = dev_get_drvdata(dev);

    //struct _buttonInfo *buttons = NULL;
    //struct _buttonInfo *pCurrentButton  = NULL;

    struct input_dev *input = NULL;

    unsigned long val;
    unsigned long val_check;

    int i = 0;
    int ret = 0;

    if (strict_strtoul(buf, 0, &val))
        return -EINVAL;

    ret = write_register(this, SX9500_CPS_CTRL0_REG, val);
    if (ret > 0) {
        val_check = val & SENSOREN_CS2;
        if (val_check == SENSOREN_DISABLE_ALL) {
            /* Initialize prox status : default FAR */
            if (this && (pDevice = this->pDevice)) {
                for (i = 0; i < pDevice->pbuttonInformation->buttonSize; i++) {
                    pDevice->pbuttonInformation->buttons[i].state = IDLE;
                }

                input = pDevice->pbuttonInformation->input;
                input_report_abs(input, ABS_DISTANCE, PROX_STATUS_FAR);
                input_sync(input);
            }
        }
    }

    return count;
}

static ssize_t sx9500_show_regproxctrl1(struct device *dev,
                     struct device_attribute *attr, char *buf)
{
    u8 reg_value = 0;
    struct sx86XX *this = dev_get_drvdata(dev);

    read_register(this, SX9500_CPS_CTRL1_REG, &reg_value);

    return sprintf(buf, "0x%02x\n", reg_value);
}

static ssize_t sx9500_store_regproxctrl1(struct device *dev,
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

static ssize_t sx9500_show_regproxctrl2(struct device *dev,
                     struct device_attribute *attr, char *buf)
{
    u8 reg_value = 0;
    struct sx86XX *this = dev_get_drvdata(dev);

    read_register(this, SX9500_CPS_CTRL2_REG, &reg_value);

    return sprintf(buf, "0x%02x\n", reg_value);
}

static ssize_t sx9500_store_regproxctrl2(struct device *dev,
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

static ssize_t sx9500_show_regproxctrl3(struct device *dev,
                     struct device_attribute *attr, char *buf)
{
    u8 reg_value = 0;
    struct sx86XX *this = dev_get_drvdata(dev);

    read_register(this, SX9500_CPS_CTRL3_REG, &reg_value);

    return sprintf(buf, "0x%02x\n", reg_value);
}

static ssize_t sx9500_store_regproxctrl3(struct device *dev,
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

static ssize_t sx9500_show_regproxctrl4(struct device *dev,
                     struct device_attribute *attr, char *buf)
{
    u8 reg_value = 0;
    struct sx86XX *this = dev_get_drvdata(dev);

    read_register(this, SX9500_CPS_CTRL4_REG, &reg_value);

    return sprintf(buf, "0x%02x\n", reg_value);
}

static ssize_t sx9500_store_regproxctrl4(struct device *dev,
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

static ssize_t sx9500_show_regproxctrl5(struct device *dev,
                     struct device_attribute *attr, char *buf)
{
    u8 reg_value = 0;
    struct sx86XX *this = dev_get_drvdata(dev);

    read_register(this, SX9500_CPS_CTRL5_REG, &reg_value);

    return sprintf(buf, "0x%02x\n", reg_value);
}

static ssize_t sx9500_store_regproxctrl5(struct device *dev,
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

static ssize_t sx9500_show_regproxctrl6(struct device *dev,
                     struct device_attribute *attr, char *buf)
{
    u8 reg_value = 0;
    struct sx86XX *this = dev_get_drvdata(dev);

    read_register(this, SX9500_CPS_CTRL6_REG, &reg_value);

    return sprintf(buf, "0x%02x\n", reg_value);
}

static ssize_t sx9500_store_regproxctrl6(struct device *dev,
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

static ssize_t sx9500_show_regproxctrl7(struct device *dev,
                     struct device_attribute *attr, char *buf)
{
    u8 reg_value = 0;
    struct sx86XX *this = dev_get_drvdata(dev);

    read_register(this, SX9500_CPS_CTRL7_REG, &reg_value);

    return sprintf(buf, "0x%02x\n", reg_value);
}

static ssize_t sx9500_store_regproxctrl7(struct device *dev,
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

static ssize_t sx9500_show_regproxctrl8(struct device *dev,
                     struct device_attribute *attr, char *buf)
{
    u8 reg_value = 0;
    struct sx86XX *this = dev_get_drvdata(dev);

    read_register(this, SX9500_CPS_CTRL8_REG, &reg_value);

    return sprintf(buf, "0x%02x\n", reg_value);
}

static ssize_t sx9500_store_regproxctrl8(struct device *dev,
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

    return sprintf(buf, "0x%02x\n", reg_value);
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
    struct sx9500 *pDevice = NULL;
    struct sx86XX *this = dev_get_drvdata(dev);
    struct input_dev *input = NULL;

    unsigned long val;
    int i = 0;

    if (strict_strtoul(buf, 0, &val))
        return -EINVAL;

    if (this && (pDevice = this->pDevice) && !running_cal_or_reset) {

        running_cal_or_reset = true;

        if (val == SX9500_SOFTRESET) {

            dev_info(this->pdev, "reset start!!!\n");

            for (i = 0; i < pDevice->pbuttonInformation->buttonSize; i++) {
                pDevice->pbuttonInformation->buttons[i].state = IDLE;
            }

            input = pDevice->pbuttonInformation->input;
            input_report_abs(input, ABS_DISTANCE, PROX_STATUS_FAR);
            input_sync(input);

            //sx86XX_suspend(this);
            //sx86XX_resume(this);
            //disable_irq(this->irq);

        #ifdef USE_THREADED_IRQ
            mutex_lock(&this->mutex);
            /* Just in case need to reset any uncaught interrupts */
            sx86XX_process_interrupt(this,0);
            mutex_unlock(&this->mutex);
        #else
            sx86XX_schedule_work(this,0);
        #endif
            initialize_device(this);

            //enable_irq(this->irq);

            dev_info(this->pdev, "reset end!!!\n");

        }

        running_cal_or_reset = false;

    }

    return count;
}

static ssize_t sx9500_store_regoffset(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t count)
{
    struct sx86XX *this = dev_get_drvdata(dev);

    unsigned int msb_val;
    unsigned int lsb_val;

    if(sscanf(buf, "%02x,%02x", &msb_val, &lsb_val) != 2)
        return -EINVAL;

    dev_dbg(this->pdev, "regoffset = %02x, %02x\n", msb_val, lsb_val);

    write_register(this, SX9500_OFFSETMSB_REG, msb_val);
    write_register(this, SX9500_OFFSETLSB_REG, lsb_val);

    return count;
}

static DEVICE_ATTR(calibrate,    0664, manual_offset_calibration_show, manual_offset_calibration_store);
static DEVICE_ATTR(skipstartup,  0664, sx9500_show_skipstartup, sx9500_store_skipstartup);
static DEVICE_ATTR(onoff,        0664, NULL, sx9500_store_onoffsensor);
static DEVICE_ATTR(docalibration,0664, NULL, sx9500_store_docalibration);
static DEVICE_ATTR(checkallnear, 0664, NULL, sx9500_store_checkallnear);
static DEVICE_ATTR(cntinputpins, 0664, sx9500_show_count_inputpins, NULL);
static DEVICE_ATTR(proxstatus,   0664, sx9500_show_proxstatus, NULL);
static DEVICE_ATTR(regproxdata,  0664, sx9500_show_regproxdata, NULL);
static DEVICE_ATTR(regproxctrl0, 0664, sx9500_show_regproxctrl0, sx9500_store_regproxctrl0);
static DEVICE_ATTR(regproxctrl1, 0664, sx9500_show_regproxctrl1, sx9500_store_regproxctrl1);
static DEVICE_ATTR(regproxctrl2, 0664, sx9500_show_regproxctrl2, sx9500_store_regproxctrl2);
static DEVICE_ATTR(regproxctrl3, 0664, sx9500_show_regproxctrl3, sx9500_store_regproxctrl3);
static DEVICE_ATTR(regproxctrl4, 0664, sx9500_show_regproxctrl4, sx9500_store_regproxctrl4);
static DEVICE_ATTR(regproxctrl5, 0664, sx9500_show_regproxctrl5, sx9500_store_regproxctrl5);
static DEVICE_ATTR(regproxctrl6, 0664, sx9500_show_regproxctrl6, sx9500_store_regproxctrl6);
static DEVICE_ATTR(regproxctrl7, 0664, sx9500_show_regproxctrl7, sx9500_store_regproxctrl7);
static DEVICE_ATTR(regproxctrl8, 0664, sx9500_show_regproxctrl8, sx9500_store_regproxctrl8);
static DEVICE_ATTR(regirqmask,   0664, sx9500_show_regirqmask, sx9500_store_regirqmask);
static DEVICE_ATTR(regreset,     0664, NULL, sx9500_store_regreset);
static DEVICE_ATTR(regoffset,    0664, NULL, sx9500_store_regoffset);

static struct attribute *sx9500_attributes[] = {
    &dev_attr_calibrate.attr,
    &dev_attr_skipstartup.attr,
    &dev_attr_onoff.attr,
    &dev_attr_docalibration.attr,
    &dev_attr_checkallnear.attr,
    &dev_attr_cntinputpins.attr,
    &dev_attr_proxstatus.attr,
    &dev_attr_regproxdata.attr,
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
    &dev_attr_regoffset.attr,
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
            dev_info(this->pdev, "SX9500_IRQSTAT_REG[0x00] = 0x%02x\n", data);
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
            dev_dbg(this->pdev, "Going to Write Reg: 0x%02x Value: 0x%02x\n", pdata->pi2c_reg[i].reg,pdata->pi2c_reg[i].val);
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




/*! \fn static int initialize_device(struct sx86XX *this)
 * \brief Performs all initialization needed to configure the device
 * \param this Pointer to main parent struct 
 * \return Last used command's return value (negative if error)
 */
static int initialize_device(struct sx86XX *this)
{
    s16 Useful_CSx = 0, Avg_CSx = 0, Diff_CSx = 0;
    u16 Offset_CSx = 0;
    int i = 0;

    unsigned char mainSensor, refSensor;
#if defined(CONFIG_MACH_MSM8926_E8LTE)
    unsigned char mainSensor2;
#endif
    struct sx9500 *pDevice = NULL;

    if (this) {
        /* Make sure we initialize that we are not in startup detection */
        this->inStartupTouch = false;
        /* prepare reset by disabling any irq handling */
        this->irq_disabled = 1;
        disable_irq(this->irq);
        /* perform a reset */
        write_register(this, SX9500_SOFTRESET_REG, SX9500_SOFTRESET);
        /* wait until the reset has finished by monitoring NIRQ */
        dev_info(this->pdev, "Sent Software Reset. Waiting until device is back from reset to continue.\n");

        /* just sleep for awhile instead of using a loop with reading irq status */
        msleep(300);

        /*while(this->get_nirq_low && this->get_nirq_low()) {
              read_regIrqStat(this);
        }*/
        dev_info(this->pdev, "Device is back from the reset, continuing. NIRQ = %d\n", this->get_nirq_low());

        hw_init(this);

        msleep(100); /* make sure everything is running */

        manual_offset_calibration(this);

        // This should cover most of the scan periods, if extremely large may
        // want to set this to 500 
    #if defined(CONFIG_MACH_MSM8926_E8LTE)
        msleep(1000); /* make sure manual offset has been fully done */
    #else
        msleep(500); /* make sure manual offset has been fully done */
    #endif

        if (likely((pDevice = this->pDevice) != NULL)) {
            mainSensor = (unsigned char) pDevice->ptouchCheckParameters->defaultStartupMainSensor;
            refSensor = (unsigned char) pDevice->ptouchCheckParameters->defaultStartupRefSensor;
        #if defined(CONFIG_MACH_MSM8926_E8LTE)
            mainSensor2 = (unsigned char) pDevice->ptouchCheckParameters->defaultSecondStartupMainSensor; 
        #endif

            for (i = 0; i < 5 ; i++) {
                read_sensor_regdata(this, mainSensor, &Useful_CSx, &Avg_CSx, &Diff_CSx, &Offset_CSx);
                dev_info(this->pdev,"[startup] capMain[%d] = %6d %6d %6d %6d\n", 
                                                    i, Useful_CSx, Avg_CSx, Diff_CSx, Offset_CSx);
            }

            touchCheckWithReferenceSensor(this, mainSensor, refSensor);
        #if defined(CONFIG_MACH_MSM8926_E8LTE)
            //touchCheckWithReferenceSensor(this, mainSensor2, refSensor);
        #endif
        }
        else {
            dev_err(this->pdev, "Couldn't open platform data containing main and ref sensors, using fallback\n");
            touchCheckWithReferenceSensor(this,(unsigned char)0x02, (unsigned char)0x03);
        #if defined(CONFIG_MACH_MSM8926_E8LTE)
            touchCheckWithReferenceSensor(this,(unsigned char)0x00, (unsigned char)0x03);
        #endif
        }

        /* re-enable interrupt handling */
        enable_irq(this->irq);
        this->irq_disabled = 0;

        /* make sure no interrupts are pending since enabling irq will only
         * work on next falling edge */
        read_regIrqStat(this);
        dev_info(this->pdev, "Exiting initialize(). NIRQ = %d\n", this->get_nirq_low());

        return 0;
    }

    return -ENOMEM;
}

/*!
 * \brief   Check if a touch(near) is on the specified mainSensor
 * \details This uses a reference sensor (refSensor) to check whether a touch
 *      (=near) is being performed. You normally would use this type of check 
 *      after a compensation has been made (especially during power up).
 * \param   this Pointer to main parent struct 
 * \param   mainSensor Main sensor that may or may not have a touch
 * \param   refSensor A known sensor that will never have a touch
 * \return  Whether a touch was detected
 */
void touchCheckWithReferenceSensor(struct sx86XX *this,
                                          unsigned char mainSensor,
                                          unsigned char refSensor) 
{
    s32 cal_Margin = -1;
    u8 cal_Offset_msByte = 0;
    u8 cal_Offset_lsByte = 0;
    u16 cal_Offset = 0; /* fullbyte */

    s32 capMain = 0;
    s32 capRef = 0;
    s32 capStartup = 0;

    int counter = 0;
    u8 CompareButtonMask = 0;
    int numberOfButtons = 0;
    struct sx9500 *pDevice = NULL;
    struct _buttonInfo *buttons = NULL;
    struct input_dev *input = NULL;

    struct _buttonInfo *pCurrentButton  = NULL;

    u8 ucTouchAvgThresh = 0;
    u8 ucReleaseAvgThresh = 0;

#if 0
    int current_xo_therm;
    int compensate_offset = 0;
#endif

    if (unlikely((this==NULL) || ((pDevice = this->pDevice)==NULL)))
        return; // ERROR!!

#if defined(CONFIG_MACH_MSM8926_E8LTE)
    if((u8)mainSensor == 0)
    {
        write_register(this, SX9500_SENSORSEL_REG, (u8)mainSensor);
        read_calibration_data(this, &cal_Margin, &cal_Offset_msByte, &cal_Offset_lsByte);
        dev_info(this->pdev, "(u8)mainSensor == 0\n");
    }


    if((u8)mainSensor == 2)
    {
        write_register(this, SX9500_SENSORSEL_REG, (u8)mainSensor);
        read_calibration2_data(this, &cal_Margin, &cal_Offset_msByte, &cal_Offset_lsByte);
        dev_info(this->pdev, "(u8)mainSensor == 2\n");
    }
#else
    read_calibration_data(this, &cal_Margin, &cal_Offset_msByte, &cal_Offset_lsByte);
#endif

    cal_Offset = (u16)((cal_Offset_msByte << 8) | cal_Offset_lsByte);
    dev_info(this->pdev, "read_calibration_data : %d, %02x, %02x\n", cal_Margin, cal_Offset_msByte, cal_Offset_lsByte);

    if (skip_startup == true)
        return;

    if(cal_Margin == -1 || cal_Offset < 500 || cal_Offset > 4000) {
        dev_err(this->pdev, "Fail!!! Check Calibratoin Data!!!\n");
        return;
    }
#if 0
    //---------------------- temp----------------------
    else {
        read_xo_therm_data(this, &current_xo_therm);
        dev_info(this->pdev, "read_xo_therm_data = %d\n", current_xo_therm);
        compensate_offset = compensate_offset_by_temperature(this, current_xo_therm);
        cal_Offset_lsByte += compensate_offset;
        dev_info(this->pdev, "compensation offset = %02x\n", compensate_offset);

        write_register(this, SX9500_OFFSETMSB_REG, cal_Offset_msByte);
        write_register(this, SX9500_OFFSETLSB_REG, cal_Offset_lsByte);
        //msleep(100);
        return;
    }
    //---------------------- temp----------------------
#endif

    pDevice->ptouchCheckParameters->calibratoin_margin = cal_Margin;

    // Calculate out the Main Cap information //
    capMain = calculate_CSx_rawdata(this, mainSensor);

    // Calculate out the Reference Cap information //
    capRef = calculate_CSx_rawdata(this, refSensor);

    // Calculate Dynamic Threshold value.
    capStartup = pDevice->ptouchCheckParameters->main_csx + 
                   (pDevice->ptouchCheckParameters->ref_csx * capRef) +
                       pDevice->ptouchCheckParameters->calibratoin_margin;

    dev_info(this->pdev, "Main[%ld] - Startup[%ld] = %ld", 
                    (long int) capMain, (long int) capStartup, (long int) (capMain - capStartup));

    // Must be sure to set the Main CS pin.
    write_register(this, SX9500_SENSORSEL_REG, mainSensor);

    // capMain is now the diff to check with startupThreshold..
    // The code below is what will send the key event.
    // We need to shift 4 more as the lower bits are the compensation.
    //  TODO: Change this to use a define index
    CompareButtonMask = 1 << (mainSensor + 4); 

    buttons = pDevice->pbuttonInformation->buttons;
    input = pDevice->pbuttonInformation->input;
    numberOfButtons = pDevice->pbuttonInformation->buttonSize;

    if (unlikely( (buttons==NULL) || (input==NULL) )) {
        dev_err(this->pdev, "ERROR!! buttons or input NULL!!!\n");
        return;
    }

    ucTouchAvgThresh = pDevice->ptouchCheckParameters->duringTouch_AvgThresh;
    ucReleaseAvgThresh = pDevice->ptouchCheckParameters->duringRelease_AvgThresh;

    // If the buttons are added to the array from 0 to max sensor,
    // then we could just skip the for and change this to have..
    // pCurrentButton = &buttons[mainSensor]
    // But since they could be added out of here this is here just in case..
    for (counter = 0; counter < numberOfButtons; counter++) {
        pCurrentButton = &buttons[counter];
        if (pCurrentButton==NULL) {
            dev_err(this->pdev, "ERROR!! current button at index: %d NULL!!!\n", counter);
            return; // ERRORR!!!!
        }

        if ( (CompareButtonMask & pCurrentButton->mask) != pCurrentButton->mask) {
            dev_dbg(this->pdev, "Mask: 0x%02x Looking For: 0x%02x Counter: %d\n",
                                       pCurrentButton->mask, CompareButtonMask, counter);
            continue; // Not the correct one so continue to next
        }

        switch (pCurrentButton->state) {
            case IDLE: /* Button is not being touched! */
                if (capMain > capStartup) {
                    /* User pressed button */
                    dev_info(this->pdev, "[startup]cap button %d touched\n", counter);
                    //input_report_key(input, pCurrentButton->keycode, 1);
                    input_report_abs(input, ABS_DISTANCE, PROX_STATUS_NEAR);
                    input_sync(input);
                    pCurrentButton->state = ACTIVE;

                    /* Set the flag since touch is detected during startup */
                    this->inStartupTouch = true;
                    write_register(this, SX9500_CPS_CTRL4_REG, ucTouchAvgThresh);

                #if 0
                    //---------------------- temp----------------------
                    write_register(this, SX9500_OFFSETMSB_REG, cal_Offset_msByte);
                    write_register(this, SX9500_OFFSETLSB_REG, cal_Offset_lsByte);
                    msleep(100);
                    //---------------------- temp----------------------
                #endif
                }
                else {
                    /* Clear the flag since no touch is detected during startup */
                    this->inStartupTouch = false;
                    write_register(this, SX9500_CPS_CTRL4_REG, ucReleaseAvgThresh);

                    dev_info(this->pdev, "[startup]Button %d already released.\n",counter);
                }
                break;

            case ACTIVE: /* Button is being touched! */ 
                if (capMain <= capStartup) {
                    /* User released button */
                    dev_info(this->pdev, "[startup]cap button %d released\n",counter);
                    //input_report_key(input, pCurrentButton->keycode, 0);
                    input_report_abs(input, ABS_DISTANCE, PROX_STATUS_FAR);
                    input_sync(input);
                    pCurrentButton->state = IDLE;

                    /* Clear the flag since no touch is detected during startup */
                    this->inStartupTouch = false;
                    write_register(this, SX9500_CPS_CTRL4_REG, ucReleaseAvgThresh);
                }
                else {
                    /* Set the flag since touch is detected during startup */
                    this->inStartupTouch = true;
                    write_register(this, SX9500_CPS_CTRL4_REG, ucTouchAvgThresh);

                    dev_info(this->pdev, "[startup]Button %d still touched.\n",counter);
                }
                break;

            default: /* Shouldn't be here, device only allowed ACTIVE or IDLE */
                dev_err(this->pdev, "ERROR! state: %d unknown!\n",pCurrentButton->state);
                break;
        };

        break; // We only want to do this once as we only are checking one sensor

    }
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
        dev_info(this->pdev, "SX9500_TCHCMPSTAT_REG[0x01] = 0x%02x\n", regstat);

        buttons = pDevice->pbuttonInformation->buttons;
        input = pDevice->pbuttonInformation->input;
        numberOfButtons = pDevice->pbuttonInformation->buttonSize;

        if (unlikely( (buttons==NULL) || (input==NULL) )) {
            dev_err(this->pdev, "ERROR!! buttons or input NULL!!!\n");
            return;
        }

        for (counter = 0; counter < numberOfButtons; counter++) {
            pCurrentButton = &buttons[counter];
            if (pCurrentButton == NULL) {
                dev_err(this->pdev, "ERROR!! current button at index: %d NULL!!!\n", counter);
                return; // ERRORR!!!!
            }

            switch (pCurrentButton->state) {
                case IDLE: /* Button is not being touched! */
                    if (((regstat & pCurrentButton->mask) == pCurrentButton->mask)) {
                        /* User pressed button */
                        dev_info(this->pdev, "cap button %d touched\n", counter);
                        /*input_report_key(input, pCurrentButton->keycode, 1);*/
#if defined(CONFIG_MACH_MSM8926_E8LTE)
                        if(buttons[0].state == ACTIVE || buttons[2].state == ACTIVE)
                        {
                           pCurrentButton->state = ACTIVE;
                           dev_info(this->pdev, "active set only\n");
                        }
                        else
                        {
                           pCurrentButton->state = ACTIVE;
                           input_report_abs(input, ABS_DISTANCE, PROX_STATUS_NEAR);
                           input_sync(input);
                           dev_info(this->pdev, "active set and PROX_STATUS_NEAR\n");
                        }
#else
                        input_report_abs(input, ABS_DISTANCE, PROX_STATUS_NEAR);
                        input_sync(input);
                        pCurrentButton->state = ACTIVE;
#endif
                    }
                    else {
                        dev_info(this->pdev, "Button %d already released.\n",counter);
                    }
                    break;
                case ACTIVE: /* Button is being touched! */ 
                    if (((regstat & pCurrentButton->mask) != pCurrentButton->mask)) {
                        /* User released button */
                        dev_info(this->pdev, "cap button %d released\n",counter);
                        /*input_report_key(input, pCurrentButton->keycode, 0);*/
#if defined(CONFIG_MACH_MSM8926_E8LTE)
                        pCurrentButton->state = IDLE;
                        if(buttons[0].state == IDLE && buttons[2].state == IDLE)
                        {
                            input_report_abs(input, ABS_DISTANCE, PROX_STATUS_FAR);
                            input_sync(input);
                            dev_info(this->pdev, "idle set and PROX_STATUS_FAR\n");
                        }
#else
                        input_report_abs(input, ABS_DISTANCE, PROX_STATUS_FAR);
                        input_sync(input);
                        pCurrentButton->state = IDLE;
#endif
                    }
                    else {
                        dev_info(this->pdev, "Button %d still touched.\n",counter);
                    }
                    break;
                default: /* Shouldn't be here, device only allowed ACTIVE or IDLE */
                    break;
            };
        }

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
        {NULL,                            NULL,                         0,              0,          0},
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
                break;
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

//    /* set functions of platform data */
//    pdata->init = sensor_platform_hw_init;
//    pdata->exit = sensor_platform_hw_exit;
//    pdata->power_on = sensor_platform_hw_power_on;

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
    struct _touchCheckParameters *pNearCheckParameters = NULL;

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

    pNearCheckParameters = devm_kzalloc(&client->dev, sizeof(struct _touchCheckParameters), GFP_KERNEL);
    if (!pNearCheckParameters) {
        dev_err(&client->dev, "Failed to allocate memory(_touchCheckParameters)\n");
        err = -ENOMEM;
        goto exit;
    }

    pplatData = devm_kzalloc(&client->dev, sizeof(struct sx9500_platform_data), GFP_KERNEL);
    if (!pplatData) {
        dev_err(&client->dev, "Failed to allocate memory(sx9500_platform_data)\n");
        err = -ENOMEM;
        goto exit;
    }

    pNearCheckParameters = &smtcTouchCheckParameters;

    pplatData->pi2c_reg = sx9500_i2c_reg_setup;
    pplatData->i2c_reg_num = ARRAY_SIZE(sx9500_i2c_reg_setup);
    pplatData->pbuttonInformation = pButtonInformationData;
    pplatData->ptouchCheckParameters = pNearCheckParameters;
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

    dev_dbg(&client->dev, "ready gpio_is_valid \n");
    if (gpio_is_valid(54)) {
        /* configure touchscreen irq gpio */
        dev_dbg(&client->dev, "gpio_is_valid(54) \n");
        error = gpio_request(54, "sx9500_irq_gpio");
        if (error)
            dev_err(&client->dev, "unable to request gpio 54\n");

        error = gpio_direction_input(54);
        if (error)
            dev_err(&client->dev, "unable to set direction for gpio 54\n");

        client->irq = gpio_to_irq(54);
        dev_dbg(&client->dev, "gpio_to_irq(54) = %d\n", gpio_to_irq(54));
    }
    else {
        dev_err(&client->dev, "irq gpio not provided\n");
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
        this->init = initialize_device;
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
            this->statusFunc[4] = Irq_Process_Close_Far; /* COMPENSATION_IRQ(COMP_STAT) */
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
            dev_err(&client->dev, "Failed to allocate memory(sx9500)\n");
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

            /* Initialize the startup parameters */
            pDevice->ptouchCheckParameters = pplatData->ptouchCheckParameters;

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

            wake_lock_init(&this->capsensor_wake_lock, WAKE_LOCK_SUSPEND, "capsensor_wakeup");
        }

        sx86XX_init(this);

        /* default sensor off */
        onoff_sensor(this, DISABLE_IRQ_MASK | DISABLE_SENSOR_PINS);

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
        disable_irq_wake(this->irq);

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
    .probe    = sx9500_probe,
    .remove   = __devexit_p(sx9500_remove),
    .suspend  = sx9500_suspend,
    .resume   = sx9500_resume,
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
