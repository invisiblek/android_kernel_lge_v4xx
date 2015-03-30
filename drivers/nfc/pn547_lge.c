/*
 *
 * Copyright (C) 2010 NXP Semiconductors
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

#include <linux/nfc/pn547_lge.h>
#include <linux/nfc/pn547_lge_hwadapter.h>
#include <linux/of_gpio.h>

#include <linux/wakelock.h>
#define NFC_POWER_OFF   false
#define NFC_POWER_ON    true
#define NFC_PACKET_HEADER   false
#define NFC_PACKET_DATA true

#define MAX_BUFFER_SIZE 512
#define pn547_RESET_CMD 0
#define pn547_DOWNLOAD_CMD  1

#ifdef CONFIG_LGE_NFC_USE_TIMEOUT_WAKELOCK // [NFC-368]
#define NFC_TIMEOUT_MS 2000
static bool sIsWakeLocked = false;
#endif

#ifdef CONFIG_LGE_NFC_SET_IRQ_WAKEUP
static bool sIrqState = false;
#endif

static bool sReadSequence = NFC_PACKET_HEADER;
static bool sPowerState = NFC_POWER_OFF;

static struct i2c_client *pn547_client;

struct wake_lock nfc_wake_lock;

static void pn547_parse_dt(struct device *dev, struct pn547_dev *pn547_dev)
{
    struct device_node *np = dev->of_node;

    /* irq gpio info */
    pn547_dev->ven_gpio = of_get_named_gpio_flags(np, "nxp,gpio_ven", 0, NULL);
    pn547_dev->firm_gpio = of_get_named_gpio_flags(np, "nxp,gpio_mode", 0, NULL);
    pn547_dev->irq_gpio = of_get_named_gpio_flags(np, "nxp,gpio_irq", 0, NULL);
}

static void pn547_disable_irq(struct pn547_dev *pn547_dev)
{
    unsigned long flags;

    spin_lock_irqsave(&pn547_dev->irq_enabled_lock, flags);
    if (pn547_dev->irq_enabled) {
        disable_irq_nosync(pn547_get_irq_pin(pn547_dev));
        disable_irq_wake(pn547_get_irq_pin(pn547_dev));
        pn547_dev->irq_enabled = false;
    }
    spin_unlock_irqrestore(&pn547_dev->irq_enabled_lock, flags);
}

static void pn547_enable_irq(struct pn547_dev *pn547_dev)
{
    unsigned long flags;
    spin_lock_irqsave(&pn547_dev->irq_enabled_lock, flags);
    if (!pn547_dev->irq_enabled) {
        pn547_dev->irq_enabled = true;
        enable_irq(pn547_dev->client->irq);
        enable_irq_wake(pn547_dev->client->irq);
    }
    spin_unlock_irqrestore(&pn547_dev->irq_enabled_lock, flags);
}
static irqreturn_t pn547_dev_irq_handler(int irq, void *dev_id)
{
    struct pn547_dev *pn547_dev = dev_id;
    unsigned long flags;
    unsigned int irq_gpio_val;

    irq_gpio_val = gpio_get_value(pn547_dev->irq_gpio);

    if (irq_gpio_val == 0) {
        pr_err("%s called, IRQ GPIO == %d\n", __func__, irq_gpio_val);
        return 0;
    }

    if (sPowerState == NFC_POWER_ON) {
        spin_lock_irqsave(&pn547_dev->irq_enabled_lock, flags);
        pn547_dev->count_irq++;
        /* Wake up waiting readers */
        wake_up(&pn547_dev->read_wq);
#ifdef CONFIG_LGE_NFC_USE_TIMEOUT_WAKELOCK
        if (sIsWakeLocked == false) {
            wake_lock(&nfc_wake_lock);
            sIsWakeLocked = true;
        }
        else {
            //pr_err("%s already wake locked!\n", __func__); // for debug
        }
#else
        wake_lock(&nfc_wake_lock);
#endif
        spin_unlock_irqrestore(&pn547_dev->irq_enabled_lock, flags);

        //pr_err("%s: wake_lock (%d)\n", __func__, pn547_dev->count_irq); // for debug
    }
    else {
         pr_err("%s, NFC IRQ Triggered during NFC OFF\n", __func__);
    }
    //                                                                                    

    return IRQ_HANDLED;
}

static ssize_t pn547_dev_read(struct file *filp, char __user *buf,
        size_t count, loff_t *offset)
{
    struct pn547_dev *pn547_dev = filp->private_data;
    static char tmp[MAX_BUFFER_SIZE];
    static unsigned char nErrorCnt = 0;
    int ret;
#ifdef CONFIG_LGE_NFC_USE_TIMEOUT_WAKELOCK
    static bool isFinalPacket = true;
#endif
    int irq_gpio_val = 0;
    unsigned long flags;

    if (count > MAX_BUFFER_SIZE)
        count = MAX_BUFFER_SIZE;

    //pr_err("%s : reading %zu bytes.\n", __func__, count); // for debug
    irq_gpio_val = gpio_get_value(pn547_dev->irq_gpio);

#ifdef CONFIG_LGE_NFC_USE_TIMEOUT_WAKELOCK
wait:
#endif
    if (sReadSequence == NFC_PACKET_HEADER) {
#ifdef CONFIG_LGE_NFC_USE_TIMEOUT_WAKELOCK
        if (isFinalPacket == true) {
            ret = wait_event_interruptible_timeout(pn547_dev->read_wq, pn547_dev->count_irq > 0, msecs_to_jiffies(NFC_TIMEOUT_MS));
            if (ret == 0) {
                //pr_err("%s: pass wait_event_interruptible by %dms timeout. restart waiting!\n", __func__, NFC_TIMEOUT_MS); // for debug
                spin_lock_irqsave(&pn547_dev->irq_enabled_lock, flags);
                wake_unlock(&nfc_wake_lock);
                sIsWakeLocked = false;
                spin_unlock_irqrestore(&pn547_dev->irq_enabled_lock, flags);
                //pr_err("%s: wake_unlock\n", __func__); // for debug
                isFinalPacket = false;
                goto wait;
            }
        }
        else {
            ret = wait_event_interruptible(pn547_dev->read_wq, pn547_dev->count_irq > 0);
            isFinalPacket = true;
        }
#else
        ret = wait_event_interruptible(pn547_dev->read_wq, pn547_dev->count_irq > 0);
#endif
        if (ret == -ERESTARTSYS) {
            if (pn547_dev->count_irq <= 0) {
                //pr_err("%s: pass wait_event_interruptible by signal. Skip\n", __func__); // for debug
                return -0xFF;
            }
#ifdef CONFIG_LGE_NFC_USE_TIMEOUT_WAKELOCK
            else {
                //pr_err("%s: pass wait_event_interruptible by signal. restart waiting!\n", __func__); // for debug
                goto wait;
            }
#endif
        }
        else {
            //pr_err("%s: pass wait_event_interruptible by condition (%d)\n", __func__, pn547_dev->count_irq); // for debug
        }
        //pr_err("%s: Read Header\n", __func__); // for debug
    }
    else {
        //pr_err("%s: Read data\n", __func__); // for debug
    }

    /* Read data */
    mutex_lock(&pn547_dev->read_mutex);
    memset(tmp, 0x00, MAX_BUFFER_SIZE);
    ret = i2c_master_recv(pn547_dev->client, tmp, count);
    mutex_unlock(&pn547_dev->read_mutex);

    if (count == 0) {
        pr_err("%s: reading 0 bytes! skip! (%d)\n", __func__, ret);
        goto skip;
    }

    if (ret < 0) {
        pr_err("%s: i2c_master_recv returned %d\n", __func__, ret);
        if (ret == -ENOTCONN) {
            if (nErrorCnt >= 2) {
                pr_err("%s: discount IRQ due to 3 times %d\n", __func__, -ENOTCONN);
                nErrorCnt = 0;
                goto skip;
            }
            else {
                nErrorCnt++;
            }
        }
        return ret;
    }
    if (ret > count) {
        pr_err("%s: received too many bytes from i2c (%d)\n",
            __func__, ret);
        return -EIO;
    }
    if (copy_to_user(buf, tmp, ret)) {
        pr_warning("%s : failed to copy to user space\n", __func__);
        return -EFAULT;
    }

    nErrorCnt = 0;

    //pr_err("%s: i2c_master_recv success (%d)\n", __func__, ret); // for debug

    if (sReadSequence == NFC_PACKET_HEADER) {
        if (ret == count) {
            sReadSequence = NFC_PACKET_DATA;
        }
        else {
            pr_err("%s: received lesser bytes(%d) than desired(%d).\n", __func__, ret, count);
        }
    }
    else {
skip:
        sReadSequence = NFC_PACKET_HEADER;
        spin_lock_irqsave(&pn547_dev->irq_enabled_lock, flags);
        if (pn547_dev->count_irq > 0) {
            pn547_dev->count_irq--;
            //pr_err("%s: current count_irq = %d\n", __func__, pn547_dev->count_irq); // for debug
        }
        if (pn547_dev->count_irq == 0) {
#ifdef CONFIG_LGE_NFC_USE_TIMEOUT_WAKELOCK

#else
            wake_unlock(&nfc_wake_lock);
            //pr_err("%s: wake_unlock\n", __func__); // for debug
#endif
        }
        spin_unlock_irqrestore(&pn547_dev->irq_enabled_lock, flags);
    }
    return ret;
}

static ssize_t pn547_dev_write(struct file *filp, const char __user *buf,
        size_t count, loff_t *offset)
{
    struct pn547_dev  *pn547_dev;
    static char tmp[MAX_BUFFER_SIZE];
    int ret;

    pn547_dev = filp->private_data;

    if (count > MAX_BUFFER_SIZE)
        count = MAX_BUFFER_SIZE;

    memset(tmp, 0x00, MAX_BUFFER_SIZE);
    if (copy_from_user(tmp, buf, count)) {
        pr_err(PN547_DRV_NAME ":%s : failed to copy from user space\n", __func__);
        return -EFAULT;
    }

    pr_debug("%s : writing %zu bytes.\n", __func__, count);
    /* Write data */
    dprintk(PN547_DRV_NAME ":write: pn547_write len=:%d\n", count);

    mutex_lock(&pn547_dev->read_mutex);
    ret = i2c_master_send(pn547_dev->client, tmp, count);
    if (ret != count) {
        pr_err("%s : i2c_master_send returned %d\n", __func__, ret);
        ret = -EIO;
    }
    mutex_unlock(&pn547_dev->read_mutex);

    return ret;
}

static int pn547_dev_open(struct inode *inode, struct file *filp)
{
    struct pn547_dev *pn547_dev = i2c_get_clientdata(pn547_client);
    filp->private_data = pn547_dev;
    pn547_dev->count_irq = 0;
    pn547_enable_irq(pn547_dev);
    pr_debug("%s : %d,%d\n", __func__, imajor(inode), iminor(inode));

    return 0;
}

static long pn547_dev_unlocked_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct pn547_dev *pn547_dev = filp->private_data;
    unsigned long flags;

    switch (cmd) {
    case pn547_SET_PWR:
        if (arg == 2) {
            /*
            power on with firmware download (requires hw reset)
            */
            dprintk(PN547_DRV_NAME ":%s power on with firmware\n", __func__);

            gpio_set_value(pn547_dev->ven_gpio, 1);
            gpio_set_value(pn547_dev->firm_gpio, 1);
            msleep(10);
            gpio_set_value(pn547_dev->ven_gpio, 0);
            msleep(10);
            gpio_set_value(pn547_dev->ven_gpio, 1);
            msleep(10);
        } else if (arg == 1) {
            /* power on */
            dprintk(PN547_DRV_NAME ":%s power on\n", __func__);
            if (sPowerState == NFC_POWER_OFF) {
                gpio_set_value(pn547_dev->firm_gpio, 0);
                gpio_set_value(pn547_dev->ven_gpio, 1);
                msleep(10);

                spin_lock_irqsave(&pn547_dev->irq_enabled_lock, flags);
#ifdef CONFIG_LGE_NFC_SET_IRQ_WAKEUP
                if (sIrqState == false) {
                    irq_set_irq_wake(pn547_dev->client->irq,1);
                    sIrqState = true;
                    dprintk(PN547_DRV_NAME ":%s enable IRQ\n", __func__);
                }
                else {
                    pr_err("%s IRQ is already enabled!\n", __func__);
                }
#endif
                //pr_err("%s NFC_POWER_ON\n", __func__); // for debug
                sPowerState = NFC_POWER_ON;
                sReadSequence = NFC_PACKET_HEADER;
                spin_unlock_irqrestore(&pn547_dev->irq_enabled_lock, flags);
            }
            else {
                pr_err("%s NFC is alread On!\n", __func__);
            }
        } else  if (arg == 0) {
            /* power off */
            dprintk(PN547_DRV_NAME ":%s power off\n", __func__);
            if (sPowerState == NFC_POWER_ON) {
                gpio_set_value(pn547_dev->firm_gpio, 0);
                gpio_set_value(pn547_dev->ven_gpio, 0);
                msleep(10);

                spin_lock_irqsave(&pn547_dev->irq_enabled_lock, flags);
#ifdef CONFIG_LGE_NFC_SET_IRQ_WAKEUP
                if (sIrqState == true) {
                    irq_set_irq_wake(pn547_dev->client->irq,0);
                    sIrqState = false;
                    dprintk(PN547_DRV_NAME ":%s disable IRQ\n", __func__);
                }
                else {
                    pr_err("%s IRQ is already disabled!\n", __func__);
                }
#endif
                if (pn547_dev->count_irq > 0) {
                    pr_err("%s: Clear IRQ(%d)\n", __func__, pn547_dev->count_irq);
                    pn547_dev->count_irq = 0;
                    wake_unlock(&nfc_wake_lock);
                }

#ifdef CONFIG_LGE_NFC_USE_TIMEOUT_WAKELOCK
                if (sIsWakeLocked == true) {
                    pr_err("%s: Wake Unlock\n", __func__);
                    wake_unlock(&nfc_wake_lock);
                    sIsWakeLocked = false;
                }
#endif

                //pr_err("%s NFC_POWER_OFF\n", __func__); // for debug
                sPowerState = NFC_POWER_OFF;
                sReadSequence = NFC_PACKET_HEADER;
                spin_unlock_irqrestore(&pn547_dev->irq_enabled_lock, flags);
            }
            else {
                pr_err("%s NFC is alread Off!\n", __func__);
            }
        } else {
                pr_err("%s bad arg %ld\n", __func__, arg);
            return -EINVAL;
        }
        break;
    case pn547_HW_REVISION:
        {
            return pn547_get_hw_revision();
        }
    default:
        pr_err("%s bad ioctl %d\n", __func__, cmd);
        return -EINVAL;
    }

    return 0;
}

static const struct file_operations pn547_dev_fops = {
    .owner  = THIS_MODULE,
    .llseek = no_llseek,
    .read   = pn547_dev_read,
    .write  = pn547_dev_write,
    .open   = pn547_dev_open,
    .unlocked_ioctl = pn547_dev_unlocked_ioctl,
};

static int pn547_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
    int ret;
    struct pn547_dev *pn547_dev = NULL;
    pn547_client = client;

    dprintk(PN547_DRV_NAME ": pn547_probe() start\n");

    pn547_dev = kzalloc(sizeof(*pn547_dev), GFP_KERNEL);
    if (pn547_dev == NULL) {
        dev_err(&client->dev,
                "failed to allocate memory for module data\n");
        ret = -ENOMEM;
        goto err_exit;
    }

    pn547_parse_dt(&client->dev, pn547_dev);

    pn547_dev->client   = client;
    dprintk(PN547_DRV_NAME ":IRQ : %d\nVEN : %d\nFIRM : %d\n",
            pn547_dev->irq_gpio, pn547_dev->ven_gpio, pn547_dev->firm_gpio);

    ret = gpio_request(pn547_dev->irq_gpio, "nfc_int");
    if (ret) {
        dprintk(PN547_DRV_NAME ":pn547_probe() : nfc_int request failed!\n");
        goto err_int;
    }
    ret = gpio_request(pn547_dev->ven_gpio, "nfc_ven");
    if (ret) {
        dprintk(PN547_DRV_NAME ":pn547_probe() : nfc_ven request failed!\n");
        goto err_ven;
    }
    ret = gpio_request(pn547_dev->firm_gpio, "nfc_firm");
    if (ret) {
        dprintk(PN547_DRV_NAME ":pn547_probe() : nfc_firm request failed!\n");
        goto err_firm;
    }

    pn547_gpio_enable(pn547_dev);

    ret = gpio_direction_output(pn547_dev->ven_gpio,1);
    ret = gpio_direction_output(pn547_dev->firm_gpio,0);
    ret = gpio_direction_input(pn547_dev->irq_gpio);

    /* init mutex and queues */
    init_waitqueue_head(&pn547_dev->read_wq);
    mutex_init(&pn547_dev->read_mutex);
    spin_lock_init(&pn547_dev->irq_enabled_lock);

    pn547_dev->pn547_device.minor = MISC_DYNAMIC_MINOR;
    pn547_dev->pn547_device.name = PN547_DRV_NAME;
    pn547_dev->pn547_device.fops = &pn547_dev_fops;

    ret = misc_register(&pn547_dev->pn547_device);
    if (ret) {
        pr_err("%s : misc_register failed\n", __FILE__);
        goto err_misc_register;
    }

    wake_lock_init(&nfc_wake_lock, WAKE_LOCK_SUSPEND, "NFCWAKE");

    /* request irq.  the irq is set whenever the chip has data available
     * for reading.  it is cleared when all data has been read.
     */
    pr_info("%s : requesting IRQ %d\n", __func__, client->irq);
    pn547_dev->irq_enabled = true;
    ret = request_irq(pn547_gpio_to_irq(pn547_dev), pn547_dev_irq_handler,
              IRQF_TRIGGER_RISING|IRQF_NO_SUSPEND, client->name, pn547_dev);
    if (ret) {
        dev_err(&client->dev, "request_irq failed\n");
        goto err_request_irq_failed;
    }
    enable_irq_wake(pn547_get_irq_pin(pn547_dev));
    pn547_disable_irq(pn547_dev);
    i2c_set_clientdata(client, pn547_dev);
    dprintk(PN547_DRV_NAME ": pn547_probe() end\n");
    return 0;

err_request_irq_failed:
    misc_deregister(&pn547_dev->pn547_device);

err_misc_register:
    mutex_destroy(&pn547_dev->read_mutex);
    gpio_free(pn547_dev->firm_gpio);

err_firm:
    gpio_free(pn547_dev->ven_gpio);

err_ven:
    gpio_free(pn547_dev->irq_gpio);

err_int:
    kfree(pn547_dev);

err_exit:
    pr_err(PN547_DRV_NAME ": pn547_dev is null\n");
    pr_err(PN547_DRV_NAME ": pn547_probe() end with error!\n");

    return ret;
}

static __devexit int pn547_remove(struct i2c_client *client)
{
    struct pn547_dev *pn547_dev;

    pn547_dev = i2c_get_clientdata(client);
    free_irq(pn547_gpio_to_irq(pn547_dev), pn547_dev);
    misc_deregister(&pn547_dev->pn547_device);
    mutex_destroy(&pn547_dev->read_mutex);
    gpio_free(pn547_dev->firm_gpio);
    gpio_free(pn547_dev->ven_gpio);
    gpio_free(pn547_dev->irq_gpio);
    kfree(pn547_dev);

    return 0;
}

static void pn547_shutdown(struct i2c_client *client)
{
    struct pn547_dev *pn547_dev;
    // Get PN547 Device Structure data
    pn547_dev = i2c_get_clientdata(client);

    pn547_shutdown_cb(pn547_dev);
    return;
}

static const struct i2c_device_id pn547_id[] = {
    { PN547_DRV_NAME, 0 },
    { }
};

static struct of_device_id pn547_match_table[] = {
    { .compatible = "nxp,pn547",},
    { },
};

static struct i2c_driver pn547_driver = {
    .driver = {
        .owner = THIS_MODULE,
        .name = PN547_DRV_NAME,
        .of_match_table = pn547_match_table,
    },
    .probe = pn547_probe,
    .remove = __devexit_p(pn547_remove),
    .shutdown   = pn547_shutdown,
    .id_table = pn547_id,
};

/*
 * module load/unload record keeping
 */

static int __init pn547_dev_init(void)
{
    int ret = 0;

    pr_info("Loading PN547 driver\n");

    ret = i2c_add_driver(&pn547_driver);
    if (ret < 0) {
        printk("[NFC]failed to i2c_add_driver\n");
    }
    pr_info("Loading PN547 driver Success! \n");
    return ret;

}
module_init(pn547_dev_init);

static void __exit pn547_dev_exit(void)
{
    pr_info("Unloading PN547 driver\n");
    i2c_del_driver(&pn547_driver);
}
module_exit(pn547_dev_exit);

MODULE_DEVICE_TABLE(i2c, pn547_id);
MODULE_AUTHOR("Sylvain Fonteneau");
MODULE_DESCRIPTION("NFC PN547 driver");
MODULE_LICENSE("GPL");
