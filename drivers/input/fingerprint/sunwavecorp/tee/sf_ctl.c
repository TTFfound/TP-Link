/**
 * The device control driver for Sunwave's fingerprint sensor.
 *
 * Copyright (C) 2016 Sunwave Corporation. <http://www.sunwavecorp.com>
 * Copyright (C) 2016 Langson L. <mailto: liangzh@sunwavecorp.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 * Public License for more details.
**/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/input.h>
#include <linux/uaccess.h>

#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/pinctrl/consumer.h>
#include <linux/spi/spi.h>
#include <linux/clk.h>

//#include "nt_smc_call.h"

#include "mt_spi.h"
#include "mt_spi_hal.h"
#include "sf_ctl.h"

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#else
#include <linux/fb.h>
#include <linux/notifier.h>
#endif

#define MODULE_NAME "sf_ctl"
#define xprintk(level, fmt, args...) printk(level MODULE_NAME": "fmt, ##args)

#if ANDROID_WAKELOCK
#include <linux/wakelock.h>
#endif
#ifndef CONFIG_OF
# error "error: this driver 'MODULE_NAME' only support dts."
#endif

#if SF_BEANPOD_COMPATIBLE
#include "nt_smc_call.h" // add lanh 2016-12-14
#endif

struct spi_device *sf_spi = NULL;
struct mt_spi_t *mt_spi = NULL;

static int sf_ctl_init_irq(void);
static int sf_ctl_init_input(void);
/**
 * Define the driver version string.
 * There is NO need to modify 'rXXXX_yyyymmdd', it should be updated automatically
 * by the building script (see the 'Driver-revision' section in 'build.sh').
 */
#define SF_DRV_VERSION "v1.2.1-20170707"

struct sf_ctl_device {
    struct miscdevice miscdev;
    int irq_num;
    struct work_struct work_queue;
    struct input_dev *input;
#if ANDROID_WAKELOCK
    struct wake_lock wakelock;
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
    struct early_suspend early_suspend;
#else
    struct notifier_block notifier;
#endif
};

typedef enum {
    SF_PIN_STATE_PWR_ON,
    SF_PIN_STATE_PWR_OFF,
    SF_PIN_STATE_RST_LOW,
    SF_PIN_STATE_RST_HIGH,
    SF_PIN_STATE_INT_SET,

    /* Array size */
    SF_PIN_STATE_MAX
} sf_pin_state_t;

static const char *sf_pinctrl_state_names[SF_PIN_STATE_MAX] = {
    "power_on", "power_off", "finger_rst_clr", "finger_rst_set", "eint",      // [liguanxiong] pinctl_name compatible wih REE
};

static struct pinctrl *sf_pinctrl = NULL;
static struct pinctrl_state *sf_pin_states[SF_PIN_STATE_MAX] = {NULL, };

/*[liguanxiong start] improve finger lightup speed */
static bool allow_speed_up = false;
#if SUNWAVE_FAST_LIGHTUP
extern void primary_display_resume(void);
struct workqueue_struct *sunwave_fingerprint_wq;
struct work_struct sunwave_lightup_screen_work;
time_t last_fast_lightup_time = 0;
atomic_t worker_running;
static void sunwave_light_screen_work_func(struct work_struct *work)
{
    time_t now = current_kernel_time().tv_sec;

    printk("%s: start fast lightup!\n", __func__);

    if(now - last_fast_lightup_time < 6){
        printk("fast_lightup_lcd_resume skip!!!!!, now: %ld, last_fast_lightup_time: %ld\n", now, last_fast_lightup_time);
        return;
    }

    atomic_set(&worker_running, 1);
    primary_display_resume();
    last_fast_lightup_time = current_kernel_time().tv_sec;
    atomic_set(&worker_running, 0);
}
static int sunwave_workqueue_init(void)
{
    sunwave_fingerprint_wq = create_singlethread_workqueue("sunwave_fingerprint_wq");
    if (sunwave_fingerprint_wq)
    {
        printk("create_singlethread_workqueue success\n");
        INIT_WORK(&sunwave_lightup_screen_work, sunwave_light_screen_work_func);
        atomic_set(&worker_running, 0);
    }
    else
    {
        goto err_workqueue_init;
    }
    return 0;

err_workqueue_init:
    printk("create_singlethread_workqueue failed\n");

    return -1;
}
#endif
/* [liguanxiong end] */

static int sf_ctl_device_power(bool on) // lanh 2016-11-21
{
    int err = 0;
    sf_pin_state_t state = on ? SF_PIN_STATE_PWR_ON : SF_PIN_STATE_PWR_OFF;
    xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);
    err = pinctrl_select_state(sf_pinctrl, sf_pin_states[state]);
    msleep(20);
    return err;
}

//extern void spi_enable_clk(void);
//extern void spi_disable_clk(void);
static int sf_spi_clock_enable(bool on)
{
    int err = 0;

    if (on) {
       // mt_spi_enable_clk(mt_spi);
	mt_spi_enable_master_clk(sf_spi);  // [liguanxiong] use new api
        //spi_enable_clk();
    }
    else {
        //mt_spi_disable_clk(mt_spi);
	mt_spi_disable_master_clk(sf_spi);  // [liguanxiong] use new api
        //spi_disable_clk();
    }

    return err;
}

static int sf_ctl_device_reset(void)
{
    int err = 0;
    xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);
    err = pinctrl_select_state(sf_pinctrl, sf_pin_states[SF_PIN_STATE_RST_HIGH]);
    msleep(1);
    err = pinctrl_select_state(sf_pinctrl, sf_pin_states[SF_PIN_STATE_RST_LOW]);
    msleep(20);
    err = pinctrl_select_state(sf_pinctrl, sf_pin_states[SF_PIN_STATE_RST_HIGH]);
    return err;
}

static void sf_ctl_device_event(struct work_struct *ws)
{
    struct sf_ctl_device *sf_ctl_dev =
        container_of(ws, struct sf_ctl_device, work_queue);
    char *uevent_env[2] = { "SPI_STATE=finger", NULL };
    xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);
    kobject_uevent_env(&sf_ctl_dev->miscdev.this_device->kobj,
                       KOBJ_CHANGE, uevent_env);
}

static irqreturn_t sf_ctl_device_irq(int irq, void *dev_id)
{
    struct sf_ctl_device *sf_ctl_dev = (struct sf_ctl_device *)dev_id;
    disable_irq_nosync(irq);
    xprintk(KERN_DEBUG, "%s(irq = %d, ..) toggled.\n", __FUNCTION__, irq);
    schedule_work(&sf_ctl_dev->work_queue);
#if ANDROID_WAKELOCK
    wake_lock_timeout(&sf_ctl_dev->wakelock, msecs_to_jiffies(5000));
#endif
    /* [liguanxiong start] Add for fingerprint unlock acceleration */
    if (allow_speed_up) {
        input_report_key(sf_ctl_dev->input, KEY_FP_SPEED_UP, 1);
        input_sync(sf_ctl_dev->input);
        input_report_key(sf_ctl_dev->input, KEY_FP_SPEED_UP, 0);
        input_sync(sf_ctl_dev->input);
        allow_speed_up = false;
    }
#if SUNWAVE_FAST_LIGHTUP
    if(sunwave_fingerprint_wq && atomic_read(&worker_running) != 1){
        queue_work (sunwave_fingerprint_wq, &sunwave_lightup_screen_work);
    }
#endif
    /* [liguanxiong end] */
    enable_irq(irq);
    return IRQ_HANDLED;
}

static int sf_ctl_report_key_event(struct input_dev *input, sf_key_event_t *kevent)
{
    int err = 0;
    unsigned int key_code = KEY_UNKNOWN;
    xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);

    switch (kevent->key) {
        case SF_KEY_HOME:
            key_code = KEY_HOME;
            break;

        case SF_KEY_MENU:
            key_code = KEY_MENU;
            break;

        case SF_KEY_BACK:
            key_code = KEY_BACK;
            break;

        case SF_KEY_F11:
            key_code = KEY_F11;
            break;

        case SF_KEY_ENTER:
            key_code = KEY_ENTER;
            break;

        case SF_KEY_CLICK:
            key_code = KEY_FP_CLICK;
            break;

        case SF_KEY_LONG_CLICK:
            key_code = KEY_FP_LONG_CLICK;
            break;

        case SF_KEY_DOUBLE_CLICK:
            key_code = KEY_FP_DOUBLE_CLICK;
            break;

        case SF_KEY_UP:
            key_code = KEY_FP_UP;
            break;

        case SF_KEY_LEFT:
            key_code = KEY_FP_LEFT;
            break;

        case SF_KEY_RIGHT:
            key_code = KEY_FP_RIGHT;
            break;

        case SF_KEY_DOWN:
            key_code = KEY_FP_DOWN;
            break;

        case SF_KEY_WAKEUP:
            key_code = KEY_WAKEUP;
            break;
        /* [liguanxiong start] for camera take picture */
        case SF_KEY_CAMERA:
            key_code = KEY_FP_CLICK;
            break;
        /* [liguanxiong end] */
        default:
            break;
    }

    /* [liguanxiong start] handle touch event */
    if (kevent->key == SF_KEY_NAV_TOUCH) {
        if (kevent->value == 1) {
            input_report_key(input, KEY_FP_TOUCH, 1);
            input_sync(input);
            xprintk(KERN_DEBUG, "%s: keycode = KEY_FP_TOUCH, key_value = 1.\n", __FUNCTION__);
        }
        return err;
    } else if (kevent->key == SF_KEY_NAV_RELEASE) {
        if (kevent->value == 1) {
            input_report_key(input, KEY_FP_TOUCH, 0);
            input_sync(input);
            xprintk(KERN_DEBUG, "%s: keycode = KEY_FP_TOUCH, key_value = 0.\n", __FUNCTION__);
        }
        return err;
    }
    /* [liguanxiong end] */
    xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);
     input_report_key(input, key_code, kevent->value);
    xprintk(KERN_DEBUG, "%s: keycode = %d, key_value =%d.\n", __FUNCTION__, kevent->key, kevent->value);
    input_sync(input);
    xprintk(KERN_DEBUG, "%s(..) leave.\n", __FUNCTION__);
    return err;
}

static const char *sf_ctl_get_version(void)
{
    static char version[SF_DRV_VERSION_LEN] = {'\0', };
    strncpy(version, SF_DRV_VERSION, SF_DRV_VERSION_LEN);
    version[SF_DRV_VERSION_LEN - 1] = '\0';
    return (const char *)version;
}

////////////////////////////////////////////////////////////////////////////////
// struct file_operations fields.




static long sf_ctl_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct miscdevice *dev = (struct miscdevice *)filp->private_data;
    struct sf_ctl_device *sf_ctl_dev =
        container_of(dev, struct sf_ctl_device, miscdev);
    int err = 0;
    sf_key_event_t kevent;
    xprintk(KERN_DEBUG, "%s(cmd = 0x%08x, ..)\n", __FUNCTION__, cmd);

    switch (cmd) {
        case SF_IOC_INIT_DRIVER: {
            // TODO:
            break;
        }

        case SF_IOC_DEINIT_DRIVER: {
            // TODO:
            break;
        }

        case SF_IOC_RESET_DEVICE: {
            sf_ctl_device_reset();
            break;
        }

        case SF_IOC_ENABLE_IRQ: {
            // TODO:
            break;
        }

        case SF_IOC_DISABLE_IRQ: {
            // TODO:
            break;
        }

        case SF_IOC_ENABLE_SPI_CLK: {
            // TODO:
            sf_spi_clock_enable(true);
            break;
        }

        case SF_IOC_DISABLE_SPI_CLK: {
            // TODO:
            sf_spi_clock_enable(false);
            break;
        }

        case SF_IOC_ENABLE_POWER: {
            // TODO:
            break;
        }

        case SF_IOC_DISABLE_POWER: {
            // TODO:
            break;
        }

        case SF_IOC_REPORT_KEY_EVENT: {
            if (copy_from_user(&kevent, (sf_key_event_t *)arg, sizeof(sf_key_event_t))) {
                xprintk(KERN_ERR, "copy_from_user(..) failed.\n");
                err = (-EFAULT);
                break;
            }

            err = sf_ctl_report_key_event(sf_ctl_dev->input, &kevent);
            break;
        }

        case SF_IOC_SYNC_CONFIG: {
            // TODO:
            break;
        }

        case SF_IOC_GET_VERSION: {
            if (copy_to_user((void *)arg, sf_ctl_get_version(), SF_DRV_VERSION_LEN)) {
                xprintk(KERN_ERR, "copy_to_user(..) failed.\n");
                err = (-EFAULT);
                break;
            }

            break;
        }

        default:
            err = (-EINVAL);
            break;
    }

    return err;
}

#ifdef CONFIG_COMPAT
static long sf_ctl_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    return sf_ctl_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#endif // CONFIG_COMPAT

static int sf_ctl_open(struct inode *inode, struct file *filp)
{
    xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);
    return 0;
}

static int sf_ctl_release(struct inode *inode, struct file *filp)
{
    xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);
    return 0;
}

#if SF_TRUSTKERNEL_COMPATIBLE
static int sf_read_sensor_id(void)
{
    int ret = -1;
    int trytimes = 3;
    char outid_8221[6] = {0};
    char readid_8221[4] = {0};
    char outid_8202[7] = {0};
    char readid_8202[5] = {0};
    char outid_swid[6] = {0};
    char readid_swid[2] = {0};
    //默认速度设置为1M, 不然8201/8211系列有可能读不到ID
    static struct mt_chip_conf smt_conf = {
        .setuptime = 15,
        .holdtime = 15,
        .high_time = 60, // 10--6m 15--4m 20--3m 30--2m [ 60--1m 120--0.5m  300--0.2m]
        .low_time  = 60,
        .cs_idletime = 20,
        .ulthgh_thrsh = 0,
        .cpol = 0,
        .cpha = 0,
        .rx_mlsb = SPI_MSB,
        .tx_mlsb = SPI_MSB,
        .tx_endian = 0,
        .rx_endian = 0,
        .com_mod = FIFO_TRANSFER,
        .pause = 0,
        .finish_intr = 1,
        .deassert = 0,
        .ulthigh = 0,
        .tckdly = 0,
    };
    msleep(10);

    do {
        /*******read 8221 begin*********/
        memset(readid_8221, 0, sizeof(readid_8221));
        readid_8221[0] = 0x60;
        readid_8221[1] = 0x28;
        readid_8221[2] = 0x02;
        readid_8221[3] = 0x00;
        ret =  tee_spi_transfer(&smt_conf, sizeof(smt_conf), readid_8221, outid_8221, 6);

        if (ret != 0) {
            printk("TEE SPI transfer failed\n");
            continue;
        }

        if (outid_8221[4] == 0x82) {
            printk("read chip is ok\n");
            return 0;
        }

        /*******8221 end**********/
        /*******read 8202 begin*********/
        memset(readid_8202, 0, sizeof(readid_8202));
        readid_8202[0] = 0x60;
        readid_8202[1] = (uint8_t)(~0x60);
        readid_8202[2] = 0x28;
        readid_8202[3] = 0x02;
        readid_8202[4] = 0x00;
        ret =  tee_spi_transfer(&smt_conf, sizeof(smt_conf), readid_8202, outid_8202, 7);

        if (ret != 0) {
            printk("TEE SPI transfer failed\n");
            continue;
        }

        if (outid_8202[5] == 0x82) {
            printk("read chip is ok\n");
            return 0;
        }

        /*******read 8202-8205-8231 end**********/
        /*******read 8205-8231 begin*********/
        memset(readid_swid, 0, sizeof(readid_swid));
        outid_swid[0] = 0xA0;
        outid_swid[1] = (uint8_t)(~0xA0);
        ret =  tee_spi_transfer(&smt_conf, sizeof(smt_conf), readid_swid, outid_swid, 6);

        if (ret != 0) {
            printk("TEE SPI transfer failed\n");
            continue;
        }

        if ((0x53 == outid_swid[2]) && (0x75 == outid_swid[3]) && (0x6e == outid_swid[4])
            && (0x57 == outid_swid[5])) {
            printk("read chip is ok\n");
            return 0;
        }

        /*******read 8205-8231 end**********/
#if 0
        /*******read 8201-8211 begin*********/
        char outid_8201[10] = {0};
        char readid_8201[8] = {0};
        char writeWakeUp_8201[3] = {0};
        /* reset脚拉高后，需等 200ms 后方可读ID */
        msleep(200);
        memset(readid_8201, 0, sizeof(readid_8201));
        memset(writeWakeUp_8201, 0, sizeof(writeWakeUp_8201));
        writeWakeUp_8201[0] = 0x1c;
        writeWakeUp_8201[1] = 0x1c;
        writeWakeUp_8201[2] = 0x1c;
        ret =  tee_spi_transfer(&smt_conf, sizeof(smt_conf), writeWakeUp_8201, outid_8201, 3);

        if (ret != 0) {
            printk("TEE SPI transfer failed\n");
            continue;
        }

        msleep(5);
        readid_8201[0] = 0x96;
        readid_8201[1] = 0x69;
        readid_8201[2] = 0x00;
        readid_8201[3] = 0x00;
        readid_8201[4] = 0x1e;
        readid_8201[5] = 0x00;
        readid_8201[6] = 0x02;
        readid_8201[7] = 0x00;
        ret =  tee_spi_transfer(&smt_conf, sizeof(smt_conf), readid_8201, outid_8201, 10);

        if (ret != 0) {
            printk("TEE SPI transfer failed\n");
            continue;
        }

        if ((outid_8201[8] == 0xfa) || (outid_8201[9] == 0xfa)) {
            printk("read chip is ok\n");
            return 0;
        }

        /*******read 8202 end**********/
#endif
    }
    while (trytimes--);

    return -1;
}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void sunwave_early_suspend(struct early_suspend *handler)
{
    char *screen[2] = { "SCREEN_STATUS=OFF", NULL };
    //sunwave_sensor_t* sunwave = container_of(handler, sunwave_sensor_t, early_suspend);
    struct sf_ctl_device *sf_ctl_dev = container_of(handler, struct sf_ctl_device, miscdev);
    // sw_info("%s enter.\n", __func__);
    kobject_uevent_env(&sf_ctl_dev->miscdev.this_device->kobj, KOBJ_CHANGE, screen);
    //kobject_uevent_env(&sunwave->spi->dev.kobj, KOBJ_CHANGE, screen);
    // sw_info("%s leave.\n", __func__);
}
static void sunwave_late_resume(struct early_suspend *handler)
{
    char *screen[2] = { "SCREEN_STATUS=ON", NULL };
    //   sunwave_sensor_t* sunwave = container_of(handler, sunwave_sensor_t, early_suspend);
    struct sf_ctl_device *sf_ctl_dev = container_of(handler, struct sf_ctl_device, miscdev);
    //  sw_info("%s enter.\n", __func__);
    kobject_uevent_env(&sf_ctl_dev->miscdev.this_device->kobj, KOBJ_CHANGE, screen);
    // kobject_uevent_env(&sunwave->spi->dev.kobj, KOBJ_CHANGE, screen);
    //   sw_info("%s leave.\n", __func__);
}

#else //SF_CFG_HAS_EARLYSUSPEND

static int sunwave_fb_notifier_callback(struct notifier_block *self,
                                        unsigned long event, void *data)
{
    static char screen_status[64] = {'\0'};
    char *screen_env[2] = { screen_status, NULL };
    struct sf_ctl_device *sf_ctl_dev;
    // sunwave_sensor_t* sunwave;
    struct fb_event *evdata = data;
    unsigned int blank;
    int retval = 0;
    //sw_info("%s enter.\n", __func__);

    if (event != FB_EVENT_BLANK /* FB_EARLY_EVENT_BLANK */) {
        return 0;
    }

    //  sunwave = container_of(self, sunwave_sensor_t, notifier);
    sf_ctl_dev = container_of(self, struct sf_ctl_device, notifier);
    blank = *(int *)evdata->data;
    //sw_info("%s enter, blank=0x%x\n", __func__, blank);

    switch (blank) {
        case FB_BLANK_UNBLANK:
            //sw_info("%s: lcd on notify\n", __func__);
            sprintf(screen_status, "SCREEN_STATUS=%s", "ON");
            //kobject_uevent_env(&sunwave->spi->dev.kobj, KOBJ_CHANGE, screen_env);
            kobject_uevent_env(&sf_ctl_dev->miscdev.this_device->kobj, KOBJ_CHANGE, screen_env);
            break;

        case FB_BLANK_POWERDOWN:
            //sw_info("%s: lcd off notify\n", __func__);
            sprintf(screen_status, "SCREEN_STATUS=%s", "OFF");
            //kobject_uevent_env(&sunwave->spi->dev.kobj, KOBJ_CHANGE, screen_env);
            kobject_uevent_env(&sf_ctl_dev->miscdev.this_device->kobj, KOBJ_CHANGE, screen_env);
            break;

        default:
            //sw_info("%s: other notifier, ignore\n", __func__);
            break;
    }

    //sw_info("%s %s leave.\n", screen_status, __func__);
    return retval;
}
#endif //SF_CFG_HAS_EARLYSUSPEND
////////////////////////////////////////////////////////////////////////////////

static struct file_operations sf_ctl_fops = {
    .owner          = THIS_MODULE,
    .unlocked_ioctl = sf_ctl_ioctl,
    .open           = sf_ctl_open,
    .release        = sf_ctl_release,
#ifdef CONFIG_COMPAT
    .compat_ioctl   = sf_ctl_compat_ioctl,
#endif // CONFIG_COMPAT
};

static struct sf_ctl_device sf_ctl_dev = {
    .miscdev = {
        .minor  = MISC_DYNAMIC_MINOR,
        .name   = "sunwave_fp",
        .fops   = &sf_ctl_fops,
    }, 0,
};

static struct of_device_id  sf_of_match[] = {
    { .compatible = "mediatek,fingerprint", },
};

// see sf_spi.c
//extern int  sf_spi_platform_init(void);
//extern void sf_spi_platform_exit(void);
/* [liguanxiong start] */
#define SUNWAVE_LIB_VERSION    "20180410"
#define SUNWAVE_DRIVER_VERSION "v1.0<20170817>"
#define SUNWAVE_CHIP_TYPE      "SW9651/SW9088"
#define SUNWAVE_FP_VENDOR      "Sunwave"

static ssize_t sunwave_fp_info_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    int ret;

    ret = snprintf(buf, PAGE_SIZE,
        "type:\t%s\n"
        "vendor:\t%s\n"
        "firmware_version:\t%s\n"
        "driver_version:\t%s\n",
        SUNWAVE_CHIP_TYPE,
        SUNWAVE_FP_VENDOR,
        SUNWAVE_LIB_VERSION,
        SUNWAVE_DRIVER_VERSION);

    return ret;
}

static ssize_t sunwave_fp_info_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{

    if (!strcmp("CMD_SPEED_UP_ON", buf)) {
        allow_speed_up = true;
    } else {
        allow_speed_up = false;
    }

    if (!strcmp("CMD_MODULE_READY", buf)) {
        sf_ctl_device_power(true);
    } else if (!strcmp("CMD_MODULE_PREPARE", buf)) {
		sf_ctl_device_power(false);
    }
    return count;
}
static DEVICE_ATTR(fp_info, S_IRUGO | S_IWUSR | S_IWGRP, sunwave_fp_info_show, sunwave_fp_info_store);
/* [liguanxiong end] */


static int sf_probe(struct spi_device *spi)
{
    int err = 0;
    sf_spi = spi;
    xprintk(KERN_ERR, "sunwave %s enter\n", __func__);
    mt_spi = spi_master_get_devdata(sf_spi->master);

    if (!mt_spi) {
        xprintk(KERN_ERR, "fail to get mediatek spi device.\n");
        dump_stack();
        return (-ENODEV);
    }

#if SF_TRUSTKERNEL_COMPATIBLE
    sf_spi_clock_enable(true);

    if (sf_read_sensor_id() < 0) {
        printk("sunwave probe read chip id is failed\n");
        sf_spi_clock_enable(false);//chenlw add
        return -1;
    }

    sf_spi_clock_enable(false);
#endif
    /* Initialize the interrupt callback. */
    err = sf_ctl_init_irq();

    if (err) {
        xprintk(KERN_ERR, "sf_ctl_init_irq failed with %d.\n", err);
        return err;
    }

    /* Initialize the input subsystem. */
    err = sf_ctl_init_input();

    if (err) {
        xprintk(KERN_ERR, "sf_ctl_init_input failed with %d.\n", err);
        free_irq(sf_ctl_dev.irq_num, (void *)&sf_ctl_dev);
        return err;
    }

    /* Register as a miscellaneous device. */
    err = misc_register(&sf_ctl_dev.miscdev);

    if (err) {
        xprintk(KERN_ERR, "misc_register(..) = %d.\n", err);
        input_unregister_device(sf_ctl_dev.input);
        free_irq(sf_ctl_dev.irq_num, (void *)&sf_ctl_dev);
        return err;
    }

    INIT_WORK(&sf_ctl_dev.work_queue, sf_ctl_device_event);
    /* [liguanxiong] Create fp_info under /sys/kernel */
    if (sysfs_create_file(kernel_kobj, &dev_attr_fp_info.attr) != 0) {
        printk("Failed to create fp_info.\n");
    }

    /* [liguanxiong start] improve finger lightup speed */
#if SUNWAVE_FAST_LIGHTUP
    if (sunwave_workqueue_init() != 0)
    {
        printk("sunwave_workqueue_init failed\n");
    }
#endif
    /*  [liguanxiong end] */
    xprintk(KERN_ERR, "%s leave\n", __func__);
    return err;
}

static int sf_remove(struct spi_device *spi)
{
    return 0;
}


static struct spi_board_info spi_board_devs[] __initdata = {
    [0] = {
        .modalias = "sunwave-fp",
        .bus_num = 0,
        .chip_select = 0,
        .mode = SPI_MODE_0,
    },
};

static struct spi_driver sf_spi_driver = {
    .driver = {
        .name = "sunwave-fp",
        .bus = &spi_bus_type,
        .owner = THIS_MODULE,
#ifdef CONFIG_OF
        .of_match_table = sf_of_match,
#endif
    },
    .probe = sf_probe,
    .remove = sf_remove,
};

////////////////////////////////////////////////////////////////////////////////

static int sf_ctl_init_gpio_pins(void)
{
    int i, err = 0;
    struct platform_device *pdev = NULL;
    struct device_node *dev_node = NULL;
    xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);
    dev_node = of_find_compatible_node(NULL, NULL, "sunwave,sunwave_fp"); // [liguanxiong] node name compatible with REE

    if (!dev_node) {
        xprintk(KERN_ERR, "of_find_compatible_node(..) failed.\n");
        return (-ENODEV);
    }

    pdev = of_find_device_by_node(dev_node);

    if (!pdev) {
        xprintk(KERN_ERR, "of_find_device_by_node(..) failed.\n");
        return (-ENODEV);
    }

    sf_pinctrl = devm_pinctrl_get(&pdev->dev);

    if (!sf_pinctrl) {
        xprintk(KERN_ERR, "devm_pinctrl_get(..) failed.\n");
        return (-ENODEV);
    }

    for (i = 0; i < SF_PIN_STATE_MAX; ++i) {
        sf_pin_states[i] = pinctrl_lookup_state(sf_pinctrl,
                                                sf_pinctrl_state_names[i]);

        if (!sf_pin_states[i]) {
            xprintk(KERN_ERR, "can't find '%s' pinctrl_state.\n",
                    sf_pinctrl_state_names[i]);
            err = (-ENODEV);
            break;
        }
    }

    if (i < SF_PIN_STATE_MAX) {
        xprintk(KERN_ERR, "%s() failed.\n", __FUNCTION__);
    }

    return err;
}

static int sf_ctl_init_irq(void)
{
    int err = 0;
    struct device_node *dev_node = NULL;
    xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);
    /* Initialize the INT pin. */
    err = pinctrl_select_state(sf_pinctrl, sf_pin_states[SF_PIN_STATE_INT_SET]);
    /* Get the irq number. */
    dev_node = of_find_compatible_node(NULL, NULL, "mediatek,fingerprint-eint"); // [liguanxiong] node name compatible with REE

    if (!dev_node) {
        xprintk(KERN_ERR, "of_find_compatible_node(..) failed.\n");
        return (-ENODEV);
    }

    sf_ctl_dev.irq_num = irq_of_parse_and_map(dev_node, 0);
    xprintk(KERN_INFO, "irq number is %d.\n", sf_ctl_dev.irq_num);
    /* Register interrupt callback. */
    err = request_irq(sf_ctl_dev.irq_num, sf_ctl_device_irq,
                      IRQF_TRIGGER_FALLING, "sf-irq", (void *)&sf_ctl_dev);

    if (err) {
        xprintk(KERN_ERR, "request_irq(..) = %d.\n", err);
    }

    enable_irq_wake(sf_ctl_dev.irq_num);
    return err;
}

static int sf_ctl_init_input(void)
{
    int err = 0;
    xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);
    sf_ctl_dev.input = input_allocate_device();

    if (!sf_ctl_dev.input) {
        xprintk(KERN_ERR, "input_allocate_device(..) failed.\n");
        return (-ENOMEM);
    }

    sf_ctl_dev.input->name = "fp_oem_kpd";  // [liguanxiong] compatible with old driver

    __set_bit(EV_KEY  , sf_ctl_dev.input->evbit );
    __set_bit(KEY_HOME, sf_ctl_dev.input->keybit);
    __set_bit(KEY_MENU, sf_ctl_dev.input->keybit);
    __set_bit(KEY_BACK, sf_ctl_dev.input->keybit);
    __set_bit(KEY_F11, sf_ctl_dev.input->keybit);
    __set_bit(KEY_ENTER  , sf_ctl_dev.input->evbit );
    __set_bit(KEY_WAKEUP, sf_ctl_dev.input->keybit);
    /* [liguanxiong start] Add OEM FP KEY */
    __set_bit(KEY_FP_UP, sf_ctl_dev.input->keybit);
    __set_bit(KEY_FP_LEFT, sf_ctl_dev.input->keybit);
    __set_bit(KEY_FP_RIGHT, sf_ctl_dev.input->keybit);
    __set_bit(KEY_FP_DOWN, sf_ctl_dev.input->keybit);
    __set_bit(KEY_FP_CLICK, sf_ctl_dev.input->keybit);
    __set_bit(KEY_FP_DOUBLE_CLICK, sf_ctl_dev.input->keybit);
    __set_bit(KEY_FP_LONG_CLICK, sf_ctl_dev.input->keybit);
    __set_bit(KEY_FP_SPEED_UP, sf_ctl_dev.input->keybit);
    __set_bit(KEY_FP_TOUCH, sf_ctl_dev.input->keybit);
    /* [liguanxiong end] */
    err = input_register_device(sf_ctl_dev.input);

    if (err) {
        xprintk(KERN_ERR, "input_register_device(..) = %d.\n", err);
        input_free_device(sf_ctl_dev.input);
        sf_ctl_dev.input = NULL;
        return (-ENODEV);
    }

    xprintk(KERN_DEBUG, "%s(..) leave.\n", __FUNCTION__);
    return err;
}

static int __init sf_ctl_driver_init(void)
{
    int err = 0;
    /* Initialize the GPIO pins. */
#if ANDROID_WAKELOCK
    wake_lock_init(&sf_ctl_dev.wakelock, WAKE_LOCK_SUSPEND, "sf_wakelock");
#endif
#if SF_BEANPOD_COMPATIBLE
    uint64_t fp_vendor_id = 0x00;
    get_t_device_id(&fp_vendor_id);
    xprintk(KERN_INFO, "'%s' fp_vendor_id = %u\n", __FUNCTION__, fp_vendor_id);

    if (fp_vendor_id != 0x02) {
        return 0;
    }

#endif
    /* Initialize the GPIO pins. */
    err = sf_ctl_init_gpio_pins();

    if (err) {
        xprintk(KERN_ERR, "sf_ctl_init_gpio_pins failed with %d.\n", err);
        return err;
    }

    err = sf_ctl_device_power(true);
    sf_ctl_device_reset();
#ifdef CONFIG_HAS_EARLYSUSPEND
    //sw_info("%s: register_early_suspend\n", __func__);
    sf_ctl_dev.early_suspend.level = (EARLY_SUSPEND_LEVEL_DISABLE_FB - 1);
    sf_ctl_dev.early_suspend.suspend = sunwave_early_suspend;
    sf_ctl_dev.early_suspend.resume = sunwave_late_resume;
    register_early_suspend(&sf_ctl_dev.early_suspend);
#else
    //sw_info("%s: fb_register_client\n", __func__);
    sf_ctl_dev.notifier.notifier_call = sunwave_fb_notifier_callback;
    fb_register_client(&sf_ctl_dev.notifier);
#endif
    /**register SPI device、driver***/
    spi_register_board_info(spi_board_devs, ARRAY_SIZE(spi_board_devs));
    err = spi_register_driver(&sf_spi_driver);

    if (err < 0) {
        xprintk(KERN_INFO, "%s, Failed to register SPI driver.\n", __FUNCTION__);
        return -EINVAL;
    }

    xprintk(KERN_INFO, "'%s' spi register success", __FUNCTION__);
    xprintk(KERN_INFO, "sunwave fingerprint device control driver registered.\n");
    xprintk(KERN_INFO, "driver version: '%s'.\n", sf_ctl_get_version());
    return err;
}

static void __exit sf_ctl_driver_exit(void)
{
    if (sf_ctl_dev.input) {
        input_unregister_device(sf_ctl_dev.input);
    }

    if (sf_ctl_dev.irq_num >= 0) {
        free_irq(sf_ctl_dev.irq_num, (void *)&sf_ctl_dev);
    }

    misc_deregister(&sf_ctl_dev.miscdev);
    spi_unregister_driver(&sf_spi_driver);
    //sf_spi_platform_exit();
#if ANDROID_WAKELOCK
    wake_lock_destroy(&sf_ctl_dev.wakelock);
#endif
    xprintk(KERN_INFO, "sunwave fingerprint device control driver released.\n");
}

module_init(sf_ctl_driver_init);
module_exit(sf_ctl_driver_exit);

MODULE_DESCRIPTION("The device control driver for Sunwave's fingerprint sensor.");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Langson L. <liangzh@sunwavecorp.com>");

