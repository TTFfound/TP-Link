/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
i * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/workqueue.h>
/* [tanghonghui start] Add waklock */
#include <linux/wakelock.h>
/* [tanghonghui end] */

#include "timed_output.h"

#include <linux/hrtimer.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/debugfs.h>

/* #include <mach/mt6577_pm_ldo.h> */

#include <vibrator.h>
#include <vibrator_hal.h>

#define VERSION					        "v 0.1"
#define VIB_DEVICE				"mtk_vibrator"

static struct dentry *vibr_droot;
static struct dentry *vibr_dklog;
int vibr_klog_en;

/* #define pr_fmt(fmt) "[vibrator]"fmt */
#define VIB_DEBUG(format, args...) do { \
	if (vibr_klog_en) {\
		pr_debug(format, ##args);\
	} \
} while (0)
#define VIB_INFO(format, args...) do { \
	if (vibr_klog_en) {\
		pr_info(format, ##args);\
	} \
} while (0)

/******************************************************************************
Error Code No.
******************************************************************************/
#define RSUCCESS        0

/******************************************************************************
Debug Message Settings
******************************************************************************/

/* Debug message event */
#define DBG_EVT_NONE		0x00000000	/* No event */
#define DBG_EVT_INT			0x00000001	/* Interrupt related event */
#define DBG_EVT_TASKLET		0x00000002	/* Tasklet related event */

#define DBG_EVT_ALL			0xffffffff

#define DBG_EVT_MASK		(DBG_EVT_TASKLET)

#if 1
#define MSG(evt, fmt, args...) \
do {	\
	if ((DBG_EVT_##evt) & DBG_EVT_MASK) { \
		VIB_DEBUG(fmt, ##args); \
	} \
} while (0)

#define MSG_FUNC_ENTRY(f)	MSG(FUC, "<FUN_ENT>: %s\n", __func__)
#else
#define MSG(evt, fmt, args...) do {} while (0)
#define MSG_FUNC_ENTRY(f)	   do {} while (0)
#endif

/******************************************************************************
Global Definations
******************************************************************************/
static struct workqueue_struct *vibrator_queue;
static struct work_struct vibrator_work;
static struct hrtimer vibe_timer;
static spinlock_t vibe_lock;
static int vibe_state;
static int ldo_state;
static int shutdown_flag;

static int vibr_Enable(void)
{
	if (!ldo_state) {
		vibr_Enable_HW();
		ldo_state = 1;
	}
	return 0;
}

static int vibr_Disable(void)
{
	if (ldo_state) {
		vibr_Disable_HW();
		ldo_state = 0;
	}
	return 0;
}

static void update_vibrator(struct work_struct *work)
{
	if (!vibe_state)
		vibr_Disable();
	else
		vibr_Enable();
}

static int vibrator_get_time(struct timed_output_dev *dev)
{
	if (hrtimer_active(&vibe_timer)) {
		ktime_t r = hrtimer_get_remaining(&vibe_timer);

		return ktime_to_ms(r);
	} else
		return 0;
}

// [tanghonghui start] Add wakelock
static struct wake_lock vibe_wlock;
// [tanghonghui end]

static void vibrator_enable(struct timed_output_dev *dev, int value)
{
	unsigned long flags;

#if 1
	struct vibrator_hw *hw = mt_get_cust_vibrator_hw();
#endif

	/* VIB_DEBUG("vibrator_enable: vibrator first in value = %d\n", value); */

	/* [tanghonghui start] Add wakelock */
	wake_lock_timeout(&vibe_wlock, msecs_to_jiffies(1000));
	/* [tanghonghui end] */

	spin_lock_irqsave(&vibe_lock, flags);
	while (hrtimer_cancel(&vibe_timer))
		VIB_DEBUG("vibrator_enable: try to cancel hrtimer[cust timer: %d(ms)], value: %d\n",
			hw->vib_timer, value);

	if (value == 0 || shutdown_flag == 1) {
		/* VIB_DEBUG("vibrator_enable: shutdown_flag = %d, cust_timer:%d\n",
			  shutdown_flag, hw->vib_timer); */
		vibe_state = 0;
	} else {
#if 1
		/* VIB_DEBUG("vibrator_enable: vibrator cust timer: %d\n",
			  hw->vib_timer); */
#ifdef CUST_VIBR_LIMIT
		if (value > hw->vib_limit && value < hw->vib_timer)
#else
		if (value >= 10 && value < hw->vib_timer)
#endif
			value = hw->vib_timer;
#endif

		value = (value > 15000 ? 15000 : value);
		vibe_state = 1;
		hrtimer_start(&vibe_timer,
			      ktime_set(value / 1000, (value % 1000) * 1000000),
			      HRTIMER_MODE_REL);
	}
	spin_unlock_irqrestore(&vibe_lock, flags);
	/* VIB_DEBUG("vibrator_enable: vibrator start: %d\n", value); */
	queue_work(vibrator_queue, &vibrator_work);
}

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
	vibe_state = 0;
	VIB_DEBUG("vibrator_timer_func: vibrator will disable\n");
	queue_work(vibrator_queue, &vibrator_work);
	return HRTIMER_NORESTART;
}

static struct timed_output_dev mtk_vibrator = {
	.name = "vibrator",
	.get_time = vibrator_get_time,
	.enable = vibrator_enable,
};

static int vib_probe(struct platform_device *pdev)
{
	return 0;
}

static int vib_remove(struct platform_device *pdev)
{
	return 0;
}

static void vib_shutdown(struct platform_device *pdev)
{
	unsigned long flags;

	VIB_DEBUG("vib_shutdown: enter!\n");
	spin_lock_irqsave(&vibe_lock, flags);
	shutdown_flag = 1;
	if (vibe_state) {
		VIB_DEBUG("vib_shutdown: vibrator will disable\n");
		vibe_state = 0;
		spin_unlock_irqrestore(&vibe_lock, flags);
		vibr_Disable();
	} else {
		spin_unlock_irqrestore(&vibe_lock, flags);
	}
}

/******************************************************************************
Device driver structure
*****************************************************************************/
static struct platform_driver vibrator_driver = {
	.probe = vib_probe,
	.remove = vib_remove,
	.shutdown = vib_shutdown,
	.driver = {
		   .name = VIB_DEVICE,
		   .owner = THIS_MODULE,
		   },
};

static struct platform_device vibrator_device = {
	.name = "mtk_vibrator",
	.id = -1,
};

static ssize_t store_vibr_on(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t size)
{
	if (buf != NULL && size != 0) {
		VIB_DEBUG("buf is %s and size is %zu\n", buf, size);
		if (buf[0] == '0')
			vibr_Disable();
		else
			vibr_Enable();
	}
	return size;
}

static DEVICE_ATTR(vibr_on, 0220, NULL, store_vibr_on);

/******************************************************************************
 * vib_mod_init
 *
 * DESCRIPTION:
 *   Register the vibrator device driver !
 *
 * PARAMETERS:
 *   None
 *
 * RETURNS:
 *   None
 *
 * NOTES:
 *   RSUCCESS : Success
 *
 ******************************************************************************/

static int vib_mod_init(void)
{
	s32 ret;
	struct vibrator_hw *hw;

	VIB_DEBUG("MediaTek MTK vibrator driver register, version %s\n",
		  VERSION);
	hw = mt_get_cust_vibrator_hw();
	if (hw == NULL) {
		VIB_INFO("%s: get dts fail.\n", __func__);
		return -1;
	}
	/* set vibr voltage if needs.  Before MT6320 vibr default voltage=2.8v,
	   but in MT6323 vibr default voltage=1.2v */
	vibr_power_set();
	ret = platform_device_register(&vibrator_device);
	if (ret != 0) {
		VIB_DEBUG("Unable to register vibrator device (%d)\n", ret);
		return ret;
	}

	vibrator_queue = create_singlethread_workqueue(VIB_DEVICE);
	if (!vibrator_queue) {
		VIB_DEBUG("Unable to create workqueue\n");
		return -ENODATA;
	}
	INIT_WORK(&vibrator_work, update_vibrator);

	spin_lock_init(&vibe_lock);
	shutdown_flag = 0;
	vibe_state = 0;
	hrtimer_init(&vibe_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vibe_timer.function = vibrator_timer_func;

	timed_output_dev_register(&mtk_vibrator);/* timed_output driver model */

	ret = platform_driver_register(&vibrator_driver);

	if (ret) {
		VIB_DEBUG("Unable to register vibrator driver (%d)\n", ret);
		return ret;
	}

	// [tanghonghui start] Add wakelock
	wake_lock_init(&vibe_wlock, WAKE_LOCK_SUSPEND, "vibe_wakelock");
	// [tanghonghui end]

	ret = device_create_file(mtk_vibrator.dev, &dev_attr_vibr_on);
	if (ret)
		VIB_DEBUG("device_create_file vibr_on fail!\n");

	/* Add vibrator debug node */
#ifdef CONFIG_MTK_ENG_BUILD
		vibr_klog_en = 1;
#else
		vibr_klog_en = 0;
#endif
		vibr_droot = debugfs_create_dir("vibrator", NULL);
		if (IS_ERR_OR_NULL(vibr_droot))
			return 0;
		vibr_dklog = debugfs_create_u32("debug", 0600, vibr_droot, &vibr_klog_en);


	VIB_DEBUG("vib_mod_init Done\n");

	return RSUCCESS;
}

/******************************************************************************
 * vib_mod_exit
 *
 * DESCRIPTION:
 *   Free the device driver !
 *
 * PARAMETERS:
 *   None
 *
 * RETURNS:
 *   None
 *
 * NOTES:
 *   None
 *
 ******************************************************************************/

static void vib_mod_exit(void)
{
	VIB_DEBUG("MediaTek MTK vibrator driver unregister, version %s\n",
		  VERSION);
	if (vibrator_queue)
		destroy_workqueue(vibrator_queue);
	// [tanghonghui start] Add wakelock
	wake_lock_destroy(&vibe_wlock);
	// [tanghonghui end]
	VIB_DEBUG("vib_mod_exit Done\n");
}

module_init(vib_mod_init);
module_exit(vib_mod_exit);
MODULE_AUTHOR("MediaTek Inc.");
MODULE_DESCRIPTION("MTK Vibrator Driver (VIB)");
MODULE_LICENSE("GPL");
