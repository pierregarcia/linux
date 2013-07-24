/*
 * smsc SCH5017 Watchdog support
 *
 * Copyright (c) 2006-2007,2013 Kurt Van Dijck
 *
 * Based on info and code taken from:
 *
 * drivers/watchdog/sch5017_wdt.c
 * SCH5017 datasheet
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * The author(s) of this software shall not be held liable for damages
 * of any nature resulting due to the use of this software. This
 * software is provided AS-IS with no warranties.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/fs.h>
#include <linux/spinlock.h>
#include <asm/uaccess.h>
#include <asm/io.h>

#define DRVNAME "sch5017_wdt"

static int nowayout = WATCHDOG_NOWAYOUT;

/* superio config registers */
#define SIO_REG	0x2e
#define SIO_VAL 0x2f

static long wdt_regs; /* Watchdog registers (4) */
#define OFFS_TIMEOUT	0
#define OFFS_VAL	1
#define OFFS_CFG	2
#define OFFS_CTRL	3
#define WDT_NREGS	4

/* lock for accessing Superio */
static DEFINE_SPINLOCK(sio_lock);
static int sio_chipid, sio_chiprev;

static int timeout = 60;
static int expect_close;
static int pinged;
static struct watchdog_info wdt_info;
/* count number of open */
static atomic_t n_fopen;

/* superio function */
static inline int superio_inb(int reg)
{
	outb(reg, SIO_REG);
	return inb(SIO_VAL);
}

static inline void superio_outb(int val, int reg)
{
	outb(reg, SIO_REG);
	outb(val, SIO_VAL);
}

static inline void superio_select(int ldn)
{
	outb(7, SIO_REG);
	outb(ldn, SIO_VAL);
}

static inline void superio_enter(void)
{
	spin_lock(&sio_lock);
	outb(0x55, SIO_REG);
}

static inline void superio_exit(void)
{
	outb(0xaa, SIO_REG);
	spin_unlock(&sio_lock);
}

/* watchdog function */
static inline int _wdt_enabled(void)
{
	return inb(wdt_regs + OFFS_VAL) != 0;
}

static inline void wdt_ping(void)
{
	spin_lock(&sio_lock);
	expect_close = 0;
	pinged = 1;
	if (!_wdt_enabled()) {
		/* disabled, don't set */
		spin_unlock(&sio_lock);
		return;
	}
	if (!timeout)
		timeout = 60;
	/* The timeout register only has 8bits wide */
	if (timeout < 256) {
		outb(timeout, wdt_regs + OFFS_VAL);
		outb(inb(wdt_regs +OFFS_TIMEOUT) | 0x80, wdt_regs +OFFS_TIMEOUT);
	} else {
		outb(inb(wdt_regs +OFFS_TIMEOUT) & ~0x80, wdt_regs +OFFS_TIMEOUT);
		outb((timeout +30) /60, wdt_regs + OFFS_VAL);
	}
	spin_unlock(&sio_lock);
}

static void wdt_force (void)
{
	spin_lock(&sio_lock);
	outb(inb(wdt_regs + OFFS_CTRL) | 0x04, wdt_regs + OFFS_CTRL);
	spin_unlock(&sio_lock);
}

static void wdt_disable(void)
{
	spin_lock(&sio_lock);
	outb(0, wdt_regs + OFFS_VAL);
	spin_unlock(&sio_lock);
}

/* reboot notifier */
static int wdt_reboot_notify(struct notifier_block *this,
		unsigned long code, void *unused)
{
	if (code == SYS_HALT || code == SYS_POWER_OFF) {
		if (!nowayout)
			wdt_disable();
	}

	return NOTIFY_DONE;
}

static struct notifier_block wdt_reboot_notifier = {
	.notifier_call = wdt_reboot_notify,
};

/* watchdog device file */
static int fops_open(struct inode *inode, struct file *file)
{
	atomic_inc(&n_fopen);
	return nonseekable_open(inode, file);
}

static int fops_close (struct inode *inode, struct file *file)
{
	if (atomic_dec_and_test(&n_fopen) && _wdt_enabled()) {
		if (nowayout)
			pr_warn(DRVNAME " module not prepared to disable watchdog on exit\n");
		else if (42 != expect_close)
			pr_warn(DRVNAME " unregular close, will not disable watchdog\n");
		else
			wdt_disable();
		expect_close = 0;
	}
	return 0;
}

static ssize_t fops_write(struct file *file, const char __user *data,
		size_t len, loff_t *ppos)
{
	/* check for a magic close character */
	if (len) {
		char c;

		if (get_user(c, data + len -1))
			return -EFAULT;
		wdt_ping();
		expect_close = ('V' == c) ? 42 : 0;
	}

	return len;
}

static long fops_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int __user *p = argp;
	int val;

	switch (cmd) {
	default:
		return -ENOTTY;
	case WDIOC_GETSUPPORT:
		if (copy_to_user(argp, &wdt_info, sizeof(wdt_info)))
			return -EFAULT;
		return 0;
		//case WDIOC_GETBOOTSTATUS:
	case WDIOC_GETSTATUS:
		val = 0;
		spin_lock(&sio_lock);
		if (pinged)
			val |= WDIOF_KEEPALIVEPING;
		pinged = 0;
		spin_unlock(&sio_lock);
		return put_user(val, p);
	case WDIOC_KEEPALIVE:
		wdt_ping();
		return 0;
	case WDIOC_SETTIMEOUT:
		if (get_user(val, p))
			return -EFAULT;
		if (val < 1)
			return -EINVAL;
		timeout = val;
		return 0;
	case WDIOC_GETTIMEOUT:
		if (put_user(timeout, p))
			return -EFAULT;
		return 0;
	case WDIOC_SETOPTIONS:
		val = arg;
		if (val & WDIOS_ENABLECARD)
			wdt_ping();
		else if (val & WDIOS_DISABLECARD) {
			if (nowayout)
				return -EINVAL;
			wdt_disable();
		}
		if (val & WDIOS_TEMPPANIC)
			wdt_force();
		return 0;
	}
}

static const struct file_operations wdt_fops = {
	.owner = THIS_MODULE,
	.open = fops_open,
	.release = fops_close,
	.llseek = no_llseek,
	.write = fops_write,
	.unlocked_ioctl = fops_ioctl,
};

static struct miscdevice wdt_miscdev = {
	.minor = WATCHDOG_MINOR,
	.name = "watchdog",
	.fops = &wdt_fops,
};

/* SYSFS */
static ssize_t show_regs(struct device * d, struct device_attribute * attr, char * buf)
{
	unsigned char reg[4];

	spin_lock(&sio_lock);
	reg[0] = inb(wdt_regs +0);
	reg[1] = inb(wdt_regs +1);
	reg[2] = inb(wdt_regs +2);
	reg[3] = inb(wdt_regs +3);
	spin_unlock(&sio_lock);

	return sprintf(buf, "%02x %02x %02x %02x\n" , reg[0] , reg[1] , reg[2] , reg[3]);
}
static DEVICE_ATTR(regs, S_IRUSR | S_IRGRP, show_regs, 0);
static struct attribute *sch5017_attrs[] = {
	&dev_attr_regs.attr,
	0,
};

static struct attribute_group sch5017_attr_group = {
	.attrs = sch5017_attrs,
};

/* INIT */
static int __init probe_dev_sch5017 (void)
{
	superio_enter();

	sio_chipid = superio_inb(0x20);
	sio_chiprev = superio_inb(0x21);

	if (0x78 != sio_chipid) {
		pr_info(DRVNAME " wrong device with id 0x%02x, rev %u\n",
				sio_chipid, sio_chiprev);
		goto exit;
	}

	/* select the PME device */
	superio_select(0x0a);
	if (!(superio_inb(0x30) & 0x01)) {
		pr_warn(DRVNAME " activating ...\n");
		superio_outb(1, 0x30);
	}
	mb();

	if (!(superio_inb(0x30) & 0x01)) {
		pr_err(DRVNAME " device not activated, skipped\n");
		goto exit;
	}

	wdt_regs = (superio_inb(0x60) << 8) | superio_inb(0x61);
	if (0 == wdt_regs) {
		pr_err(DRVNAME " base address not set, skipped\n");
		goto exit;
	}
	/* wdt control is 0x65-0x69 */
	wdt_regs += 0x65;
	pr_info(DRVNAME " found id 0x%02x, rev %u, @0x%04lx\n",
			sio_chipid, sio_chiprev, wdt_regs);
	superio_exit();

	/* debug */
	if (1) {
		unsigned char regs[4];
		spin_lock(&sio_lock);
		regs[0] = inb(wdt_regs +0);
		regs[1] = inb(wdt_regs +1);
		regs[2] = inb(wdt_regs +2);
		regs[3] = inb(wdt_regs +3);
		spin_unlock(&sio_lock);
		pr_info(DRVNAME " %02x %02x %02x %02x\n",
				regs[0] , regs[1] , regs[2] , regs[3]);
	}

	return 0;

exit:
	superio_exit();
	return -ENODEV;
}

static int __init mod_init(void)
{
	int err = 0;

	/* probe */
	err = probe_dev_sch5017();
	if (err < 0)
		return err;
	err = -EBUSY;
	if (!request_region(wdt_regs, WDT_NREGS, DRVNAME)) {
		pr_err(DRVNAME " could not request io 0x%04lx:%u\n",
				wdt_regs, WDT_NREGS);
		goto exit;
	}

	spin_lock(&sio_lock);
	/* enable keyboard reset */
	outb(inb(wdt_regs + OFFS_CFG) | 0x02, wdt_regs + OFFS_CFG);
	spin_unlock(&sio_lock);
	/* reboot notifier */
	err = register_reboot_notifier(&wdt_reboot_notifier);
	if (err < 0) {
		pr_err(DRVNAME " could not register reboot notifier\n");
		goto exit_with_io;
	}

	err = misc_register(&wdt_miscdev);
	if (err < 0) {
		pr_err(DRVNAME " could not register misc device\n");
		goto exit_with_reboot;
	}
	err = sysfs_create_group(&wdt_miscdev.this_device->kobj,
			&sch5017_attr_group);

	if (err < 0) {
		pr_err(DRVNAME " could not create sysfs entries\n");
		goto exit_with_misc;
	}

	/* prepare watchdog indentity */
	strncpy(wdt_info.identity, "smsc-sch5017", sizeof(wdt_info.identity));
	wdt_info.firmware_version = sio_chiprev;
	wdt_info.options = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE;
	return 0;

exit_with_misc:
	misc_deregister(&wdt_miscdev);
exit_with_reboot:
	unregister_reboot_notifier(&wdt_reboot_notifier);
exit_with_io:
	release_region(wdt_regs, WDT_NREGS);
exit:
	return err ? err : -ENODEV;
}

static void __exit mod_exit(void)
{
	sysfs_remove_group(&wdt_miscdev.this_device->kobj,
			&sch5017_attr_group);
	misc_deregister(&wdt_miscdev);
	unregister_reboot_notifier(&wdt_reboot_notifier);
	release_region(wdt_regs, WDT_NREGS);
}

module_init(mod_init);
module_exit(mod_exit);

MODULE_AUTHOR("Kurt Van Dijck <kurt.van.dijck@skynet.be>");
MODULE_DESCRIPTION("smsc-sch5017 watchdog driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
module_param(timeout, int, 60);
MODULE_PARM_DESC(timeout, "watchdog timeout in seconds");
module_param(nowayout, int, 0);
MODULE_PARM_DESC(nowayout, "disable watchdog shutdown on close");

