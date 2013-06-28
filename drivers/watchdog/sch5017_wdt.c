/*
 *	smsc SCH5017 Watchdog support
 *
 *	Copyright (c) 2006-2007 Kurt Van Dijck
 *
 *	Based on info and code taken from:
 *
 * drivers/watchdog/sch5017_wdt.c
 * SCH5017 datasheet
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation; either version 2 of the
 *	License, or (at your option) any later version.
 *
 *	The author(s) of this software shall not be held liable for damages
 *	of any nature resulting due to the use of this software. This
 *	software is provided AS-IS with no warranties.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/fs.h>
#include <linux/pci.h>
#include <linux/spinlock.h>

#include <asm/uaccess.h>
#include <asm/io.h>

static int margin = 60;		/* in seconds */

static int nowayout = WATCHDOG_NOWAYOUT;

struct dev {
   unsigned long regs;
      #define OFFS_TIMEOUT 0
      #define OFFS_VAL     1
      #define OFFS_CFG     2
      #define OFFS_CTRL    3
   int nregs;
   struct {
      int id;
      int rev;
   } chip;
   unsigned int timeout;
   int enabled;
   int expect_close;
   int pinged;
   struct {
      // superio config regs
      unsigned long reg;
      unsigned long val;
   } sio;
   const char * name;
   atomic_t    n_instances;
   spinlock_t  lock;
   struct miscdevice       misc;
   struct notifier_block   reboot;
   struct file_operations  fops;
	struct watchdog_info    wdt;
   struct attribute_group  sysfs;
};
#define misc2dev(x)     container_of((x), struct dev, misc  )
#define reboot2dev(x)   container_of((x), struct dev, reboot)

#define dev_msg(level, dev, proto, ...) printk(level "%s: " proto "\n", dev->name, ##__VA_ARGS__)

// keep global pointer
static struct dev * pdev = 0;

/* superio function */
static int
superio_inb(struct dev * dev, int reg)
{
	outb(reg, dev->sio.reg);
	return inb(dev->sio.val);
}

static void
superio_outb(struct dev * dev, int val, int reg)
{
	outb(reg, dev->sio.reg);
	outb(val, dev->sio.val);
}

static inline void
superio_select(struct dev * dev, int ldn)
{
	outb(7, dev->sio.reg);
	outb(ldn, dev->sio.val);
}

static inline void
superio_enter(struct dev * dev)
{
	spin_lock(&dev->lock);
	outb(0x55, dev->sio.reg);
}

static inline void
superio_exit(struct dev * dev)
{
	outb(0xaa, dev->sio.reg);
	spin_unlock(&dev->lock);
}
/* ~superio functions */
/* watchdog function */
static void
wdt_reset(struct dev * dev)
{
   /* be sure to have lock */
	/* The timeout register only has 8bits wide */
   if (margin < 256) {
      outb(dev->timeout, dev->regs + OFFS_VAL);
      outb(inb(dev->regs +OFFS_TIMEOUT) | 0x80, dev->regs +OFFS_TIMEOUT);
   } else {
      outb(inb(dev->regs +OFFS_TIMEOUT) & ~0x80, dev->regs +OFFS_TIMEOUT);
      outb((dev->timeout +30) /60, dev->regs + OFFS_VAL);
   }
}

static inline void
wdt_ping(struct dev * dev)
{
   spin_lock(&dev->lock);
   dev->expect_close = 0;
   dev->pinged = 1;
   if (!dev->enabled) {
      /* disabled, don't set */
      spin_unlock(&dev->lock);
      return;
   }
   if (!dev->timeout)
      dev->timeout = 60;
   wdt_reset(dev);
   spin_unlock(&dev->lock);
}

static void
wdt_force (struct dev * dev)
{
   dev_msg(KERN_DEBUG, dev, "force watchdog");
   spin_lock(&dev->lock);
   outb(inb(dev->regs + OFFS_CTRL) | 0x04, dev->regs + OFFS_CTRL);
   spin_unlock(&dev->lock);
}

static void
wdt_enable(struct dev * dev)
{
   spin_lock(&dev->lock);
   if (!dev->timeout)
      dev->timeout = 60;
   dev->enabled = 1;
   wdt_reset(dev);
   spin_unlock(&dev->lock);

   dev_msg(KERN_DEBUG, dev, "enable watchdog, %us", dev->timeout);
}

static void
wdt_disable(struct dev * dev)
{
   dev_msg(KERN_DEBUG, dev, "disable watchdog");
	spin_lock(&dev->lock);
   outb(0, dev->regs + OFFS_VAL);
   dev->enabled = 0;
	spin_unlock(&dev->lock);
}
/* ~watchdog function */

static int
reboot_notify(struct notifier_block *this,
		    unsigned long code, void *unused)
{
   struct dev * dev = reboot2dev(this);

	if (code == SYS_HALT || code == SYS_POWER_OFF) {
		if (!nowayout)
			wdt_disable(dev);
      dev_msg(KERN_DEBUG, dev, "reboot notifier, %s disabled watchdog", nowayout ? "not" : "");
   }

	return NOTIFY_DONE;
}
static int fops_open(struct inode *inode, struct file *file)
{
   struct dev * dev;
   dev = pdev; //container_of(inode->i_fop, struct dev, fops);
   file->private_data = dev;
   atomic_inc(&dev->n_instances);
	return nonseekable_open(inode, file);
}

static int
fops_close (struct inode *inode, struct file *file)
{
   struct dev * dev = (struct dev *)file->private_data;
   if (atomic_dec_and_test(&dev->n_instances)) {
      /* this is the last one */
      if (!dev->enabled) {
         dev_msg(KERN_INFO, dev, "watchdog not enabled, leaving it disabled");
      } else if (nowayout) {
         dev_msg(KERN_WARNING, dev, "module not prepared to disable watchdog on exit");
      } else if (42 != dev->expect_close) {
         dev_msg(KERN_WARNING, dev, "unregular close, will not disable watchdog");
      } else {
         wdt_disable(dev);
      }
      dev->expect_close = 0;
   }
	return 0;
}

static ssize_t
fops_wr (struct file *file, const char __user *data,
	size_t len, loff_t *ppos)
{
   struct dev * dev = (struct dev *)file->private_data;
	/* check for a magic close character */
	if (len) {
      char c;

      if (get_user(c, data + len -1))
         return -EFAULT;
      wdt_ping(dev);
		dev->expect_close = ('V' == c) ? 42 : 0;
	}

	return len;
}

static int
fops_ioctl(struct inode *inode, struct file *file,
	unsigned int cmd, unsigned long arg)
{
   struct dev * dev = (struct dev *)file->private_data;
	void __user *argp = (void __user *)arg;
	int __user *p = argp;
	int val;

	switch (cmd) {
	default:
		return -ENOTTY;
	case WDIOC_GETSUPPORT:
		if (copy_to_user(argp, &dev->wdt, sizeof(dev->wdt)))
			return -EFAULT;
		return 0;
	//case WDIOC_GETBOOTSTATUS:
	case WDIOC_GETSTATUS:
      val = 0;
      spin_lock(&dev->lock);
      if (dev->pinged)
         val |= WDIOF_KEEPALIVEPING;
      dev->pinged = 0;
      spin_unlock(&dev->lock);
		return put_user(val, p);
	case WDIOC_KEEPALIVE:
      wdt_ping(dev);
      return 0;
	case WDIOC_SETTIMEOUT:
		if (get_user(val, p))
			return -EFAULT;
		if (val < 1)
			return -EINVAL;
      dev->timeout = val;
      return 0;
	case WDIOC_GETTIMEOUT:
		if (put_user(dev->timeout, p))
			return -EFAULT;
		return 0;
   case WDIOC_SETOPTIONS:
      val = arg;
      if (val & WDIOS_ENABLECARD) {
         wdt_enable(dev);
      } else if (val & WDIOS_DISABLECARD) {
         if (nowayout)
            return -EINVAL;
         wdt_disable(dev);
      }
      if (val & WDIOS_TEMPPANIC) {
         wdt_force(dev);
      }
      return 0;
	}
}


static ssize_t show_cfg (struct device * d, struct device_attribute * attr, char * buf) {
   struct dev * dev = (struct dev *)d->driver_data;
   unsigned char reg[4];

   spin_lock(&dev->lock);
   reg[0] = inb(dev->regs +0);
   reg[1] = inb(dev->regs +1);
   reg[2] = inb(dev->regs +2);
   reg[3] = inb(dev->regs +3);
   spin_unlock(&dev->lock);

   return sprintf(buf, "%02x %02x %02x %02x\n" , reg[0] , reg[1] , reg[2] , reg[3]);
}
static DEVICE_ATTR(cfg, S_IRUSR | S_IRGRP, show_cfg, 0);
static struct attribute * sch5017_sysfs [] = {
   &dev_attr_cfg.attr,
   0,
};
static int __init
probe_dev_sch5017 (struct dev * dev)
{
   superio_enter(dev);

   dev->chip.id  = superio_inb(dev, 0x20);
   dev->chip.rev = superio_inb(dev, 0x21);

	if (0x78 != dev->chip.id) {
      dev_msg(KERN_INFO, dev, "device with id 0x%02x, rev %u" 
            , dev->chip.id, dev->chip.rev);
      goto exit;
   }

   /* select the PME device */
	superio_select(dev, 0x0a);
	if (!(superio_inb(dev, 0x30) & 0x01)) {
      dev_msg(KERN_ERR, dev, "device not activated, activating ...");
	   superio_outb(dev, 1, 0x30);
	}
   mb();

	if (!(superio_inb(dev, 0x30) & 0x01)) {
      dev_msg(KERN_ERR, dev, "device not activated, skipped");
      goto exit;
	}

   dev->regs = (superio_inb(dev, 0x60) << 8) | superio_inb(dev, 0x61);
	if (0 == dev->regs) {
      dev_msg(KERN_ERR, dev, "base address not set, skipped");
      goto exit;
	}
   /* wdt control is 0x65-0x69 */
   dev->regs += 0x65;
   dev->nregs = 4;
   dev_msg(KERN_INFO, dev, "found id 0x%02x, rev %u, @0x%04lx"
         , dev->chip.id, dev->chip.rev
         , dev->regs
         );
   dev->enabled = inb(dev->regs + OFFS_VAL) != 0;
	superio_exit(dev);

   if (1) {
      unsigned char regs[4];
      spin_lock(&dev->lock);
      regs[0] = inb(dev->regs +0);
      regs[1] = inb(dev->regs +1);
      regs[2] = inb(dev->regs +2);
      regs[3] = inb(dev->regs +3);
      spin_unlock(&dev->lock);
      dev_msg(KERN_INFO, dev, "%02x %02x %02x %02x" , regs[0] , regs[1] , regs[2] , regs[3]);
   }

   dev->sysfs.name  = "sch5017";
   dev->sysfs.attrs = sch5017_sysfs;
   return 0;

exit:
	superio_exit(dev);
	return -ENODEV;
}

#define DYNAMIC 0
#if !DYNAMIC
static struct dev st_dev;
#endif
static int __init
mod_init(void)
{
	int err = 0;
   struct dev * dev;

   if (pdev)
      return -EEXIST;

   #if DYNAMIC
   if (0 == (dev = kmalloc(sizeof(*dev), GFP_ATOMIC)))
      return -ENOMEM;
   #else
   dev = &st_dev;
   #endif
   memset(dev, 0, sizeof(*dev));
   spin_lock_init(&dev->lock);
   dev->name = "smsc-sch5017-wdt";
   dev->timeout = margin;
   /* superio config registers */
   dev->sio.reg = 0x2e;
   dev->sio.val = 0x2f;
   /* probe */
   if (0 != (err = probe_dev_sch5017(dev)))
      goto exit;
   err = -EBUSY;
   if (!request_region(dev->regs, dev->nregs, dev->name)) {
      dev_msg(KERN_ERR, dev, "could not request io 0x%04lx:%u", dev->regs, dev->nregs);
      goto exit;
   }
   /* init defaults */
   spin_lock(&dev->lock);
   /* enable keyboard reset */
   outb(inb(dev->regs + OFFS_CFG) | 0x02, dev->regs + OFFS_CFG);
   spin_unlock(&dev->lock);
   /* reboot notifier */
   dev->reboot.notifier_call = reboot_notify;
   if (0 != (err = register_reboot_notifier(&dev->reboot))) {
      dev_msg(KERN_ERR, dev, "could not register reboot notifier");
      goto exit_with_io;
   }
   /* fops */
   dev->fops.owner   = THIS_MODULE;
   dev->fops.llseek  = no_llseek;
   dev->fops.write   = fops_wr;
   dev->fops.ioctl   = fops_ioctl;
   dev->fops.open    = fops_open;
   dev->fops.release = fops_close;
   /* miscdevice init */
   dev->misc.name = "watchdog";
   dev->misc.minor = WATCHDOG_MINOR;
   dev->misc.fops  = &dev->fops;

   if (0 != (err = misc_register(&dev->misc))) {
      dev_msg(KERN_ERR, dev, "could not register misc device");
      goto exit_with_reboot;
   }
   if (dev->sysfs.attrs) {
      dev->misc.this_device->driver_data = dev;
      if (0 != (err = sysfs_create_group(&dev->misc.this_device->kobj, &dev->sysfs))) {
         dev_msg(KERN_ERR, dev, "could not create sysfs entries (%i)", err);
      }
   }
   
   /* prepare watchdog indentity */
   strncpy(dev->wdt.identity, dev->name, sizeof(dev->wdt.identity) -1);
   dev->wdt.firmware_version = dev->chip.rev;
   dev->wdt.options = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE;
   pdev = dev;
	return 0;

exit_with_reboot:
	unregister_reboot_notifier(&dev->reboot);
exit_with_io:
	release_region(dev->regs, dev->nregs);
exit:
	return err ? err : -ENODEV;
}

static void __exit
mod_exit(void)
{
   struct dev * dev;
   if (0 != (dev = pdev)) {
      pdev = 0;
      if (dev->sysfs.attrs) {
         sysfs_remove_group(&dev->misc.this_device->kobj, &dev->sysfs);
         dev->misc.this_device->driver_data = 0;
      }
      misc_deregister(&dev->misc);
      unregister_reboot_notifier(&dev->reboot);
      release_region(dev->regs, dev->nregs);
      dev_msg(KERN_INFO, dev, "module shutdown");
      memset(dev, 0, sizeof(*dev));
      #if DYNAMIC
      kfree(dev);
      #endif
   }
}

module_init(mod_init);
module_exit(mod_exit);

MODULE_AUTHOR("Kurt Van Dijck <kurt.van.dijck@skynet.be>");
MODULE_DESCRIPTION("smsc-sch5017 watchdog driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
module_param(margin, int, 0);
MODULE_PARM_DESC(margin, "watchdog margin in seconds");
module_param(nowayout, int, 0);
MODULE_PARM_DESC(nowayout, "disable watchdog shutdown on close");

