 /* drivers/usb/otg/otg_event.c
 *
 * OTG event handling function
 *
 * Copyright (C) 2010, 2011 Sony Ericsson Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/usb/otg.h>
#include <linux/usb/otg_event.h>
#include <linux/cdev.h>
#include <linux/semaphore.h>

#define MAX_NAME_SIZE 16
#define MAX_MODULE_NAME_SIZE 16
#define MAX_EVENT_STRING_SIZE 32

struct otg_event_drv {
	struct class *class;
	struct device *dev;
	char name[MAX_NAME_SIZE];
};

static DECLARE_MUTEX(sem);

static struct otg_event_drv *otg_event;

static const char *event_string(enum usb_otg_event event)
{
	switch (event) {
	case OTG_EVENT_DEV_CONN_TMOUT:
		return "DEV_CONN_TMOUT";
	case OTG_EVENT_NO_RESP_FOR_HNP_ENABLE:
		return "NO_RESP_FOR_HNP_ENABLE";
	case OTG_EVENT_HUB_NOT_SUPPORTED:
		return "HUB_NOT_SUPPORTED";
	case OTG_EVENT_DEV_NOT_SUPPORTED:
		return "DEV_NOT_SUPPORTED,";
	case OTG_EVENT_HNP_FAILED:
		return "HNP_FAILED";
	case OTG_EVENT_NO_RESP_FOR_SRP:
		return "NO_RESP_FOR_SRP";
#ifdef CONFIG_USB_OTG_NOTIFICATION
	case OTG_EVENT_ACA_CONNECTED:
		return "ACA_CONNECTED";
	case OTG_EVENT_ACA_DISCONNECTED:
		return "ACA_DISCONNECTED";
	case OTG_EVENT_VBUS_DROP:
		return "VBUS_DROP";
#endif
	default:
		return "UNDEFINED";
	}
}

static int otg_send_uevent(struct otg_transceiver *xceiv,
			enum usb_otg_event event)
{
	struct otg_event_drv *dev = otg_event;

	char udev_event[MAX_EVENT_STRING_SIZE];
	char module[MAX_MODULE_NAME_SIZE];
	char *envp[] = {module, udev_event, NULL};
	int ret;

	if (dev == NULL)
		return -ENODEV;

	ret = down_interruptible(&sem);
	if (ret < 0)
		return ret;

	pr_debug("sending %s event\n", event_string(event));

	snprintf(udev_event, MAX_EVENT_STRING_SIZE, "EVENT=%s",
			event_string(event));
	snprintf(module, MAX_MODULE_NAME_SIZE, "MODULE=%s", dev->name);
	ret = kobject_uevent_env(&dev->dev->kobj, KOBJ_CHANGE, envp);

	up(&sem);

	if (ret < 0)
		pr_info("uevent sending failed with ret = %d\n", ret);

	return ret;
}

int otg_event_driver_register(struct otg_transceiver *xceiv)
{
	struct otg_event_drv *dev;
	int ret;

	if (otg_event)
		return -EBUSY;

	dev = kzalloc(sizeof(struct otg_event_drv), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	otg_event = dev;

	dev->class = class_create(THIS_MODULE, "usb_otg");
	if (IS_ERR(dev->class)) {
		pr_info("%s :failed to create class", __func__);
		ret = PTR_ERR(dev->class);
		goto class_create_fail;
	}

	if (xceiv->label)
		strlcpy(dev->name, xceiv->label, MAX_NAME_SIZE);
	else
		strlcpy(dev->name, "Unknown", MAX_NAME_SIZE);

	dev->dev = device_create(dev->class, NULL, MKDEV(0, 0), NULL,
					dev->name);
	if (IS_ERR(dev->dev)) {
		pr_info("%s :failed to create device", __func__);
		ret = PTR_ERR(dev->dev);
		goto device_create_fail;
	}
	xceiv->send_event = otg_send_uevent;

	return 0;

device_create_fail:
	class_destroy(dev->class);
class_create_fail:
	kfree(dev);
	otg_event = NULL;
	return ret;
}

void otg_event_driver_unregister(void)
{
	struct otg_event_drv *dev = otg_event;

	down(&sem);

	device_destroy(dev->class, MKDEV(0, 0));
	class_destroy(dev->class);
	kfree(dev);
	otg_event = NULL;

	up(&sem);
}
