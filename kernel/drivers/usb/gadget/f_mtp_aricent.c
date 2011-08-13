/*++

 Product Name:
	MTP Over USB

 Module Name:
	MTP USB Gadget driver

 File name:
	mtp.c

 License:
	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

 Abstract:
	This module is responsible for following functionalities
	1.Act as a USB Gadget driver
	2.Enumerate the device as MTP to the Host
	3.Transfer/Receive data between MTP stack and host[MTP Initiator]
	4.Handle Interrupts from host and notify to userspace

 Environment:
	Linux Kernel 2.9
	Android version 1.5/1.6

 Notes:

 Revision History:
	Aricent:
	July 02, 2009  By RamaSubramanian A - Initial Code.
	Sep  01, 2009  By RamaSubramanian A - Code review changes done.
	Jan  11, 2010  By RamaSubramanian A - Added support to notify host
						event to Application.
	Jan  27, 2010  By RamaSubramanian A - Added support to send events to
						host through Interrupt pipe.
	March 5, 2010  By RamaSubramanian A - Removed Hard coded values for
						Event notification kill_fasync.

	April 2, 2010 By RamaSubramanian A  - Added fix for WLK Transport test
 --*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/interrupt.h>

#include <linux/unistd.h>
#include <linux/syscalls.h>

#include <linux/types.h>
#include <linux/device.h>
#include <linux/miscdevice.h>

#include <linux/usb/ch9.h>
#include <linux/usb/android_composite.h>
#include <linux/kmod.h>

#include <linux/kernel.h>
#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/uaccess.h>

MODULE_AUTHOR("Aricent");
MODULE_DESCRIPTION("MTP Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

/* Uncomment to enable Debug */
/* #define DEBUG */

#define xprintk(d, level, fmt, args...) \
	{ if ((d)->m_pGadget) dev_printk(level , &(d)->m_pGadget->dev , \
	    fmt , ## args); else printk(fmt , ## args); }

#ifdef DEBUG
#define MTPDBG(dev, fmt, args...) \
		xprintk(dev , KERN_DEBUG , fmt , ## args)
#else
#define MTPDBG(dev, fmt, args...) do { } while (0)
#endif /* DEBUG */

#define MTPERROR(dev, fmt, args...) \
	xprintk(dev , KERN_ERR , fmt , ## args)
#define MTPINFO(dev, fmt, args...) \
	xprintk(dev , KERN_INFO , fmt , ## args)

#define E_MTP_SUCCESS			0
#define E_MTP_FAIL			-1

/* ID for our one configuration. */
#define CONFIG_VALUE			1

/* Buffer Size allocated for each Endpoints*/
#define BULK_BUFFER_SIZE	4096
#define INTR_BUFFER_SIZE	64

#define USB_NUM_ENDPOINTS	3
#define USB_EP_BULK_MAX_SIZE	512
#define USB_EP_INTR_MAX_SIZE	64
#define USB_EP_INTR_INTERVAL	5

/* Number of rx and tx requests to allocate */
#define RX_REQ_MAX		4
#define TX_REQ_MAX		4
#define INTR_REQ_MAX		4

/* Control codes for MTP operations */
#define MTP_CONTROL_CANCEL_TRANSACTION	0x64
#define MTP_CONTROL_DEVICE_RESET	0x66
#define MTP_CONTROL_GET_DEVICE_STATUS	0x67

#define MTP_CONTROL_DEVICE_READY	0x2001
#define MTP_CONTROL_DEVICE_BUSY		0x2019

/* Interrupt code used to communicate events to Application */
#define MTP_INTR_CODE		100

/* MTP Event codes */
enum MTP_EVENT_CODES {
	/* host events */
	enUsbConnected = 1,
	enUsbDisconnected,
	enDeviceReset,
	enCloseSession,
	enCancelTransaction,

	/* Device side events */
	enSdCardInserted ,
	enSdCardRemoved,
	enAppEvent,
};

/* MTP Device status */
enum MTP_DEVICE_STATUS {
	enStatusDeviceReady = 1,
	enStatusDeviceBusy,
};

/* MTP Ioctl Commands first two constants are reserved for Dual mode */

enum MTP_IOCTL_CMD {
	enCmdAdbEnable = 1,
	enCmdAdbDisable,
	enCmdSetDeviceStatus = 3,
	enCmdGetCurrentMtpEvent,
	enCmdSendMtpEvent,
	enCmdNotifyAppEvent,
	enCmdCancelTransaction,

};

static int MtpOpen(struct inode *pInode, struct file *pFile);
static int MtpFasync(int nFileDescriptor, struct file *pFile, int nMode);
static int MtpClose(struct inode *pInode, struct file *pFile);

static ssize_t MtpRead(struct file *pFile, char __user *pBuff,
		size_t nCount, loff_t *pPosition);
static ssize_t MtpWrite(struct file *pFile, const char __user *pBuff,
		size_t nCount, loff_t *pPosition);

static int MtpGadgetBind(struct usb_configuration *c, struct usb_function *f);

static void MtpGadgetUnbind(struct usb_configuration *c,
		struct usb_function *f);

static int MtpGadgetSetup(struct usb_function *f,
		const struct usb_ctrlrequest *pControlRequest);

static void MtpGadgetDisconnect(struct usb_function *f);

static int MtpGadgetSetAlt(struct usb_function *f, unsigned intf,
		unsigned alt);

static int MtpIoctl(struct inode *pInode, struct file *pFile,
		unsigned int uCmd, unsigned long uArg);

/* Device Details Structure */
struct Mtp_Device_Details {
	struct usb_function function;
	struct usb_composite_dev *cdev;

	int m_nConfig;
	int m_nOnline;
	int m_nError;

	int m_nAdbEnable;
	int m_nMtpEvent;
	int m_nAppEvent;
	int m_nDeviceStatus;

	unsigned char *m_pReadBuff;
	unsigned m_uReadCount;

	spinlock_t m_Lock;

	struct usb_gadget *m_pGadget;
	struct usb_request *m_pRequestep0; /* request for ep0 */

	struct usb_ep *m_pEpIn;
	struct usb_ep *m_pEpOut;
	struct usb_ep *m_pEpIntr;
	struct list_head m_TxIdle;
	struct list_head m_IntrIdle;
	struct list_head m_RxIdle;
	struct list_head m_RxDone;
	struct usb_request *m_pReadReq;
	struct fasync_struct *m_AsyncQueue;

	atomic_t m_ReadExcl;
	atomic_t m_WriteExcl;
	atomic_t m_OpenExcl;

	wait_queue_head_t m_Readwq;
	wait_queue_head_t m_Writewq;
	wait_queue_head_t m_Intrwq;
};

/* Used to send/receive Device Status response/Request */
struct DeviceStatus {
	unsigned short wLength;
	unsigned short Code;
};

static const char gszDrvLongName[] = "MTP Gadget Driver";
static const char gszDrvShortName[] = "mtp";

static struct usb_interface_descriptor gMtpInterfaceDesc = {
		.bLength = USB_DT_INTERFACE_SIZE,
		.bDescriptorType = USB_DT_INTERFACE,
		.bInterfaceNumber = 0,
		.bNumEndpoints = USB_NUM_ENDPOINTS,
		.bInterfaceClass = 6,
		.bInterfaceSubClass = 1,
		.bInterfaceProtocol = 1,
};

static struct usb_endpoint_descriptor gMtpHighspeed_IN_Desc = {
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = USB_DIR_IN,
		.bmAttributes = USB_ENDPOINT_XFER_BULK,
		.wMaxPacketSize = __constant_cpu_to_le16(USB_EP_BULK_MAX_SIZE),
};

static struct usb_endpoint_descriptor gMtpHighspeed_OUT_Desc = {
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = USB_DIR_OUT,
		.bmAttributes = USB_ENDPOINT_XFER_BULK,
		.wMaxPacketSize = __constant_cpu_to_le16(USB_EP_BULK_MAX_SIZE),
};

static struct usb_endpoint_descriptor gMtpHighspeed_INTR_IN_Desc = {
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = USB_DIR_IN,
		.bmAttributes = USB_ENDPOINT_XFER_INT,
		.wMaxPacketSize = __constant_cpu_to_le16(USB_EP_INTR_MAX_SIZE),
		.bInterval = USB_EP_INTR_INTERVAL,
};

static struct usb_endpoint_descriptor gMtpFullspeed_IN_Desc = {
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = USB_DIR_IN,
		.bmAttributes = USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor gMtpFullspeed_OUT_Desc = {
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = USB_DIR_OUT,
		.bmAttributes = USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor gMtpFullspeed_INTR_IN_Desc = {
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = USB_DIR_IN,
		.bmAttributes = USB_ENDPOINT_XFER_INT,
		.wMaxPacketSize = __constant_cpu_to_le16(USB_EP_INTR_MAX_SIZE),
		.bInterval = USB_EP_INTR_INTERVAL,
};

static struct usb_descriptor_header *hs_descriptors[] = {
		(struct usb_descriptor_header *) &gMtpInterfaceDesc,
		(struct usb_descriptor_header *) &gMtpHighspeed_IN_Desc,
		(struct usb_descriptor_header *) &gMtpHighspeed_OUT_Desc,
		(struct usb_descriptor_header *) &gMtpHighspeed_INTR_IN_Desc,
		NULL,
};

static struct usb_descriptor_header *fs_descriptors[] = {
		(struct usb_descriptor_header *) &gMtpInterfaceDesc,
		(struct usb_descriptor_header *) &gMtpFullspeed_IN_Desc,
		(struct usb_descriptor_header *) &gMtpFullspeed_OUT_Desc,
		(struct usb_descriptor_header *) &gMtpFullspeed_INTR_IN_Desc,
		NULL,
};

/* file operations for mtp device /dev/mtp */
static const struct file_operations gMtpFileOperations = {
		.owner = THIS_MODULE,
		.read = MtpRead,
		.write = MtpWrite,
		.open = MtpOpen,
		.release = MtpClose,
		.fasync = MtpFasync,
		.ioctl = MtpIoctl,
};

static struct miscdevice gMtpDevice = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = gszDrvShortName,
		.fops = &gMtpFileOperations,
};

/* temporary variable used between MtpOpen() and MtpGadgetBind() */
static struct Mtp_Device_Details *gpMtpDev;

static inline struct Mtp_Device_Details *func_to_dev(struct usb_function *f)
{
	return container_of(f, struct Mtp_Device_Details, function);
}

/*++

 Routine Name :
	 MtpRequestNew

 Routine Description:

	 Creates new USB request & its associated buffer.

 Arguments:

	 pEndPoint -Pointer to Endpoint for which USB request to be allocated.
	 nBufferSize - Size of the Paload buffer.

 Return Value:

	new usb_request

 --*/
static struct usb_request *MtpRequestNew(struct usb_ep *pEndPoint,
		int nBufferSize)
{
	struct usb_request *pUsbRequest = NULL;

	if (NULL == pEndPoint) {
		MTPERROR(gpMtpDev, "NULL == pEndPoint\n");
		return NULL;
	}

	pUsbRequest = usb_ep_alloc_request(pEndPoint, GFP_KERNEL);

	if (NULL == pUsbRequest) {
		MTPERROR(gpMtpDev, "usb_ep_alloc_request() failed\n");
		return NULL;
	}

	/* now allocate buffers for the requests */
	pUsbRequest->buf = kmalloc(nBufferSize, GFP_KERNEL);
	if (NULL == pUsbRequest->buf) {
		usb_ep_free_request(pEndPoint, pUsbRequest);
		MTPERROR(gpMtpDev, "kmalloc() failed\n");
		return NULL;
	}

	return pUsbRequest;
}

/*++

 Routine Name :
	 MtpRequestFree

 Routine Description:

	 Frees USB request & its associated buffer.

 Arguments:

	 pEndPoint -Pointer to Endpoint for which USB request to be allocated.
	 pUsbRequest - Pointer to USB request.

 Return Value:

	 None.

 --*/
static void MtpRequestFree(struct usb_request *pUsbRequest,
		struct usb_ep *pEndPoint)
{
	if (NULL == pEndPoint) {
		MTPERROR(gpMtpDev, "NULL == pEndPoint\n");
		return;
	}

	if (pUsbRequest) {
		kfree(pUsbRequest->buf);
		usb_ep_free_request(pEndPoint, pUsbRequest);
	}
}

/*++

 Routine Name :
	 MtpLock

 Routine Description:

	 Locks the specified variable for exclusive access.

 Arguments:

	 pExcl	- Pointer to variable to be locked.

 Return Value:

	 Error code.

 --*/
static inline int MtpLock(atomic_t *pExcl)
{
	if (!pExcl) {
		MTPERROR(gpMtpDev, "MtpLock: NULL Pointer\n");
		return E_MTP_FAIL;
	}

	if (atomic_inc_return(pExcl) == 1) {
		return E_MTP_SUCCESS;
	} else {
		atomic_dec(pExcl);
		MTPERROR(gpMtpDev, "MtpLock: atomic_inc_return failure\n");
		return E_MTP_FAIL;
	}
}

/*++

 Routine Name :
	 MtpUnlock

 Routine Description:

	 Unlocks the specified variable for exclusive access.

 Arguments:

	 pExcl	- Pointer to variable to be unlocked.

 Return Value:

	 None.

 --*/
static inline void MtpUnlock(atomic_t *pExcl)
{
	if (pExcl)
		atomic_dec(pExcl);
}

/*++

 Routine Name :
	PushUsbRequest

 Routine Description:

	Adds a UsbRequest to the end of list.

 Arguments:

	pDevice - Pointer to device details
	pHead	- Pointer to listhead to which UsbRequest to be added

 Return Value:

	None.

 --*/
void PushUsbRequest(struct Mtp_Device_Details *pDevice, struct list_head *pHead,
		struct usb_request *pUsbRequest)
{
	unsigned long uFlags = 0;

	if (!pDevice || !pHead || !pUsbRequest) {
		MTPERROR(gpMtpDev, "MtpLock: NULL Pointer\n");
		return;
	}

	spin_lock_irqsave(&pDevice->m_Lock, uFlags);

	list_add_tail(&pUsbRequest->list, pHead);

	spin_unlock_irqrestore(&pDevice->m_Lock, uFlags);
}

/*++

 Routine Name :
	PullUsbRequest

 Routine Description:

	Removes a UsbRequest from the end of Queue.

 Arguments:

	pDevice - Pointer to device details
	pHead	- Pointer to listhead to which UsbRequest to be added

 Return Value:

	UsbRequest.

 --*/
struct usb_request *PullUsbRequest(struct Mtp_Device_Details *pDevice,
		struct list_head *pHead)
{
	unsigned long uFlags = 0;
	struct usb_request *pUsbRequest;

	spin_lock_irqsave(&pDevice->m_Lock, uFlags);

	if (list_empty(pHead)) {
		pUsbRequest = 0;
	} else {
		pUsbRequest = list_first_entry(pHead, struct usb_request,
					list);
		list_del(&pUsbRequest->list);
	}

	spin_unlock_irqrestore(&pDevice->m_Lock, uFlags);

	return pUsbRequest;
}

/*++
 Routine Name :
	MtpCompleteEP_Intr

 Routine Description:
	Callback function for EP_Intr.Called when a EP_Intr request completes.

 Arguments:
	pEndPoint	- Pointer to EndPoint
	pUsbRequest	- Pointer to UsbRequest

 Return Value:
	None.

 --*/
static void MtpCompleteEP_Intr(struct usb_ep *pEndPoint,
		struct usb_request *pUsbRequest)
{
	/* No Implementation. May be used in future */
	struct Mtp_Device_Details *pDevice = NULL;

	MTPDBG(gpMtpDev, "MtpCompleteEP_Intr: IN\n");

	pDevice = pEndPoint->driver_data;
	if (pUsbRequest->status != 0) {
		MTPERROR(gpMtpDev, "MtpCompleteEP_Intr: Intr pkt failure\n");
		pDevice->m_nError = 1;
	}

	PushUsbRequest(pDevice, &pDevice->m_IntrIdle, pUsbRequest);

	wake_up(&pDevice->m_Intrwq);
	MTPDBG(gpMtpDev, "MtpCompleteEP_Intr: OUT\n");
}

/*++
 Routine Name :
	MtpCompleteIN

 Routine Description:
	Callback function for EP IN.Called when a EP IN request completes.
	It adds used UsbRequest to m_TxIdle list and wakeup waiting threads.

 Arguments:
	pEndPoint	- Pointer to EndPoint
	pUsbRequest	- Pointer to UsbRequest

 Return Value:
	None.

 --*/
static void MtpCompleteIN(struct usb_ep *pEndPoint,
		struct usb_request *pUsbRequest)
{
	struct Mtp_Device_Details *pDevice = NULL;

	pDevice = pEndPoint->driver_data;
	if (pUsbRequest->status != 0)
		pDevice->m_nError = 1;

	PushUsbRequest(pDevice, &pDevice->m_TxIdle, pUsbRequest);

	wake_up(&pDevice->m_Writewq);
}

/*++
 Routine Name :
	MtpCompleteOUT

 Routine Description:
	Callback function for EP OUT.Called when a EP OUT request
	completes. It adds the used UsbRequest to m_RxIdle or m_RxDone
	list and wakeup waiting threads.

 Arguments:
	pEndPoint	- Pointer to EndPoint
	pUsbRequest	- Pointer to UsbRequest

 Return Value:
	None.

 --*/
static void MtpCompleteOUT(struct usb_ep *pEndPoint,
		struct usb_request *pUsbRequest)
{
	struct Mtp_Device_Details *pDevice = NULL;

	pDevice = pEndPoint->driver_data;

	if (pUsbRequest->status != 0) {
		pDevice->m_nError = 1;
		PushUsbRequest(pDevice, &pDevice->m_RxIdle, pUsbRequest);
	} else {
		PushUsbRequest(pDevice, &pDevice->m_RxDone, pUsbRequest);
	}

	wake_up(&pDevice->m_Readwq);
}

/*++
 Routine Name :
	MtpSetConfiguration

 Routine Description:
	Function sets the configuration as per host request.

 Arguments:
	pDevice		- Pointer to device
	nConfig		- Configuration to set
	nSpeed		- HighSpeed/FullSpeed

 Return Value:
	None.

 --*/
static int MtpSetConfiguration(struct Mtp_Device_Details *pDevice, int nConfig,
		int nSpeed)
{
	int nRetVal = 0;

	MTPDBG(pDevice, "set configuration: %d\n", nConfig);
	if (pDevice->m_nConfig == nConfig) {

		MTPDBG(pDevice, "device already in this configuration %d\n",
				nConfig);

		pDevice->m_nMtpEvent = enUsbConnected;
		/* Notify device event occurence to MTP stack*/
		kill_fasync(&pDevice->m_AsyncQueue, MTP_INTR_CODE, POLL_IN);
		return E_MTP_SUCCESS;
	}

	if (nConfig == CONFIG_VALUE) {
		MTPDBG(pDevice, "Gng to enable end points\n");

		pDevice->m_pEpIn->driver_data = pDevice;

		nRetVal = usb_ep_enable(pDevice->m_pEpIn,
				(nSpeed == USB_SPEED_HIGH ?
						&gMtpHighspeed_IN_Desc :
						&gMtpFullspeed_IN_Desc));

		if (nRetVal) {
			MTPERROR(pDevice, "usb_ep_enable(EPIN) failed, "
					"nRetVal - %d\n", nRetVal);
			return nRetVal;
		}

		pDevice->m_pEpOut->driver_data = pDevice;
		nRetVal = usb_ep_enable(pDevice->m_pEpOut,
				(nSpeed == USB_SPEED_HIGH ?
						&gMtpHighspeed_OUT_Desc :
						&gMtpFullspeed_OUT_Desc));

		if (nRetVal) {
			MTPERROR(pDevice, "usb_ep_enable(EPOUT) failed, "
					"nRetVal - %d\n", nRetVal);
			return nRetVal;
		}

		pDevice->m_pEpIntr->driver_data = pDevice;
		nRetVal = usb_ep_enable(pDevice->m_pEpIntr,
				(nSpeed == USB_SPEED_HIGH ?
						&gMtpHighspeed_INTR_IN_Desc :
						&gMtpFullspeed_INTR_IN_Desc));

		if (nRetVal) {
			MTPERROR(pDevice, "usb_ep_enable(EP_INTR) failed, "
					"nRetVal - %d\n", nRetVal);
			return nRetVal;
		}

		pDevice->m_nOnline = 1;
		pDevice->m_nMtpEvent = enUsbConnected;
		/* Notify device event occurence to MTP stack*/
		kill_fasync(&pDevice->m_AsyncQueue, MTP_INTR_CODE, POLL_IN);
		MTPDBG(pDevice, "******End points enabled success - "
				"Device Onlibne state****\n");

	} else {
		MTPDBG(pDevice, "Gng to disable end points\n");
		pDevice->m_nMtpEvent = enUsbDisconnected;
		pDevice->m_nOnline = 0;
		pDevice->m_nError = 1;
		usb_ep_disable(pDevice->m_pEpIn);
		usb_ep_disable(pDevice->m_pEpOut);
		usb_ep_disable(pDevice->m_pEpIntr);
	}

	pDevice->m_nConfig = nConfig;

	/* readers may be blocked waiting for us to go online */
	wake_up(&pDevice->m_Readwq);
	return E_MTP_SUCCESS;
}

/*++
 Routine Name :
	MtpCreateBulkEndpoints

 Routine Description:
	Creates Bulk Endpoints, EP_IN, EP_OUT, EP_INTR.
	Allocates MAX_BUFFERS = 4 for BULK_IN, BULK_OUT endpoints.

 Arguments:
	pGadget		 - Pointer to Gadget
	pDevice		 - Pointer to device
	pEP_InDesc	 - Pointer to EP_IN Description
	pEP_OutDesc	 - Pointer to EP_OUT Description
	pEP_IntrDesc - Pointer to EP_INTR Description

 Return Value:
	Error code.

 --*/

static int __init MtpCreateBulkEndpoints(struct usb_composite_dev *cdev,
		struct Mtp_Device_Details *pDevice,
		struct usb_endpoint_descriptor *pEP_InDesc,
		struct usb_endpoint_descriptor *pEP_OutDesc,
		struct usb_endpoint_descriptor *pEP_IntrDesc)
{
	struct usb_request *pUsbRequest = NULL;
	struct usb_ep *pEndPoint = NULL;
	int i = 0;

	if (!cdev || !pDevice || !pEP_InDesc ||
			!pEP_IntrDesc || !pEP_OutDesc) {
		printk(KERN_ERR "MTP:Error:MtpCreateBulkEndpoints "
				"NULL Pointer\n");
		return E_MTP_FAIL;
	}

	MTPDBG(pDevice, "MtpCreateBulkEndpoints\n");

	pEndPoint = usb_ep_autoconfig(cdev->gadget, pEP_InDesc);

	if (!pEndPoint) {
		MTPDBG(pDevice, "usb_ep_autoconfig for ep_in failed\n");
		return -ENODEV;
	}

	MTPDBG(pDevice, "usb_ep_autoconfig for ep_in got %s\n",
			pEndPoint->name);

	pEndPoint->driver_data = pDevice;
	pDevice->m_pEpIn = pEndPoint;

	pEndPoint = usb_ep_autoconfig(cdev->gadget, pEP_OutDesc);
	if (!pEndPoint) {
		MTPERROR(pDevice, "usb_ep_autoconfig for ep_out failed\n");
		return -ENODEV;
	}

	MTPDBG(pDevice, "usb_ep_autoconfig for mtp ep_out got %s\n",
			pEndPoint->name);

	pEndPoint->driver_data = pDevice;
	pDevice->m_pEpOut = pEndPoint;

	pEndPoint = usb_ep_autoconfig(cdev->gadget, pEP_IntrDesc);
	if (!pEndPoint) {
		MTPERROR(pDevice, "usb_ep_autoconfig for m_pEpIntr failed\n");
		return -ENODEV;
	}

	MTPDBG(pDevice, "usb_ep_autoconfig for m_pEpIntr got %s\n",
			pEndPoint->name);

	pEndPoint->driver_data = pDevice;
	pDevice->m_pEpIntr = pEndPoint;

	/* now allocate requests for interupt endpoints */
	for (i = 0; i < INTR_REQ_MAX; i++) {
		pUsbRequest = MtpRequestNew(pDevice->m_pEpIntr,
				INTR_BUFFER_SIZE);

		if (!pUsbRequest) {
			MTPERROR(pDevice, "EP_OUT buffer allocation failed\n");
			goto fail;
		}
		pUsbRequest->complete = MtpCompleteEP_Intr;
		PushUsbRequest(pDevice, &pDevice->m_IntrIdle, pUsbRequest);
	}

	/* now allocate requests for our endpoints */
	for (i = 0; i < RX_REQ_MAX; i++) {
		pUsbRequest = MtpRequestNew(pDevice->m_pEpOut,
				BULK_BUFFER_SIZE);

		if (!pUsbRequest) {
			MTPERROR(pDevice, "EP_OUT buffer allocation failed\n");
			goto fail;
		}
		pUsbRequest->complete = MtpCompleteOUT;
		PushUsbRequest(pDevice, &pDevice->m_RxIdle, pUsbRequest);
	}

	for (i = 0; i < TX_REQ_MAX; i++) {
		pUsbRequest = MtpRequestNew(pDevice->m_pEpIn,
				BULK_BUFFER_SIZE);

		if (!pUsbRequest) {
			MTPERROR(pDevice, "EP_IN buffer allocation failed\n");
			goto fail;
		}
		pUsbRequest->complete = MtpCompleteIN;
		PushUsbRequest(pDevice, &pDevice->m_TxIdle, pUsbRequest);
	}

	return E_MTP_SUCCESS;

fail:
	printk(KERN_ERR "MTP:mtp_bind() could not allocate requests\n");
	return E_MTP_FAIL;
}

/*++
 Routine Name :
	MtpGadgetUnbind

 Routine Description:
	Changes the device to unknown configuration. Releases
	buffers associated with each End Point. Sets the device to error state.

 Arguments:
	pGadget - Pointer to Gadget

 Return Value:
	None.

 --*/

static void MtpGadgetUnbind(struct usb_configuration *c,
		struct usb_function *f)
{
	struct Mtp_Device_Details *pDevice = NULL;
	struct usb_request *pUsbRequest = NULL;
	int nRetval = 0;

	if (!c || !f) {
		MTPERROR(gpMtpDev, "Error: MtpGadgetUnbind() NULL Pointer\n");
		return;
	}
	pDevice = func_to_dev(f);

	MTPDBG(pDevice, "MtpGadgetUnbind\n");

	MtpSetConfiguration(pDevice, 0, USB_SPEED_UNKNOWN);

	while ((pUsbRequest = PullUsbRequest(pDevice, &pDevice->m_RxIdle)))
		MtpRequestFree(pUsbRequest, pDevice->m_pEpOut);

	while ((pUsbRequest = PullUsbRequest(pDevice, &pDevice->m_TxIdle)))
		MtpRequestFree(pUsbRequest, pDevice->m_pEpIn);

	while ((pUsbRequest = PullUsbRequest(pDevice, &pDevice->m_IntrIdle)))
		MtpRequestFree(pUsbRequest, pDevice->m_pEpIntr);

	pDevice->m_nOnline = 0;
	pDevice->m_nError = 1;

	nRetval = misc_deregister(&gMtpDevice);
	if (0 != nRetval)
		MTPDBG(gpMtpDev, "misc_deregister failed\n");

	if (NULL != gpMtpDev) {
		kfree(gpMtpDev);
		gpMtpDev = NULL;
	}
}

/*++
 Routine Name :
	MtpGadgetBind

 Routine Description:
	It configures the bulk endpoints & Calls MtpCreateBulkEndpoints()
	to create BulkEndpoints. It also sets the device to self powered state.

 Arguments:
	pGadget	 - Pointer to Gadget

 Return Value:
	Error Code.

 --*/
static int MtpGadgetBind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = NULL;
	struct Mtp_Device_Details *pDevice = NULL;
	int nRetVal = 0;
	int id;

	if (!c || !f) {
		printk(KERN_ERR "MTP:Error: MtpGadgetBind() NULL param\n");
		return E_MTP_FAIL;
	}
	cdev = c->cdev;
	pDevice = func_to_dev(f);

	MTPDBG(pDevice, "MtpGadgetBind\n");

	MTPDBG(pDevice, "MtpGadgetBind MtpCreateBulkEndpoints\n");

	nRetVal = MtpCreateBulkEndpoints(cdev, pDevice, &gMtpFullspeed_IN_Desc,
			&gMtpFullspeed_OUT_Desc, &gMtpFullspeed_INTR_IN_Desc);
	if (nRetVal != 0) {
		MTPERROR(pDevice, "Error: MtpGadgetBind() - "
				"MtpCreateBulkEndpoints() failed\n");
		return nRetVal;
	}

	/* allocate interface ID(s) */
	id = usb_interface_id(c, f);
	if (id < 0)
		return id;
	gMtpInterfaceDesc.bInterfaceNumber = id;

	/* copy endpoint addresses computed by usb_ep_autoconfig()
	 * to the high speed descriptors
	 */
	if (gadget_is_dualspeed(cdev->gadget)) {
		/* Assume endpoint addresses are the same for both speeds */
		gMtpHighspeed_IN_Desc.bEndpointAddress
				= gMtpFullspeed_IN_Desc.bEndpointAddress;
		gMtpHighspeed_OUT_Desc.bEndpointAddress
				= gMtpFullspeed_OUT_Desc.bEndpointAddress;
		gMtpHighspeed_INTR_IN_Desc.bEndpointAddress
				= gMtpFullspeed_INTR_IN_Desc.bEndpointAddress;

		f->hs_descriptors = hs_descriptors;
	}

	return E_MTP_SUCCESS;
}

/*++
 Routine Name :
	MtpGadgetSetup

 Routine Description:
	Handles all control commands from Host and sends appropriate
	response to the host through EP0 buffer.

 Arguments:
	pGadget		- Pointer to Gadget.
	pControlRequest - Pointer to control request.

 Return Value:
	Error Code.

 --*/
static int MtpGadgetSetup(struct usb_function *f,
		const struct usb_ctrlrequest *pControlRequest)
{
	struct Mtp_Device_Details *pDevice = NULL;
	struct usb_composite_dev *cdev = NULL;
	u8 uRequestType = 0;
	u8 uDirection = 0;
	u8 uRequest = 0;
	u16 uIndex = 0;
	u16 uValue = 0;
	u16 uLength = 0;
	int nResult = -EOPNOTSUPP;

	struct usb_request *pUsbRequest = NULL;

	if (!f || !pControlRequest) {
		printk(KERN_ERR "MTP:Error:MtpGadgetSetup()-NULLpointer\n");
		return -EINVAL;
	}
	pDevice = func_to_dev(f);
	cdev = pDevice->cdev;
	pUsbRequest = cdev->req;

	uRequestType = (pControlRequest->bRequestType & USB_TYPE_MASK);
	uDirection = (pControlRequest->bRequestType & USB_DIR_IN);
	uRequest = pControlRequest->bRequest;
	uIndex = le16_to_cpu(pControlRequest->wIndex);
	uValue = le16_to_cpu(pControlRequest->wValue);
	uLength = le16_to_cpu(pControlRequest->wLength);

	MTPDBG(pDevice, "SETUP type: %02x, uRequest: %02x, uIndex: %04x, "
			"uValue: %04x, uLength: %04x\n",
			pControlRequest->bRequestType,
			uRequest, uIndex, uValue, uLength);

	switch (uRequest) {
	case MTP_CONTROL_GET_DEVICE_STATUS: /* get device status */
	{
		struct DeviceStatus *pDevStatus =
				(struct DeviceStatus *)pUsbRequest->buf;

		pDevStatus->wLength = sizeof(struct DeviceStatus);

		if (pDevice->m_nDeviceStatus == enStatusDeviceBusy)
			pDevStatus->Code = MTP_CONTROL_DEVICE_BUSY;
		else
			pDevStatus->Code = MTP_CONTROL_DEVICE_READY;

		nResult = sizeof(struct DeviceStatus);
		break;
	}

	case MTP_CONTROL_CANCEL_TRANSACTION: /* cancel request */
	{
		struct usb_request *pUsbReq;
		pDevice->m_nMtpEvent = enCancelTransaction;
		pDevice->m_nDeviceStatus = enStatusDeviceBusy; /* device busy */

		MTPDBG(pDevice, "Before cancelling trasaction\n");

		/* clear all rx done buffer and put to idle buffer */
		while ((pUsbReq = PullUsbRequest(pDevice,
						&pDevice->m_RxDone))) {
			MTPDBG(pDevice, "submitting buffer to idle queue\n");
			PushUsbRequest(pDevice, &pDevice->m_RxIdle, pUsbReq);
		}
		usb_ep_fifo_flush(pDevice->m_pEpOut);
		/*usb_ep_fifo_flush(pDevice->m_pEpIn);
		 usb_ep_fifo_flush(pDevice->m_pEpIntr);*/

		kill_fasync(&pDevice->m_AsyncQueue, MTP_INTR_CODE, POLL_IN);
		wake_up(&pDevice->m_Readwq);

		pDevice->m_nError = 0;
		pDevice->m_nDeviceStatus = enStatusDeviceReady;
		/* nResult = 0; */
		MTPDBG(pDevice, "After kill_fasync of cancel transaction\n");
		break;
	}

	case MTP_CONTROL_DEVICE_RESET: /* device reset */
	{
		pDevice->m_nMtpEvent = enDeviceReset;
		pDevice->m_nDeviceStatus = enStatusDeviceBusy; /* device busy */
		kill_fasync(&pDevice->m_AsyncQueue, MTP_INTR_CODE, POLL_IN);
		wake_up(&pDevice->m_Readwq);
		nResult = 0;
		break;
	}

	default:
		MTPERROR(pDevice, "Unsupported SETUP type: %02x, uRequest:"
			"%02x, uIndex: %04x, uValue: %04x, uLength: %04x\n",
			pControlRequest->bRequestType,
			uRequest, uIndex, uValue, uLength);
	}

	/* send response */
	if (nResult >= 0) {
		pUsbRequest->length = min(uLength, (u16)nResult);

		if (nResult < uLength &&
				(nResult % cdev->gadget->ep0->maxpacket) == 0) {
			pUsbRequest->zero = 1;
		} else {
			pUsbRequest->zero = 0;
		}

		nResult = usb_ep_queue(cdev->gadget->ep0,
				 pUsbRequest, GFP_ATOMIC);

		if (nResult < 0) {
			MTPERROR(pDevice, "usb_ep_queue returned %d\n",
				nResult);
			pUsbRequest->status = 0;
		}
	}

	return nResult;
}

/*++

 Routine Name :
	MtpGadgetDisconnect

 Routine Description:
	USB controller calls this function when device is disconnected.
	It changes driver configuration to UNKNOWN by calling SetConfiguration.
	Also it changes the device state to Offline.
	It notifies the device status to Application.

 Arguments:
	pGadget - Pointer to USB gadget.

 Return Value:
	None.

 --*/
static void MtpGadgetDisconnect(struct usb_function *f)
{
	struct Mtp_Device_Details *pDevice = NULL;

	if (!f) {
		printk(KERN_ERR "MTP:MtpGadgetDisconnect: NULL param\n");
		return;
	}
	pDevice = func_to_dev(f);

	/* Change device configuration to unknown and state to Offline*/
	pDevice->m_nOnline = 0;
	MtpSetConfiguration(pDevice, 0, USB_SPEED_UNKNOWN);

}

/*++

 Routine Name :
	MtpRead

 Routine Description:
	This function used to read the data from the
	gadget driver to user space buffer. It gets a free BULK_OUT buffer from
	pDevice->m_RxIdle list and submits a data request to USB controller.
	If data already exist in kernel buffer it copies to Userspace buffer.
	If the requested bytes are transferred to Userspace buffer,
	function exits else it waits for data callback from usb controller.

 Arguments:

	pFile	- File pointer
	pBuff	- Pointer to Buffer for transmission
	nCount	- Number of bytes to transfer
	pPosition - Offset


 Return Value:
	Actual Number of bytes read from USB host or error code.

 --*/

static ssize_t MtpRead(struct file *pFile, char __user *pBuff,
		size_t nCount, loff_t *pPosition)
{
	struct Mtp_Device_Details *pDevice = NULL;
	struct usb_request *pUsbRequest = NULL;
	int nRetVal = nCount;
	int nTemp = 0;
	int nTempRetVal = 0;

	pDevice = pFile->private_data;

	if (!pBuff || !pDevice) {
		MTPERROR(gpMtpDev, "Returning error: Invalid argument\n");
		return -EINVAL;
	}

	MTPDBG(pDevice, "MtpRead(%d)\n", nCount);

	/* Lock m_ReadExcl for exclusive read*/
	if (MtpLock(&pDevice->m_ReadExcl)) {
		MTPERROR(pDevice, "MtpRead return -EBUSY\n");
		return -EBUSY;
	}

	/* we will block until we're online */
	while (!(pDevice->m_nOnline || pDevice->m_nError)) {
		MTPDBG(pDevice, "MtpRead: waiting for online state\n");
		nTempRetVal = wait_event_interruptible(pDevice->m_Readwq,
				(pDevice->m_nOnline || pDevice->m_nError));

		if (nTempRetVal < 0) {
			MtpUnlock(&pDevice->m_ReadExcl);
			MTPERROR(pDevice, "MtpRead return m_nError %d - "
					"device not online\n", nTempRetVal);
			return nTempRetVal;
		}
	}

	while (nCount > 0) {
		if (pDevice->m_nError) {
			nRetVal = -EIO;
			MTPERROR(pDevice, "MtpRead return -EIO\n");
			break;
		}

		/* if we have idle read requests, get them queued */
		while ((pUsbRequest = PullUsbRequest(pDevice,
						&pDevice->m_RxIdle))) {
requeue_req:
			pUsbRequest->length = BULK_BUFFER_SIZE;

			nTempRetVal = usb_ep_queue(pDevice->m_pEpOut,
						pUsbRequest, GFP_ATOMIC);

			if (nTempRetVal < 0) {

				nRetVal = -EIO;
				pDevice->m_nError = 1;

				PushUsbRequest(pDevice, &pDevice->m_RxIdle,
					pUsbRequest);

				MTPERROR(pDevice, "MtpRead: usb_ep_queue() "
						"failed returning -EIO\n");
				goto fail;

			} else {
				MTPDBG(pDevice, "rx %p queue\n", pUsbRequest);
			}
		}

		/* if we have data pending, give it to userspace */
		if (pDevice->m_uReadCount > 0) {

			if (pDevice->m_uReadCount < nCount)
				nTemp = pDevice->m_uReadCount;
			else
				nTemp = nCount;

			if (copy_to_user(pBuff, pDevice->m_pReadBuff, nTemp)) {
				nRetVal = -EFAULT;

				MTPERROR(pDevice, "MtpRead: copy_to_user() "
					" failed return -EFAULT\n");
				break;
			}

			pDevice->m_pReadBuff += nTemp;
			pDevice->m_uReadCount -= nTemp;
			pBuff += nTemp;
			nCount -= nTemp;

			/* if we've emptied the buffer, release the request */
			if (pDevice->m_uReadCount == 0) {
				PushUsbRequest(pDevice, &pDevice->m_RxIdle,
						pDevice->m_pReadReq);

				pDevice->m_pReadReq = 0;
			}
			continue;
		}

		/* wait for a request to complete */
		pUsbRequest = NULL;
		MTPDBG(pDevice, "Wait for request to complete\n");

		nTempRetVal = wait_event_interruptible(pDevice->m_Readwq,
					((pUsbRequest = PullUsbRequest(pDevice,
							&pDevice->m_RxDone)) ||
					pDevice->m_nError ||
					((enCancelTransaction ==
						pDevice->m_nMtpEvent) ||
					(enDeviceReset ==
						pDevice->m_nMtpEvent)) ||
					(1 == pDevice->m_nAppEvent)));

		if (pUsbRequest != NULL) {
			/* if we got a 0-len one we need to put it back into
			** service.  if we made it the current read
			** pUsbRequest we'd be stuck forever
			*/
			if (pUsbRequest->actual == 0)
				goto requeue_req;

			pDevice->m_pReadReq = pUsbRequest;
			pDevice->m_uReadCount = pUsbRequest->actual;
			pDevice->m_pReadBuff = pUsbRequest->buf;

			MTPDBG(pDevice, "rx %p %d\n", pUsbRequest,
					pUsbRequest->actual);
		}

		if ((enCancelTransaction == pDevice->m_nMtpEvent) ||
				(enDeviceReset == pDevice->m_nMtpEvent)) {
			MTPDBG(pDevice, "mtp_read returning due to "
				"m_nMtpEvent - %d\n", pDevice->m_nMtpEvent);

			nRetVal = 0;
			pDevice->m_uReadCount = 0;
			break;
		}

		if (pDevice->m_nAppEvent == 1) {
			MTPDBG(pDevice, "mtp_read returning for App_Event\n");
			nRetVal = 0;
			pDevice->m_nAppEvent = 0;
			break;
		}

		if (nTempRetVal < 0) {
			nRetVal = nTempRetVal;
			break;
		}
	}

	MtpUnlock(&pDevice->m_ReadExcl);
	return nRetVal;

fail:
	MTPERROR(pDevice, "Read failed\n");
	MtpUnlock(&pDevice->m_ReadExcl);
	return nRetVal;
}

/*++

 Routine Name :
	MtpWrite

 Routine Description:
	The purpose of this fuction is to write the data from the
	user space buffer to gadget driver. It gets a free BULK_IN buffer
	from pDevice->m_TxIdle list and submits a data transfer request
	to USB controller. Copies data to kernel buffer from Userspace buffer
	and submits the buffer to USB controller.
	Procedure continues till the requested bytes are transferred .

 Arguments:

	pFile	- File pointer
	pBuff	- Pointer to Buffer for transmission
	nCount	- Number of bytes to transfer
	pPosition - Offset


 Return Value:
	Actual Number of bytes send to USB host.

 --*/
static ssize_t MtpWrite(struct file *pFile, const char __user *pBuff,
		size_t nCount, loff_t *pPosition) {
	struct Mtp_Device_Details *pDevice = NULL;
	struct usb_request *pUsbRequest = NULL;
	int nRetVal = nCount;
	int nTemp = 0;
	int nTempRetVal = 0;

	pDevice = pFile->private_data;

	if (!pBuff || !pDevice) {
		MTPERROR(gpMtpDev, "Returning error: Invalid argument\n");
		return -EINVAL;
	}

	MTPDBG(pDevice, "MtpWrite(%d)\n", nCount);

	if ((enCancelTransaction == pDevice->m_nMtpEvent) ||
			(enDeviceReset == pDevice->m_nMtpEvent)) {
		MTPDBG(pDevice, "mtp_write returning due to "
				"Event Code %d\n", pDevice->m_nMtpEvent);

		return E_MTP_SUCCESS;
	}

	/*Lock m_WriteExcl for exclusive write*/
	if (MtpLock(&pDevice->m_WriteExcl)) {
		MTPERROR(pDevice, "Returning error: Device Busy\n");
		return -EBUSY;
	}

	while (nCount > 0) {
		if (pDevice->m_nError) {
			nRetVal = -EIO;
			MTPERROR(pDevice, "MtpRead return -EIO\n");
			break;
		}

		pUsbRequest = NULL;
		/* Wait till we get an idle tx request to use or error state*/
		nTempRetVal = wait_event_interruptible(pDevice->m_Writewq,
				((pUsbRequest = PullUsbRequest(pDevice,
							&pDevice->m_TxIdle)) ||
				pDevice->m_nError));

		if (nTempRetVal < 0) {
			nRetVal = nTempRetVal;

			MTPERROR(pDevice, "MtpRead: Error "
				"wait_event_interruptible() %d\n", nTempRetVal);

			break;
		}

		if (pUsbRequest != NULL) {

			if (nCount > BULK_BUFFER_SIZE)
				nTemp = BULK_BUFFER_SIZE;
			else
				nTemp = nCount;

			/* Copy data from Userspace buffer to kernel buffer */
			if (copy_from_user(pUsbRequest->buf, pBuff, nTemp)) {
				nRetVal = -EFAULT;
				MTPERROR(pDevice, "MtpRead return -EFAULT\n");
				break;
			}

			pUsbRequest->length = nTemp;

			/* Submitting usbrequest-transfer data to BulkIn EP*/
			nTempRetVal = usb_ep_queue(pDevice->m_pEpIn,
					pUsbRequest, GFP_ATOMIC);

			if (nTempRetVal < 0) {
				MTPERROR(pDevice, "MtpWrite: usb_ep_queue() "
				"error nTempRetVal - %d\n", nTempRetVal);

				pDevice->m_nError = 1;
				nRetVal = -EIO;
				break;
			}

			pBuff += nTemp;
			nCount -= nTemp;

			/* zero this so we don't try to free it on error exit*/
			pUsbRequest = NULL;
		}
	}

	/*Add unused UsbRequest to the m_TxIdle queue*/
	if (NULL != pUsbRequest)
		PushUsbRequest(pDevice, &pDevice->m_TxIdle, pUsbRequest);

	/*UnLock m_WriteExcl */
	MtpUnlock(&pDevice->m_WriteExcl);
	return nRetVal;
}

/*++

 Routine Name :
	MtpIoctl

 Routine Description:
	The purpose of this fuction is to handle miscellaneous commands
	required specific to MTP Application.

 Arguments:

	pFile	- File pointer
	pBuff	- Pointer to Buffer for transmission
	uCmd	- Command specified by Application
	uArg	- Optional Argument for the command


 Return Value:
	Actual Number of bytes send to USB host.

 --*/
static int MtpIoctl(struct inode *pInode, struct file *pFile,
		unsigned int uCmd, unsigned long uArg)
{
	int *pEvent;
	struct Mtp_Device_Details *pDevice = NULL;

	pDevice = pFile->private_data;

	MTPDBG(pDevice, "mtp_ioctl cmd - %d\n", uCmd);

	switch (uCmd) {
	case enCmdAdbEnable:
		MTPDBG(pDevice, "ignore enCmdAdbEnable request\n");
		return E_MTP_SUCCESS;

	case enCmdAdbDisable:
		MTPDBG(pDevice, "ignore enCmdAdbDisable request\n");
		return E_MTP_SUCCESS;

	case enCmdSetDeviceStatus:
		if ((uArg == enStatusDeviceReady) ||
			(uArg == enStatusDeviceBusy))
			pDevice->m_nDeviceStatus = (int)uArg;
		else
			MTPDBG(pDevice, "Invalid device status\n");
		break;

	case enCmdGetCurrentMtpEvent: {
		pEvent = (int *)uArg;

		if (NULL == pEvent) {
			MTPDBG(pDevice, "pEvent == NULL\n");
			break;
		}

		if (copy_to_user(pEvent, &(pDevice->m_nMtpEvent),
				sizeof(pDevice->m_nMtpEvent))) {

			MTPDBG(pDevice, "copy_to_user() failed. "
				"returning -EFAULT\n");
			return -EFAULT;
		}
		pDevice->m_nMtpEvent = 0;
		break;
	}

	case enCmdSendMtpEvent: {
		int nOffset = sizeof(unsigned long);
		int nRetVal = -EIO;
		struct usb_request *pUsbRequest = NULL;
		unsigned long uPktSize = 0;
		void *pBuffer = (void *) uArg;

		do {
			if (NULL == pBuffer) {
				MTPDBG(pDevice, "NULL == pBuffer\n");
				break;
			}

			MTPDBG(pDevice, "Waiting to Get Idle Intr buffer\n");

			pUsbRequest = PullUsbRequest(pDevice,
						&pDevice->m_IntrIdle);

			MTPDBG(pDevice, "After Getting Idle Intr buffer\n");

			if (pUsbRequest == NULL) {
				nRetVal = -1;
				MTPERROR(pDevice, "MtpIoctl:NoFreeBuffer()\n");
				break;
			}

			/*  get the size of the pkt */
			if (copy_from_user(pUsbRequest->buf, pBuffer,
					nOffset)) {
				MTPDBG(pDevice, "copy_from_user failed"
						"for pkt size\n");
				break;
			}

			uPktSize = *((unsigned long *)(pUsbRequest->buf));

			MTPDBG(pDevice, "Event Pkt Size uPktSize - %lu\n",
					uPktSize);

			if (uPktSize > INTR_BUFFER_SIZE) {
				MTPDBG(pDevice, "Invalid Buffer size\n");
				break;
			}

			/*  copy the buffer in driver context */
			if (copy_from_user(pUsbRequest->buf + nOffset,
				(pBuffer + nOffset), (uPktSize - nOffset))) {
				MTPDBG(pDevice, "copy_from_user"
						" failed-remaining buffer\n");
				break;
			}

			pUsbRequest->length = uPktSize;

			/*  send it to intr queue */
			nRetVal = usb_ep_queue(pDevice->m_pEpIntr,
					pUsbRequest, GFP_ATOMIC);

			if (nRetVal < 0) {

				MTPDBG(pDevice, "Error in queing data"
						" to intr EP\n");
				break;
			}

			nRetVal = 0;
			MTPDBG(pDevice, "Event sent success\n");

		} while (0);

		return nRetVal;

	}

	case enCmdNotifyAppEvent:
		pDevice->m_nAppEvent = 1;
		wake_up(&pDevice->m_Readwq);
		break;

	case enCmdCancelTransaction: {
		struct usb_request *pUsbReq;

		MTPDBG(pDevice, "Before cancelling trasaction\n");

		/* clear all rx done buffer and put to idle buffer */
		while ((pUsbReq = PullUsbRequest(pDevice,
						&pDevice->m_RxDone))) {
			MTPDBG(pDevice, "submitting buffer to idle queue\n");
			PushUsbRequest(pDevice, &pDevice->m_RxIdle, pUsbReq);
		}
		usb_ep_fifo_flush(pDevice->m_pEpOut);

		pDevice->m_nDeviceStatus = 0;
		pDevice->m_uReadCount = 0;
		pDevice->m_nError = 0;
		break;
	}
	default:
		MTPDBG(pDevice, "No IOCTL Handler returning -EINVAL\n");
		return -EINVAL;

	}

	return E_MTP_SUCCESS;
}

/*++

 Routine Name :
	MtpOpen

 Routine Description:
	Locks the /dev/mtp file for exclusive access.

 Arguments:

	pInode	 - Pointer to Inode(/dev/mtp)
	pFile	 - File pointer

 Return Value:
	Error Code

 --*/
static int MtpOpen(struct inode *pInode, struct file *pFile)
{
	int nRetVal = 0;
	MTPDBG(gpMtpDev, "MtpOpen() enter\n");

	if (!pInode || !pFile)
		return -EINVAL;

	if (0 != MtpLock(&gpMtpDev->m_OpenExcl)) {
		MTPDBG(gpMtpDev, "MtpOpen - already opened - failed\n");
		nRetVal = -EBUSY;
	} else {

		pFile->private_data = gpMtpDev;
		/* clear the error latch */
		gpMtpDev->m_nError = 0;
	}

	MTPDBG(gpMtpDev, "MtpOpen() exit\n");
	return nRetVal;
}

/*++

 Routine Name :
	MtpFasync

 Routine Description:
	Registers with kernel for signalling interrupts to userspace.

 Arguments:

	nFileDescriptor - File descriptor(/dev/mtp)
	pFile			- File pointer
	nMode			- File Opening mode

 Return Value:
	Error Code

 --*/

static int MtpFasync(int nFileDescriptor, struct file *pFile, int nMode)
{
	int nRetVal = 0;
	struct Mtp_Device_Details *pDevice = NULL;

	MTPDBG(gpMtpDev, "MtpFasync() Enter\n");
	do {
		if (NULL == pFile->private_data) {
			MTPDBG(gpMtpDev, "Invalid Argument NULL ptr\n");
			nRetVal = -1;
			break;
		}

		pDevice = pFile->private_data;
		nRetVal = fasync_helper(nFileDescriptor, pFile, nMode,
				&pDevice->m_AsyncQueue);

		if (0 > nRetVal) {
			MTPERROR(pDevice, "fasync_helper() failure\n");
			break;
		}

	} while (false);

	return nRetVal;
}

/*++

 Routine Name :
	mtp_close

 Routine Description:
	Unlocks the file /dev/mtp.

 Arguments:

	pInode - Pointer to inode of the file.
	pFile  - File pointer

 Return Value:

	Error Code

 --*/

int MtpClose(struct inode *pInode, struct file *pFile)
{
	MTPDBG(gpMtpDev, "MtpClose\n");
	MtpUnlock(&gpMtpDev->m_OpenExcl);
	return E_MTP_SUCCESS;
}

static int MtpGadgetSetAlt(struct usb_function *f, unsigned intf, unsigned alt)
{
	struct Mtp_Device_Details *pDevice = NULL;
	struct usb_composite_dev *cdev = NULL;

	if (!f) {
		printk(KERN_ERR "MTP:MtpGadgetSetAlt() NULL param\n");
		return E_MTP_FAIL;
	}
	pDevice = func_to_dev(f);
	cdev = pDevice->cdev;

	return MtpSetConfiguration(pDevice, CONFIG_VALUE, cdev->gadget->speed);
}


/*++

Routine Name :
	mtp_bind_config

 Routine Description:

	Entry point for this module. It initializes all the variables and
	Datastructures.

 Arguments:

	None

 Return Value:

    Error Code

--*/

static int mtp_bind_config(struct usb_configuration *c)
{
	struct Mtp_Device_Details *pDevice = NULL;
	int nRetVal = 0;
	int nDereg = 0;

	printk(KERN_INFO "MTP:android mtp driver\n");

	do {
		/* Allocatin memory to store device details */
		pDevice = kzalloc(sizeof(*pDevice), GFP_KERNEL);
		if (NULL == pDevice) {
			printk(KERN_ERR "MTP: android mtp driver\n");
			return -ENOMEM;
		}

		/* Initialize spinlock */
		spin_lock_init(&pDevice->m_Lock);

		/* Initialize wait queues */
		init_waitqueue_head(&pDevice->m_Readwq);
		init_waitqueue_head(&pDevice->m_Writewq);
		init_waitqueue_head(&pDevice->m_Intrwq);

		atomic_set(&pDevice->m_OpenExcl, 0);
		atomic_set(&pDevice->m_ReadExcl, 0);
		atomic_set(&pDevice->m_WriteExcl, 0);

		INIT_LIST_HEAD(&pDevice->m_RxIdle);
		INIT_LIST_HEAD(&pDevice->m_RxDone);
		INIT_LIST_HEAD(&pDevice->m_TxIdle);
		INIT_LIST_HEAD(&pDevice->m_IntrIdle);

		/*set gpMtpDev before calling usb_gadget_register_driver*/
		gpMtpDev = pDevice;
		gpMtpDev->m_nMtpEvent = 0;

		nRetVal = misc_register(&gMtpDevice);
		if (0 != nRetVal) {
			printk(KERN_ERR "mtp gadget driver misc_register failed\n");
			break;
		}

		gpMtpDev->cdev = c->cdev;
		gpMtpDev->function.name = gszDrvShortName;
		gpMtpDev->function.descriptors = fs_descriptors;
		gpMtpDev->function.bind = MtpGadgetBind;
		gpMtpDev->function.unbind = MtpGadgetUnbind;
		gpMtpDev->function.setup = MtpGadgetSetup;
		gpMtpDev->function.set_alt = MtpGadgetSetAlt;
		gpMtpDev->function.disable = MtpGadgetDisconnect;

		nRetVal = usb_add_function(c, &gpMtpDev->function);
		if (nRetVal != 0) {
			printk(KERN_ERR "mtp_bind_config: usb_add_function failed\n");
			nDereg = 1;
			break;
		}

		printk(KERN_ERR "mtp gadget driver initialize - success\n");
		return E_MTP_SUCCESS;

	} while (0);

	/* ERROR HANDLING */
	if (0 != nDereg)
		misc_deregister(&gMtpDevice);

	if (NULL != pDevice) {
		kfree(pDevice);
		gpMtpDev = NULL;
	}

	printk(KERN_ERR "mtp gadget driver failed to initialize\n");
	return nRetVal;
}

static struct android_usb_function mtp_function = {
	.name = "mtp",
	.bind_config = mtp_bind_config,
};

static int __init mtp_init(void)
{
	printk(KERN_INFO "f_mtp init\n");
	android_register_function(&mtp_function);
	return 0;
}
module_init(mtp_init);
