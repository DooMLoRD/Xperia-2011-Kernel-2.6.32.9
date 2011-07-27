#ifndef _ZEUS_ESD_H_
#define _ZEUS_ESD_H_

#include <linux/platform_device.h>

typedef void (*eventESD_t)(void);

struct platform_esd_data {
	/* Which GPIO and how to detect an event */
	const uint32_t	positive;
	const uint32_t	positive_detect;

	/* Which GPIO and how to detect an event */
	const uint32_t	negative;
	const uint32_t	negative_detect;

	/* Initialization function, sets up the HW so we can sign up for IRQs */
	void (*init)(void);

	/**
	 * Called whenever a ESD event is detected.
	 *
	 * Must contain atleast one entry and the last entry MUST always be
	 * a NULL so the driver is aware of when there are no more functions
	 * to call.
	 */
	eventESD_t eventESD[];
};

#endif
