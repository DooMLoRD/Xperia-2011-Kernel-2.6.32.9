/* /kernel/arch/arm/mach-msm/fpc_test_hallon.c
 *
 * Copyright (C) [2010] Sony Ericsson Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */
#include <linux/kernel.h>
#include <linux/fpc_connector_test.h>

static struct fpc_connector fpc_test_connections[] = {
	{ .pin = 36, .name = "touch", },
	{ .pin = 38, .name = "camera", },
	{ .pin = 39, .name = "keypad", },
};

struct fpc_connections_set fpc_connections_set = {
	.connections = fpc_test_connections,
	.num = ARRAY_SIZE(fpc_test_connections),
};
