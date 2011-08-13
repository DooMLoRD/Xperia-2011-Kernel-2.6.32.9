#ifndef __LINUX_FPC_CONNECTOR_TEST_H
#define __LINUX_FPC_CONNECTOR_TEST_H
struct fpc_connector {
	const char *name;
	int pin;
};

struct fpc_connections_set {
	struct fpc_connector *connections;
	unsigned num;
};

struct fpc_test_platform_data {
	struct fpc_connections_set *fpc;
	int (*fpc_pin_read)(int pin);
};

#define FPC_TEST_DRV_NAME "fpc-connection-test"
#endif
