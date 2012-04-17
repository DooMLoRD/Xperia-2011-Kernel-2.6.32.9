#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>

#include <asm/gpio.h>
#include <asm/io.h>
#include <linux/wl12xx.h>

#define MOGAMI_WIFI_PMENA_GPIO	57
#define MOGAMI_WIFI_IRQ_GPIO	147

static int mogami_wifi_power_state;

int mogami_wifi_power(int on)
{
	if (on && (on == mogami_wifi_power_state))
		return 0;
	if (on) {
		gpio_set_value(MOGAMI_WIFI_PMENA_GPIO, 1);
		mdelay(15);
		gpio_set_value(MOGAMI_WIFI_PMENA_GPIO, 0);
		mdelay(1);
		gpio_set_value(MOGAMI_WIFI_PMENA_GPIO, 1);
		mdelay(70);
	} else {
		gpio_set_value(MOGAMI_WIFI_PMENA_GPIO, 0);
	}
	mogami_wifi_power_state = on;
	return 0;
}

struct wl12xx_platform_data mogami_wlan_data __initdata = {
	.irq = MSM_GPIO_TO_INT(MOGAMI_WIFI_IRQ_GPIO),
	.board_ref_clock = WL12XX_REFCLOCK_19,
	.platform_quirks = WL12XX_PLATFORM_QUIRK_EDGE_IRQ,
};

static int __init mogami_wifi_init(void)
{
	int ret;

	ret = gpio_request(MOGAMI_WIFI_IRQ_GPIO, "wifi_irq");
	if (ret < 0) {
		printk(KERN_ERR "%s: can't reserve GPIO: %d\n", __func__,
			MOGAMI_WIFI_IRQ_GPIO);
		goto out;
	}
	ret = gpio_request(MOGAMI_WIFI_PMENA_GPIO, "wifi_pmena");
	if (ret < 0) {
		printk(KERN_ERR "%s: can't reserve GPIO: %d\n", __func__,
			MOGAMI_WIFI_PMENA_GPIO);
		gpio_free(MOGAMI_WIFI_IRQ_GPIO);
		goto out;
	}
	gpio_direction_input(MOGAMI_WIFI_IRQ_GPIO);
	gpio_direction_output(MOGAMI_WIFI_PMENA_GPIO, 0);
	if (wl12xx_set_platform_data(&mogami_wlan_data))
		pr_err("error setting wl12xx data\n");
out:
	return ret;
}

device_initcall(mogami_wifi_init);
