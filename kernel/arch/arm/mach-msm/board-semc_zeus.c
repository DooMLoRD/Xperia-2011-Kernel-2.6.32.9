/*
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/bootmem.h>
#include <linux/io.h>
#include <linux/spi/spi.h>
#include <linux/mfd/pmic8058.h>
#include <linux/mfd/marimba.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/smsc911x.h>
#include <linux/ofn_atlab.h>
#include <linux/power_supply.h>
#include <linux/input/pmic8058-keypad.h>
#include <linux/pwm.h>
#include <linux/pmic8058-pwm.h>
#include <linux/leds-pmic8058.h>
#include <linux/msm_adc.h>
#include <linux/hrtimer.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>

#include <mach/bcm_bt_lpm.h>
#include <mach/msm_serial_hs.h>
#include <mach/mpp.h>
#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/camera.h>
#include <mach/memory.h>
#include <mach/msm_iomap.h>
#include <mach/msm_hsusb.h>
#include <mach/rpc_hsusb.h>
#include <mach/msm_spi.h>
#include <mach/qdsp5v2/msm_lpa.h>
#include <mach/qdsp5v2/mi2s.h>
#include <mach/qdsp5v2/audio_dev_ctl.h>
#include <mach/dma.h>
#include <linux/android_pmem.h>
#include <mach/pmic.h>
#include <mach/rpc_pmapp.h>
#include <mach/qdsp5v2/aux_pcm.h>
#include <mach/msm_battery.h>
#include <mach/msm_tsif.h>

#include <asm/mach/mmc.h>
#include <asm/mach/flash.h>
#include <mach/vreg.h>
#include "devices.h"
#include "timer.h"
#include "socinfo.h"
#include "cpufreq.h"
#include <linux/usb/android.h>
#ifdef CONFIG_USB_ANDROID_ACCESSORY
#include <linux/usb/f_accessory.h>
#endif
#include "pm.h"
#include "spm.h"
#include <linux/msm_kgsl.h>
#include <mach/dal_axi.h>
#include <mach/msm_serial_hs.h>
#include <mach/msm_reqs.h>

#include <linux/spi/cypress_touch.h>
#include <linux/leds-as3676.h>
#include <linux/gp2ap002a00f.h>
#include <linux/i2c/synaptics_touchpad.h>
#include <linux/bma150_ng.h>
#include <linux/i2c/akm8975.h>
#include <mach/mddi_novatek_fwvga.h>
#include <linux/max17040.h>

#include <linux/if.h>
#include <linux/inet.h>
#include <linux/wlan_plat.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <mach/semc_battery_data.h>

#include <mach/semc_rpc_server_handset.h>
#ifdef CONFIG_SIMPLE_REMOTE_PLATFORM
#include <mach/simple_remote_msm7x30_pf.h>
#endif

#define CYPRESS_TOUCH_GPIO_RESET		40
#define CYPRESS_TOUCH_GPIO_IRQ			42
#define GP2A_GPIO				20
#define SYNAPTICS_TOUCHPAD_GPIO			33
#define BMA150_GPIO				51
#define AKM8975_GPIO				92
#define NOVATEK_GPIO_RESET			157

#define MSM_PMEM_SF_SIZE	0x500000
#define MSM_FB_SIZE		0x500000
#define MSM_GPU_PHYS_SIZE       SZ_2M
#define MSM_PMEM_CAMERA_SIZE    0x2000000
#define MSM_PMEM_ADSP_SIZE      0x1800000
#define PMEM_KERNEL_EBI1_SIZE   0x600000
#define MSM_PMEM_AUDIO_SIZE     0x200000

#define PMIC_GPIO_INT		27
#define PMIC_VREG_WLAN_LEVEL	2900
#define PMIC_GPIO_SD_DET	22
#ifdef CONFIG_PMIC_GPIO_25
#define PMIC_GPIO_SD_POWER	25
#endif

#define BT_GPIO_EN			(103)
#define BT_GPIO_WAKE			(106)
#define BT_GPIO_HOST_WAKE		(18)
#define WIFI_GPIO_EN			(57)
#define WIFI_GPIO_IRQ			(147)

/* Macros assume PMIC GPIOs start at 0 */
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)
#define PM8058_GPIO_SYS_TO_PM(sys_gpio)    (sys_gpio - NR_GPIO_IRQS)

#ifdef CONFIG_ANDROID_RAM_CONSOLE
#define MSM_RAM_CONSOLE_START   (0x50000000 - MSM_RAM_CONSOLE_SIZE)
#define MSM_RAM_CONSOLE_SIZE    (128 * SZ_1K)
#endif

#define USB_VREG_MV		3500	/* usb voltage regulator mV */

/* GPIO hardware device identification */
enum board_hwid {
	BOARD_HWID_UNK,
	BOARD_HWID_DP1,
	BOARD_HWID_SP1,
	BOARD_HWID_DBZ3,
	BOARD_HWID_SP1_5,
	BOARD_HWID_SP1_6,
	BOARD_HWID_DBZ3_1,
	BOARD_HWID_SP2,
	BOARD_HWID_DBZ3_2,
	BOARD_HWID_SP3,
	BOARD_HWID_AP1,
	BOARD_HWID_PQ,
};
static u8 board_hwid;


extern void msm_init_pmic_vibrator(void);

static int vreg_helper_on(const char *pzName, unsigned mv)
{
	struct vreg *reg = NULL;
	int rc = 0;

	reg = vreg_get(NULL, pzName);
	if (reg == NULL) {
		printk(KERN_ERR "Unable to resolve VREG name \"%s\"\n", pzName);
		return rc;
	}

	if (mv != (unsigned int)-1)
		rc = vreg_set_level(reg, mv);

	if (rc) {
		printk(KERN_ERR "Unable to set vreg \"%s\" level\n", pzName);
		return rc;
	}

	rc = vreg_enable(reg);
	if (rc) {
		printk(KERN_ERR "Unable to enable vreg \"%s\" level\n", pzName);
		return rc;
	}

	printk(KERN_INFO "Enabled VREG \"%s\" at %u mV\n", pzName, mv);
	return rc;
}

static void vreg_helper_off(const char *pzName)
{
	struct vreg *reg = NULL;
	int rc;

	reg = vreg_get(NULL, pzName);
	if (reg == NULL) {
		printk(KERN_ERR "Unable to resolve VREG name \"%s\"\n", pzName);
		return;
	}

	rc = vreg_disable(reg);
	if (rc) {
		printk(KERN_ERR "Unable to disable vreg \"%s\" level\n",
		       pzName);
		return;
	}

	printk(KERN_INFO "Disabled VREG \"%s\"\n", pzName);
}

static int pm8058_gpios_init(void)
{
	int rc;

	struct pm8058_gpio sdcc_det = {
		.direction = PM_GPIO_DIR_IN,
		.pull = PM_GPIO_PULL_NO,
		.vin_sel = 2,
		.function = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol = 0,
	};

	rc = pm8058_gpio_config(PMIC_GPIO_SD_DET - 1, &sdcc_det);
	if (rc) {
		pr_err("%s PMIC_GPIO_SD_DET config failed\n", __func__);
		return rc;
	}

	return 0;
}

static const unsigned int keymap_game[] = {
	KEY(7, 0, KEY_VOLUMEUP),   /* DBZ2, VOL_UP */
	KEY(7, 1, KEY_VOLUMEDOWN), /* DBZ2, VOL_DOWN */
	KEY(7, 2, BTN_SELECT),     /* DBZ2, S1 */
	KEY(7, 3, KEY_ENTER),      /* DBZ2, S2 */
};

static struct resource resources_keypad_game[] = {
	{
	 .start = PM8058_KEYPAD_IRQ(PMIC8058_IRQ_BASE),
	 .end = PM8058_KEYPAD_IRQ(PMIC8058_IRQ_BASE),
	 .flags = IORESOURCE_IRQ,
	 },
	{
	 .start = PM8058_KEYSTUCK_IRQ(PMIC8058_IRQ_BASE),
	 .end = PM8058_KEYSTUCK_IRQ(PMIC8058_IRQ_BASE),
	 .flags = IORESOURCE_IRQ,
	 },
};

static struct matrix_keymap_data keymap_game_data = {
	.keymap_size = ARRAY_SIZE(keymap_game),
	.keymap = keymap_game,
};

static struct pmic8058_keypad_data zeus_keypad_data = {
	.input_name = "keypad-game-zeus",
	.input_phys_device = "keypad-game-zeus/input0",
	.num_rows = 8,
	.num_cols = 8,
	.rows_gpio_start = 8,
	.cols_gpio_start = 0,
	.debounce_ms = {8, 10},
	.scan_delay_ms = 32,
	.row_hold_ns = 91500,
	.wakeup = 1,
	.keymap_data = &keymap_game_data,
};

/* Put sub devices with fixed location first in sub_devices array */
#define	PM8058_SUBDEV_KPD	0
#define	PM8058_SUBDEV_LED	1

static struct pm8058_gpio_platform_data pm8058_gpio_data = {
	.gpio_base = PM8058_GPIO_PM_TO_SYS(0),
	.irq_base = PM8058_GPIO_IRQ(PMIC8058_IRQ_BASE, 0),
	.init = pm8058_gpios_init,
};

static struct pm8058_gpio_platform_data pm8058_mpp_data = {
	.gpio_base = PM8058_GPIO_PM_TO_SYS(PM8058_GPIOS),
	.irq_base = PM8058_MPP_IRQ(PMIC8058_IRQ_BASE, 0),
};

static struct mfd_cell pm8058_subdevs[] = {
	{.name = "pm8058-keypad",
	 .id = -1,
	 .num_resources = ARRAY_SIZE(resources_keypad_game),
	 .resources = resources_keypad_game,
	 },
	{.name = "pm8058-gpio",
	 .id = -1,
	 .platform_data = &pm8058_gpio_data,
	 .data_size = sizeof(pm8058_gpio_data),
	 }
	,
	{.name = "pm8058-mpp",
	 .id = -1,
	 .platform_data = &pm8058_mpp_data,
	 .data_size = sizeof(pm8058_mpp_data),
	 }
	,
	{.name = "pm8058-nfc",
	 .id = -1,
	 }
	,
	{	.name = "pm8058-upl",
		.id		= -1,
	},
};

static struct pm8058_platform_data pm8058_7x30_data = {
	.irq_base = PMIC8058_IRQ_BASE,

	.num_subdevs = ARRAY_SIZE(pm8058_subdevs),
	.sub_devices = pm8058_subdevs,
};

static struct i2c_board_info pm8058_boardinfo[] __initdata = {
	{
	 I2C_BOARD_INFO("pm8058-core", 0),
	 .irq = MSM_GPIO_TO_INT(PMIC_GPIO_INT),
	 .platform_data = &pm8058_7x30_data,
	 },
};

static void msm_camera_vreg_enable(void)
{
	vreg_helper_on("gp15", 1200); /* L22 */
	vreg_helper_on("lvsw1", 1800); /* LVS1 */
	vreg_helper_on("gp2", 2800); /* L11 */
}

static void msm_camera_vreg_disable(void)
{
	vreg_helper_off("gp2");
	vreg_helper_off("lvsw1");
	vreg_helper_off("gp15");
}

static uint32_t camera_off_gpio_table[] = {
	/* parallel CAMERA interfaces 5MAF & VT */
	GPIO_CFG(0,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* RST */
	GPIO_CFG(4, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* DAT4 */
	GPIO_CFG(5, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* DAT5 */
	GPIO_CFG(6, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* DAT6 */
	GPIO_CFG(7, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* DAT7 */
	GPIO_CFG(8, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* DAT8 */
	GPIO_CFG(9, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* DAT9 */
	GPIO_CFG(10, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* DAT10 */
	GPIO_CFG(11, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* DAT11 */
	GPIO_CFG(12, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* PCLK */
	GPIO_CFG(13, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* HSYNC_IN */
	GPIO_CFG(14, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* VSYNC_IN */
	GPIO_CFG(15, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA), /* MCLK */
	/* NIRQ */
	GPIO_CFG(21, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(31, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* VT VGA_PWDN */
	GPIO_CFG(143, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* STB */
	GPIO_CFG(104, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* LED_IND */
	GPIO_CFG(105, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* LED_TORCH */
};

static uint32_t camera_on_gpio_table[] = {
	/* parallel CAMERA interfaces 5MAF & VT */
	GPIO_CFG(0, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* RST */
	GPIO_CFG(4, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* DAT4 */
	GPIO_CFG(5, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* DAT5 */
	GPIO_CFG(6, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* DAT6 */
	GPIO_CFG(7, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* DAT7 */
	GPIO_CFG(8, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* DAT8 */
	GPIO_CFG(9, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* DAT9 */
	GPIO_CFG(10, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* DAT10 */
	GPIO_CFG(11, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* DAT11 */
	GPIO_CFG(12, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* PCLK */
	GPIO_CFG(13, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* HSYNC_IN */
	GPIO_CFG(14, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* VSYNC_IN */
	GPIO_CFG(15, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA), /* MCLK */
	/* NIRQ */
	GPIO_CFG(21, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(31, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* VT VGA_PWDN */
	GPIO_CFG(143, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* STB */
	GPIO_CFG(104, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* LED_IND */
	GPIO_CFG(105, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* LED_TORCH */
};

static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}

static void config_camera_on_gpios(void)
{
    gpio_request(31, "vtcam_rst");
    gpio_set_value(31, 1); // SCAM_RST_N
    gpio_free(31);
	config_gpio_table(camera_on_gpio_table,
				ARRAY_SIZE(camera_on_gpio_table));
	msm_camera_vreg_enable();
}

static void config_camera_off_gpios(void)
{
    gpio_request(31, "vtcam_rst");
    gpio_set_value(31, 1); // SCAM_RST_N
    gpio_free(31);
	msm_camera_vreg_disable();

	config_gpio_table(camera_off_gpio_table,
				ARRAY_SIZE(camera_off_gpio_table));
}

struct gpio_led as3685a_leds[] = {
		{
				.name = "indicator", /*MSM_CAMERA_LED_LOW*/
				.default_trigger = 0,
				.gpio = 104,
				.active_low = 0
		},
		{
				.name = "torch", /*MSM_CAMERA_LED_HIGH*/
				.default_trigger = 0,
				.gpio = 105,
				.active_low = 0
		}
};

struct gpio_led_platform_data as3685a_data = {
		.num_leds = ARRAY_SIZE(as3685a_leds),
		.leds = as3685a_leds
};

struct msm_camera_sensor_flash_src seix006_flash_src = {
		.flash_sr_type = MSM_CAMERA_FLASH_SRC_LED,
		._fsrc.gpio_led_src = &as3685a_data
};

struct resource msm_camera_resources[] = {
	{
	 .start = 0xA6000000,
	 .end = 0xA6000000 + SZ_1M - 1,
	 .flags = IORESOURCE_MEM,
	 },
	{
	 .start = INT_VFE,
	 .end = INT_VFE,
	 .flags = IORESOURCE_IRQ,
	 },
};

struct msm_camera_device_platform_data msm_camera_device_data_seix006 = {
	.camera_gpio_on = config_camera_on_gpios,
	.camera_gpio_off = config_camera_off_gpios,
	.ioext.mdcphy = MSM_MDC_PHYS,
	.ioext.mdcsz  = MSM_MDC_SIZE,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
	.ioext.camifpadphy = 0xAB000000,
	.ioext.camifpadsz = 0x00000400,
	.ioext.csiphy = 0xA6100000,
	.ioext.csisz  = 0x00000400,
	.ioext.csiirq = INT_CSI,
	.ioclk.mclk_clk_rate = 19200000,
	.ioclk.vfe_clk_rate  = 122880000,
};

struct msm_camera_device_platform_data msm_camera_device_data_ovm7692 = {
	.camera_gpio_on = config_camera_on_gpios,
	.camera_gpio_off = config_camera_off_gpios,
	.ioext.mdcphy = MSM_MDC_PHYS,
	.ioext.mdcsz  = MSM_MDC_SIZE,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
	.ioext.camifpadphy = 0xAB000000,
	.ioext.camifpadsz = 0x00000400,
	.ioext.csiphy = 0xA6100000,
	.ioext.csisz  = 0x00000400,
	.ioext.csiirq = INT_CSI,
	.ioclk.mclk_clk_rate = 24000000,
	.ioclk.vfe_clk_rate  = 122880000,
};

struct msm_camera_device_platform_data msm_camera_device_data_mt9v114 = {
	.camera_gpio_on = config_camera_on_gpios,
	.camera_gpio_off = config_camera_off_gpios,
	.ioext.mdcphy = MSM_MDC_PHYS,
	.ioext.mdcsz  = MSM_MDC_SIZE,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
	.ioext.camifpadphy = 0xAB000000,
	.ioext.camifpadsz = 0x00000400,
	.ioext.csiphy = 0xA6100000,
	.ioext.csisz  = 0x00000400,
	.ioext.csiirq = INT_CSI,
	.ioclk.mclk_clk_rate = 19200000,
	.ioclk.vfe_clk_rate  = 122880000,
};



#ifdef CONFIG_SEIX006
static struct msm_camera_sensor_flash_data flash_seix006 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &seix006_flash_src
};
static struct msm_camera_sensor_info msm_camera_sensor_seix006_data = {
	.sensor_name    = "seix006",
	.sensor_reset   = 0,
	.sensor_pwd     = 143,
	.vcm_pwd        = 0,
	.pdata          = &msm_camera_device_data_seix006,
	.flash_data     = &flash_seix006,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources)
};

static struct platform_device msm_camera_sensor_seix006 = {
	.name      = "msm_camera_seix006",
	.dev       = {
		.platform_data = &msm_camera_sensor_seix006_data,
	},
};
#endif /*CONFIG_SEIX006 ZEUS 5MAF CAMERA*/

#ifdef CONFIG_OVM7692

static struct msm_camera_sensor_flash_data flash_ovm7692_data = {
	.flash_type = MSM_CAMERA_FLASH_NONE,
	.flash_src  = 0
};

static struct msm_camera_sensor_info msm_camera_sensor_ovm7692_data = {
	.sensor_name    = "ovm7692",
	.sensor_reset   = 0,
	.sensor_pwd     = 31,
	.vcm_pwd        = 0,
	.pdata          = &msm_camera_device_data_ovm7692,
	.flash_data     = &flash_ovm7692_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources)
};

static struct platform_device msm_camera_sensor_ovm7692 = {
	.name      = "msm_camera_ovm7692",
	.dev       = {
		.platform_data = &msm_camera_sensor_ovm7692_data,
	},
};
#endif /*CONFIG_OVM7692 ZEUS VGA VT CAMERA*/

#ifdef CONFIG_MT9V114

static struct msm_camera_sensor_flash_data flash_mt9v114_data = {
	.flash_type = MSM_CAMERA_FLASH_NONE,
	.flash_src  = 0
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9v114_data = {
	.sensor_name    = "mt9v114",
	.sensor_reset   = 0,
	.sensor_pwd     = 31,
	.vcm_pwd        = 0,
	.pdata          = &msm_camera_device_data_mt9v114,
	.flash_data     = &flash_mt9v114_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources)
};

static struct platform_device msm_camera_sensor_mt9v114 = {
	.name      = "msm_camera_mt9v114",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9v114_data,
	},
};
#endif /*CONFIG_MT9V114 ZEUS VGA VT CAMERA*/

static struct resource msm_gemini_resources[] = {
	{
	 .start = 0xA3A00000,
	 .end = 0xA3A00000 + 0x0150 - 1,
	 .flags = IORESOURCE_MEM,
	 },
	{
	 .start = INT_JPEG,
	 .end = INT_JPEG,
	 .flags = IORESOURCE_IRQ,
	 },
};

static struct platform_device msm_gemini_device = {
	.name = "msm_gemini",
	.resource = msm_gemini_resources,
	.num_resources = ARRAY_SIZE(msm_gemini_resources),
};

static struct resource msm_vpe_resources[] = {
	{
		.start	= 0xAD200000,
		.end	= 0xAD200000 + SZ_1M - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_VPE,
		.end	= INT_VPE,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device msm_vpe_device = {
	.name = "msm_vpe",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_vpe_resources),
	.resource = msm_vpe_resources,
};

static uint32_t audio_pamp_gpio_config =
   GPIO_CFG(82, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);

static uint32_t HAC_amp_gpio_config =
   GPIO_CFG(109, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);

static int __init snddev_poweramp_gpio_init(void)
{
	int rc;

	pr_info("snddev_poweramp_gpio_init \n");
	rc = gpio_tlmm_config(audio_pamp_gpio_config, GPIO_CFG_ENABLE);
	if (rc) {
		printk(KERN_ERR
			"%s: gpio_tlmm_config(%#x)=%d\n",
			__func__, audio_pamp_gpio_config, rc);
	}

	/* Enabling HAC amplifier */
	rc = gpio_tlmm_config(HAC_amp_gpio_config, GPIO_CFG_ENABLE);
	if (rc) {
		printk(KERN_ERR
			"%s: gpio_tlmm_config(%#x)=%d\n",
			__func__, HAC_amp_gpio_config, rc);
	}

	/* Make sure we start with a known state */
	msm_snddev_poweramp_off();
	msm_hac_amp_off();

	return rc;
}

void msm_snddev_tx_route_config(void)
{
}

void msm_snddev_tx_route_deconfig(void)
{
}

void msm_hac_amp_on(void)
{
	pr_info("%s: power on HAC amplifier\n", __func__);
	gpio_set_value(109, 1); /* enable HAC poweramp */
}

void msm_hac_amp_off(void)
{
	pr_info("%s: power off HAC amplifier\n", __func__);
	gpio_set_value(109, 0); /* disable HAC poweramp */
}

void msm_snddev_poweramp_on(void)
{
	pr_info("%s: power on amplifier\n", __func__);
	gpio_set_value(82, 1);  /* enable spkr poweramp */
}

void msm_snddev_poweramp_off(void)
{
	pr_info("%s: power off amplifier\n", __func__);
	gpio_set_value(82, 0);  /* disable spkr poweramp */
}

void msm_snddev_hsed_voltage_on(void)
{
	pr_info("%s: power on negative charge pump\n", __func__);
	vreg_helper_on("ncp", (unsigned int)-1);
}

void msm_snddev_hsed_voltage_off(void)
{
	pr_info("%s: power off negative charge pump\n", __func__);
	vreg_helper_off("ncp");
}

static unsigned aux_pcm_gpio_on[] = {
	GPIO_CFG(138, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* PCM_DOUT */
	GPIO_CFG(139, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* PCM_DIN  */
	GPIO_CFG(140, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* PCM_SYNC */
	GPIO_CFG(141, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* PCM_CLK  */
};

static int __init aux_pcm_gpio_init(void)
{
	int pin, rc;

	pr_info("aux_pcm_gpio_init \n");
	for (pin = 0; pin < ARRAY_SIZE(aux_pcm_gpio_on); pin++) {
		rc = gpio_tlmm_config(aux_pcm_gpio_on[pin], GPIO_CFG_ENABLE);
		if (rc) {
			printk(KERN_ERR
				"%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, aux_pcm_gpio_on[pin], rc);
		}
	}
	return rc;
}

static int __init buses_init(void)
{
	if (gpio_tlmm_config(GPIO_CFG(PMIC_GPIO_INT, 0, GPIO_CFG_INPUT,
			GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE))
		pr_err("%s: gpio_tlmm_config (gpio=%d) failed\n",
		       __func__, PMIC_GPIO_INT);

	pm8058_7x30_data.sub_devices[PM8058_SUBDEV_KPD].platform_data
	    = &zeus_keypad_data;
	pm8058_7x30_data.sub_devices[PM8058_SUBDEV_KPD].data_size
	    = sizeof(zeus_keypad_data);

	i2c_register_board_info(6 /* I2C_SSBI ID */ , pm8058_boardinfo,
				ARRAY_SIZE(pm8058_boardinfo));

	return 0;
}

static struct vreg *vreg_marimba_1;
static struct vreg *vreg_marimba_2;

static unsigned int msm_marimba_setup_power(void)
{
	int rc;

	rc = vreg_enable(vreg_marimba_1);
	if (rc) {
		printk(KERN_ERR "%s: vreg_enable() = %d \n", __func__, rc);
		goto out;
	}
	rc = vreg_enable(vreg_marimba_2);
	if (rc) {
		printk(KERN_ERR "%s: vreg_enable() = %d \n", __func__, rc);
		goto out;
	}

 out:
	return rc;
};

static void msm_marimba_shutdown_power(void)
{
	int rc;

	rc = vreg_disable(vreg_marimba_1);
	if (rc)
		printk(KERN_ERR "%s: return val: %d \n", __func__, rc);
	rc = vreg_disable(vreg_marimba_2);
	if (rc)
		printk(KERN_ERR "%s: return val: %d \n", __func__, rc);
};

/* Slave id address for FM/CDC/QMEMBIST
 * Values can be programmed using Marimba slave id 0
 * should there be a conflict with other I2C devices
 * */
#define MARIMBA_SLAVE_ID_FM_ADDR	0x2A
#define MARIMBA_SLAVE_ID_CDC_ADDR	0x77
#define MARIMBA_SLAVE_ID_QMEMBIST_ADDR	0X66

static struct vreg *vreg_codec_s4;
static int msm_marimba_codec_power(int vreg_on)
{
	int rc = 0;

	if (!vreg_codec_s4) {

		vreg_codec_s4 = vreg_get(NULL, "s4");

		if (IS_ERR(vreg_codec_s4)) {
			printk(KERN_ERR "%s: vreg_get() failed (%ld)\n",
				__func__, PTR_ERR(vreg_codec_s4));
			rc = PTR_ERR(vreg_codec_s4);
			goto vreg_codec_s4_fail;
		}
	}

	if (vreg_on) {
		rc = vreg_enable(vreg_codec_s4);
		if (rc)
			printk(KERN_ERR "%s: vreg_enable() = %d \n",
				__func__, rc);
		goto vreg_codec_s4_fail;
	} else {
		rc = vreg_disable(vreg_codec_s4);
		if (rc)
			printk(KERN_ERR "%s: vreg_disable() = %d \n",
				__func__, rc);
		goto vreg_codec_s4_fail;
	}

 vreg_codec_s4_fail:
	return rc;
}

static struct marimba_codec_platform_data mariba_codec_pdata = {
	.marimba_codec_power = msm_marimba_codec_power,
};

static struct marimba_platform_data marimba_pdata = {
	.slave_id[MARIMBA_SLAVE_ID_CDC] = MARIMBA_SLAVE_ID_CDC_ADDR,
	.slave_id[MARIMBA_SLAVE_ID_QMEMBIST] = MARIMBA_SLAVE_ID_QMEMBIST_ADDR,
	.marimba_setup = msm_marimba_setup_power,
	.marimba_shutdown = msm_marimba_shutdown_power,
	.codec = &mariba_codec_pdata,
};

static void __init msm7x30_init_marimba(void)
{
	vreg_marimba_1 = vreg_get(NULL, "s3");
	if (IS_ERR(vreg_marimba_1)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg_marimba_1));
		return;
	}
	vreg_marimba_2 = vreg_get(NULL, "gp16");
	if (IS_ERR(vreg_marimba_2)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg_marimba_2));
		return;
	}
}

static struct resource msm_aictl_resources[] = {
	{
	 .name = "aictl",
	 .start = 0xa5000100,
	 .end = 0xa5000100,
	 .flags = IORESOURCE_MEM,
	 }
};

static struct resource msm_mi2s_resources[] = {
	{
	 .name = "hdmi",
	 .start = 0xac900000,
	 .end = 0xac900038,
	 .flags = IORESOURCE_MEM,
	 },
	{
	 .name = "codec_rx",
	 .start = 0xac940040,
	 .end = 0xac940078,
	 .flags = IORESOURCE_MEM,
	 },
	{
	 .name = "codec_tx",
	 .start = 0xac980080,
	 .end = 0xac9800B8,
	 .flags = IORESOURCE_MEM,
	 }

};

static struct msm_lpa_platform_data lpa_pdata = {
	.obuf_hlb_size = 0x2BFF8,
	.dsp_proc_id = 0,
	.app_proc_id = 2,
	.nosb_config = {
			.llb_min_addr = 0,
			.llb_max_addr = 0x3ff8,
			.sb_min_addr = 0,
			.sb_max_addr = 0,
	},
	.sb_config = {
		      .llb_min_addr = 0,
		      .llb_max_addr = 0x37f8,
		      .sb_min_addr = 0x3800,
		      .sb_max_addr = 0x3ff8,
	}
};

static struct resource msm_lpa_resources[] = {
	{
	 .name = "lpa",
	 .start = 0xa5000000,
	 .end = 0xa50000a0,
	 .flags = IORESOURCE_MEM,
	 }
};

static struct resource msm_aux_pcm_resources[] = {

	{
	 .name = "aux_codec_reg_addr",
	 .start = 0xac9c00c0,
	 .end = 0xac9c00c8,
	 .flags = IORESOURCE_MEM,
	 },
	{
	 .name = "aux_pcm_dout",
	 .start = 138,
	 .end = 138,
	 .flags = IORESOURCE_IO,
	 },
	{
	 .name = "aux_pcm_din",
	 .start = 139,
	 .end = 139,
	 .flags = IORESOURCE_IO,
	 },
	{
	 .name = "aux_pcm_syncout",
	 .start = 140,
	 .end = 140,
	 .flags = IORESOURCE_IO,
	 },
	{
	 .name = "aux_pcm_clkin_a",
	 .start = 141,
	 .end = 141,
	 .flags = IORESOURCE_IO,
	 },
};

static struct platform_device msm_aux_pcm_device = {
	.name = "msm_aux_pcm",
	.id = 0,
	.num_resources = ARRAY_SIZE(msm_aux_pcm_resources),
	.resource = msm_aux_pcm_resources,
};

struct platform_device msm_aictl_device = {
	.name = "audio_interct",
	.id = 0,
	.num_resources = ARRAY_SIZE(msm_aictl_resources),
	.resource = msm_aictl_resources,
};

struct platform_device msm_mi2s_device = {
	.name = "mi2s",
	.id = 0,
	.num_resources = ARRAY_SIZE(msm_mi2s_resources),
	.resource = msm_mi2s_resources,
};

struct platform_device msm_lpa_device = {
	.name = "lpa",
	.id = 0,
	.num_resources = ARRAY_SIZE(msm_lpa_resources),
	.resource = msm_lpa_resources,
	.dev = {
		.platform_data = &lpa_pdata,
		},
};

#define DEC0_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC1_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC2_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC3_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC4_FORMAT (1<<MSM_ADSP_CODEC_MIDI)

static unsigned int dec_concurrency_table[] = {
	/* Audio LP */
	0,
	(DEC3_FORMAT | (1 << MSM_ADSP_MODE_NONTUNNEL) | (1 << MSM_ADSP_OP_DM)),
	(DEC2_FORMAT | (1 << MSM_ADSP_MODE_NONTUNNEL) | (1 << MSM_ADSP_OP_DM)),
	(DEC1_FORMAT | (1 << MSM_ADSP_MODE_NONTUNNEL) | (1 << MSM_ADSP_OP_DM)),
	(DEC0_FORMAT | (1 << MSM_ADSP_MODE_TUNNEL) | (1 << MSM_ADSP_MODE_LP) |
	 (1 << MSM_ADSP_OP_DM)),

	/* Concurrency 1 */
	(DEC4_FORMAT),
	(DEC3_FORMAT | (1 << MSM_ADSP_MODE_TUNNEL) | (1 << MSM_ADSP_OP_DM)),
	(DEC2_FORMAT | (1 << MSM_ADSP_MODE_TUNNEL) | (1 << MSM_ADSP_OP_DM)),
	(DEC1_FORMAT | (1 << MSM_ADSP_MODE_TUNNEL) | (1 << MSM_ADSP_OP_DM)),
	(DEC0_FORMAT | (1 << MSM_ADSP_MODE_TUNNEL) | (1 << MSM_ADSP_OP_DM)),

	/* Concurrency 2 */
	(DEC4_FORMAT),
	(DEC3_FORMAT | (1 << MSM_ADSP_MODE_TUNNEL) | (1 << MSM_ADSP_OP_DM)),
	(DEC2_FORMAT | (1 << MSM_ADSP_MODE_TUNNEL) | (1 << MSM_ADSP_OP_DM)),
	(DEC1_FORMAT | (1 << MSM_ADSP_MODE_TUNNEL) | (1 << MSM_ADSP_OP_DM)),
	(DEC0_FORMAT | (1 << MSM_ADSP_MODE_TUNNEL) | (1 << MSM_ADSP_OP_DM)),

	/* Concurrency 3 */
	(DEC4_FORMAT),
	(DEC3_FORMAT | (1 << MSM_ADSP_MODE_TUNNEL) | (1 << MSM_ADSP_OP_DM)),
	(DEC2_FORMAT | (1 << MSM_ADSP_MODE_TUNNEL) | (1 << MSM_ADSP_OP_DM)),
	(DEC1_FORMAT | (1 << MSM_ADSP_MODE_TUNNEL) | (1 << MSM_ADSP_OP_DM)),
	(DEC0_FORMAT | (1 << MSM_ADSP_MODE_NONTUNNEL) | (1 << MSM_ADSP_OP_DM)),

	/* Concurrency 4 */
	(DEC4_FORMAT),
	(DEC3_FORMAT | (1 << MSM_ADSP_MODE_TUNNEL) | (1 << MSM_ADSP_OP_DM)),
	(DEC2_FORMAT | (1 << MSM_ADSP_MODE_TUNNEL) | (1 << MSM_ADSP_OP_DM)),
	(DEC1_FORMAT | (1 << MSM_ADSP_MODE_NONTUNNEL) | (1 << MSM_ADSP_OP_DM)),
	(DEC0_FORMAT | (1 << MSM_ADSP_MODE_NONTUNNEL) | (1 << MSM_ADSP_OP_DM)),

	/* Concurrency 5 */
	(DEC4_FORMAT),
	(DEC3_FORMAT | (1 << MSM_ADSP_MODE_TUNNEL) | (1 << MSM_ADSP_OP_DM)),
	(DEC2_FORMAT | (1 << MSM_ADSP_MODE_NONTUNNEL) | (1 << MSM_ADSP_OP_DM)),
	(DEC1_FORMAT | (1 << MSM_ADSP_MODE_NONTUNNEL) | (1 << MSM_ADSP_OP_DM)),
	(DEC0_FORMAT | (1 << MSM_ADSP_MODE_NONTUNNEL) | (1 << MSM_ADSP_OP_DM)),

	/* Concurrency 6 */
	(DEC4_FORMAT),
	(DEC3_FORMAT | (1 << MSM_ADSP_MODE_NONTUNNEL) | (1 << MSM_ADSP_OP_DM)),
	(DEC2_FORMAT | (1 << MSM_ADSP_MODE_NONTUNNEL) | (1 << MSM_ADSP_OP_DM)),
	(DEC1_FORMAT | (1 << MSM_ADSP_MODE_NONTUNNEL) | (1 << MSM_ADSP_OP_DM)),
	(DEC0_FORMAT | (1 << MSM_ADSP_MODE_NONTUNNEL) | (1 << MSM_ADSP_OP_DM)),
};

#define DEC_INFO(name, queueid, decid, nr_codec) { .module_name = name, \
	.module_queueid = queueid, .module_decid = decid, \
	.nr_codec_support = nr_codec}

#define DEC_INSTANCE(max_instance_same, max_instance_diff) { \
	.max_instances_same_dec = max_instance_same, \
	.max_instances_diff_dec = max_instance_diff}

static struct msm_adspdec_info dec_info_list[] = {
	DEC_INFO("AUDPLAY4TASK", 17, 4, 1),	/* AudPlay4BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY3TASK", 16, 3, 11),	/* AudPlay3BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY2TASK", 15, 2, 11),	/* AudPlay2BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY1TASK", 14, 1, 11),	/* AudPlay1BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY0TASK", 13, 0, 11),	/* AudPlay0BitStreamCtrlQueue */
};

static struct dec_instance_table dec_instance_list[][MSM_MAX_DEC_CNT] = {
	/* Non Turbo Mode */
	{
	 DEC_INSTANCE(4, 3),	/* WAV */
	 DEC_INSTANCE(4, 3),	/* ADPCM */
	 DEC_INSTANCE(4, 2),	/* MP3 */
	 DEC_INSTANCE(0, 0),	/* Real Audio */
	 DEC_INSTANCE(4, 2),	/* WMA */
	 DEC_INSTANCE(3, 2),	/* AAC */
	 DEC_INSTANCE(0, 0),	/* Reserved */
	 DEC_INSTANCE(0, 0),	/* MIDI */
	 DEC_INSTANCE(4, 3),	/* YADPCM */
	 DEC_INSTANCE(4, 3),	/* QCELP */
	 DEC_INSTANCE(4, 3),	/* AMRNB */
	 DEC_INSTANCE(1, 1),	/* AMRWB/WB+ */
	 DEC_INSTANCE(4, 3),	/* EVRC */
	 DEC_INSTANCE(1, 1),	/* WMAPRO */
	 },
	/* Turbo Mode */
	{
	 DEC_INSTANCE(4, 3),	/* WAV */
	 DEC_INSTANCE(4, 3),	/* ADPCM */
	 DEC_INSTANCE(4, 3),	/* MP3 */
	 DEC_INSTANCE(0, 0),	/* Real Audio */
	 DEC_INSTANCE(4, 3),	/* WMA */
	 DEC_INSTANCE(4, 3),	/* AAC */
	 DEC_INSTANCE(0, 0),	/* Reserved */
	 DEC_INSTANCE(0, 0),	/* MIDI */
	 DEC_INSTANCE(4, 3),	/* YADPCM */
	 DEC_INSTANCE(4, 3),	/* QCELP */
	 DEC_INSTANCE(4, 3),	/* AMRNB */
	 DEC_INSTANCE(2, 3),	/* AMRWB/WB+ */
	 DEC_INSTANCE(4, 3),	/* EVRC */
	 DEC_INSTANCE(1, 2),	/* WMAPRO */
	 },
};

static struct msm_adspdec_database msm_device_adspdec_database = {
	.num_dec = ARRAY_SIZE(dec_info_list),
	.num_concurrency_support = (ARRAY_SIZE(dec_concurrency_table) /
				    ARRAY_SIZE(dec_info_list)),
	.dec_concurrency_table = dec_concurrency_table,
	.dec_info_list = dec_info_list,
	.dec_instance_list = &dec_instance_list[0][0],
};

static struct platform_device msm_device_adspdec = {
	.name = "msm_adspdec",
	.id = -1,
	.dev = {
		.platform_data = &msm_device_adspdec_database},
};

static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns		= 1,
	.vendor		= "SEMC",
	.product	= "Mass Storage",
	.release	= 0x0100,

	.cdrom_nluns	= 1,
	.cdrom_vendor	= "SEMC",
	.cdrom_product	= "CD-ROM",
	.cdrom_release	= 0x0100,
};

static struct platform_device mass_storage_device = {
	.name = "usb_mass_storage",
	.id = -1,
	.dev = {
		.platform_data = &mass_storage_pdata,
	},
};

static struct usb_ether_platform_data rndis_pdata = {
	/* ethaddr is filled by board_serialno_setup */
	.vendorID	= 0x0FCE,
	.vendorDescr	= "SEMC",
};

static struct platform_device rndis_device = {
	.name	= "rndis",
	.id	= -1,
	.dev	= {
		.platform_data = &rndis_pdata,
	},
};

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id		= 0x0FCE,
	.version		= 0x0100,
	.product_name		= "SEMC HSUSB Device",
	.manufacturer_name	= "SEMC",
	.usb_mass_storage_device = &mass_storage_device,
	/* .serial_number filled_in by board_serialno_setup */
};

static struct platform_device android_usb_device = {
	.name = "android_usb",
	.id = -1,
	.dev = {
		.platform_data = &android_usb_pdata,
	},
};

static int __init board_serialno_setup(char *serialno)
{
	int ix, len, i;
	static char usb_serial_number[21];
	char *src = serialno;

	/* create a MAC address from our serial number.
	 * first byte is 0x02 to signify locally administered.
	 */
	rndis_pdata.ethaddr[0] = 0x02;
	for (i = 0; *src; i++) {
		/* XOR the USB serial across the remaining bytes */
		rndis_pdata.ethaddr[i % (ETH_ALEN - 1) + 1] ^= *src++;
	}

	/* The USB mass storage spec states in section 4.1.2 that
	 * the serial number may only contain characters '0' to '9'
	 * and 'A' to 'F'. With this change the serial is instead
	 * generated from the hex value of the individual chars
	 * in the phone's serial number.
	 */
	len = strlen(serialno);
	ix = 0;
	while (ix < 20) {
		if (*serialno && ix >= 20 - (len << 1)) {
			sprintf(&usb_serial_number[ix], "%02X",
					(unsigned char)*serialno);
			serialno++;
		} else {
			sprintf(&usb_serial_number[ix], "%02X", 0);
		}
		ix += 2;
	}
	usb_serial_number[20] = '\0';
	android_usb_pdata.serial_number = usb_serial_number;
	mass_storage_pdata.serial_number = usb_serial_number;

	printk(KERN_INFO "USB serial number: %s\n",
			android_usb_pdata.serial_number);
	return 1;
}
__setup("serialno=", board_serialno_setup);


static void hr_usleep(int us)
{
	struct timespec req_time;
	long ret;

	req_time.tv_sec = us / 1000000;
	req_time.tv_nsec = (us % 1000000) * 1000;

	ret = hrtimer_nanosleep(&req_time, NULL, HRTIMER_MODE_REL,
							CLOCK_MONOTONIC);
	if (ret != 0)
		printk(KERN_ERR "%s: nanosleep failed, ret = %ld\n", __func__,
									ret);
}

static void hr_msleep(int ms)
{
	hr_usleep(1000 * ms);
}

static int novatek_power(int on)
{
	static int enabled_once;
	int rc = 0;

	if (on) {
		if (!enabled_once) {
			rc = vreg_helper_on("gp6", 2850);
			if (rc)
				goto out;
			rc = vreg_helper_on("gp9", 1800);
			if (rc) {
				vreg_helper_off("gp6");
				goto out;
			}
			hr_usleep(21); /* spec says > 20us */
			gpio_set_value(NOVATEK_GPIO_RESET, 1);
			hr_msleep(11); /* spec says > 11ms */
			enabled_once = 1;
		} else {
			gpio_set_value(NOVATEK_GPIO_RESET, 0);
			hr_msleep(4); /* spec says: > 4ms */
			gpio_set_value(NOVATEK_GPIO_RESET, 1);
			hr_msleep(11); /* spec says: > 10 ms */
		}
	}
	/* Do not do anything at power off */
out:
	return rc;
}

static struct novatek_fwvga_platform_data novatek_platform_data = {
	.power = novatek_power,
	.reset = NULL,
};

static struct platform_device novatek_device = {
	.name	= MDDI_NOVATEK_FWVGA_NAME,
	.id	= -1,
	.dev	= {
		.platform_data = &novatek_platform_data,
	}
};

static const struct panel_id *novatek_panels[] = {
#ifdef CONFIG_MDDI_NOVATEK_PANEL_TMD_MDP42
	&novatek_panel_id_tmd_mdp42_rev_c,
	&novatek_panel_id_tmd_mdp42_rev_d,
#endif
#ifdef CONFIG_MDDI_NOVATEK_PANEL_SHARP_LS040T8LX01
	&novatek_panel_id_sharp_ls040t8lx01_rev_c_x,	/* SP2.1 support */
	&novatek_panel_id_sharp_ls040t8lx01_rev_c,
	&novatek_panel_id_sharp_ls040t8lx01_rev_d,
#endif
	NULL,
};

struct novatek_i2c_pdata novatek_i2c_pdata = {
	.panels = novatek_panels,
};

static struct as3676_als_config as3676_als_config = {
	.gain = AS3676_GAIN_1,
	.filter_up = AS3676_FILTER_1HZ,
	.filter_down = AS3676_FILTER_1HZ,
	.source = AS3676_ALS_SOURCE_GPIO2,
	.curve = {
		[AS3676_AMB_GROUP_1] = {
			.y0 = 49,
			.y3 = 255,
			.k1 = 71,
			.k2 = 54,
			.x1 =  1,
			.x2 = 33,
		},
	},
};

static struct as3676_platform_led as3676_pdata_leds[] = {
	{
		.name = "lcd-backlight",
		.sinks = BIT(AS3676_SINK_01),
		.flags = AS3676_FLAG_ALS | AS3676_FLAG_PWM_INIT
		| AS3676_FLAG_WAIT_RESUME
			| AS3676_FLAG_DLS,
		.max_current = 20000,
		.default_brightness = LED_FULL,
	},
	{
		.name = "red",
		.sinks = BIT(AS3676_SINK_41),
		.flags = AS3676_FLAG_RGB | AS3676_FLAG_BLINK,
		.max_current = 3000,
	},
	{
		.name = "green",
		.sinks = BIT(AS3676_SINK_42),
		.flags = AS3676_FLAG_RGB | AS3676_FLAG_BLINK,
		.max_current = 3000,
	},
	{
		.name = "blue",
		.sinks = BIT(AS3676_SINK_43),
		.flags = AS3676_FLAG_RGB | AS3676_FLAG_BLINK,
		.max_current = 3000,
	},
};

static struct as3676_platform_data as3676_platform_data = {
	.leds = as3676_pdata_leds,
	.num_leds = ARRAY_SIZE(as3676_pdata_leds),
	.als_config = &as3676_als_config,
	.als_connected = 1,
	.dls_connected = 1,
};

static struct msm_gpio synaptics_gpio_config_data[] = {
	{ GPIO_CFG(SYNAPTICS_TOUCHPAD_GPIO, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "synaptics_touchpad_irq" },
};

static int synaptics_touchpad_gpio_setup(void)
{
	int rc;

	rc = msm_gpios_request_enable(synaptics_gpio_config_data,
		ARRAY_SIZE(synaptics_gpio_config_data));

	return rc;
}

static void synaptics_touchpad_gpio_teardown(void)
{
	msm_gpios_disable_free(synaptics_gpio_config_data,
		ARRAY_SIZE(synaptics_gpio_config_data));
}

static struct synaptics_touchpad_platform_data synaptics_touchpad_data = {
	.gpio_setup	= synaptics_touchpad_gpio_setup,
	.gpio_teardown	= synaptics_touchpad_gpio_teardown,
};

static int bma150_gpio_setup(bool request)
{
	if (request)
		return gpio_request(BMA150_GPIO, "bma150_irq");
	else
		gpio_free(BMA150_GPIO);
	return 0;
}

struct bma150_platform_data bma150_ng_platform_data = {
	.gpio_setup = bma150_gpio_setup,
};

static int akm8975_gpio_setup(void)
{
	int rc;
	rc = gpio_request(AKM8975_GPIO, "akm8975_drdy_irq");
	return rc;
}

static void akm8975_gpio_shutdown(void)
{
	gpio_free(AKM8975_GPIO);
}

static struct akm8975_platform_data akm8975_platform_data = {
	.setup = akm8975_gpio_setup,
	.shutdown = akm8975_gpio_shutdown,
};

static struct msm_gpio gp2a_gpio_config_data[] = {
	{ GPIO_CFG(GP2A_GPIO, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GP2A_GPIO_NAME },
};

static int gp2a_gpio_setup(void)
{
	int rc;
	rc = msm_gpios_request_enable(gp2a_gpio_config_data,
		ARRAY_SIZE(gp2a_gpio_config_data));

	return rc;
}

static int gp2a_gpio_teardown(void)
{
	msm_gpios_disable_free(gp2a_gpio_config_data,
		ARRAY_SIZE(gp2a_gpio_config_data));

	return 0;
}

static struct gp2a_platform_data gp2a_platform_data = {
	.gpio = GP2A_GPIO,
	.wake = 1,
	.gpio_setup = gp2a_gpio_setup,
	.gpio_shutdown = gp2a_gpio_teardown,
};

static struct cypress_callback *cy_callback;

static void cy_register_cb(struct cypress_callback *cy)
{
	cy_callback = cy;
}

static struct cypress_touch_platform_data cypress_touch_data = {
	.x_min		= 0,
	.x_max		= 479,
	.y_min		= 0,
	.y_max		= 853,
	.irq		= MSM_GPIO_TO_INT(CYPRESS_TOUCH_GPIO_IRQ),
	.gpio_irq_pin	= CYPRESS_TOUCH_GPIO_IRQ,
	.gpio_reset_pin	= CYPRESS_TOUCH_GPIO_RESET,
	.reset_polarity	= 1,
	.irq_polarity	= IRQF_TRIGGER_FALLING,
	.no_fw_update = 0,
	.register_cb	= cy_register_cb,
};

void charger_connected(int on)
{
	if (cy_callback && cy_callback->cb)
		cy_callback->cb(cy_callback, on);
}

static void cypress_touch_gpio_init(void)
{
	vreg_helper_on("gp13", 3000);

	/* Avoid writing firmward on SP3 */
	if (BOARD_HWID_SP3 == board_hwid)
		cypress_touch_data.no_fw_update = 1;
	else
		cypress_touch_data.no_fw_update = 0;

	gpio_request(CYPRESS_TOUCH_GPIO_RESET, "cy8_reset");
	/* intial reset */
	msleep(1);
	gpio_set_value(CYPRESS_TOUCH_GPIO_RESET,
			cypress_touch_data.reset_polarity);
	gpio_free(CYPRESS_TOUCH_GPIO_RESET);
}

static struct i2c_board_info msm_camera_boardinfo[] __initdata = {
	{
		I2C_BOARD_INFO(MDDI_NOVATEK_I2C_NAME, 0x98 >> 1),
		.type = MDDI_NOVATEK_I2C_NAME,
		.platform_data = &novatek_i2c_pdata,
	},
	{
		I2C_BOARD_INFO("synaptics_touchpad", 0x40 >> 1),
		.irq		= MSM_GPIO_TO_INT(SYNAPTICS_TOUCHPAD_GPIO),
		.platform_data	= &synaptics_touchpad_data,
	},
	{
		I2C_BOARD_INFO(GP2A_I2C_NAME, 0x88 >> 1),
		.irq		= MSM_GPIO_TO_INT(GP2A_GPIO),
		.platform_data	= &gp2a_platform_data
	},
#ifdef CONFIG_SEIX006 /* zeus camera */
	{
		I2C_BOARD_INFO("seix006", 0x1A),
		.type = "seix006"
	},
#endif /* CONFIG_SEIX006 */
#ifdef CONFIG_OVM7692 /* zeus vt camera */
	{
		I2C_BOARD_INFO("ovm7692", 0x3C),
		.type = "ovm7692"
	},
#endif /* CONFIG_OVM7692 */
#ifdef CONFIG_MT9V114 /* zeus vt camera version 2*/
	{
		I2C_BOARD_INFO("mt9v114", 0x3D),
		.type = "mt9v114"
	},
#endif /* CONFIG_MT9V114 */
};

/* Driver(s) to be notified upon change in bdata */
static char *bdata_supplied_to[] = {
	MAX17040_NAME,
};

static struct semc_battery_platform_data semc_battery_platform_data = {
	.supplied_to = bdata_supplied_to,
	.num_supplicants = ARRAY_SIZE(bdata_supplied_to),
};

static struct platform_device bdata_driver = {
	.name = SEMC_BDATA_NAME,
	.id = -1,
	.dev = {
		.platform_data = &semc_battery_platform_data,
	},
};

static struct max17040_platform_data max17040_platform_data = {
	.model_desc = {
		.ocv_test = { 0xD9, 0x80 },
		.soc_low = 0xF4,
		.soc_high = 0xF6,
		.model_data = {
			{
				0xA6, 0xA0, 0xB7, 0x50, 0xB8, 0xB0, 0xB8, 0xE0,
				0xB9, 0x30, 0xBB, 0x60, 0xBB, 0xF0, 0xBC, 0x40
			},
			{
				0xBC, 0xA0, 0xBD, 0x50, 0xBE, 0x20, 0xC0, 0x20,
				0xC3, 0xF0, 0xC6, 0xE0, 0xCB, 0x40, 0xCF, 0x80
			},
			{
				0x03, 0xA0, 0x1A, 0x80, 0xAD, 0x60, 0x43, 0x60,
				0x00, 0x40, 0x7E, 0x40, 0x0E, 0x80, 0x72, 0x00
			},
			{
				0x4C, 0x20, 0x3B, 0x40, 0x29, 0xE0, 0x1B, 0x00,
				0x1B, 0x20, 0x13, 0x60, 0x12, 0x40, 0x12, 0x40
			}
		},
		.exp = 1
	},
	.rcomp_data = {
		.rcomp0 = 0x55,
		.temp_co_hot = -1400,
		.temp_co_cold = -9725,
		.temp_div = 1000,
	},
	.chg_max_temp = 550,
	.chg_min_temp = 50,
};

static struct i2c_board_info msm_i2c_board_info[] = {
	{
		I2C_BOARD_INFO(MAX17040_NAME, 0x6C >> 1),
		.platform_data = &max17040_platform_data,
	},
	{
		I2C_BOARD_INFO("as3676", 0x80 >> 1),
		.platform_data = &as3676_platform_data,
	},
	{
		I2C_BOARD_INFO("bma150", 0x70 >> 1),
		.irq		= MSM_GPIO_TO_INT(BMA150_GPIO),
		.platform_data = &bma150_ng_platform_data,
	},
	{
		I2C_BOARD_INFO(AKM8975_I2C_NAME, 0x18 >> 1),
		.irq		= MSM_GPIO_TO_INT(AKM8975_GPIO),
		.platform_data	= &akm8975_platform_data,
	},
};
static struct spi_board_info msm_spi_board_info[] __initdata = {
	{
		.modalias	= "cypress_touchscreen",
		.mode		= SPI_MODE_0,
		.platform_data	= &cypress_touch_data,
		.bus_num	= 0,
		.chip_select	= 0,
		.max_speed_hz	= 1 * 1000 * 1000,
	},
};

static struct i2c_board_info msm_marimba_board_info[] = {
	{
	 I2C_BOARD_INFO("marimba", 0xc),
	 .platform_data = &marimba_pdata,
	 }
};

static struct msm_pm_platform_data msm_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].supported = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].latency = 8594,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].residency = 23740,

	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].supported = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].latency = 4594,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].residency = 23740,

	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE].supported = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE].suspend_enabled = 0,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE].latency = 500,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE].residency = 6000,

	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].supported = 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].suspend_enabled
	    = 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].idle_enabled = 0,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency = 443,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].residency = 1098,

	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].supported = 1,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].latency = 2,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].residency = 0,
};

static struct resource qsd_spi_resources[] = {
	{
	 .name = "spi_irq_in",
	 .start = INT_SPI_INPUT,
	 .end = INT_SPI_INPUT,
	 .flags = IORESOURCE_IRQ,
	 },
	{
	 .name = "spi_irq_out",
	 .start = INT_SPI_OUTPUT,
	 .end = INT_SPI_OUTPUT,
	 .flags = IORESOURCE_IRQ,
	 },
	{
	 .name = "spi_irq_err",
	 .start = INT_SPI_ERROR,
	 .end = INT_SPI_ERROR,
	 .flags = IORESOURCE_IRQ,
	 },
	{
	 .name = "spi_base",
	 .start = 0xA8000000,
	 .end = 0xA8000000 + SZ_4K - 1,
	 .flags = IORESOURCE_MEM,
	 },
	{
	 .name = "spidm_channels",
	 .flags = IORESOURCE_DMA,
	 },
	{
	 .name = "spidm_crci",
	 .flags = IORESOURCE_DMA,
	 },
};

#define AMDH0_BASE_PHYS		0xAC200000
#define ADMH0_GP_CTL		(ct_adm_base + 0x3D8)
static int msm_qsd_spi_dma_config(void)
{
	void __iomem *ct_adm_base = 0;
	u32 spi_mux = 0;
	int ret = 0;

	ct_adm_base = ioremap(AMDH0_BASE_PHYS, PAGE_SIZE);
	if (!ct_adm_base) {
		pr_err("%s: Could not remap %x\n", __func__, AMDH0_BASE_PHYS);
		return -ENOMEM;
	}

	spi_mux = (ioread32(ADMH0_GP_CTL) & (0x3 << 12)) >> 12;

	qsd_spi_resources[4].start = DMOV_USB_CHAN;
	qsd_spi_resources[4].end = DMOV_TSIF_CHAN;

	switch (spi_mux) {
	case (1):
		qsd_spi_resources[5].start = DMOV_HSUART1_RX_CRCI;
		qsd_spi_resources[5].end = DMOV_HSUART1_TX_CRCI;
		break;
	case (2):
		qsd_spi_resources[5].start = DMOV_HSUART2_RX_CRCI;
		qsd_spi_resources[5].end = DMOV_HSUART2_TX_CRCI;
		break;
	case (3):
		qsd_spi_resources[5].start = DMOV_CE_OUT_CRCI;
		qsd_spi_resources[5].end = DMOV_CE_IN_CRCI;
		break;
	default:
		ret = -ENOENT;
	}

	iounmap(ct_adm_base);

	return ret;
}

static struct platform_device qsd_device_spi = {
	.name = "spi_qsd",
	.id = 0,
	.num_resources = ARRAY_SIZE(qsd_spi_resources),
	.resource = qsd_spi_resources,
};

static struct msm_gpio qsd_spi_gpio_config_data[] = {
	{GPIO_CFG(45, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA), "spi_clk"},
	{GPIO_CFG(46, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA), "spi_cs0"},
	{GPIO_CFG(47, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA), "spi_mosi"},
	{GPIO_CFG(48, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "spi_miso"},
};

static int msm_qsd_spi_gpio_config(void)
{
	return msm_gpios_request_enable(qsd_spi_gpio_config_data,
					ARRAY_SIZE(qsd_spi_gpio_config_data));
}

static void msm_qsd_spi_gpio_release(void)
{
	msm_gpios_disable_free(qsd_spi_gpio_config_data,
				ARRAY_SIZE(qsd_spi_gpio_config_data));
}

static struct msm_spi_platform_data qsd_spi_pdata = {
	.max_clock_speed = 26331429,
	.clk_name = "spi_clk",
	.pclk_name = "spi_pclk",
	.gpio_config = msm_qsd_spi_gpio_config,
	.gpio_release = msm_qsd_spi_gpio_release,
	.dma_config = msm_qsd_spi_dma_config,
};

static void __init msm_qsd_spi_init(void)
{
	qsd_device_spi.dev.platform_data = &qsd_spi_pdata;
}

static int hsusb_rpc_connect(int connect)
{
	if (connect)
		return msm_hsusb_rpc_connect();
	else
		return msm_hsusb_rpc_close();
}

static struct vreg *vreg_3p3;
static int msm_hsusb_ldo_init(int init)
{
	if (init) {
		vreg_3p3 = vreg_get(NULL, "usb");
		if (IS_ERR(vreg_3p3))
			return PTR_ERR(vreg_3p3);
		vreg_set_level(vreg_3p3, USB_VREG_MV);
	} else
		vreg_put(vreg_3p3);

	return 0;
}

static int msm_hsusb_ldo_enable(int enable)
{
	static int ldo_status;

	if (!vreg_3p3 || IS_ERR(vreg_3p3))
		return -ENODEV;

	if (ldo_status == enable)
		return 0;

	ldo_status = enable;

	if (enable)
		return vreg_enable(vreg_3p3);

	return vreg_disable(vreg_3p3);
}

static int msm_hsusb_ldo_set_voltage(int mV)
{
	static int cur_voltage = USB_VREG_MV;

	if (!vreg_3p3 || IS_ERR(vreg_3p3))
		return -ENODEV;

	if (cur_voltage == mV)
		return 0;

	charger_connected(USB_VREG_MV == mV);

	cur_voltage = mV;

	pr_debug("%s: (%d)\n", __func__, mV);

	return vreg_set_level(vreg_3p3, mV);
}

static struct msm_hsusb_gadget_platform_data msm_gadget_pdata;

static void msm_hsusb_vbus_power(unsigned phy_info, int on)
{
}

static struct msm_usb_host_platform_data msm_usb_host_pdata = {
	.phy_info = (USB_PHY_INTEGRATED | USB_PHY_MODEL_45NM),
	.power_budget = 180,
};

/* Driver(s) to be notified upon change in USB */
static char *hsusb_chg_supplied_to[] = {
	MAX17040_NAME,
};

static struct msm_otg_platform_data msm_otg_pdata = {
	.rpc_connect		= hsusb_rpc_connect,
	.core_clk		= 1,
	.vbus_power		= msm_hsusb_vbus_power,
	.pemp_level		= PRE_EMPHASIS_WITH_20_PERCENT,
	.cdr_autoreset		= CDR_AUTO_RESET_DISABLE,
	.drv_ampl		= HS_DRV_AMPLITUDE_DEFAULT,
	.se1_gating		= SE1_GATING_DISABLE,
	.chg_vbus_draw		= hsusb_chg_vbus_draw,
	.chg_connected		= hsusb_chg_connected,
	.chg_init		= hsusb_chg_init,
	.ldo_enable		= msm_hsusb_ldo_enable,
	.ldo_init		= msm_hsusb_ldo_init,
	.ldo_set_voltage	= msm_hsusb_ldo_set_voltage,
};

static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.allocator_type = PMEM_ALLOCATORTYPE_ALLORNOTHING,
	.cached = 1,
};

static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = {.platform_data = &android_pmem_pdata},
};

static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
	.inject_rx_on_wakeup = 0,
	.exit_lpm_cb = bcm_bt_lpm_exit_lpm_locked,
};

static struct bcm_bt_lpm_platform_data bcm_bt_lpm_pdata = {
	.gpio_wake = BT_GPIO_WAKE,
	.gpio_host_wake = BT_GPIO_HOST_WAKE,
	.request_clock_off_locked = msm_hs_request_clock_off,
	.request_clock_on_locked = msm_hs_request_clock_on,
};

struct platform_device bcm_bt_lpm_device = {
	.name = "bcm_bt_lpm",
	.id = 0,
	.dev = {
		.platform_data = &bcm_bt_lpm_pdata,
	},
};

static struct resource msm_fb_resources[] = {
	{
	 .flags = IORESOURCE_DMA,
	 }
};

static struct msm_fb_platform_data msm_fb_pdata = {
	.mddi_prescan = 1,
};

static struct platform_device msm_fb_device = {
	.name = "msm_fb",
	.id = 0,
	.num_resources = ARRAY_SIZE(msm_fb_resources),
	.resource = msm_fb_resources,
	.dev = {
		.platform_data = &msm_fb_pdata,
		}
};

static struct android_pmem_platform_data android_pmem_kernel_ebi1_pdata = {
	.name = PMEM_KERNEL_EBI1_DATA_NAME,
	/* if no allocator_type, defaults to PMEM_ALLOCATORTYPE_BITMAP,
	 * the only valid choice at this time. The board structure is
	 * set to all zeros by the C runtime initialization and that is now
	 * the enum value of PMEM_ALLOCATORTYPE_BITMAP, now forced to 0 in
	 * include/linux/android_pmem.h.
	 */
	.cached = 0,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
};

static struct android_pmem_platform_data android_pmem_camera_pdata = {
	.name = "pmem_camera",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
};

static struct android_pmem_platform_data android_pmem_audio_pdata = {
	.name = "pmem_audio",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
};

static struct platform_device android_pmem_kernel_ebi1_device = {
	.name = "android_pmem",
	.id = 1,
	.dev = {.platform_data = &android_pmem_kernel_ebi1_pdata},
};

static struct platform_device android_pmem_adsp_device = {
	.name = "android_pmem",
	.id = 2,
	.dev = {.platform_data = &android_pmem_adsp_pdata},
};

static struct platform_device android_pmem_camera_device = {
	.name = "android_pmem",
	.id = 3,
	.dev = {.platform_data = &android_pmem_camera_pdata},
};

static struct platform_device android_pmem_audio_device = {
	.name = "android_pmem",
	.id = 4,
	.dev = {.platform_data = &android_pmem_audio_pdata},
};

struct kgsl_cpufreq_voter {
	int idle;
	struct msm_cpufreq_voter voter;
};

static int kgsl_cpufreq_vote(struct msm_cpufreq_voter *v)
{
	struct kgsl_cpufreq_voter *kv =
			container_of(v, struct kgsl_cpufreq_voter, voter);

	return kv->idle ? MSM_CPUFREQ_IDLE : MSM_CPUFREQ_ACTIVE;
}

static struct kgsl_cpufreq_voter kgsl_cpufreq_voter = {
	.idle = 1,
	.voter = {
		.vote = kgsl_cpufreq_vote,
	},
};

static void kgsl_idle_cb(int idle)
{
	if (idle != kgsl_cpufreq_voter.idle) {
		kgsl_cpufreq_voter.idle = idle;
		msm_cpufreq_voter_update(&kgsl_cpufreq_voter.voter);
	}
}

static struct kgsl_platform_data kgsl_pdata = {
	/* AXI rates in KHz */
	.high_axi_3d = 192000,
	.high_axi_2d = 192000,

	.max_grp2d_freq = 0,
	.min_grp2d_freq = 0,
	.set_grp2d_async = NULL,	/* HW workaround, run Z180 SYNC @ 192 MHZ */
	.max_grp3d_freq = 245760000,
	.min_grp3d_freq = 192 * 1000*1000,
	.set_grp3d_async = set_grp3d_async,
	.imem_clk_name = "imem_clk",
	.grp3d_clk_name = "grp_clk",
	.grp2d0_clk_name = "grp_2d_clk",
	.idle_callback = kgsl_idle_cb,
};

static struct resource kgsl_resources[] = {
	{
	 .name = "kgsl_reg_memory",
	 .start = 0xA3500000,	/* 3D GRP address */
	 .end = 0xA351ffff,
	 .flags = IORESOURCE_MEM,
	 },
	{
	 .name = "kgsl_phys_memory",
	 .start = 0,
	 .end = 0,
	 .flags = IORESOURCE_MEM,
	 },
	{
	 .name = "kgsl_yamato_irq",
	 .start = INT_GRP_3D,
	 .end = INT_GRP_3D,
	 .flags = IORESOURCE_IRQ,
	 },
	{
	 .name = "kgsl_2d0_reg_memory",
	 .start = 0xA3900000,	/* Z180 base address */
	 .end = 0xA3900FFF,
	 .flags = IORESOURCE_MEM,
	 },
	{
	 .name = "kgsl_2d0_irq",
	 .start = INT_GRP_2D,
	 .end = INT_GRP_2D,
	 .flags = IORESOURCE_IRQ,
	 },
};

static struct platform_device msm_device_kgsl = {
	.name = "kgsl",
	.id = -1,
	.num_resources = ARRAY_SIZE(kgsl_resources),
	.resource = kgsl_resources,
	.dev = {
		.platform_data = &kgsl_pdata,
		},
};

static int msm_fb_mddi_sel_clk(u32 *clk_rate)
{
	*clk_rate *= 2;
	return 0;
}

static struct mddi_platform_data mddi_pdata = {
/*      .mddi_power_save = display_common_power, */
	.mddi_sel_clk = msm_fb_mddi_sel_clk,
};

static struct msm_panel_common_pdata mdp_pdata = {
	.hw_revision_addr = 0xac001270,
	.gpio = 30,
	.mdp_core_clk_rate = 122880000,
};

static void __init msm_fb_add_devices(void)
{
	msm_fb_register_device("mdp", &mdp_pdata);
	msm_fb_register_device("pmdh", &mddi_pdata);
}

#ifdef CONFIG_ANDROID_RAM_CONSOLE
static struct resource ram_console_resources[] = {
	[0] = {
		.start  = MSM_RAM_CONSOLE_START,
		.end    = MSM_RAM_CONSOLE_START+MSM_RAM_CONSOLE_SIZE-1,
		.flags  = IORESOURCE_MEM,
	},
};

static struct platform_device ram_console_device = {
	.name           = "ram_console",
	.id             = -1,
};

static void ram_console_reserve_mem(void)
{
	if(reserve_bootmem(MSM_RAM_CONSOLE_START, MSM_RAM_CONSOLE_SIZE,
						BOOTMEM_EXCLUSIVE)) {
		printk(KERN_ERR "ram_console reserve memory failed\n");
		return;
	}
	ram_console_device.num_resources  = ARRAY_SIZE(ram_console_resources);
	ram_console_device.resource       = ram_console_resources;
}
#endif

/* BT/WiFi */

unsigned wifi_mac_addr[IFHWADDRLEN];
unsigned bt_mac_addr[IFHWADDRLEN];

static struct platform_device msm_bt_power_device = {
	.name = "bt_power",
};

static struct msm_gpio bt_config_power_on[] = {
	{ GPIO_CFG(134, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_4MA),
		   "UART1DM_RFR" },
	{ GPIO_CFG(135, 1, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		   "UART1DM_CTS" },
	{ GPIO_CFG(136, 1, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		   "UART1DM_Rx" },
	{ GPIO_CFG(137, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_4MA),
		   "UART1DM_Tx" },
	{ GPIO_CFG(138, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
		   "AUX_PCM_DOUT" },
	{ GPIO_CFG(139, 1, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		   "AUX_PCM_DIN" },
	{ GPIO_CFG(140, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		   "AUX_PCM_SYNC" },
	{ GPIO_CFG(141, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
		   "AUX_PCM_CLK" },
};

static struct msm_gpio bt_config_power_off[] = {
	{ GPIO_CFG(134, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
		   "UART1DM_RFR" },
	{ GPIO_CFG(135, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		   "UART1DM_CTS" },
	{ GPIO_CFG(136, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		   "UART1DM_Rx" },
	{ GPIO_CFG(137, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_4MA),
		   "UART1DM_Tx" },
	{ GPIO_CFG(138, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
		   "AUX_PCM_DOUT" },
	{ GPIO_CFG(139, 0, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
		   "AUX_PCM_DIN" },
	{ GPIO_CFG(140, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		   "AUX_PCM_SYNC" },
	{ GPIO_CFG(141, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
		   "AUX_PCM_CLK" },
};

static struct msm_gpio bluetooth_en_gpio[] = {
	{ GPIO_CFG(BT_GPIO_EN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "BT_EN" },
	{ GPIO_CFG(BT_GPIO_WAKE, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "MSM_WAKES_BT" },
	{ GPIO_CFG(BT_GPIO_HOST_WAKE, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "BT_WAKES_MSM" },
};

static int bluetooth_power(int on)
{
	int rc = -EIO;

	if (on) {
		rc = msm_gpios_enable(bt_config_power_on,
				      ARRAY_SIZE(bt_config_power_on));
		if (rc)
			goto out;
		gpio_set_value(BT_GPIO_EN, 1);
	} else {
		gpio_set_value(BT_GPIO_EN, 0);
		msm_gpios_enable(bt_config_power_off,
				 ARRAY_SIZE(bt_config_power_off));
	}
	return 0;
out:
	return rc;
}

static void __init bt_power_init(void)
{
	int rc;

	rc = msm_gpios_enable(bluetooth_en_gpio, ARRAY_SIZE(bluetooth_en_gpio));
	if (rc < 0) {
		printk(KERN_ERR "%s: enable gpio config failed: %d\n", __func__, rc);
		return;
	}

	bluetooth_power(0);

	msm_bt_power_device.dev.platform_data = &bluetooth_power;
}

int zeus_wifi_power(int on)
{
	printk(KERN_INFO "%s: %d\n", __func__, on);
	gpio_set_value(WIFI_GPIO_EN, on);
	mdelay(100);
	return 0;
};

int zeus_wifi_reset(int on)
{
	printk(KERN_INFO "%s: %d\n", __func__, on);
	mdelay(100);
	return 0;
};

static struct resource zeus_wifi_resources[] = {
	{
		.name	= "bcm4329_wlan_irq",
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_LOWEDGE,
	},
};

extern struct wifi_platform_data zeus_wifi_control;

static struct platform_device zeus_wifi = {
	.name		= "bcm4329_wlan",
	.id		= 1,
	.dev		= {
		.platform_data = &zeus_wifi_control,
	},
};

static int zeus_get_mac_address(unsigned char *buf)
{
	int i = IFHWADDRLEN;
	while(i-- > 0)
		buf[i] = wifi_mac_addr[i];
	return 0;
};

static void zeus_init_wlan_pdata(void)
{
	int i = IFHWADDRLEN, cksum = 0;

	while (i-- > 0)
		cksum += (wifi_mac_addr[i] ? 1 : 0);

	if (!cksum) {
		printk(KERN_ERR "%s: BAD WiFi MAC Address- not registering WiFi device\n", __func__);
		return;
	};

	zeus_wifi_control.get_mac_addr = zeus_get_mac_address;
	zeus_wifi_resources[0].start = MSM_GPIO_TO_INT(WIFI_GPIO_IRQ);
	zeus_wifi_resources[0].end = MSM_GPIO_TO_INT(WIFI_GPIO_IRQ);
	zeus_wifi.resource = zeus_wifi_resources;
	zeus_wifi.num_resources = ARRAY_SIZE(zeus_wifi_resources);
}

/* SEMC tally led */
#ifdef CONFIG_LEDS_LOW_CURRENT_PMIC
static struct platform_device zeus_tally_led_device = {
	.name   = "pmic-low-current-leds",
};
#endif

#ifdef CONFIG_PMIC_TIME
static struct platform_device pmic_time_device = {
	.name = "pmic_time",
};
#endif /* CONFIG_PMIC_TIME */

static struct input_dev *input_dev_pwr_key = NULL;
static void msm_pmic_pwr_key_rpc_callback(uint32_t key, uint32_t event)
{
	if (!input_dev_pwr_key)
		return;
	switch (key) {
	case HS_PWR_K:
		key = KEY_POWER;
		break;
	case HS_END_K:
		key = KEY_END;
		break;
	default:
		return;
	}
	input_report_key(input_dev_pwr_key, key, event != HS_REL_K);
}

static int __init msm_pmic_pwr_key_init(void)
{
	input_dev_pwr_key = input_allocate_device();
	if (!input_dev_pwr_key) {
		printk(KERN_ERR "%s: Error, unable to alloc pwr key device\n",
			__func__);
		return -1;
	}
	input_dev_pwr_key->name = "msm_pmic_pwr_key";
	input_dev_pwr_key->phys = "semc_rpc_server_handset";
	input_set_capability(input_dev_pwr_key, EV_KEY, KEY_POWER);
	input_set_capability(input_dev_pwr_key, EV_KEY, KEY_END);
	if (input_register_device(input_dev_pwr_key)) {
		printk(KERN_ERR "%s: Error, unable to reg pwr key device\n",
			__func__);
		input_free_device(input_dev_pwr_key);
		return -1;
	}
	return 0;
}
module_init(msm_pmic_pwr_key_init);

/*
 * Add callbacks here. Every defined callback will receive
 * all events. The types are defined in the file
 * semc_rpc_server_handset.h
 */

static handset_cb_array_t semc_rpc_hs_callbacks = {
	&msm_pmic_pwr_key_rpc_callback,
#ifdef CONFIG_SIMPLE_REMOTE_PLATFORM
	&simple_remote_pf_button_handler,
#endif
};

static struct semc_handset_data semc_rpc_hs_data = {
	.callbacks = semc_rpc_hs_callbacks,
	.num_callbacks = ARRAY_SIZE(semc_rpc_hs_callbacks),
};

static struct platform_device semc_rpc_handset_device = {
	.name = SEMC_HANDSET_DRIVER_NAME,
	.id = -1,
	.dev = {
		.platform_data = &semc_rpc_hs_data,
	},
};

#ifdef CONFIG_SIMPLE_REMOTE_PLATFORM
#define PLUG_DET_ENA_PIN 80
#define PLUG_DET_READ_PIN 26
#define MODE_SWITCH_PIN -1

int simple_remote_pf_initialize_gpio(struct simple_remote_platform_data *data)
{
	int err = 0;
	int i;

	if (!data || -1 == data->headset_detect_enable_pin) {
		printk(KERN_ERR
		       "*** %s - Error: Invalid inparameter (GPIO Pins)."
		       " Aborting!\n", __func__);
		return -EIO;
	}

	err = gpio_request(data->headset_detect_enable_pin,
			   "Simple_remote_plug_detect_enable");
	if (err) {
		printk(KERN_CRIT "%s: Error %d - Request hs_detect_enable pin",
		       __func__, err);
		goto out;
	}

	err = gpio_direction_output(data->headset_detect_enable_pin, 1);
	if (err) {
		printk(KERN_CRIT "%s: Error %d - Set hs_detect_enable pin"
		       " as output high\n", __func__, err);
		goto out_hs_det_enable;
	}

	err = gpio_request(data->headset_detect_read_pin,
			   "Simple_remote_plug_detect_read");
	if (err) {
		printk(KERN_CRIT "%s - Error %d - Request hs-detect_read pin",
		       __func__, err);
		goto out_hs_det_enable;
	}

	err = gpio_direction_input(data->headset_detect_read_pin);
	if (err) {
		printk(KERN_CRIT "%s - Error %d - Set hs-detect pin as input\n",
		       __func__, err);
		goto out_hs_det_read;
	}

	if (BOARD_HWID_DBZ3 != board_hwid || BOARD_HWID_DBZ3_1 != board_hwid ||
	    BOARD_HWID_DBZ3 != board_hwid)
		data->invert_plug_det = 1;
	else
		data->invert_plug_det = 0;

	if (0 < data->headset_mode_switch_pin) {
		printk(KERN_INFO "%s - This device supports HS Mode switch\n",
		       __func__);
		err = gpio_request(data->headset_mode_switch_pin,
				   "Simple_remote_headset_mode_switch");
		if (err) {
			printk(KERN_CRIT
			       "%s - Error %d - Request hs-mode_switch pin",
			       __func__, err);
			goto out_hs_det_read;
		}

		err = gpio_direction_output(data->headset_mode_switch_pin, 0);
		if (err) {
			printk(KERN_CRIT
			       "%s - Error %d - Set hs-mode_switch pin as "
			       "input\n", __func__, err);
			goto out_hs_mode_switch;
		}
	}

	for (i = 0; i < data->num_regs; i++) {
		data->regs[i].reg = vreg_get(NULL, data->regs[i].name);
		if (IS_ERR(data->regs[i].reg)) {
			printk(KERN_ERR "%s - Failed to find regulator %s\n",
			       __func__, data->regs[i].name);
			err = PTR_ERR(data->regs[i].reg);
			if (0 <= data->headset_mode_switch_pin)
				goto out_hs_mode_switch;
			else
				goto out_hs_det_read;
		}
	}

	return err;

out_hs_mode_switch:
	gpio_free(data->headset_mode_switch_pin);

out_hs_det_read:
	gpio_free(data->headset_detect_read_pin);

out_hs_det_enable:
	gpio_free(data->headset_detect_enable_pin);
out:
	return err;
}

void simple_remote_pf_deinitialize_gpio(
	struct simple_remote_platform_data *data)
{
	gpio_free(data->headset_detect_read_pin);
	gpio_free(data->headset_detect_enable_pin);
}

static struct simple_remote_platform_regulators regs[] =  {
	{
		.name = "ncp",
	},
	{
		.name = "s3",
	},
	{
		.name = "s2",
	},
};

static struct simple_remote_platform_data simple_remote_pf_data = {
	.headset_detect_enable_pin = PLUG_DET_ENA_PIN,
	.headset_detect_read_pin = PLUG_DET_READ_PIN,
	.headset_mode_switch_pin = MODE_SWITCH_PIN,
	.initialize = &simple_remote_pf_initialize_gpio,
	.deinitialize = &simple_remote_pf_deinitialize_gpio,

	.regs = regs,
	.num_regs = ARRAY_SIZE(regs),

	.controller = PM_HSED_CONTROLLER_1,
};

static struct platform_device simple_remote_pf_device = {
	.name = SIMPLE_REMOTE_PF_NAME,
	.dev = {
		.platform_data = &simple_remote_pf_data,
	},
};
#endif

static struct platform_device *devices[] __initdata = {
	&msm_device_smd,
	&msm_device_dmov,
	&msm_device_nand,
	&msm_device_otg,
	&msm_device_gadget_peripheral,
	&mass_storage_device,
	&rndis_device,
	&android_usb_device,
	&bcm_bt_lpm_device,
	&qsd_device_spi,
	&msm_device_ssbi6,
	&msm_device_ssbi7,
	&android_pmem_device,
	&msm_fb_device,
	&msm_rotator_device,
	&android_pmem_kernel_ebi1_device,
	&android_pmem_adsp_device,
	&android_pmem_camera_device,
	&android_pmem_audio_device,
	&msm_device_i2c,
	&msm_device_i2c_2,
	&msm_device_uart_dm1,
	&msm_aictl_device,
	&msm_mi2s_device,
	&msm_lpa_device,
	&msm_aux_pcm_device,
	&msm_device_adspdec,
	&qup_device_i2c,
	&msm_device_kgsl,
	&msm_device_uart3,
	&msm_device_vidc_720p,
	&msm_gemini_device,
	&msm_vpe_device,
	&bdata_driver,
	/*      &msm_batt_device, */
	/*      &msm_adc_device,  */
	&msm_bt_power_device,
#ifdef CONFIG_SEIX006
	&msm_camera_sensor_seix006,
#endif /* CONFIG_SEIX006 */
#ifdef CONFIG_OVM7692
	&msm_camera_sensor_ovm7692,
#endif /* CONFIG_OVM7692 */
#ifdef CONFIG_MT9V114
	&msm_camera_sensor_mt9v114,
#endif /* CONFIG_MT9V114 */
	&zeus_wifi,
	&novatek_device,
#ifdef CONFIG_PMIC_TIME
	&pmic_time_device,
#endif /* CONFIG_PMIC_TIME */
#ifdef CONFIG_ANDROID_RAM_CONSOLE
	&ram_console_device,
#endif
	&semc_rpc_handset_device,
#ifdef CONFIG_SIMPLE_REMOTE_PLATFORM
	&simple_remote_pf_device,
#endif
};

static struct msm_gpio msm_i2c_gpios_hw[] = {
	{GPIO_CFG(70, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "i2c_scl"},
	{GPIO_CFG(71, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "i2c_sda"},
};

static struct msm_gpio msm_i2c_gpios_io[] = {
	{GPIO_CFG(70, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "i2c_scl"},
	{GPIO_CFG(71, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "i2c_sda"},
};

static struct msm_gpio qup_i2c_gpios_io[] = {
	{GPIO_CFG(16, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "qup_scl"},
	{GPIO_CFG(17, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "qup_sda"},
};

static struct msm_gpio qup_i2c_gpios_hw[] = {
	{GPIO_CFG(16, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "qup_scl"},
	{GPIO_CFG(17, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "qup_sda"},
};

static void msm_i2c_gpio_config(int adap_id, int config_type)
{
	struct msm_gpio *msm_i2c_table;

	/* Each adapter gets 2 lines from the table */
	if (adap_id > 0)
		return;
	if (config_type)
		msm_i2c_table = &msm_i2c_gpios_hw[adap_id * 2];
	else
		msm_i2c_table = &msm_i2c_gpios_io[adap_id * 2];
	msm_gpios_enable(msm_i2c_table, 2);

	vreg_helper_on("gp7", 1800);
}

static void qup_i2c_gpio_config(int adap_id, int config_type)
{
	int rc = 0;
	struct msm_gpio *qup_i2c_table;
	/* Each adapter gets 2 lines from the table */
	if (adap_id != 4)
		return;
	if (config_type)
		qup_i2c_table = qup_i2c_gpios_hw;
	else
		qup_i2c_table = qup_i2c_gpios_io;
	rc = msm_gpios_enable(qup_i2c_table, 2);
	if (rc < 0)
		printk(KERN_ERR "QUP GPIO enable failed: %d\n", rc);
}

static struct msm_i2c_platform_data msm_i2c_pdata = {
	.clk_freq = 400000,
	.pri_clk = 70,
	.pri_dat = 71,
	.rmutex = 1,
	.rsl_id = "D:I2C02000021",
	.msm_i2c_config_gpio = msm_i2c_gpio_config,
};

static void __init msm_device_i2c_init(void)
{
	if (msm_gpios_request(msm_i2c_gpios_hw, ARRAY_SIZE(msm_i2c_gpios_hw)))
		pr_err("failed to request I2C gpios\n");

	msm_device_i2c.dev.platform_data = &msm_i2c_pdata;
}

static struct msm_i2c_platform_data msm_i2c_2_pdata = {
	.clk_freq = 400000,
	.rmutex = 0,
	.msm_i2c_config_gpio = msm_i2c_gpio_config,
};

static void __init msm_device_i2c_2_init(void)
{
	msm_device_i2c_2.dev.platform_data = &msm_i2c_2_pdata;
}

static struct msm_i2c_platform_data qup_i2c_pdata = {
	.clk_freq = 400000,
	.pclk = "camif_pad_pclk",
	.msm_i2c_config_gpio = qup_i2c_gpio_config,
};

static void __init qup_device_i2c_init(void)
{
	if (msm_gpios_request(qup_i2c_gpios_hw, ARRAY_SIZE(qup_i2c_gpios_hw)))
		pr_err("failed to request I2C gpios\n");

	qup_device_i2c.dev.platform_data = &qup_i2c_pdata;
}

static struct msm_ssbi_platform_data msm_i2c_ssbi6_pdata = {
	.rsl_id = "D:PMIC_SSBI",
	.controller_type = MSM_SBI_CTRL_SSBI2,
};

static struct msm_ssbi_platform_data msm_i2c_ssbi7_pdata = {
	.rsl_id = "D:CODEC_SSBI",
	.controller_type = MSM_SBI_CTRL_SSBI,
};

static struct msm_acpu_clock_platform_data msm7x30_clock_data = {
	.acpu_switch_time_us = 50,
	.vdd_switch_time_us = 62,
};

static void __init msm7x30_init_irq(void)
{
	msm_init_irq();
}

static struct msm_gpio mi2s_clk_gpios[] = {
	{ GPIO_CFG(145, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_SCLK"},
	{ GPIO_CFG(144, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_WS"},
	{ GPIO_CFG(120, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_MCLK_A"},
};

static struct msm_gpio mi2s_rx_data_lines_gpios[] = {
	{ GPIO_CFG(121, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_DATA_SD0_A"},
	{ GPIO_CFG(122, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_DATA_SD1_A"},
	{ GPIO_CFG(123, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_DATA_SD2_A"},
	{ GPIO_CFG(146, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_DATA_SD3"},
};

static struct msm_gpio mi2s_tx_data_lines_gpios[] = {
	{ GPIO_CFG(146, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_DATA_SD3"},
};

int mi2s_config_clk_gpio(void)
{
	int rc = 0;

	rc = msm_gpios_request_enable(mi2s_clk_gpios,
			ARRAY_SIZE(mi2s_clk_gpios));
	if (rc) {
		pr_err("%s: enable mi2s clk gpios  failed\n",
					__func__);
		return rc;
	}
	return 0;
}

int  mi2s_unconfig_data_gpio(u32 direction, u8 sd_line_mask)
{
	int i, rc = 0;
	sd_line_mask &= MI2S_SD_LINE_MASK;

	switch (direction) {
	case DIR_TX:
		msm_gpios_disable_free(mi2s_tx_data_lines_gpios, 1);
		break;
	case DIR_RX:
		i = 0;
		while (sd_line_mask) {
			if (sd_line_mask & 0x1)
				msm_gpios_disable_free(
					mi2s_rx_data_lines_gpios + i , 1);
			sd_line_mask = sd_line_mask >> 1;
			i++;
		}
		break;
	default:
		pr_err("%s: Invaild direction  direction = %u\n",
						__func__, direction);
		rc = -EINVAL;
		break;
	}
	return rc;
}

int mi2s_config_data_gpio(u32 direction, u8 sd_line_mask)
{
	int i , rc = 0;
	u8 sd_config_done_mask = 0;

	sd_line_mask &= MI2S_SD_LINE_MASK;

	switch (direction) {
	case DIR_TX:
		if ((sd_line_mask & MI2S_SD_0) || (sd_line_mask & MI2S_SD_1) ||
		   (sd_line_mask & MI2S_SD_2) || !(sd_line_mask & MI2S_SD_3)) {
			pr_err("%s: can not use SD0 or SD1 or SD2 for TX"
				".only can use SD3. sd_line_mask = 0x%x\n",
				__func__ , sd_line_mask);
			rc = -EINVAL;
		} else {
			rc = msm_gpios_request_enable(mi2s_tx_data_lines_gpios,
							 1);
			if (rc)
				pr_err("%s: enable mi2s gpios for TX failed\n",
					   __func__);
		}
		break;
	case DIR_RX:
		i = 0;
		while (sd_line_mask && (rc == 0)) {
			if (sd_line_mask & 0x1) {
				rc = msm_gpios_request_enable(
					mi2s_rx_data_lines_gpios + i , 1);
				if (rc) {
					pr_err("%s: enable mi2s gpios for"
					 "RX failed.  SD line = %s\n",
					 __func__,
					 (mi2s_rx_data_lines_gpios + i)->label);
					mi2s_unconfig_data_gpio(DIR_RX,
						sd_config_done_mask);
				} else
					sd_config_done_mask |= (1 << i);
			}
			sd_line_mask = sd_line_mask >> 1;
			i++;
		}
		break;
	default:
		pr_err("%s: Invaild direction  direction = %u\n",
			__func__, direction);
		rc = -EINVAL;
		break;
	}
	return rc;
}

int mi2s_unconfig_clk_gpio(void)
{
	msm_gpios_disable_free(mi2s_clk_gpios, ARRAY_SIZE(mi2s_clk_gpios));
	return 0;
}

struct sdcc_gpio {
	struct msm_gpio *cfg_data;
	uint32_t size;
};

static struct msm_gpio sdc3_cfg_on_data[] = {
	{GPIO_CFG(110, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
								"sdc3_clk"},
	{GPIO_CFG(111, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
								"sdc3_cmd"},
	{GPIO_CFG(116, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
								"sdc3_dat_3"},
	{GPIO_CFG(117, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
								"sdc3_dat_2"},
	{GPIO_CFG(118, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
								"sdc3_dat_1"},
	{GPIO_CFG(119, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
								"sdc3_dat_0"},
};

static struct msm_gpio sdc3_cfg_off_data[] = {
	{GPIO_CFG(110, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
								"sdc3_clk"},
	{GPIO_CFG(111, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
								"sdc3_cmd"},
	{GPIO_CFG(116, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
								"sdc3_dat_3"},
	{GPIO_CFG(117, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
								"sdc3_dat_2"},
	{GPIO_CFG(118, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
								"sdc3_dat_1"},
	{GPIO_CFG(119, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
								"sdc3_dat_0"},
};

static struct msm_gpio sdc4_cfg_on_data[] = {
	{GPIO_CFG(58, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
								"sdc4_clk"},
	{GPIO_CFG(59, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
								"sdc4_cmd"},
	{GPIO_CFG(60, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
								"sdc4_dat_3"},
	{GPIO_CFG(61, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
								"sdc4_dat_2"},
	{GPIO_CFG(62, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
								"sdc4_dat_1"},
	{GPIO_CFG(63, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
								"sdc4_dat_0"},
};

#ifdef CONFIG_PMIC_GPIO_25
static struct msm_gpio sdc4_cfg_off_data[] = {
	{GPIO_CFG(58, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
								"sdc4_clk"},
	{GPIO_CFG(59, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
								"sdc4_cmd"},
	{GPIO_CFG(60, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
								"sdc4_dat_3"},
	{GPIO_CFG(61, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
								"sdc4_dat_2"},
	{GPIO_CFG(62, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
								"sdc4_dat_1"},
	{GPIO_CFG(63, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
								"sdc4_dat_0"},
};
#else
/* uSD is never turned off. Same configuration is used for ON and OFF */
static struct msm_gpio sdc4_cfg_off_data[] = {
	{GPIO_CFG(58, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
								"sdc4_clk"},
	{GPIO_CFG(59, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
								"sdc4_cmd"},
	{GPIO_CFG(60, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
								"sdc4_dat_3"},
	{GPIO_CFG(61, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
								"sdc4_dat_2"},
	{GPIO_CFG(62, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
								"sdc4_dat_1"},
	{GPIO_CFG(63, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
								"sdc4_dat_0"},
};
#endif

static struct sdcc_gpio sdcc_cfg_on_data[] = {
	{
		.cfg_data = NULL,
		.size = 0,
	},
	{
		.cfg_data = NULL,
		.size = 0,
	},
	{
		.cfg_data = sdc3_cfg_on_data,
		.size = ARRAY_SIZE(sdc3_cfg_on_data),
	},
	{
		.cfg_data = sdc4_cfg_on_data,
		.size = ARRAY_SIZE(sdc4_cfg_on_data),
	},
};

static struct sdcc_gpio sdcc_cfg_off_data[] = {
	{
		.cfg_data = NULL,
		.size = 0,
	},
	{
		.cfg_data = NULL,
		.size = 0,
	},
	{
		.cfg_data = sdc3_cfg_off_data,
		.size = ARRAY_SIZE(sdc3_cfg_off_data),
	},
	{
		.cfg_data = sdc4_cfg_off_data,
		.size = ARRAY_SIZE(sdc4_cfg_off_data),
	},
};

static unsigned long gpio_sts;

static int cdcc_pm_gpio_config(unsigned int enable)
{
	int rc = 0;
#ifdef CONFIG_PMIC_GPIO_25
	/*Set PMIC_GPIO_25 to Out, no_pull*/
	struct pm8058_gpio pmic_gpio_25 = {
		.direction = PM_GPIO_DIR_OUT,
		.pull = PM_GPIO_PULL_NO,
		.vin_sel = PM_GPIO_VIN_L5,
		.out_strength = PM_GPIO_STRENGTH_HIGH,
		.function = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol = 0,
	};
	if (enable)
		pr_info("%s: uSD power on\n", __func__);
	else
		pr_info("%s: uSD power off\n", __func__);
	/* Set output_value to low if enabling power and
	 * high if disabling power
	 */
	pmic_gpio_25.output_value = !enable;
	rc = pm8058_gpio_config(PMIC_GPIO_SD_POWER - 1, &pmic_gpio_25);
	if (rc)
		pr_err("%s: Config PMIC_GPIO_SD_POWER failed\n", __func__);
#endif
	return rc;
}

static uint32_t msm_sdcc_setup_gpio(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct sdcc_gpio *curr;

	if ((dev_id > 0) &&
	    (dev_id <= ARRAY_SIZE(sdcc_cfg_off_data))) {
		curr = enable ? &sdcc_cfg_on_data[dev_id - 1] :
				&sdcc_cfg_off_data[dev_id - 1];
	} else {
		pr_err("%s: Incorrect device id %d\n", __func__, dev_id);
		goto out;
	}

	if (!curr->cfg_data)
		goto out;

	if (!(test_bit(dev_id, &gpio_sts) ^ enable))
		goto out;

	if (enable) {
		set_bit(dev_id, &gpio_sts);
		rc = msm_gpios_request_enable(curr->cfg_data, curr->size);
		if (rc) {
			/* Restore gpio_sts to power off */
			clear_bit(dev_id, &gpio_sts);
			pr_err("%s: Failed to turn on GPIOs for slot %d\n",
			       __func__, dev_id);
		}
		/* uSD power on */
		if ((dev_id == 4) && !rc && cdcc_pm_gpio_config(enable)) {
			/* Do nothing here if we failed to set PMIC_GPIO_25.
			 * Let it be restored in the off sequence.
			 */
		}
	} else {
		clear_bit(dev_id, &gpio_sts);
		/* uSD power off */
		if ((dev_id == 4) && cdcc_pm_gpio_config(enable)) {
			msm_gpios_free(curr->cfg_data, curr->size);
			goto out;
		}
		rc = msm_gpios_enable(curr->cfg_data, curr->size);
		if (rc) {
			if (dev_id != 4) {
				/* Restore gpio_sts to power on */
				set_bit(dev_id, &gpio_sts);
			}
			pr_err("%s: Failed to turn off GPIOs for slot %d\n",
			       __func__, dev_id);
		}
		msm_gpios_free(curr->cfg_data, curr->size);
#ifdef CONFIG_PMIC_GPIO_25
		if (dev_id == 4) {
			/*
			 * 200 milliseconds delay should be sufficient to allow
			 * microSD reaches zero voltage when uSD is power off.
			 */
			msleep(200);
		}
#endif
	}
out:
	return 0;
}

static uint32_t msm_sdcc_setup_power(struct device *dv, unsigned int vdd)
{
	struct platform_device *pdev;

	pdev = container_of(dv, struct platform_device, dev);

	return msm_sdcc_setup_gpio(pdev->id, (vdd ? 1 : 0));
}

static unsigned int msm7x30_sdcc_slot_status(struct device *dev)
{
	return (unsigned int)!gpio_get_value(
			PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_SD_DET - 1));
}

static int zeus_wifi_cd;
static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;

static int zeus_wifi_status_register(void (*callback)(int card_present, void *dev_id), void *dev_id)
{
	printk(KERN_DEBUG "%s: %p %p\n", __func__, callback, dev_id);
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

static unsigned int zeus_wifi_status(struct device *dev)
{
	printk(KERN_DEBUG "%s: %d\n", __func__, zeus_wifi_cd);
	return zeus_wifi_cd;
}

int zeus_wifi_set_carddetect(int val)
{
	printk(KERN_DEBUG "%s: %d\n", __func__, val);
	zeus_wifi_cd = val;
	if (wifi_status_cb)
		wifi_status_cb(val, wifi_status_cb_devid);
	else
		printk(KERN_WARNING "%s: Nobody to notify\n", __func__);
	return 0;
}

static struct mmc_platform_data msm7x30_sdc3_data = {
	.ocr_mask	= MMC_VDD_165_195 | MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd = msm_sdcc_setup_power,
	.mmc_bus_width = MMC_CAP_4_BIT_DATA,
	.sdiowakeup_irq = MSM_GPIO_TO_INT(118),
	.msmsdcc_fmin = 144000,
	.msmsdcc_fmid = 24576000,
	.msmsdcc_fmax = 49152000,
	.nonremovable = 1,
	.status			= zeus_wifi_status,
	.register_status_notify = zeus_wifi_status_register,
	.dummy52_required	= 1,
};

static struct mmc_platform_data msm7x30_sdc4_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd = msm_sdcc_setup_power,
	.mmc_bus_width = MMC_CAP_4_BIT_DATA,
	.status = msm7x30_sdcc_slot_status,
	.status_irq = PM8058_GPIO_IRQ(PMIC8058_IRQ_BASE, PMIC_GPIO_SD_DET - 1),
	.irq_flags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
	.msmsdcc_fmin = 144000,
	.msmsdcc_fmid = 24576000,
	.msmsdcc_fmax = 49152000,
	.nonremovable = 0,
};

static void __init msm7x30_init_mmc(void)
{
	msm_add_sdcc(3, &msm7x30_sdc3_data);
	msm_add_sdcc(4, &msm7x30_sdc4_data);
}

static struct msm_gpio uart3_config_data[] = {
	{GPIO_CFG(53, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "UART3_Rx"},
	{GPIO_CFG(54, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
								"UART3_Tx"},
};

static void msm7x30_init_uart3(void)
{
	msm_gpios_request_enable(uart3_config_data,
				 ARRAY_SIZE(uart3_config_data));

}

static struct msm_spm_platform_data msm_spm_data __initdata = {
	.reg_base_addr = MSM_SAW_BASE,

	.reg_init_values[MSM_SPM_REG_SAW_CFG] = 0x05,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_CTL] = 0x18,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_SLP_TMR_DLY] = 0x00006666,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_WAKE_TMR_DLY] = 0xFF000666,

	.reg_init_values[MSM_SPM_REG_SAW_SLP_CLK_EN] = 0x01,
	.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_PRECLMP_EN] = 0x03,
	.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_POSTCLMP_EN] = 0x00,

	.reg_init_values[MSM_SPM_REG_SAW_SLP_CLMP_EN] = 0x01,
	.reg_init_values[MSM_SPM_REG_SAW_SLP_RST_EN] = 0x00,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_MPM_CFG] = 0x00,

	.awake_vlevel = 0xF2,
	.retention_vlevel = 0xE0,
	.collapse_vlevel = 0x72,
	.retention_mid_vlevel = 0xE0,
	.collapse_mid_vlevel = 0xE0,

	.vctl_timeout_us = 50,
};

/*
 * Attempts to identify the the hardware ID
 * of Zeus HW.
 */
static void __init zeus_detect_product(void)
{
	u8 hwid;
	const char *pzName[] = {
		[BOARD_HWID_UNK]    = "Unknown",
		[BOARD_HWID_DP1]    = "DP1",
		[BOARD_HWID_SP1]    = "SP1",
		[BOARD_HWID_DBZ3]   = "DBZ3",
		[BOARD_HWID_SP1_5]  = "SP1.5",
		[BOARD_HWID_SP1_6]  = "SP1.6",
		[BOARD_HWID_DBZ3_1] = "DBZ3.1",
		[BOARD_HWID_SP2]    = "SP2",
		[BOARD_HWID_DBZ3_2] = "DBZ3.2",
		[BOARD_HWID_SP3]    = "SP3",
		[BOARD_HWID_AP1]    = "AP1",
		[BOARD_HWID_PQ]     = "PQ",
	};

	int idmap[] = {
		BOARD_HWID_UNK,    BOARD_HWID_UNK,	/* 0,1 */
		BOARD_HWID_UNK,    BOARD_HWID_UNK,	/* 2,3 */
		BOARD_HWID_UNK,    BOARD_HWID_DBZ3_2,	/* 4,5 */
		BOARD_HWID_DBZ3_1, BOARD_HWID_DBZ3,	/* 6,7 */
		BOARD_HWID_DP1,    BOARD_HWID_SP1,	/* 8,9 */
		BOARD_HWID_SP1_5,  BOARD_HWID_SP1_6,	/* a,b */
		BOARD_HWID_SP2,    BOARD_HWID_SP3,	/* c,d */
		BOARD_HWID_AP1,    BOARD_HWID_PQ,	/* e,f */
	};

	/* Request GPIOs, and set the TLMM in a state where we can read them */
	gpio_request(43, "hwid_3");
	gpio_request(148, "hwid_2");
	gpio_request(149, "hwid_1");
	gpio_request(150, "hwid_0");
	gpio_request(38, "hwid_3_ex");
	gpio_tlmm_config(GPIO_CFG(43, 0,
			GPIO_CFG_INPUT, GPIO_CFG_PULL_UP,
			GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(148, 0,
			GPIO_CFG_INPUT, GPIO_CFG_PULL_UP,
			GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(149, 0,
			GPIO_CFG_INPUT, GPIO_CFG_PULL_UP,
			GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(150, 0,
			GPIO_CFG_INPUT, GPIO_CFG_PULL_UP,
			GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	/* GPIO 38 (NC) pin should be configured as no pull for DBZ3.0x */
	gpio_tlmm_config(GPIO_CFG(38, 0,
			GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
			GPIO_CFG_2MA), GPIO_CFG_ENABLE);

	hwid = 0;
	hwid |= (gpio_get_value(150) & 1) << 0;
	hwid |= (gpio_get_value(149) & 1) << 1;
	hwid |= (gpio_get_value(148) & 1) << 2;
	hwid |= (gpio_get_value(43) & 1) << 3;

	board_hwid = idmap[hwid];
	system_rev = board_hwid;

	printk(KERN_INFO "Zeus HWID: 0x%x (%s)\n", hwid, pzName[board_hwid]);

	/* Reconfigure the GPIOs so that we won't leak current */
	gpio_tlmm_config(GPIO_CFG(43, 0,
			GPIO_CFG_INPUT, GPIO_CFG_NO_PULL,
			GPIO_CFG_2MA), GPIO_CFG_DISABLE);
	gpio_tlmm_config(GPIO_CFG(148, 0,
			GPIO_CFG_INPUT, GPIO_CFG_NO_PULL,
			GPIO_CFG_2MA), GPIO_CFG_DISABLE);
	gpio_tlmm_config(GPIO_CFG(149, 0,
			GPIO_CFG_INPUT, GPIO_CFG_NO_PULL,
			GPIO_CFG_2MA), GPIO_CFG_DISABLE);
	gpio_tlmm_config(GPIO_CFG(150, 0,
			GPIO_CFG_INPUT, GPIO_CFG_NO_PULL,
			GPIO_CFG_2MA), GPIO_CFG_DISABLE);

	/* Free our allocations */
	gpio_free(43);
	gpio_free(148);
	gpio_free(149);
	gpio_free(150);
	gpio_free(38);

}

/*
 * Temporary place for hardware initialization until the devices in question
 * gets proper drivers
 */
static void __init zeus_temp_fixups(void)
{
	/* Power up cameras, but keeps both in RST */
	gpio_request(0, "maincam_rst");
	gpio_set_value(0, 0);	/* MCAM_RST_N */
	gpio_free(0);

	/* Tweak the NT3550 power */
	gpio_tlmm_config(GPIO_CFG(157, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
				GPIO_CFG_ENABLE);

	/* Since the sequencing for AKM & BMA needs to be L10 -> L8 */
	vreg_helper_on("gp4", 2850);	/* L10 */

	vreg_helper_off("gp3");	/* L0 */
	vreg_helper_off("gp5");	/* L23 */
	vreg_helper_on("wlan", 1800);	/* L13: touchpad VDIO */
	vreg_helper_on("gp10", 2800);	/* L16: touchpad */

	/* This should be moved to the driver instead */
	cypress_touch_gpio_init();
}

static void zeus_init_wlan_gpios(void)
{
	int pin, rc;
	unsigned wlan_gpio_on[] = {
		GPIO_CFG(WIFI_GPIO_IRQ, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),  /* WLAN IRQ */
		GPIO_CFG(WIFI_GPIO_EN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),  /* WLAN EN  */
	};

	for (pin = 0; pin < ARRAY_SIZE(wlan_gpio_on); pin++) {
		rc = gpio_tlmm_config(wlan_gpio_on[pin], GPIO_CFG_ENABLE);
		if (rc) {
			printk(KERN_INFO "%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, wlan_gpio_on[pin], rc);
			return;
		}
	}
}

#ifdef CONFIG_PROC_FS
static void *frag_start(struct seq_file *m, loff_t *pos)
{
	pg_data_t *pgdat;
	loff_t node = *pos;
	for (pgdat = first_online_pgdat();
	     pgdat && node;
	     pgdat = next_online_pgdat(pgdat))
		--node;

	return pgdat;
}

static void *frag_next(struct seq_file *m, void *arg, loff_t *pos)
{
	pg_data_t *pgdat = (pg_data_t *)arg;

	(*pos)++;
	return next_online_pgdat(pgdat);
}

static void frag_stop(struct seq_file *m, void *arg)
{
}

static int bt_addr_file_show(struct seq_file *m, void *arg)
{
	seq_printf(m, "%02X:%02X:%02X:%02X:%02X:%02X\n",
		bt_mac_addr[0], bt_mac_addr[1], bt_mac_addr[2],
		bt_mac_addr[3], bt_mac_addr[4], bt_mac_addr[5]);
	return 0;

};
static const struct seq_operations bt_addr_file_op = {
	.start		= frag_start,
	.next		= frag_next,
	.stop		= frag_stop,
	.show		= bt_addr_file_show,
};

static int bt_addr_file_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &bt_addr_file_op);
};

static const struct file_operations bt_addr_file_ops = {
	.open		= bt_addr_file_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};
#endif

static int __init bt_addr_proc_init(void)
{
#ifdef CONFIG_PROC_FS
	proc_create("bt_mac_addr", S_IRUGO, NULL, &bt_addr_file_ops);
#endif
	return 0;
}

static void __init msm7x30_init(void)
{
	zeus_detect_product();

	zeus_temp_fixups();

	if (socinfo_init() < 0)
		printk(KERN_ERR "%s: socinfo_init() failed!\n", __func__);
	msm_clock_init(msm_clocks_7x30, msm_num_clocks_7x30);

	msm7x30_init_uart3();

	msm_spm_init(&msm_spm_data, 1);
	msm_acpu_clock_init(&msm7x30_clock_data);

	msm_device_otg.dev.platform_data = &msm_otg_pdata;
	msm_otg_pdata.swfi_latency =
	    msm_pm_data
	    [MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	msm_device_gadget_peripheral.dev.platform_data = &msm_gadget_pdata;

	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
	platform_add_devices(devices, ARRAY_SIZE(devices));

	msm_add_host(0, &msm_usb_host_pdata);

	hsusb_chg_set_supplicants(hsusb_chg_supplied_to,
				  ARRAY_SIZE(hsusb_chg_supplied_to));
	msm7x30_init_mmc();
	msm_qsd_spi_init();
	msm_fb_add_devices();
	spi_register_board_info(msm_spi_board_info,
				ARRAY_SIZE(msm_spi_board_info));

	msm_pm_set_platform_data(msm_pm_data, ARRAY_SIZE(msm_pm_data));
	msm_device_i2c_init();
	msm_device_i2c_2_init();
	qup_device_i2c_init();
	buses_init();
	msm7x30_init_marimba();

	snddev_poweramp_gpio_init();
	msm_snddev_init();
	aux_pcm_gpio_init();

	msm_init_pmic_vibrator();

	bt_addr_proc_init();
	bt_power_init();

	zeus_init_wlan_gpios();
	zeus_init_wlan_pdata();

	msm_cpufreq_register_voter(&kgsl_cpufreq_voter.voter);

	i2c_register_board_info(0, msm_i2c_board_info,
				ARRAY_SIZE(msm_i2c_board_info));

	i2c_register_board_info(2, msm_marimba_board_info,
				ARRAY_SIZE(msm_marimba_board_info));

	i2c_register_board_info(4 /* QUP ID */ , msm_camera_boardinfo,
				ARRAY_SIZE(msm_camera_boardinfo));

	msm_device_ssbi6.dev.platform_data = &msm_i2c_ssbi6_pdata;
	msm_device_ssbi7.dev.platform_data = &msm_i2c_ssbi7_pdata;
}

static unsigned pmem_sf_size = MSM_PMEM_SF_SIZE;
static void __init pmem_sf_size_setup(char **p)
{
	pmem_sf_size = memparse(*p, p);
}

__early_param("pmem_sf_size=", pmem_sf_size_setup);

static unsigned fb_size = MSM_FB_SIZE;
static void __init fb_size_setup(char **p)
{
	fb_size = memparse(*p, p);
}

__early_param("fb_size=", fb_size_setup);

static unsigned gpu_phys_size = MSM_GPU_PHYS_SIZE;
static void __init gpu_phys_size_setup(char **p)
{
	gpu_phys_size = memparse(*p, p);
}

__early_param("gpu_phys_size=", gpu_phys_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;
static void __init pmem_adsp_size_setup(char **p)
{
	pmem_adsp_size = memparse(*p, p);
}

__early_param("pmem_adsp_size=", pmem_adsp_size_setup);

static unsigned pmem_camera_size = MSM_PMEM_CAMERA_SIZE;
static void __init pmem_camera_size_setup(char **p)
{
	pmem_camera_size = memparse(*p, p);
}

__early_param("pmem_camera_size=", pmem_camera_size_setup);

static unsigned pmem_audio_size = MSM_PMEM_AUDIO_SIZE;
static void __init pmem_audio_size_setup(char **p)
{
	pmem_audio_size = memparse(*p, p);
}

__early_param("pmem_audio_size=", pmem_audio_size_setup);

static unsigned pmem_kernel_ebi1_size = PMEM_KERNEL_EBI1_SIZE;
static void __init pmem_kernel_ebi1_size_setup(char **p)
{
	pmem_kernel_ebi1_size = memparse(*p, p);
}

__early_param("pmem_kernel_ebi1_size=", pmem_kernel_ebi1_size_setup);

static void __init msm7x30_allocate_memory_regions(void)
{
	void *addr;
	unsigned long size;

	size = pmem_sf_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_pdata.start = __pa(addr);
		android_pmem_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for sf "
			"pmem arena\n", size, addr, __pa(addr));
	}

	size = fb_size ? : MSM_FB_SIZE;
	addr = alloc_bootmem(size);
	msm_fb_resources[0].start = __pa(addr);
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	pr_info("allocating %lu bytes at %p (%lx physical) for fb\n",
		size, addr, __pa(addr));

	size = gpu_phys_size;
	if (size) {
		addr = alloc_bootmem(size);
		kgsl_resources[1].start = __pa(addr);
		kgsl_resources[1].end = kgsl_resources[1].start + size - 1;
		pr_info("allocating %lu bytes at %p (%lx physical) for "
			"KGSL\n", size, addr, __pa(addr));
	}

	size = pmem_adsp_size;

	if (size) {
		addr = __alloc_bootmem(size, 8*1024, __pa(MAX_DMA_ADDRESS));
		android_pmem_adsp_pdata.start = __pa(addr);
		android_pmem_adsp_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for adsp "
			"pmem arena\n", size, addr, __pa(addr));
	}

	size = pmem_camera_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_camera_pdata.start = __pa(addr);
		android_pmem_camera_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for camera "
			"pmem arena\n", size, addr, __pa(addr));
	}

	size = pmem_audio_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_audio_pdata.start = __pa(addr);
		android_pmem_audio_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for audio "
			"pmem arena\n", size, addr, __pa(addr));
	}

	size = pmem_kernel_ebi1_size;
	if (size) {
		addr = alloc_bootmem_aligned(size, 0x100000);
		android_pmem_kernel_ebi1_pdata.start = __pa(addr);
		android_pmem_kernel_ebi1_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for kernel"
			" ebi1 pmem arena\n", size, addr, __pa(addr));
	}

}

static void __init msm7x30_map_io(void)
{
	msm_shared_ram_phys = 0x00100000;
	msm_map_msm7x30_io();
#ifdef CONFIG_ANDROID_RAM_CONSOLE
	ram_console_reserve_mem();
#endif
	msm7x30_allocate_memory_regions();
}

static void __init msm7x30_fixup(struct machine_desc *desc, struct tag *tags,
				 char **cmdline, struct meminfo *mi)
{
#define MSM_BANK0_BASE			PHYS_OFFSET
#define MSM_BANK0_SIZE			0x03C00000

#define MSM_BANK1_BASE			0x07400000
#define MSM_BANK1_SIZE			0x08C00000

#define MSM_BANK2_BASE			0x40000000
#define MSM_BANK2_SIZE			0x10000000

	mi->nr_banks = 3;
	mi->bank[0].start = MSM_BANK0_BASE;
	mi->bank[0].node = PHYS_TO_NID(MSM_BANK0_BASE);
	mi->bank[0].size = MSM_BANK0_SIZE;

	mi->bank[1].start = MSM_BANK1_BASE;
	mi->bank[1].node = PHYS_TO_NID(mi->bank[1].start);
	mi->bank[1].size = MSM_BANK1_SIZE;

	mi->bank[2].start = MSM_BANK2_BASE;
	mi->bank[2].size = MSM_BANK2_SIZE;
	mi->bank[2].node = PHYS_TO_NID(mi->bank[2].start);
}

static int __init board_bt_addr_setup(char *btaddr)
{
	sscanf(btaddr, "%02X:%02X:%02X:%02X:%02X:%02X",
		&bt_mac_addr[0], &bt_mac_addr[1], &bt_mac_addr[2],
		&bt_mac_addr[3], &bt_mac_addr[4], &bt_mac_addr[5]);
	return 1;
};

static int __init board_wifi_addr_setup(char *wifiaddr)
{
	sscanf(wifiaddr, "%02X:%02X:%02X:%02X:%02X:%02X",
		&wifi_mac_addr[0], &wifi_mac_addr[1], &wifi_mac_addr[2],
		&wifi_mac_addr[3], &wifi_mac_addr[4], &wifi_mac_addr[5]);
	return 1;
};

__setup("bt0.ieee_addr=", board_bt_addr_setup);
__setup("wifi0.eth_addr=", board_wifi_addr_setup);

MACHINE_START(SEMC_ZEUS, "zeus")
	.fixup = msm7x30_fixup,
	.boot_params = PHYS_OFFSET + 0x100,
	.map_io = msm7x30_map_io,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
MACHINE_END
