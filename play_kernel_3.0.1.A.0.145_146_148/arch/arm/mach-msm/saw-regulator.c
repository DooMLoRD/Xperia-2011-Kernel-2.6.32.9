/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
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
 */

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>

#include "spm.h"
#include "saw-regulator.h"

#define SMPS_BAND_MASK			0xC0
#define SMPS_VPROG_MASK			0x3F

#define SMPS_BAND_1			0x40
#define SMPS_BAND_2			0x80
#define SMPS_BAND_3			0xC0

#define SMPS_BAND_1_VMIN		350000
#define SMPS_BAND_1_VMAX		650000
#define SMPS_BAND_1_VSTEP		6250

#define SMPS_BAND_2_VMIN		700000
#define SMPS_BAND_2_VMAX		1487500
#define SMPS_BAND_2_VSTEP		12500

#define SMPS_BAND_3_VMIN		1400000
#define SMPS_BAND_3_VMAX		3300000
#define SMPS_BAND_3_VSTEP		50000

static int saw_set_voltage(struct regulator_dev *dev, int min_uV, int max_uV)
{
	int rc;
	u8 vlevel, band;

	if (min_uV < SMPS_BAND_2_VMIN) {
		vlevel = ((min_uV - SMPS_BAND_1_VMIN) / SMPS_BAND_1_VSTEP);
		band = SMPS_BAND_1;
	} else if (min_uV < SMPS_BAND_3_VMIN) {
		vlevel = ((min_uV - SMPS_BAND_2_VMIN) / SMPS_BAND_2_VSTEP);
		band = SMPS_BAND_2;
	} else {
		vlevel = ((min_uV - SMPS_BAND_3_VMIN) / SMPS_BAND_3_VSTEP);
		band = SMPS_BAND_3;
	}

	rc = msm_spm_set_vdd(rdev_get_id(dev), vlevel | band);
	if (rc)
		pr_err("%s: msm_spm_set_vdd failed %d\n", __func__, rc);

	return rc;
}

static struct regulator_ops saw_ops = {
	.set_voltage = saw_set_voltage,
};

static struct regulator_desc saw_descrip[] = {
	{
		.name	= "8901_s0",
		.id	= SAW_VREG_ID_S0,
		.ops	= &saw_ops,
		.type	= REGULATOR_VOLTAGE,
		.owner	= THIS_MODULE,
	},
	{
		.name	= "8901_s1",
		.id	= SAW_VREG_ID_S1,
		.ops	= &saw_ops,
		.type	= REGULATOR_VOLTAGE,
		.owner	= THIS_MODULE,
	},
};

static struct regulator_dev *saw_rdev[2];

static int __devinit saw_probe(struct platform_device *pdev)
{
	struct regulator_init_data *init_data;
	int rc = 0;

	if (pdev == NULL)
		return -EINVAL;

	if (pdev->id != SAW_VREG_ID_S0 && pdev->id != SAW_VREG_ID_S1)
		return -ENODEV;

	init_data = pdev->dev.platform_data;

	saw_rdev[pdev->id] = regulator_register(&saw_descrip[pdev->id],
			&pdev->dev, init_data, NULL);
	if (IS_ERR(saw_rdev[pdev->id]))
		rc = PTR_ERR(saw_rdev[pdev->id]);

	pr_info("%s: id=%d, rc=%d\n", __func__, pdev->id, rc);

	return rc;
}

static int __devexit saw_remove(struct platform_device *pdev)
{
	regulator_unregister(saw_rdev[pdev->id]);
	return 0;
}

static struct platform_driver saw_driver = {
	.probe = saw_probe,
	.remove = __devexit_p(saw_remove),
	.driver = {
		.name = "saw-regulator",
		.owner = THIS_MODULE,
	},
};

static int __init saw_init(void)
{
	return platform_driver_register(&saw_driver);
}

static void __exit saw_exit(void)
{
	platform_driver_unregister(&saw_driver);
}

postcore_initcall(saw_init);
module_exit(saw_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("SAW regulator driver");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:saw-regulator");
