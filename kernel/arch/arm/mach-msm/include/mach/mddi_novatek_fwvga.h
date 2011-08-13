#ifndef __ARCH_ARM_MACH_MSM_MDDI_NOVATEK_FWVGA_H
#define __ARCH_ARM_MACH_MSM_MDDI_NOVATEK_FWVGA_H

#include <linux/types.h>

#define MDDI_NOVATEK_FWVGA_NAME "mddi_novatek_fwvga"
#define MDDI_NOVATEK_I2C_NAME "novatek_i2c"

struct novatek_fwvga_platform_data {
	int (*power)(int on);
	int (*reset)(void);
};

struct panel_id;

struct novatek_i2c_pdata {
	const struct panel_id **panels;
};

extern const struct panel_id novatek_panel_id_tmd_mdp42_rev_c;
extern const struct panel_id novatek_panel_id_tmd_mdp42_rev_d;
extern const struct panel_id novatek_panel_id_sharp_ls040t8lx01_rev_c_x;
extern const struct panel_id novatek_panel_id_sharp_ls040t8lx01_rev_c;
extern const struct panel_id novatek_panel_id_sharp_ls040t8lx01_rev_d;
extern const struct panel_id novatek_panel_id_sharp_ls042t3lx_type1;
extern const struct panel_id novatek_panel_id_sharp_ls042t3lx;
extern const struct panel_id novatek_panel_id_sony_acx424akm_type1;
extern const struct panel_id novatek_panel_id_sony_acx424akm;
extern const struct panel_id novatek_panel_id_sony_acx427ak;
extern const struct panel_id novatek_panel_id_sony_acx424ak;
extern const struct panel_id novatek_panel_id_hitachi_dx09d09vm_type1;
extern const struct panel_id novatek_panel_id_hitachi_dx09d09vm;
extern const struct panel_id novatek_panel_id_sharp_ls033t3lx01;
extern const struct panel_id novatek_panel_id_tmd_lt033mdv1000;

#endif
