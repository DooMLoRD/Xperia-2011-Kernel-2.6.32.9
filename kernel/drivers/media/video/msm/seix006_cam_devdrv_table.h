/* drivers/media/video/msm/seix006_cam_devdrv_table.h
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef _CAMSENSOR_SEIX006_CAM_DEVDRV_TABLE_H_
#define _CAMSENSOR_SEIX006_CAM_DEVDRV_TABLE_H_

enum reg_bits {
	REG_BITS_8 = 0,
	REG_BITS_16,
	REG_BITS_32
};

struct reg_entry {
	uint16_t address;
	uint32_t data;
	enum reg_bits reg_bits;
};

#define seix006_set_regs(table) seix006_send_reg_table(table, (sizeof_##table/sizeof(struct reg_entry)))

extern const struct reg_entry seix006_GEN_period_1_ES1[];
extern int32_t sizeof_seix006_GEN_period_1_ES1;
extern const struct reg_entry seix006_GEN_period_1_ES2[];
extern int32_t sizeof_seix006_GEN_period_1_ES2;
extern const struct reg_entry seix006_vendor_1_period_2_ES1[];
extern int32_t sizeof_seix006_vendor_1_period_2_ES1;
extern const struct reg_entry seix006_vendor_1_period_2_ES2[];
extern int32_t sizeof_seix006_vendor_1_period_2_ES2;
extern const struct reg_entry seix006_vendor_0_period_2_ES1[];
extern int32_t sizeof_seix006_vendor_0_period_2_ES1;
extern const struct reg_entry seix006_vendor_0_period_2_ES2[];
extern int32_t sizeof_seix006_vendor_0_period_2_ES2;
extern const struct reg_entry seix006_vendor_1_period_3_ES1[];
extern int32_t sizeof_seix006_vendor_1_period_3_ES1;
extern const struct reg_entry seix006_vendor_1_period_3_ES2[];
extern int32_t sizeof_seix006_vendor_1_period_3_ES2;
extern const struct reg_entry seix006_vendor_0_period_3_ES1[];
extern int32_t sizeof_seix006_vendor_0_period_3_ES1;
extern const struct reg_entry seix006_vendor_0_period_3_ES2[];
extern int32_t sizeof_seix006_vendor_0_period_3_ES2;
extern const struct reg_entry seix006_vendor_0_SHD_1_ES2[];
extern int32_t sizeof_seix006_vendor_0_SHD_1_ES2;
extern const struct reg_entry seix006_vendor_0_SHD_2_ES2[];
extern int32_t sizeof_seix006_vendor_0_SHD_2_ES2;
extern const struct reg_entry seix006_vendor_0_SHD_3_ES2[];
extern int32_t sizeof_seix006_vendor_0_SHD_3_ES2;
extern const struct reg_entry seix006_vendor_1_SHD_1_ES2[];
extern int32_t sizeof_seix006_vendor_1_SHD_1_ES2;
extern const struct reg_entry seix006_vendor_1_SHD_2_ES2[];
extern int32_t sizeof_seix006_vendor_1_SHD_2_ES2;
extern const struct reg_entry seix006_vendor_1_SHD_3_ES2[];
extern int32_t sizeof_seix006_vendor_1_SHD_3_ES2;
extern const struct reg_entry seix006_vf_resolution_1280x720[];
extern int32_t sizeof_seix006_vf_resolution_1280x720;
extern const struct reg_entry seix006_vf_resolution_800x480[];
extern int32_t sizeof_seix006_vf_resolution_800x480;
extern const struct reg_entry seix006_vf_resolution_640x480[];
extern int32_t sizeof_seix006_vf_resolution_640x480;
extern const struct reg_entry seix006_vf_resolution_320x240[];
extern int32_t sizeof_seix006_vf_resolution_320x240;
extern const struct reg_entry seix006_snapshot_resolution_640x480[];
extern int32_t sizeof_seix006_snapshot_resolution_640x480;
extern const struct reg_entry seix006_snapshot_resolution_800x480[];
extern int32_t sizeof_seix006_snapshot_resolution_800x480;
extern const struct reg_entry seix006_snapshot_resolution_1280x960[];
extern int32_t sizeof_seix006_snapshot_resolution_1280x960;
extern const struct reg_entry seix006_snapshot_resolution_1632x1224[];
extern int32_t sizeof_seix006_snapshot_resolution_1632x1224;
extern const struct reg_entry seix006_snapshot_resolution_176x144[];
extern int32_t sizeof_seix006_snapshot_resolution_176x144;
extern const struct reg_entry seix006_snapshot_resolution_320x240[];
extern int32_t sizeof_seix006_snapshot_resolution_320x240;
extern const struct reg_entry seix006_snapshot_resolution_352x288[];
extern int32_t sizeof_seix006_snapshot_resolution_352x288;
extern const struct reg_entry seix006_snapshot_resolution_800x600[];
extern int32_t sizeof_seix006_snapshot_resolution_800x600;
extern const struct reg_entry seix006_snapshot_resolution_1024x768[];
extern int32_t sizeof_seix006_snapshot_resolution_1280x768;
extern const struct reg_entry seix006_snapshot_resolution_1280x768[];
extern int32_t sizeof_seix006_snapshot_resolution_1024x768;
extern const struct reg_entry seix006_snapshot_resolution_1280x720[];
extern int32_t sizeof_seix006_snapshot_resolution_1280x720;
extern const struct reg_entry seix006_snapshot_resolution_1600x1200[];
extern int32_t sizeof_seix006_snapshot_resolution_1600x1200;
extern const struct reg_entry seix006_snapshot_resolution_1920x1080[];
extern int32_t sizeof_seix006_snapshot_resolution_1920x1080;
extern const struct reg_entry seix006_snapshot_resolution_2048x1536[];
extern int32_t sizeof_seix006_snapshot_resolution_2048x1536;
extern const struct reg_entry seix006_snapshot_resolution_2592x1944[];
extern int32_t sizeof_seix006_snapshot_resolution_2592x1944;
extern const struct reg_entry seix006_thumbnail_size_WVGA[];
extern int32_t sizeof_seix006_thumbnail_size_WVGA;
extern const struct reg_entry seix006_thumbnail_size_VGA[];
extern int32_t sizeof_seix006_thumbnail_size_VGA;
extern const struct reg_entry seix006_thumbnail_size_QVGA[];
extern int32_t sizeof_seix006_thumbnail_size_QVGA;
extern const struct reg_entry seix006_thumbnail_size_QCIF[];
extern int32_t sizeof_seix006_thumbnail_size_QCIF;
extern const struct reg_entry seix006_thumbnail_size_WQVGA[];
extern int32_t sizeof_seix006_thumbnail_size_WQVGA;
extern const struct reg_entry seix006_thumbnail_size_512x384[];
extern int32_t sizeof_seix006_thumbnail_size_512x384;
extern const struct reg_entry seix006_thumbnail_size_512x288[];
extern int32_t sizeof_seix006_thumbnail_size_512x288;
extern const struct reg_entry seix006_thumbnail_size_480x288[];
extern int32_t sizeof_seix006_thumbnail_size_480x288;
extern const struct reg_entry seix006_thumbnail_size_432x288[];
extern int32_t sizeof_seix006_thumbnail_size_432x288;
extern const struct reg_entry seix006_thumbnail_size_352x288[];
extern int32_t sizeof_seix006_thumbnail_size_352x288;
extern const struct reg_entry seix006_mode_monitor[];
extern int32_t sizeof_seix006_mode_monitor;
extern const struct reg_entry seix006_mode_half_release[];
extern int32_t sizeof_seix006_mode_half_release;
extern const struct reg_entry seix006_hr_auto_start[];
extern int32_t sizeof_seix006_hr_auto_start;
extern const struct reg_entry seix006_hr_auto_reset[];
extern int32_t sizeof_seix006_hr_auto_reset;
extern const struct reg_entry seix006_mode_movie[];
extern int32_t sizeof_seix006_mode_movie;
extern const struct reg_entry seix006_hr_twilight[];
extern int32_t sizeof_seix006_hr_twilight;
extern const struct reg_entry seix006_hr_reset[];
extern int32_t sizeof_seix006_hr_reset;
extern const struct reg_entry seix006_hr_LED[];
extern int32_t sizeof_seix006_hr_LED;
extern const struct reg_entry seix006_hr_LED_reset[];
extern int32_t sizeof_seix006_hr_LED_reset;
extern const struct reg_entry seix006_test_pattern_on[];
extern int32_t sizeof_seix006_test_pattern_on;
extern const struct reg_entry seix006_test_pattern_off[];
extern int32_t sizeof_seix006_test_pattern_off;
extern const struct reg_entry seix006_prepare_mode_capture[];
extern int32_t sizeof_seix006_prepare_mode_capture;
extern const struct reg_entry seix006_mode_capture[];
extern int32_t sizeof_seix006_mode_capture;
extern const struct reg_entry seix006_INTCLR[];
extern int32_t sizeof_seix006_INTCLR;
extern const struct reg_entry seix006_MONI_REFRESH_F[];
extern int32_t sizeof_seix006_MONI_REFRESH_F;
extern const struct reg_entry seix006_GEN_scene_normal[];
extern int32_t sizeof_seix006_GEN_scene_normal;
extern const struct reg_entry seix006_GEN_scene_macro[];
extern int32_t sizeof_seix006_GEN_scene_macro;
extern const struct reg_entry seix006_GEN_scene_sports[];
extern int32_t sizeof_seix006_GEN_scene_sports;
extern const struct reg_entry seix006_GEN_scene_twilight[];
extern int32_t sizeof_seix006_GEN_scene_twilight;
extern const struct reg_entry seix006_GEN_scene_beach_and_snow[];
extern int32_t sizeof_seix006_GEN_scene_beach_and_snow;
extern const struct reg_entry seix006_GEN_scene_landscape[];
extern int32_t sizeof_seix006_GEN_scene_landscape;
extern const struct reg_entry seix006_GEN_scene_portrait[];
extern int32_t sizeof_seix006_GEN_scene_portrait;
extern const struct reg_entry seix006_GEN_scene_twilight_portrait[];
extern int32_t sizeof_seix006_GEN_scene_twilight_portrait;
extern const struct reg_entry seix006_GEN_scene_document[];
extern int32_t sizeof_seix006_GEN_scene_document;

extern const struct reg_entry seix006_framerate_30[];
extern int32_t sizeof_seix006_framerate_30;
extern const struct reg_entry seix006_framerate_15[];
extern int32_t sizeof_seix006_framerate_15;
extern const struct reg_entry seix006_framerate_variable[];
extern int32_t sizeof_seix006_framerate_variable;
extern const struct reg_entry seix006_framerate_fixed[];
extern int32_t sizeof_seix006_framerate_fixed;
extern const uint16_t AEO_table[];
extern const struct reg_entry seix006_mode_capture_YUV[];
extern int32_t sizeof_seix006_mode_capture_YUV;
extern const struct reg_entry seix006_mode_capture_RGB[];
extern int32_t sizeof_seix006_mode_capture_RGB;
extern const struct reg_entry seix006_iso_auto[];
extern int32_t sizeof_seix006_iso_auto;
extern const struct reg_entry seix006_iso_100[];
extern int32_t sizeof_seix006_iso_100;
extern const struct reg_entry seix006_iso_200[];
extern int32_t sizeof_seix006_iso_200;
extern const struct reg_entry seix006_iso_400[];
extern int32_t sizeof_seix006_iso_400;
extern const struct reg_entry seix006_iso_800[];
extern int32_t sizeof_seix006_iso_800;
extern const struct reg_entry seix006_iso_1600[];
extern int32_t sizeof_seix006_iso_1600;

extern const struct reg_entry seix006_focus_mode_auto[];
extern int32_t sizeof_seix006_focus_mode_auto;
extern const struct reg_entry seix006_focus_mode_continuous[];
extern int32_t sizeof_seix006_focus_mode_continuous;
extern const struct reg_entry seix006_primary_focus_window_continuous[];
extern int32_t sizeof_seix006_primary_focus_window_continuous;
extern const struct reg_entry seix006_primary_focus_window_auto[];
extern int32_t sizeof_seix006_primary_focus_window_auto;
#endif /* _CAMSENSOR_SEIX006_CAM_DEVDRV_TABLE_H_ */
