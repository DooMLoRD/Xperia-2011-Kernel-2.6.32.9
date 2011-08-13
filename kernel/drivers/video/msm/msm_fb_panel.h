/* Copyright (c) 2008-2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef MSM_FB_PANEL_H
#define MSM_FB_PANEL_H

#include "msm_fb_def.h"
#include <linux/msm_fb_panel.h>

/* panel id type */
typedef struct panel_id_s {
	uint16 id;
	uint16 type;
} panel_id_type;

/* panel class */
typedef enum {
	DISPLAY_LCD = 0,	/* lcd = ebi2/mddi */
	DISPLAY_LCDC,		/* lcdc */
	DISPLAY_TV,		/* TV Out */
	DISPLAY_EXT_MDDI,	/* External MDDI */
} DISP_TARGET;


/*===========================================================================
  FUNCTIONS PROTOTYPES
============================================================================*/
struct platform_device *msm_fb_device_alloc(struct msm_fb_panel_data *pdata,
						u32 type, u32 id);
int panel_next_on(struct platform_device *pdev);
int panel_next_off(struct platform_device *pdev);

int lcdc_device_register(struct msm_panel_info *pinfo);

int mddi_toshiba_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel);

#endif /* MSM_FB_PANEL_H */
