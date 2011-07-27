/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
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

#ifndef __MSM_VPE_H__

#include <linux/types.h>
#include <media/msm_camera.h>

#define MSM_VPE_IOCTL_MAGIC 'v'

#define MSM_VPE_IOCTL_CONFIG_VPE  \
	_IOWR(MSM_VPE_IOCTL_MAGIC, 1, struct msm_vpe_cfg_cmd *)
#define MSM_VPE_IOCTL_VPE_TRANSFER \
	_IOWR(MSM_VPE_IOCTL_MAGIC, 2, struct msm_vpe_transfer_cfg *)
#define MSM_VPE_IOCTL_VPE_REGISTER \
	_IOWR(MSM_VPE_IOCTL_MAGIC, 3, struct msm_vpe_register_cfg *)
#define MSM_VPE_IOCTL_VPE_UNREGISTER \
	_IO(MSM_VPE_IOCTL_MAGIC, 4)

struct msm_vpe_crop_info {
	int x;
	int y;
	int w;
	int h;
};

struct msm_vpe_transfer_cfg {
	void *srcAddr;
	void *dstAddr;
	struct msm_pmem_info src_info;
	struct msm_pmem_info dst_info;
	struct msm_vpe_crop_info src_crop;
	struct msm_vpe_crop_info dst_crop;
};

struct msm_vpe_register_cfg {
	struct msm_pmem_info inf;
	struct msm_vpe_crop_info crop;
};
#endif
