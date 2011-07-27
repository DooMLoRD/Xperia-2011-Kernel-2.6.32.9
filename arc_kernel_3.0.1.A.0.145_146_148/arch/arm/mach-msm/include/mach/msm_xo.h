/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
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
 */
#ifndef __MACH_MSM_XO_H
#define __MACH_MSM_XO_H

enum msm_xo_ids {
	TCXO_D0,
	TCXO_D1,
	TCXO_A0,
	TCXO_A1,
	PXO,
	NUM_XO_IDS
};

enum msm_xo_modes {
	XO_MODE_OFF,
	XO_MODE_PIN_CTRL,
	XO_MODE_ON,
	NUM_XO_MODES
};

struct msm_xo_voter;

#ifdef CONFIG_MSM_XO
struct msm_xo_voter *msm_xo_get(enum msm_xo_ids xo_id, const char *voter);
void msm_xo_put(struct msm_xo_voter *xo_voter);
int msm_xo_mode_vote(struct msm_xo_voter *xo_voter, enum msm_xo_modes xo_mode);
#else
static inline struct msm_xo_voter *msm_xo_get(enum msm_xo_ids xo_id,
		const char *voter)
{
	return NULL;
}

static inline void msm_xo_put(struct msm_xo_voter *xo_voter) { }

static inline int msm_xo_mode_vote(struct msm_xo_voter *xo_voter,
		enum msm_xo_modes xo_mode)
{
	return 0;
}
#endif /* CONFIG_MSM_XO */

#endif
