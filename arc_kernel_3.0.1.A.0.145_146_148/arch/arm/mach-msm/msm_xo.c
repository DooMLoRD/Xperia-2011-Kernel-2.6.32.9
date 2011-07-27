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

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/mutex.h>

#include <mach/msm_xo.h>

#include "rpm.h"

static DEFINE_MUTEX(msm_xo_mutex);

struct msm_xo {
	unsigned votes[NUM_XO_MODES];
	unsigned mode;
};

struct msm_xo_voter {
	const char *name;
	unsigned mode;
	struct msm_xo *xo;
};

static struct msm_xo msm_xo_sources[NUM_XO_IDS];

static int msm_xo_update_vote(struct msm_xo *xo)
{
	int ret;
	unsigned vote, prev_vote = xo->mode;
	struct msm_rpm_iv_pair cmd;

	if (xo->votes[XO_MODE_ON])
		vote = XO_MODE_ON;
	else if (xo->votes[XO_MODE_PIN_CTRL])
		vote = XO_MODE_PIN_CTRL;
	else
		vote = XO_MODE_OFF;

	if (vote == prev_vote)
		return 0;

	/*
	 * Change the vote here to simplify the TCXO logic. If the RPM
	 * command fails we'll rollback.
	 */
	xo->mode = vote;

	if (xo == &msm_xo_sources[PXO]) {
		cmd.id = MSM_RPM_ID_PXO_CLK;
		cmd.value = msm_xo_sources[PXO].mode ? 1 : 0;
		ret = 0;
		/* TODO: Implement PXO voting */
	} else {
		cmd.id = MSM_RPM_ID_CXO_BUFFERS;
		cmd.value = (msm_xo_sources[TCXO_D0].mode << 0) |
			    (msm_xo_sources[TCXO_D1].mode << 8) |
			    (msm_xo_sources[TCXO_A0].mode << 16) |
			    (msm_xo_sources[TCXO_A1].mode << 24);
		/* FIXME: not yet supported.
		 * ret = msm_rpm_set(MSM_RPM_CTX_SET_0, &cmd, 1);
		 */
		ret = 0;
	}

	if (ret)
		xo->mode = prev_vote;

	return ret;
}

static int __msm_xo_mode_vote(struct msm_xo_voter *xo_voter, unsigned mode)
{
	int ret;
	struct msm_xo *xo = xo_voter->xo;

	if (xo_voter->mode == mode)
		return 0;

	xo->votes[mode]++;
	xo->votes[xo_voter->mode]--;
	ret = msm_xo_update_vote(xo);
	if (ret) {
		xo->votes[xo_voter->mode]++;
		xo->votes[mode]--;
		goto out;
	}
	xo_voter->mode = mode;
out:
	return ret;
}

/**
 * msm_xo_mode_vote() - Vote for an XO to be ON, OFF, or under PIN_CTRL
 * @xo_voter - Valid handle returned from msm_xo_get()
 * @mode - Mode to vote for (ON, OFF, PIN_CTRL)
 *
 * Vote for an XO to be either ON, OFF, or under PIN_CTRL. Votes are
 * aggregated with ON taking precedence over PIN_CTRL taking precedence
 * over OFF.
 *
 * This function returns 0 on success or a negative error code on failure.
 */
int msm_xo_mode_vote(struct msm_xo_voter *xo_voter, enum msm_xo_modes mode)
{
	int ret;

	if (mode >= NUM_XO_MODES)
		return -EINVAL;

	mutex_lock(&msm_xo_mutex);
	ret = __msm_xo_mode_vote(xo_voter, mode);
	mutex_unlock(&msm_xo_mutex);

	return ret;
}
EXPORT_SYMBOL(msm_xo_mode_vote);

/**
 * msm_xo_get() - Get a voting handle for an XO
 * @xo_id - XO identifier
 * @voter - Debug string to identify users
 *
 * XO voters vote for OFF by default. This function returns a pointer
 * indicating success. An ERR_PTR is returned on failure.
 *
 * If XO voting is disabled, %NULL is returned.
 */
struct msm_xo_voter *msm_xo_get(enum msm_xo_ids xo_id, const char *voter)
{
	int ret;
	struct msm_xo_voter *xo_voter;

	if (xo_id >= NUM_XO_IDS) {
		ret = -EINVAL;
		goto err;
	}

	xo_voter = kzalloc(sizeof(*xo_voter), GFP_KERNEL);
	if (!xo_voter) {
		ret = -ENOMEM;
		goto err;
	}

	xo_voter->name = kstrdup(voter, GFP_KERNEL);
	if (!xo_voter->name) {
		ret = -ENOMEM;
		goto err_name;
	}

	xo_voter->xo = &msm_xo_sources[xo_id];

	/* Voters vote for OFF by default */
	mutex_lock(&msm_xo_mutex);
	xo_voter->xo->votes[XO_MODE_OFF]++;
	mutex_unlock(&msm_xo_mutex);

	return xo_voter;

err_name:
	kfree(xo_voter);
err:
	return ERR_PTR(ret);
}
EXPORT_SYMBOL(msm_xo_get);

/**
 * msm_xo_put() - Release a voting handle
 * @xo_voter - Valid handle returned from msm_xo_get()
 *
 * Release a reference to an XO voting handle. This also removes the voter's
 * vote, therefore calling msm_xo_mode_vote(xo_voter, XO_MODE_OFF) beforehand
 * is unnecessary.
 */
void msm_xo_put(struct msm_xo_voter *xo_voter)
{
	mutex_lock(&msm_xo_mutex);
	__msm_xo_mode_vote(xo_voter, XO_MODE_OFF);
	xo_voter->xo->votes[XO_MODE_OFF]--;
	mutex_unlock(&msm_xo_mutex);

	kfree(xo_voter->name);
	kfree(xo_voter);
}
EXPORT_SYMBOL(msm_xo_put);
