/*
 * Support for Intel Camera Imaging ISP subsystem.
 *
 * Copyright (c) 2010 - 2014 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
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

#include "ia_css_types.h"
#include "sh_css_defs.h"
#include "ia_css_debug.h"
#include "sh_css_frac.h"

#include "bnr/bnr_1.0/ia_css_bnr.host.h"
#include "ia_css_ynr.host.h"

const struct ia_css_nr_config default_nr_config = {
	16384,
	8192,
	1280,
	0,
	0
};

const struct ia_css_ee_config default_ee_config = {
	8192,
	128,
	2048
};

void
ia_css_nr_encode(
	struct sh_css_isp_ynr_params *to,
	const struct ia_css_nr_config *from)
{
	/* YNR (Y Noise Reduction) */
	to->threshold =
		uDIGIT_FITTING((unsigned)8192, 16, SH_CSS_BAYER_BITS);
	to->gain_all =
	    uDIGIT_FITTING(from->ynr_gain, 16, SH_CSS_YNR_GAIN_SHIFT);
	to->gain_dir =
	    uDIGIT_FITTING(from->ynr_gain, 16, SH_CSS_YNR_GAIN_SHIFT);
	to->threshold_cb =
	    uDIGIT_FITTING(from->threshold_cb, 16, SH_CSS_BAYER_BITS);
	to->threshold_cr =
	    uDIGIT_FITTING(from->threshold_cr, 16, SH_CSS_BAYER_BITS);
}

void
ia_css_yee_encode(
	struct sh_css_isp_yee_params *to,
	const struct ia_css_yee_config *from)
{
	int asiWk1 = (int) from->ee.gain;
	int asiWk2 = asiWk1 / 8;
	int asiWk3 = asiWk1 / 4;

	/* YEE (Y Edge Enhancement) */
	to->dirthreshold_s =
	    min((uDIGIT_FITTING(from->nr.direction, 16, SH_CSS_BAYER_BITS)
				    << 1),
		SH_CSS_BAYER_MAXVAL);
	to->dirthreshold_g =
	    min((uDIGIT_FITTING(from->nr.direction, 16, SH_CSS_BAYER_BITS)
				    << 4),
		SH_CSS_BAYER_MAXVAL);
	to->dirthreshold_width_log2 =
	    uFRACTION_BITS_FITTING(8);
	to->dirthreshold_width =
	    1 << to->dirthreshold_width_log2;
	to->detailgain =
	    uDIGIT_FITTING(from->ee.detail_gain, 11,
			   SH_CSS_YEE_DETAIL_GAIN_SHIFT);
	to->coring_s =
	    (uDIGIT_FITTING((unsigned)56, 16, SH_CSS_BAYER_BITS) *
	     from->ee.threshold) >> 8;
	to->coring_g =
	    (uDIGIT_FITTING((unsigned)224, 16, SH_CSS_BAYER_BITS) *
	     from->ee.threshold) >> 8;
	/* 8; // *1.125 ->[s4.8] */
	to->scale_plus_s =
	    (asiWk1 + asiWk2) >> (11 - SH_CSS_YEE_SCALE_SHIFT);
	/* 8; // ( * -.25)->[s4.8] */
	to->scale_plus_g =
	    (0 - asiWk3) >> (11 - SH_CSS_YEE_SCALE_SHIFT);
	/* 8; // *0.875 ->[s4.8] */
	to->scale_minus_s =
	    (asiWk1 - asiWk2) >> (11 - SH_CSS_YEE_SCALE_SHIFT);
	/* 8; // ( *.25 ) ->[s4.8] */
	to->scale_minus_g =
	    (asiWk3) >> (11 - SH_CSS_YEE_SCALE_SHIFT);
	to->clip_plus_s =
	    uDIGIT_FITTING((unsigned)32760, 16, SH_CSS_BAYER_BITS);
	to->clip_plus_g = 0;
	to->clip_minus_s =
	    uDIGIT_FITTING((unsigned)504, 16, SH_CSS_BAYER_BITS);
	to->clip_minus_g =
	    uDIGIT_FITTING((unsigned)32256, 16, SH_CSS_BAYER_BITS);
	to->Yclip = SH_CSS_BAYER_MAXVAL;
}

void
ia_css_nr_dump(
	const struct sh_css_isp_ynr_params *ynr,
	unsigned level)
{
	ia_css_debug_dtrace(level,
		"Y Noise Reduction:\n");
	ia_css_debug_dtrace(level, "\t%-32s = %d\n",
			"ynr_threshold", ynr->threshold);
	ia_css_debug_dtrace(level, "\t%-32s = %d\n",
			"ynr_gain_all", ynr->gain_all);
	ia_css_debug_dtrace(level, "\t%-32s = %d\n",
			"ynr_gain_dir", ynr->gain_dir);
	ia_css_debug_dtrace(level, "\t%-32s = %d\n",
			"ynr_threshold_cb", ynr->threshold_cb);
	ia_css_debug_dtrace(level, "\t%-32s = %d\n",
			"ynr_threshold_cr", ynr->threshold_cr);
}

void
ia_css_yee_dump(
	const struct sh_css_isp_yee_params *yee,
	unsigned level)
{
	ia_css_debug_dtrace(level,
		"Y Edge Enhancement:\n");
	ia_css_debug_dtrace(level, "\t%-32s = %d\n",
			"ynryee_dirthreshold_s",
			yee->dirthreshold_s);
	ia_css_debug_dtrace(level, "\t%-32s = %d\n",
			"ynryee_dirthreshold_g",
			yee->dirthreshold_g);
	ia_css_debug_dtrace(level, "\t%-32s = %d\n",
			"ynryee_dirthreshold_width_log2",
			yee->dirthreshold_width_log2);
	ia_css_debug_dtrace(level, "\t%-32s = %d\n",
			"ynryee_dirthreshold_width",
			yee->dirthreshold_width);
	ia_css_debug_dtrace(level, "\t%-32s = %d\n",
			"yee_detailgain",
			yee->detailgain);
	ia_css_debug_dtrace(level, "\t%-32s = %d\n",
			"yee_coring_s",
			yee->coring_s);
	ia_css_debug_dtrace(level, "\t%-32s = %d\n",
			"yee_coring_g",
			yee->coring_g);
	ia_css_debug_dtrace(level, "\t%-32s = %d\n",
			"yee_scale_plus_s",
			yee->scale_plus_s);
	ia_css_debug_dtrace(level, "\t%-32s = %d\n",
			"yee_scale_plus_g",
			yee->scale_plus_g);
	ia_css_debug_dtrace(level, "\t%-32s = %d\n",
			"yee_scale_minus_s",
			yee->scale_minus_s);
	ia_css_debug_dtrace(level, "\t%-32s = %d\n",
			"yee_scale_minus_g",
			yee->scale_minus_g);
	ia_css_debug_dtrace(level, "\t%-32s = %d\n",
			"yee_clip_plus_s",
			yee->clip_plus_s);
	ia_css_debug_dtrace(level, "\t%-32s = %d\n",
			"yee_clip_plus_g",
			yee->clip_plus_g);
	ia_css_debug_dtrace(level, "\t%-32s = %d\n",
			"yee_clip_minus_s",
			yee->clip_minus_s);
	ia_css_debug_dtrace(level, "\t%-32s = %d\n",
			"yee_clip_minus_g",
			yee->clip_minus_g);
	ia_css_debug_dtrace(level, "\t%-32s = %d\n",
			"ynryee_Yclip",
			yee->Yclip);
}

void
ia_css_nr_debug_dtrace(
	const struct ia_css_nr_config *config,
	unsigned level)
{
	ia_css_debug_dtrace(level,
		"config.direction=%d, "
		"config.bnr_gain=%d, config.ynr_gain=%d, "
		"config.threshold_cb=%d, config.threshold_cr=%d\n",
		config->direction,
		config->bnr_gain, config->ynr_gain,
		config->threshold_cb, config->threshold_cr);
}

void
ia_css_ee_debug_dtrace(
	const struct ia_css_ee_config *config,
	unsigned level)
{
	ia_css_debug_dtrace(level,
		"config.gain=%d, config.detail_gain=%d\n",
		config->threshold,
		config->gain, config->detail_gain);
}
