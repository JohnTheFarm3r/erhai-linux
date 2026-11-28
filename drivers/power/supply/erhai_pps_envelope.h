/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _ERHAI_PPS_ENVELOPE_H
#define _ERHAI_PPS_ENVELOPE_H
/*
 * Stock pps_charge_third_strategy of the erhai charger devicetree
 * (oplus,pps_charge): what the stock charger gives a third-party PD PPS
 * source. Rows are the cell
 * voltage up to which the step applies and the 2:1 bus current of the step.
 * The table is the same in the overlays with and without the pack gauge.
 *
 * It ends lower than the SUPERVOOC table of erhai_cell_envelope.h (4.49 V
 * against 4.545 V, 5 A against 6.1 A) and is what the engine uses where the
 * firmware gauges the pack, see erhai_pps.c.
 */
#include "erhai_charge_curve.h"

static const struct erhai_charge_step pps_little_cold_soc20[] = {
	{ 4150, 3850, 0 },
	{ 4150, 3500, 0 },
	{ 4150, 3000, 0 },
	{ 4480, 2650, 0 },
	{ 4480, 2500, 0 },
	{ 4480, 2000, 0 },
	{ 4480, 1500, 0 },
	{ 4490, 1000, 1 },
};

static const struct erhai_charge_step pps_cool_soc20[] = {
	{ 4150, 4850, 0 },
	{ 4150, 4500, 0 },
	{ 4150, 4000, 0 },
	{ 4150, 3000, 0 },
	{ 4200, 2950, 0 },
	{ 4480, 2500, 0 },
	{ 4480, 2000, 0 },
	{ 4480, 1500, 0 },
	{ 4490, 1000, 1 },
};

static const struct erhai_charge_step pps_normal_low_soc20[] = {
	{ 4230, 5000, 0 },
	{ 4230, 4500, 0 },
	{ 4230, 4000, 0 },
	{ 4380, 3950, 0 },
	{ 4380, 3500, 0 },
	{ 4380, 3000, 0 },
	{ 4480, 2950, 0 },
	{ 4480, 2500, 0 },
	{ 4480, 2000, 0 },
	{ 4480, 1500, 0 },
	{ 4490, 1000, 1 },
};

static const struct erhai_charge_step pps_normal_high_soc20[] = {
	{ 4200, 5000, 0 },
	{ 4230, 4850, 0 },
	{ 4230, 4500, 0 },
	{ 4230, 4000, 0 },
	{ 4380, 3900, 0 },
	{ 4380, 3500, 0 },
	{ 4380, 3000, 0 },
	{ 4460, 2950, 0 },
	{ 4460, 2500, 0 },
	{ 4460, 2000, 0 },
	{ 4460, 1500, 0 },
	{ 4470, 1000, 1 },
};

static const struct erhai_charge_step pps_warm_soc20[] = {
	{ 4120, 3200, 0 },
	{ 4120, 3000, 0 },
	{ 4120, 2500, 0 },
	{ 4120, 2000, 0 },
	{ 4120, 1500, 0 },
	{ 4130, 1000, 1 },
};

static const struct erhai_charge_step pps_little_cold_soc55[] = {
	{ 4480, 2000, 0 },
	{ 4480, 1500, 0 },
	{ 4490, 1000, 1 },
};

static const struct erhai_charge_step pps_cool_soc55[] = {
	{ 4480, 2500, 0 },
	{ 4480, 2000, 0 },
	{ 4480, 1500, 0 },
	{ 4490, 1000, 1 },
};

static const struct erhai_charge_step pps_little_cool_soc55[] = {
	{ 4150, 3000, 0 },
	{ 4200, 2950, 0 },
	{ 4480, 2500, 0 },
	{ 4480, 2000, 0 },
	{ 4480, 1500, 0 },
	{ 4490, 1000, 1 },
};

static const struct erhai_charge_step pps_little_cool_high_soc55[] = {
	{ 4380, 3000, 0 },
	{ 4480, 2950, 0 },
	{ 4480, 2500, 0 },
	{ 4480, 2000, 0 },
	{ 4480, 1500, 0 },
	{ 4490, 1000, 1 },
};

static const struct erhai_charge_step pps_cold_soc75[] = {
	{ 4480, 1500, 0 },
	{ 4490, 1000, 1 },
};

static const struct erhai_charge_step pps_normal_high_soc75[] = {
	{ 4380, 3000, 0 },
	{ 4460, 2950, 0 },
	{ 4460, 2500, 0 },
	{ 4460, 2000, 0 },
	{ 4460, 1500, 0 },
	{ 4470, 1000, 1 },
};

static const struct erhai_charge_step pps_normal_high_soc85[] = {
	{ 4460, 2500, 0 },
	{ 4460, 2000, 0 },
	{ 4460, 1500, 0 },
	{ 4470, 1000, 1 },
};

static const struct erhai_charge_step pps_normal_high_soc90[] = {
	{ 4460, 1500, 0 },
	{ 4470, 1000, 1 },
};

static const struct erhai_charge_step pps_last[] = {
	{ 4490, 1000, 1 },
};

static const struct erhai_charge_step pps_warm_last[] = {
	{ 4130, 1000, 1 },
};

/*
 * [state of charge up to 20, 35, 55, 75, 85, 90 %][0, 5, 12, 16, 21, 35,
 * 44 .. 51 C], the regions of erhai_curve_select().
 */
static const struct erhai_charge_curve pps_curves[6][7] = {
	{
		ERHAI_CURVE(pps_little_cold_soc20),
		ERHAI_CURVE(pps_cool_soc20),
		ERHAI_CURVE(pps_cool_soc20),
		ERHAI_CURVE(pps_normal_low_soc20),
		ERHAI_CURVE(pps_normal_low_soc20),
		ERHAI_CURVE(pps_normal_high_soc20),
		ERHAI_CURVE(pps_warm_soc20),
	},
	{
		ERHAI_CURVE(pps_little_cold_soc20),
		ERHAI_CURVE(pps_cool_soc20),
		ERHAI_CURVE(pps_cool_soc20),
		ERHAI_CURVE(pps_normal_low_soc20),
		ERHAI_CURVE(pps_normal_low_soc20),
		ERHAI_CURVE(pps_normal_high_soc20),
		ERHAI_CURVE(pps_warm_soc20),
	},
	{
		ERHAI_CURVE(pps_little_cold_soc55),
		ERHAI_CURVE(pps_cool_soc55),
		ERHAI_CURVE(pps_little_cool_soc55),
		ERHAI_CURVE(pps_little_cool_high_soc55),
		ERHAI_CURVE(pps_normal_low_soc20),
		ERHAI_CURVE(pps_normal_high_soc20),
		ERHAI_CURVE(pps_warm_soc20),
	},
	{
		ERHAI_CURVE(pps_cold_soc75),
		ERHAI_CURVE(pps_cold_soc75),
		ERHAI_CURVE(pps_little_cold_soc55),
		ERHAI_CURVE(pps_little_cold_soc55),
		ERHAI_CURVE(pps_cool_soc55),
		ERHAI_CURVE(pps_normal_high_soc75),
		ERHAI_CURVE(pps_warm_last),
	},
	{
		ERHAI_CURVE(pps_last),
		ERHAI_CURVE(pps_last),
		ERHAI_CURVE(pps_cold_soc75),
		ERHAI_CURVE(pps_cold_soc75),
		ERHAI_CURVE(pps_little_cold_soc55),
		ERHAI_CURVE(pps_normal_high_soc85),
		ERHAI_CURVE(pps_warm_last),
	},
	{
		ERHAI_CURVE(pps_last),
		ERHAI_CURVE(pps_last),
		ERHAI_CURVE(pps_last),
		ERHAI_CURVE(pps_last),
		ERHAI_CURVE(pps_cold_soc75),
		ERHAI_CURVE(pps_normal_high_soc90),
		ERHAI_CURVE(pps_warm_last),
	},
};

/* The last step of every table of the region: where PPS charging ends. */
static inline int erhai_pps_cv(int temp)
{
	if (temp >= 440)
		return 4130;
	if (temp >= 350)
		return 4470;
	return 4490;
}

/* oplus,pps_charge_strategy_soc: no PPS session starts above this. */
#define ERHAI_PPS_ENTRY_SOC_MAX		90
/* oplus,pps_warm_allow_soc / _vol: entry from 44 C needs both. */
#define ERHAI_PPS_WARM_SOC_MAX		55
#define ERHAI_PPS_WARM_MV_MAX		4000
/* oplus,pps_strategy_batt_high_temp / _high_current: from 43 C (back under 42 C). */
#define ERHAI_PPS_HOT_ON		430
#define ERHAI_PPS_HOT_OFF		420
#define ERHAI_PPS_HOT_MA		3000
/* oplus,pps_ibat_over_third for the 55 W class: 8 one-second samples. */
#define ERHAI_PPS_IBAT_MAX_MA		11400
#endif
