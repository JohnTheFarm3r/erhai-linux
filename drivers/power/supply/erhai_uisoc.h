/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _ERHAI_UISOC_H
#define _ERHAI_UISOC_H
/*
 * The percentage the stock charger shows, for the board without the pack
 * gauge, where the charger firmware's own figure is all there is and stops
 * short of 100 % with a full pack, with the charger devicetree values of
 * this board:
 *
 * - oplus_comm_smooth_to_soc(): oplus,smooth_switch, oplus,reserve_chg_soc 1
 *   (from 55 % the firmware figure is shown one higher, averaged over up to
 *   four samples around that step);
 * - oplus_comm_ui_soc_update(): one step at a time, up at most every 10 s
 *   while charging, towards 100 % once the AP has declared the pack full,
 *   down while charging only with no charge current, down without a charger
 *   at the stock ladder (40 to 300 s a step), never below 1 % except below
 *   the undervoltage threshold, the catch-up after a suspend;
 * - oplus_comm_report_full(): full after 300 s at 100 % on a charger, cleared
 *   below 100 % unless the AP has declared it;
 * - the start-up value from the percentage saved before the reboot
 *   (oplus_comm_init_ui_soc path of the gauge topic, BATT_RTC_SOC).
 *
 * Not ported: what this board's devicetree leaves off (low battery control,
 * deep discharge keep-2/3 %, recharge by UI percentage, decimal display),
 * the parallel-battery and wireless paths, the charge up limit of the stock
 * settings app.
 *
 * Pure logic on jiffies, so that a userspace harness can drive it.
 */

#define ERHAI_UISOC_SMOOTH_FROM		55	/* ui_soc_smooth_table[1][0] */
#define ERHAI_UISOC_JUMP_RANGE		1	/* SOC_JUMP_RANGE_VAL */
#define ERHAI_UISOC_FIFO		4	/* SMOOTH_SOC_MAX_FIFO_LEN */
#define ERHAI_UISOC_SMOOTH_MS		4500
#define ERHAI_UISOC_UP_S		10
#define ERHAI_UISOC_FULL_S		300	/* oplus_spec,batt_full_time */
#define ERHAI_UISOC_UV_MV		3300	/* oplus_spec,vbat_uv_thr_mv */
#define ERHAI_UISOC_CHG_UV_MV		3200	/* oplus_spec,vbat_charging_uv_thr_mv */
#define ERHAI_UISOC_UV_HOLD_S		20	/* NON_CHARGE_FORCE_DEC_INTERVAL */
#define ERHAI_UISOC_CHG_UV_HOLD_S	60	/* CHARGE_FORCE_DEC_INTERVAL */
#define ERHAI_UISOC_UV_STEP_S		15	/* UI_SOC_DEC_SPEED_OF_UV_BATT */
#define ERHAI_UISOC_DOWN_MA		(-50)	/* ALLOW_UISOC_DOWN_CURRENT */
#define ERHAI_UISOC_PLUGOUT		95	/* PLUGOUT_SOC_THRESHOLD */
#define ERHAI_UISOC_BOOT_GAP		20
#define ERHAI_UISOC_POLL_S		10	/* when nothing else is due */

struct erhai_uisoc {
	bool ready;
	bool full;		/* stock batt_full: what the status reports */
	bool online;
	int ui, smooth, delta;
	int pre_soc;
	int fifo[ERHAI_UISOC_FIFO];
	unsigned int fifo_idx, avg_cnt;
	bool jump;
	unsigned long smooth_at, up_at, down_at, uv_at, full_at;
	unsigned long sleep_s, save_sleep_s;
};

struct erhai_uisoc_in {
	int soc;		/* the firmware's percentage */
	int vbat_mv;
	int ibat_ma;		/* stock sign: negative while charging */
	bool online;		/* a charger is attached */
	bool mmi;		/* and charging is allowed (no bypass, no limit) */
	bool ap_full;		/* the AP's full decision (sw/hw full) */
	bool fastchg;		/* a fast charging session is running */
	bool warm;		/* temp_region >= WARM (45 C) */
};

/* soc_down_ladder, soc_down_delay_hz_ladder, soc_down_chg_delay_hz_ladder:
 * the stock defaults, the devicetree sets none of them.
 */
static inline unsigned int erhai_uisoc_down_s(int ui, bool online)
{
	static const int ladder[] = { 2, 11, 60, 95, 100 };
	static const unsigned int off[] = { 40, 40, 40, 60, 150, 300 };
	static const unsigned int chg[] = { 90, 40, 40, 60, 150, 300 };
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ladder); i++)
		if (ui < ladder[i])
			break;
	return online ? chg[i] : off[i];
}

static inline int erhai_uisoc_clamp_delta(int d)
{
	return d < 1 ? 1 : d > 10 ? 10 : d;	/* MIN_DELTA_SOC, MAX_DELTA_SOC */
}

/* oplus_comm_smooth_to_soc() with reserve 1. */
static inline void erhai_uisoc_smooth(struct erhai_uisoc *u, int soc,
				      unsigned long now, bool force)
{
	int shown, sum = 0, valid = 0, cnt, idx, i;

	if (force || u->pre_soc < 0) {
		u->jump = false;
		u->avg_cnt = 1;
		for (i = 0; i < ERHAI_UISOC_FIFO; i++)
			u->fifo[i] = -1;
		u->fifo_idx = 0;
		u->smooth_at = now + msecs_to_jiffies(ERHAI_UISOC_SMOOTH_MS);
	} else if (u->pre_soc == soc && time_before(now, u->smooth_at)) {
		return;
	} else {
		u->smooth_at = now + msecs_to_jiffies(ERHAI_UISOC_SMOOTH_MS);
	}

	shown = soc >= ERHAI_UISOC_SMOOTH_FROM ? soc + 1 : soc;
	if (shown > 100)
		shown = 100;
	u->fifo[u->fifo_idx] = shown;

	if (force || u->pre_soc != soc) {
		u->jump = false;
		if (u->pre_soc >= 0 && abs(u->pre_soc - soc) > 1)
			u->avg_cnt = 1;
		if (soc >= ERHAI_UISOC_SMOOTH_FROM - ERHAI_UISOC_JUMP_RANGE &&
		    soc <= ERHAI_UISOC_SMOOTH_FROM + ERHAI_UISOC_JUMP_RANGE) {
			u->jump = true;
			u->avg_cnt = ERHAI_UISOC_FIFO;
		}
	}

	if (u->avg_cnt == 1) {
		u->smooth = u->fifo[u->fifo_idx];
	} else {
		cnt = u->avg_cnt;
		idx = u->fifo_idx;
		while (cnt--) {
			if (u->fifo[idx] > 0) {
				valid++;
				sum += u->fifo[idx];
			}
			idx = (idx + ERHAI_UISOC_FIFO - 1) % ERHAI_UISOC_FIFO;
		}
		u->smooth = valid ? sum / valid : u->fifo[u->fifo_idx];
	}
	u->pre_soc = soc;
	u->fifo_idx = (u->fifo_idx + 1) % ERHAI_UISOC_FIFO;
	if (!u->jump && u->avg_cnt > 1)
		u->avg_cnt--;
}

/*
 * Start-up: the percentage shown before the reboot if it is still within
 * 20 of the firmware's, else the smoothed firmware figure, never 0.
 */
static inline void erhai_uisoc_init(struct erhai_uisoc *u, const struct erhai_uisoc_in *in,
				    int saved, unsigned long now)
{
	*u = (struct erhai_uisoc){ .pre_soc = -1, .delta = 1 };
	erhai_uisoc_smooth(u, in->soc, now, true);
	if (u->smooth == 0)
		u->smooth = in->soc;
	if (saved > 0 && saved <= 100 && abs(saved - in->soc) < ERHAI_UISOC_BOOT_GAP)
		u->ui = saved;
	else
		u->ui = u->smooth > 0 ? u->smooth : 1;
	u->online = in->online;
	u->up_at = u->down_at = u->uv_at = u->full_at = now;
	u->ready = true;
}

/* Charger attached or removed: oplus_comm_plugin_work(). */
static inline void erhai_uisoc_plug(struct erhai_uisoc *u, bool online, unsigned long now)
{
	if (online == u->online)
		return;
	u->online = online;
	if (online) {
		u->full_at = now;	/* no full report right after plug-in */
		return;
	}
	u->full = false;
	if (u->smooth >= ERHAI_UISOC_PLUGOUT && u->ui >= ERHAI_UISOC_PLUGOUT)
		u->down_at = now;
}

/*
 * One pass of oplus_comm_ui_soc_update() and oplus_comm_report_full().
 * Returns the jiffies until the next pass is due; *changed tells whether
 * the shown percentage or the full state moved.
 */
static inline unsigned long erhai_uisoc_update(struct erhai_uisoc *u,
					       const struct erhai_uisoc_in *in,
					       unsigned long now, bool *changed)
{
	unsigned long up_at, down_at, uv_at, delay = 0;
	bool charging = in->online, was_full = u->full, force_down = false;
	int ui = u->ui, old_ui = u->ui, smooth;

	erhai_uisoc_plug(u, in->online, now);
	erhai_uisoc_smooth(u, in->soc, now, false);
	smooth = u->smooth;
	/* oplus_comm_set_batt_full(): sw_full and hw_full_by_sw are full. */
	if (in->online && in->ap_full)
		u->full = true;

	up_at = u->up_at + ERHAI_UISOC_UP_S * HZ;
	down_at = u->down_at + erhai_uisoc_down_s(ui, charging) * HZ;

	/* oplus_comm_is_under_uv(): towards 0 % at 15 s a step. */
	if (in->vbat_mv < (charging ? ERHAI_UISOC_CHG_UV_MV : ERHAI_UISOC_UV_MV)) {
		uv_at = u->uv_at + (charging ? ERHAI_UISOC_CHG_UV_HOLD_S :
					       ERHAI_UISOC_UV_HOLD_S) * HZ;
		delay = 5 * HZ;
		if (time_after(now, uv_at)) {
			down_at = u->down_at + ERHAI_UISOC_UV_STEP_S * HZ;
			force_down = true;
		}
	} else {
		u->uv_at = now;
	}
	if (force_down) {
		if (time_after(now, down_at)) {
			ui = ui > 0 ? ui - 1 : 0;
			u->delta = erhai_uisoc_clamp_delta(ui - smooth);
		}
		goto check_update_time;
	}

	/* After a suspend: catch up by 1 % per 10 min slept. */
	if (u->sleep_s > 0 && ui > smooth && !u->full) {
		unsigned long margin = u->sleep_s / 600;
		bool update = true;

		if (margin == 0) {
			u->save_sleep_s += u->sleep_s;
			if (u->save_sleep_s >= 60 && ui - smooth > 2) {
				ui = ui > 1 ? ui - 1 : 1;
				u->save_sleep_s = 0;
			} else {
				update = false;
			}
		} else if (margin < (unsigned long)(ui - smooth)) {
			ui -= margin;
			u->save_sleep_s = 0;
		} else {
			ui = smooth;
			u->save_sleep_s = 0;
		}
		u->sleep_s = 0;
		if (ui < 1)
			ui = 1;
		if (update) {
			u->delta = erhai_uisoc_clamp_delta(ui - smooth);
			goto done;
		}
	}

	if (charging && in->mmi) {
		if (ui < smooth && time_after(now, up_at)) {
			ui = ui < 100 ? ui + 1 : 100;
			u->sleep_s = u->save_sleep_s = 0;
		} else if (ui < 100 && in->ap_full && !in->warm && time_after(now, up_at)) {
			ui++;
			u->sleep_s = u->save_sleep_s = 0;
		} else if (ui > smooth + u->delta && !in->ap_full &&
			   time_after(now, down_at) && in->ibat_ma > ERHAI_UISOC_DOWN_MA) {
			ui = ui > 1 ? ui - 1 : 1;
			u->save_sleep_s = 0;
		}
		if (ui == smooth)
			u->delta = 1;
	} else {
		if (ui > smooth && time_after(now, down_at)) {
			ui = ui > 1 ? ui - 1 : 1;
			u->save_sleep_s = 0;
		}
		u->delta = erhai_uisoc_clamp_delta(ui - smooth);
	}

check_update_time:
	/* The next pass just after a gate: the stock steps strictly after it. */
	if (!delay)
		delay = ERHAI_UISOC_POLL_S * HZ;
	if (!time_after(now, down_at) && delay > down_at - now + 1)
		delay = down_at - now + 1;
	if (!time_after(now, up_at) && delay > up_at - now + 1)
		delay = up_at - now + 1;
done:
	if (!delay)
		delay = ERHAI_UISOC_POLL_S * HZ;
	if (u->ui != ui) {
		u->ui = ui;
		u->up_at = u->down_at = u->full_at = now;
	} else if (!u->full && ui == 100 && charging && in->mmi && !in->fastchg) {
		/* oplus_comm_report_full(): 100 % for 300 s on a charger. */
		unsigned long at = u->full_at + ERHAI_UISOC_FULL_S * HZ;

		if (time_after(now, at))
			u->full = true;
		else if (delay > at - now + 1)
			delay = at - now + 1;
	} else if (ui < 100 && u->full && !in->ap_full) {
		u->full = false;
	} else {
		u->full_at = now;
	}
	*changed = u->ui != old_ui || u->full != was_full;
	return delay ?: 1;
}

#endif
