// SPDX-License-Identifier: GPL-2.0-only
/*
 * Userspace walk-through of drivers/power/supply/erhai_uisoc.h, the stock UI
 * percentage of the Erhai charger without the pack gauge. Build and run:
 *
 *   gcc -O2 -Wall -o /tmp/uisoc-sim tools/testing/erhai/uisoc-sim.c && /tmp/uisoc-sim
 *
 * Each scenario prints the shown percentage against the firmware's as time
 * passes and checks what the stock charger guarantees; the exit status is
 * the number of failed checks.
 */
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#define HZ			1000UL
#define ARRAY_SIZE(a)		(sizeof(a) / sizeof((a)[0]))
#define time_after(a, b)	((long)((b) - (a)) < 0)
#define time_before(a, b)	time_after(b, a)
#define msecs_to_jiffies(m)	((unsigned long)(m))

#include "../../../drivers/power/supply/erhai_uisoc.h"

static int failures;
static unsigned long now = 1000000;
static struct erhai_uisoc u;
static struct erhai_uisoc_in in;
static int max_up_step, prev_ui;

#define CHECK(cond, ...) do {						\
	if (!(cond)) {							\
		failures++;						\
		printf("  FAIL %s:%d: ", __func__, __LINE__);		\
		printf(__VA_ARGS__);					\
		printf("\n");						\
	}								\
} while (0)

/* Advance @secs of awake time, running the state machine when it asks. */
static void run(unsigned long secs, const char *what)
{
	unsigned long end = now + secs * HZ, next = now;
	bool changed;
	int min_ui = 101, max_ui = -1;

	while (time_before(now, end)) {
		if (!time_before(now, next)) {
			prev_ui = u.ui;
			next = now + erhai_uisoc_update(&u, &in, now, &changed);
			if (u.ui - prev_ui > max_up_step)
				max_up_step = u.ui - prev_ui;
			if (!in.online || !in.mmi)
				CHECK(u.ui <= prev_ui, "went up without charging: %d -> %d", prev_ui, u.ui);
			if (u.ui == 0)
				CHECK(in.vbat_mv < ERHAI_UISOC_UV_MV, "0 %% above the undervoltage threshold");
		}
		if (u.ui < min_ui)
			min_ui = u.ui;
		if (u.ui > max_ui)
			max_ui = u.ui;
		now += HZ;
	}
	printf("  %-46s fw %3d%%  ui %3d%% (range %d-%d)  full %d\n", what, in.soc, u.ui,
	       min_ui, max_ui, u.full);
}

/* @secs during which the firmware figure moves from its value to @to. */
static void drift(int to, unsigned long secs, const char *what)
{
	int steps = abs(to - in.soc);
	unsigned long per = steps ? secs / steps : secs;

	while (in.soc != to) {
		run(per, what);
		in.soc += to > in.soc ? 1 : -1;
	}
	run(per, what);
}

static void boot(int soc, int saved, bool online)
{
	in = (struct erhai_uisoc_in){ .soc = soc, .vbat_mv = 4300, .online = online, .mmi = true,
				      .ibat_ma = online ? -2000 : 800 };
	erhai_uisoc_init(&u, &in, saved, now);
	max_up_step = 0;
}

static void charge_to_full(void)
{
	printf("charge to full (PPS, then the buck; the firmware stops at 93 %%)\n");
	boot(80, -1, true);
	CHECK(u.ui == 81, "start: ui %d, expected 81 (80 + reserve 1)", u.ui);
	drift(93, 13 * 180, "charging 80 -> 93 %");
	CHECK(u.ui == 94, "ui %d before full, expected 94", u.ui);
	CHECK(!u.full, "full before the AP declared it");
	in.ibat_ma = -40;
	in.vbat_mv = 4484;
	in.ap_full = true;	/* sw full: 4484 mV, 40 mA */
	run(5, "AP full declared");
	CHECK(u.full && u.ui < 100, "full at once, ui still ramping (%d)", u.ui);
	run(60, "60 s later");
	CHECK(u.ui == 100, "ui %d one minute after full, expected 100", u.ui);
	CHECK(max_up_step == 1, "jumped by %d", max_up_step);
	run(3600, "an hour on the charger at full");
	CHECK(u.ui == 100 && u.full, "full lost on the charger");
}

static void unplug_and_discharge(void)
{
	printf("unplug at full, then discharge\n");
	in.online = false;
	in.ap_full = false;
	in.ibat_ma = 900;
	run(10, "unplugged, firmware 93 %");
	/* Smoothed 94 < 95: no hold after plug-out, as stock. */
	CHECK(u.ui >= 99 && !u.full, "ui %d full %d right after unplug", u.ui, u.full);
	run(140, "150 s after unplug");
	CHECK(u.ui == 99, "ui %d, expected 99: 150 s steps from 99", u.ui);
	run(1200, "20 min after unplug");
	CHECK(u.ui == 94, "ui %d, expected to meet the firmware's 93 + 1", u.ui);
	drift(40, 53 * 300, "discharging 93 -> 40 %");
	CHECK(u.ui >= 40 && u.ui <= 42, "ui %d does not follow the firmware's 40", u.ui);
	in.soc = 30;	/* the firmware jumps down by 10 */
	run(200, "firmware jumps 40 -> 30 %, 200 s");
	CHECK(u.ui > 30 && u.ui < 40, "ui %d: expected a gradual move from 40 towards 30", u.ui);
	run(1200, "20 min later");
	CHECK(u.ui == 30, "ui %d, expected 30", u.ui);
}

static void low_battery(void)
{
	printf("low battery\n");
	boot(2, -1, false);
	in.vbat_mv = 3400;
	run(600, "firmware 2 %, 3.40 V");
	CHECK(u.ui == 2, "ui %d at 2 %%", u.ui);
	in.soc = 0;
	run(600, "firmware 0 %, 3.40 V");
	CHECK(u.ui == 1, "ui %d: never 0 above the undervoltage threshold", u.ui);
	in.vbat_mv = 3250;
	/* 20 s from the last pass that saw the voltage above the threshold. */
	run(9, "3.25 V for 9 s");
	CHECK(u.ui == 1, "ui %d dropped before 10 s under 3.3 V", u.ui);
	run(40, "3.25 V for a minute");
	CHECK(u.ui == 0, "ui %d, expected 0 under 3.3 V", u.ui);
}

static void replug_at_95(void)
{
	printf("replug at 95 %% (no PPS above 90 %%)\n");
	boot(94, 95, false);
	CHECK(u.ui == 95, "start: ui %d, expected 95", u.ui);
	run(60, "on battery");
	in.online = true;
	in.ibat_ma = -1500;
	run(120, "plugged in");
	CHECK(u.ui == 95 && !u.full, "ui %d full %d after plug-in", u.ui, u.full);
	drift(96, 600, "buck charging 94 -> 96 %");
	CHECK(u.ui == 97, "ui %d, expected 97", u.ui);
	in.ap_full = true;
	in.ibat_ma = -500;
	run(40, "AP full at 96 %");
	CHECK(u.ui == 100 && u.full, "ui %d full %d", u.ui, u.full);
	in.ap_full = false;	/* recharge: output on again */
	in.soc = 95;
	run(600, "recharge, firmware 95 %");
	CHECK(u.ui == 100 && u.full, "ui %d full %d: stays full while at 100 on the charger", u.ui, u.full);
}

static void reboot_at_98(void)
{
	printf("reboot at 98 %%\n");
	boot(93, 98, false);
	CHECK(u.ui == 98, "start: ui %d, expected the saved 98", u.ui);
	run(149, "on battery for 149 s");
	CHECK(u.ui == 98, "ui %d", u.ui);
	run(30, "on battery for 3 min");
	CHECK(u.ui == 97, "ui %d, expected 97 after the 150 s step", u.ui);
	boot(93, 60, true);
	CHECK(u.ui == 94, "start with a stale saved 60: ui %d, expected 94", u.ui);
	boot(93, 98, true);
	in.ibat_ma = -300;
	run(600, "reboot at 98 on the charger");
	CHECK(u.ui == 98, "ui %d: not up without the firmware or full, not down while charging", u.ui);
	in.ap_full = true;
	in.ibat_ma = -30;
	run(30, "AP full");
	CHECK(u.ui == 100, "ui %d", u.ui);
}

static void unplug_at_full_firmware_99(void)
{
	printf("unplug at full with the firmware at 99 %%\n");
	boot(99, 100, true);
	in.ap_full = true;
	in.ibat_ma = -20;
	run(60, "full on the charger");
	in.online = false;
	in.ap_full = false;
	in.ibat_ma = 700;
	run(290, "290 s after unplug");
	CHECK(u.ui == 100, "ui %d: smoothed 100 >= 95 holds 100 for 300 s", u.ui);
	run(20, "310 s after unplug");
	CHECK(u.ui == 100, "ui %d: the firmware's 99 + 1 is 100", u.ui);
	in.soc = 97;
	run(400, "firmware 97 %");
	CHECK(u.ui == 98, "ui %d, expected 98", u.ui);
}

static void suspend_catch_up(void)
{
	printf("suspend\n");
	boot(60, 70, false);
	CHECK(u.ui == 70, "start %d", u.ui);
	u.sleep_s = 3600;	/* an hour asleep: 6 steps */
	run(1, "after an hour asleep");
	CHECK(u.ui == 64, "ui %d, expected 70 - 6", u.ui);
}

int main(void)
{
	charge_to_full();
	unplug_and_discharge();
	low_battery();
	replug_at_95();
	reboot_at_98();
	unplug_at_full_firmware_99();
	suspend_catch_up();
	printf("%s: %d failed checks\n", failures ? "FAIL" : "PASS", failures);
	return failures;
}
