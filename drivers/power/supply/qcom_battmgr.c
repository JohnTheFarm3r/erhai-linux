// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2019-2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022, Linaro Ltd
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * Copyright (C) 2020-2022 Oplus. All rights reserved.
 */
#include <linux/auxiliary_bus.h>
#include <linux/cleanup.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pm_wakeup.h>
#include <linux/power_supply.h>
#include <linux/pm_wakeup.h>
#include <linux/property.h>
#include <linux/soc/qcom/pdr.h>
#include <linux/soc/qcom/pmic_glink.h>
#include <linux/spinlock.h>
#include <linux/math.h>
#include <linux/suspend.h>
#include <linux/thermal.h>
#include <linux/units.h>
#include <linux/usb/pd.h>
#include <linux/usb/role.h>
#include <linux/workqueue.h>

#include "erhai_cell_envelope.h"
#include "erhai_buck.h"
#include "erhai_uisoc.h"
#include "erhai_otg_glink.h"

#define BATTMGR_CHEMISTRY_LEN	4
#define BATTMGR_STRING_LEN	128

enum qcom_battmgr_variant {
	QCOM_BATTMGR_SC8280XP,
	QCOM_BATTMGR_SM8350,
	QCOM_BATTMGR_SM8550,
	QCOM_BATTMGR_X1E80100,
};

/* Battery current is positive while discharging */
#define QCOM_BATTMGR_QUIRK_BATT_CURRENT_INVERTED	BIT(0)
/* BATT_POWER_NOW does not follow the battery power */
#define QCOM_BATTMGR_QUIRK_NO_BATT_POWER_NOW		BIT(1)
/* The charge control opcode and properties mean something else */
#define QCOM_BATTMGR_QUIRK_NO_CHARGE_CONTROL		BIT(2)
/*
 * Battery property 24 switches charging on (1) and off (0), so the driver
 * can enforce the charge control thresholds itself.
 */
#define QCOM_BATTMGR_QUIRK_CHARGE_ENABLE		BIT(3)
/*
 * The firmware leaves the USB VBUS boost to the OS: once allowed to, it
 * reports an attached sink with a notification and waits for the OS to
 * switch the boost on.
 */
#define QCOM_BATTMGR_QUIRK_OTG_VBUS			BIT(4)
/* The OS can ask the firmware for a USB PD PPS contract and its limits */
#define QCOM_BATTMGR_QUIRK_PPS				BIT(5)
/* OPLUS adapter type, PD notifications and fixed PDO voltage requests */
#define QCOM_BATTMGR_QUIRK_OPLUS_PD			BIT(6)

#define BATTMGR_BAT_STATUS		0x1

#define BATTMGR_REQUEST_NOTIFICATION	0x4

#define BATTMGR_NOTIFICATION		0x7
#define NOTIF_BAT_PROPERTY		0x30
#define NOTIF_USB_PROPERTY		0x32
#define NOTIF_WLS_PROPERTY		0x34
#define NOTIF_BAT_STATUS		0x80
#define NOTIF_BAT_INFO			0x81
#define NOTIF_BAT_CHARGING_STATE	0x83
/* OPLUS firmware: USB port events */
#define NOTIF_OPLUS_OTG_ENABLE		0x50
#define NOTIF_OPLUS_OTG_DISABLE		0x51
#define NOTIF_OPLUS_CID_DETECT		0x53
#define NOTIF_OPLUS_TYPEC_STATE		0x55
#define NOTIF_OPLUS_PLUGIN		0x57
#define NOTIF_OPLUS_APSD_DONE		0x58
/* OPLUS firmware: UFCS and reverse charging events */
#define NOTIF_OPLUS_PD_SVOOC		0x56
#define NOTIF_OPLUS_UFCS_HANDSHAKE_OK	0x71
#define NOTIF_OPLUS_UFCS_DISABLE_MOS	0x73
#define NOTIF_OPLUS_SOURCECAP_DONE	0x79
#define NOTIF_OPLUS_POWER_ROLE		0x7d
#define NOTIF_OPLUS_PD_HARD_RESET	0x7f

/* OPLUS buffer opcodes, separate from BATTMGR_NOTIFICATION */
#define BATTMGR_OPLUS_READ_PDOS		0x10004
#define BATTMGR_OPLUS_PD_INFO		0x1000a
#define OPLUS_READ_BUFFER_WORDS		128
#define OPLUS_PD_INFO_WORDS		8
#define OPLUS_PD_INFO_VDM_ID		0
#define OPLUS_USB_TYPE_PD_SDP		17
#define OPLUS_USB_TYPE_APPLE_BRICK_ID	9

#define BATTMGR_BAT_INFO		0x9

#define BATTMGR_BAT_DISCHARGE_TIME	0xc

#define BATTMGR_BAT_CHARGE_TIME		0xd

#define BATTMGR_BAT_PROPERTY_GET	0x30
#define BATTMGR_BAT_PROPERTY_SET	0x31
#define BATT_STATUS			0
#define BATT_HEALTH			1
#define BATT_PRESENT			2
#define BATT_CHG_TYPE			3
#define BATT_CAPACITY			4
#define BATT_SOH			5
#define BATT_VOLT_OCV			6
#define BATT_VOLT_NOW			7
#define BATT_VOLT_MAX			8
#define BATT_CURR_NOW			9
#define BATT_CHG_CTRL_LIM		10
#define BATT_CHG_CTRL_LIM_MAX		11
#define BATT_TEMP			12
#define BATT_TECHNOLOGY			13
#define BATT_CHG_COUNTER		14
#define BATT_CYCLE_COUNT		15
#define BATT_CHG_FULL_DESIGN		16
#define BATT_CHG_FULL			17
#define BATT_MODEL_NAME			18
#define BATT_TTF_AVG			19
#define BATT_TTE_AVG			20
#define BATT_RESISTANCE			21
#define BATT_POWER_NOW			22
#define BATT_POWER_AVG			23
#define BATT_CHG_CTRL_EN		24
#define BATT_CHG_CTRL_START_THR		25
#define BATT_CHG_CTRL_END_THR		26
/* OPLUS firmware: set to 0 to stop charging, the system keeps running on input */
/* Charger firmware gauge qualification properties. */
#define BATT_OPLUS_AUTH			30
/* BATT_RTC_SOC: where the stock charger keeps the percentage over a reboot. */
#define BATT_OPLUS_RTC_SOC		31
#define BATT_OPLUS_HMAC			40
#define BATT_OPLUS_FULL_PARA		48
#define BATT_OPLUS_SET_FULL		64
#define BATT_OPLUS_CHG_EN		24
/* Fixed PDO voltage in mV, written with BATTMGR_BAT_PROPERTY_SET */
#define BATT_OPLUS_SET_QC		26	/* non-soccp BATT_SET_QC, mV */
#define BATT_OPLUS_SET_PDO		25

#define BATTMGR_USB_PROPERTY_GET	0x32
#define BATTMGR_USB_PROPERTY_SET	0x33
#define USB_ONLINE			0
#define USB_VOLT_NOW			1
#define USB_VOLT_MAX			2
#define USB_CURR_NOW			3
#define USB_CURR_MAX			4
#define USB_INPUT_CURR_LIMIT		5
#define USB_TYPE			6
#define USB_ADAP_TYPE			7
#define USB_MOISTURE_DET_EN		8
#define USB_MOISTURE_DET_STS		9
#define USB_OPLUS_RERUN_AICL		67
#define USB_OPLUS_CID_STATUS		21 /* USB_CID_STATUS */
#define USB_REAL_TYPE			11
/* OPLUS firmware: VBUS boost control */
#define USB_OPLUS_OTG_AP_ENABLE		17
#define USB_OPLUS_OTG_SWITCH		18
#define USB_OPLUS_OTG_VBUS_ENABLE	24
#define USB_OPLUS_OTG_BOOST_CURRENT	64
/* OPLUS firmware: USB PD PPS, voltage in mV and current in mA */
#define USB_OPLUS_PPS_TYPE		32
#define USB_OPLUS_PPS_VOLT		34
#define USB_OPLUS_PPS_CURR		35
#define USB_OPLUS_PPS_MAX_CURR		36

#define BATTMGR_WLS_PROPERTY_GET	0x34
#define BATTMGR_WLS_PROPERTY_SET	0x35
#define WLS_ONLINE			0
#define WLS_VOLT_NOW			1
#define WLS_VOLT_MAX			2
#define WLS_CURR_NOW			3
#define WLS_CURR_MAX			4
#define WLS_TYPE			5
#define WLS_BOOST_EN			6

#define BATTMGR_CHG_CTRL_LIMIT_EN	0x48
#define CHARGE_CTRL_START_THR_MIN	50
#define CHARGE_CTRL_START_THR_MAX	95
#define CHARGE_CTRL_END_THR_MIN		55
#define CHARGE_CTRL_END_THR_MAX		100
#define CHARGE_CTRL_DELTA_SOC		5

/* Interval of the state of charge check while an end threshold is set */
#define CHARGE_CTRL_POLL_MS		30000

/* The stock system waits this long for the answer to a property write */
#define BATTMGR_SET_TIMEOUT_MS		2500

/* VBUS boost current limit for a USB sink, from the vendor devicetree */
#define OTG_BOOST_CURRENT_MA		1250

/* Fixed PD limits from the stock charger devicetree. */
#define PD_FIXED_MV			9000
#define PD_DEFAULT_MV			5000
/* PM8550: top HAL usb_icl[] step, also used for the PPS buck path. */
#define PD_INPUT_MAX_UA			3000000
/* Stock oplus_spec,pd-iclmax-ma: 18 W from a 9 V source. */
#define PD_INPUT_STOCK_UA		2000000
/* 24924 common.dtsi input-power-mw: 10 W for VOOC/DCP at 5 V. */
#define ERHAI_SERIAL_5V_UA		2000000
#define PD_TRANSITION_UA			500000
#define PD_VBUS_RISEN_UV			7500000
#define PD_VBUS_MAX_UV			11000000
#define PD_VBAT_START_UV			5000000
#define PD_VBAT_STOP_UV			5100000
/* The stock PD configuration waits up to 15 s, checking at 1 s intervals. */
#define PD_POLL_MS			1000
#define PD_CONFIG_TIMEOUT_MS		15000
#define PD_SHUTDOWN_WAIT_MS		100

enum qcom_battmgr_pd_state {
	PD_IDLE,
	PD_WAIT,
	PD_ACTIVE,
	PD_FAILED,
};

enum qcom_battmgr_pd_event {
	PD_EVENT_CAPS,
	PD_EVENT_RESET,
	PD_EVENT_SERVICE_RESET,
	PD_EVENT_PARTNER,
	PD_EVENT_TAIL,
	PD_EVENT_DETACH,
	PD_EVENT_SVOOC_RESPONSE,
	PD_EVENT_UFCS_RESPONSE,
};

/* Whether the gauge of oplus,external-gauge is fitted on this board. */
enum qcom_battmgr_gauge_state {
	GAUGE_UNKNOWN,
	GAUGE_PRESENT,
	GAUGE_ABSENT,
};

/* Successful delivery and discovery attempts belong to this attachment. */
struct erhai_route_retry {
	bool delivered;
	unsigned int attempts;
	unsigned int recovery_step;
	unsigned long next;
};

static DEFINE_MUTEX(erhai_bridge_lock);
static struct qcom_battmgr *erhai_battmgr;

struct qcom_battmgr_enable_request {
	struct pmic_glink_hdr hdr;
	__le32 battery_id;
	__le32 power_state;
	__le32 low_capacity;
	__le32 high_capacity;
};

struct qcom_battmgr_property_request {
	struct pmic_glink_hdr hdr;
	__le32 battery;
	__le32 property;
	__le32 value;
};

struct qcom_battmgr_read_pdos_request {
	struct pmic_glink_hdr hdr;
	__le32 data_size;
};

struct qcom_battmgr_read_pdos_response {
	struct pmic_glink_hdr hdr;
	__le32 data[OPLUS_READ_BUFFER_WORDS];
	__le32 data_size;
};

struct qcom_battmgr_pd_info {
	struct pmic_glink_hdr hdr;
	__le32 data[OPLUS_PD_INFO_WORDS];
	__le32 data_size;
	__le32 msg_id;
};

struct qcom_battmgr_update_request {
	struct pmic_glink_hdr hdr;
	__le32 battery_id;
};

struct qcom_battmgr_charge_time_request {
	struct pmic_glink_hdr hdr;
	__le32 battery_id;
	__le32 percent;
	__le32 reserved;
};

struct qcom_battmgr_discharge_time_request {
	struct pmic_glink_hdr hdr;
	__le32 battery_id;
	__le32 rate; /* 0 for current rate */
	__le32 reserved;
};

struct qcom_battmgr_charge_ctrl_request {
	struct pmic_glink_hdr hdr;
	__le32 enable;
	__le32 target_soc;
	__le32 delta_soc;
};

struct qcom_battmgr_message {
	struct pmic_glink_hdr hdr;
	union {
		struct {
			__le32 property;
			__le32 value;
			__le32 result;
		} intval;
		struct {
			__le32 property;
			char model[BATTMGR_STRING_LEN];
		} strval;
		struct {
			/*
			 * 0: mWh
			 * 1: mAh
			 */
			__le32 power_unit;
			__le32 design_capacity;
			__le32 last_full_capacity;
			/*
			 * 0 nonrechargable
			 * 1 rechargable
			 */
			__le32 battery_tech;
			__le32 design_voltage; /* mV */
			__le32 capacity_low;
			__le32 capacity_warning;
			__le32 cycle_count;
			/* thousandth of percent */
			__le32 accuracy;
			__le32 max_sample_time_ms;
			__le32 min_sample_time_ms;
			__le32 max_average_interval_ms;
			__le32 min_average_interval_ms;
			/* granularity between low and warning */
			__le32 capacity_granularity1;
			/* granularity between warning and full */
			__le32 capacity_granularity2;
			/*
			 * 0: no
			 * 1: cold
			 * 2: hot
			 */
			__le32 swappable;
			__le32 capabilities;
			char model_number[BATTMGR_STRING_LEN];
			char serial_number[BATTMGR_STRING_LEN];
			char battery_type[BATTMGR_STRING_LEN];
			char oem_info[BATTMGR_STRING_LEN];
			char battery_chemistry[BATTMGR_CHEMISTRY_LEN];
			char uid[BATTMGR_STRING_LEN];
			__le32 critical_bias;
			u8 day;
			u8 month;
			__le16 year;
			__le32 battery_id;
		} info;
		struct {
			/*
			 * BIT(0) discharging
			 * BIT(1) charging
			 * BIT(2) critical low
			 */
			__le32 battery_state;
			/* mWh or mAh, based on info->power_unit */
			__le32 capacity;
			__le32 rate;
			/* mv */
			__le32 battery_voltage;
			/*
			 * BIT(0) power online
			 * BIT(1) discharging
			 * BIT(2) charging
			 * BIT(3) battery critical
			 */
			__le32 power_state;
			/*
			 * 1: AC
			 * 2: USB
			 * 3: Wireless
			 */
			__le32 charging_source;
			__le32 temperature;
		} status;
		__le32 time;
		__le32 notification;
	};
};

#define BATTMGR_CHARGING_SOURCE_AC	1
#define BATTMGR_CHARGING_SOURCE_USB	2
#define BATTMGR_CHARGING_SOURCE_WIRELESS 3

enum qcom_battmgr_unit {
	QCOM_BATTMGR_UNIT_mWh = 0,
	QCOM_BATTMGR_UNIT_mAh = 1
};

struct qcom_battmgr_info {
	bool valid;

	bool present;
	unsigned int charge_type;
	unsigned int design_capacity;
	unsigned int last_full_capacity;
	unsigned int voltage_max_design;
	unsigned int voltage_max;
	unsigned int capacity_low;
	unsigned int capacity_warning;
	unsigned int cycle_count;
	unsigned int charge_count;
	unsigned int charge_ctrl_start;
	unsigned int charge_ctrl_end;
	char model_number[BATTMGR_STRING_LEN];
	char serial_number[BATTMGR_STRING_LEN];
	char oem_info[BATTMGR_STRING_LEN];
	unsigned char technology;
	unsigned char day;
	unsigned char month;
	unsigned short year;
};

struct qcom_battmgr_status {
	unsigned int status;
	unsigned int health;
	unsigned int capacity;
	unsigned int percent;
	int current_now;
	int power_now;
	unsigned int voltage_now;
	unsigned int voltage_ocv;
	unsigned int temperature;
	unsigned int resistance;
	unsigned int soh_percent;

	unsigned int discharge_time;
	unsigned int charge_time;
};

struct qcom_battmgr_ac {
	bool online;
};

struct qcom_battmgr_usb {
	bool online;
	unsigned int voltage_now;
	unsigned int voltage_max;
	unsigned int current_now;
	unsigned int current_max;
	unsigned int current_limit;
	unsigned int usb_type;
};

struct qcom_battmgr_wireless {
	bool online;
	unsigned int voltage_now;
	unsigned int voltage_max;
	unsigned int current_now;
	unsigned int current_max;
};

struct qcom_battmgr {
	struct device *dev;
	struct pmic_glink_client *client;

	enum qcom_battmgr_variant variant;
	unsigned long quirks;

	struct power_supply *ac_psy;
	struct power_supply *bat_psy;
	struct power_supply *gauge;	/* oplus,external-gauge */
	/*
	 * Written by the charge worker under @charge_lock, see
	 * qcom_battmgr_gauge_detect(); read without it.
	 */
	enum qcom_battmgr_gauge_state gauge_state;
	unsigned int gauge_polls, gauge_nacks;
	unsigned long gauge_poll_at;
	struct power_supply *usb_psy;
	struct power_supply *wls_psy;

	enum qcom_battmgr_unit unit;

	int error;
	struct completion ack;
	/* Only a reply to the outstanding opcode may complete @ack. */
	u32 request_opcode;

	bool service_up;

	struct qcom_battmgr_info info;
	struct qcom_battmgr_status status;
	struct qcom_battmgr_ac ac;
	struct qcom_battmgr_usb usb;
	struct qcom_battmgr_wireless wireless;

	struct work_struct enable_work;

	/*
	 * @lock is used to prevent concurrent power supply requests to the
	 * firmware, as it then stops responding.
	 */
	struct mutex lock;

	/* Set under @lock once @client is about to be released */
	bool client_gone;

	/*
	 * @notify_lock orders @removing against the PMIC GLINK callbacks, which
	 * may still run while the power supplies and @enable_work are torn down.
	 */
	spinlock_t notify_lock;
	bool removing;

	/*
	 * The battery temperature as a thermal sensor, in millidegrees: the
	 * last reading passed to the zone and the last one read. Only the
	 * zone's get_temp(), under the zone lock, uses these.
	 */
	int bat_tz_temp;
	int bat_tz_last;
	bool bat_tz_known;
	bool bat_tz_failing;

	/* Property id and value in the answer to the last accepted request */
	u32 resp_property;
	u32 resp_value;
	/* Source PDO reply, protected by @lock and the request completion. */
	u32 src_pdos[PDO_MAX_OBJECTS];
	unsigned int num_src_pdos;

	/*
	 * Charge control thresholds enforced by the driver: @charge_work runs
	 * on a freezable workqueue, so no request is sent while the system
	 * is suspended.  @charge_inhibited is set while the driver holds
	 * charging off, @charge_error while the firmware refuses to switch.
	 * @charge_bypass holds charging off regardless of the thresholds, so
	 * a connected charger powers the system while the battery rests.
	 */
	struct delayed_work charge_work;
	bool charge_inhibited;
	bool charge_bypass;
	bool charge_error;
	bool charge_policy_hold;
	struct wakeup_source *charge_ws;
	bool charge_awake; /* protected by notify_lock */

	/*
	 * USB VBUS boost: @otg_request follows the firmware notifications,
	 * @otg_work (freezable) switches the boost to match and records the
	 * result in @otg_vbus.
	 */
	struct work_struct otg_work;
	bool otg_request;
	bool otg_vbus;

	/* Both charging policies are serialized by @charge_lock. */
	struct mutex charge_lock;
	struct notifier_block pm_nb;
	bool pps_suspended;
	bool pps_input_used;
	const struct erhai_pps_ops *pps_ops;
	/* Registered, but held back until the pack gauge has answered. */
	const struct erhai_pps_ops *pps_parked;
	void *pps_data;

	/* Fixed PD shares @charge_work and @charge_lock with PPS. */
	struct notifier_block psy_nb;
	unsigned long pd_events;
	enum qcom_battmgr_pd_state pd_state;
	unsigned long pd_since;
	bool pd_restore;
	bool qc_contract;
	bool serial_probe;
	bool stable_seen;
	unsigned long stable_since;
	unsigned long fixed_stable_since;
	unsigned int missing_routes;
	unsigned int retry_evidence;
	unsigned int retry_once;
	unsigned int retry_pending;
	bool retry_stopping;
	unsigned long retry_deadline;
	struct erhai_route_retry serial_retry_state, ufcs_retry_state;
	struct erhai_route_retry pps_retry_state, fixed_retry_state;
	unsigned int restore_route;
	unsigned int suspend_route;
	unsigned int tail_route;
	bool tail_owned;
	unsigned int recharge_samples;
	unsigned long recharge_at;
	unsigned long serial_tail_check_at;
	u32 charge_detected_type;
	const char *fixed_exit;
	unsigned int serial_failures, ufcs_failures;
	unsigned long serial_retry;
	unsigned long ufcs_retry;
	bool ufcs_backoff;
	bool pd_charge_only;
	/* The source carries data: its contract is left alone, unpolled. */
	bool pd_data_partner;
	bool pd_partner_checked;
	struct class_interface partner_events;
	bool fixed_backoff;
	unsigned long fixed_retry;
	unsigned long pd_classified_at;
	bool qc_probed;
	unsigned long qc_probe_at;
	bool serial_backoff;
	bool serial_blocked;
	bool aicl_done;
	bool serial_icl_vote;
	bool serial_buck_suspended;
	bool buck_online;
	struct erhai_buck buck;
	struct erhai_buck_sample buck_sample;
	/* gaugeless_ffc: the AP runs the stock tail, see qcom_battmgr_ffc_tick() */
	struct {
		bool active;		/* the AP owns float voltage and current */
		bool fault;		/* a cross-check failed: not again on this plug */
		const char *ending;	/* hand-back not yet acknowledged */
		const char *end;	/* why the last tail ended */
		int restore_fv_mv;	/* handed back to the firmware */
		int fw_fv;		/* BATT_VOLT_MAX as the firmware had it, raw */
		int fw_mv, pump_mv;	/* last cross-check pair */
		u32 last_ua;
		unsigned int stale, vdiff, iover;
		unsigned long next;
	} ffc;
	/* gaugeless_full, see qcom_battmgr_uisoc_worker() */
	bool guard_withheld;
	int pack;			/* stock battery profile: -1 not known yet */
	unsigned long pack_at;
	struct mutex uisoc_lock;
	struct erhai_uisoc uisoc;	/* @uisoc_lock */
	struct delayed_work uisoc_work;
	int uisoc_fw_soc;
	bool uisoc_fastchg;
	atomic_long_t uisoc_sleep_s;
	ktime_t uisoc_suspend_at;
	bool uisoc_backup_failed;
	int buck_fv_applied, buck_fcc_applied, buck_enable_applied;
	int buck_para_applied, buck_full_applied, buck_error, buck_gauge_error;
	bool buck_saved_fcc_valid;
	u32 buck_saved_fcc_ua;
	unsigned long buck_sample_at;
	int serial_icl_ua, serial_input_ua, serial_max_ua;
	unsigned long serial_sample_at;
	bool pd_logged;
	u32 pd_input_limit_ua;
	u32 pd_saved_fcc_ua;
	bool pd_restore_fcc;
	u32 pd_saved_limit_ua;
};

static int __qcom_battmgr_request(struct qcom_battmgr *battmgr, void *data,
				  size_t len, unsigned long timeout)
{
	const struct pmic_glink_hdr *hdr = data;
	unsigned long left;
	int ret;

	/* All request paths, including the timed OPLUS ones, hold @lock. */
	if (battmgr->client_gone)
		return -ENODEV;

	reinit_completion(&battmgr->ack);
	WRITE_ONCE(battmgr->request_opcode, le32_to_cpu(hdr->opcode));

	battmgr->error = 0;

	ret = pmic_glink_send(battmgr->client, data, len);
	if (!ret) {
		left = wait_for_completion_timeout(&battmgr->ack, timeout);
		ret = left ? battmgr->error : -ETIMEDOUT;
	}
	WRITE_ONCE(battmgr->request_opcode, U32_MAX);

	return ret;
}

static int qcom_battmgr_request(struct qcom_battmgr *battmgr, void *data, size_t len)
{
	return __qcom_battmgr_request(battmgr, data, len, HZ);
}

static int qcom_battmgr_request_property(struct qcom_battmgr *battmgr, int opcode,
					 int property, u32 value)
{
	struct qcom_battmgr_property_request request = {
		.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.hdr.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
		.hdr.opcode = cpu_to_le32(opcode),
		.battery = cpu_to_le32(0),
		.property = cpu_to_le32(property),
		.value = cpu_to_le32(value),
	};

	return qcom_battmgr_request(battmgr, &request, sizeof(request));
}

/*
 * Write a battery (BATTMGR_BAT_PROPERTY_SET) or USB (BATTMGR_USB_PROPERTY_SET)
 * property and wait until the firmware has accepted it.  The answer must
 * name the written property, so a late answer to an earlier request that
 * timed out is not taken for this one.
 */
static int qcom_battmgr_set_property_locked(struct qcom_battmgr *battmgr, int opcode,
				     u32 property, u32 value)
{
	struct qcom_battmgr_property_request request = {
		.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.hdr.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
		.hdr.opcode = cpu_to_le32(opcode),
		.battery = cpu_to_le32(0),
		.property = cpu_to_le32(property),
		.value = cpu_to_le32(value),
	};
	int ret;

	battmgr->resp_property = U32_MAX;
	ret = __qcom_battmgr_request(battmgr, &request, sizeof(request),
				     msecs_to_jiffies(BATTMGR_SET_TIMEOUT_MS));
	if (!ret && battmgr->resp_property != property)
		ret = -EIO;

	return ret;
}

static int qcom_battmgr_set_property(struct qcom_battmgr *battmgr, int opcode,
				     u32 property, u32 value)
{
	int ret;

	if (opcode == BATTMGR_BAT_PROPERTY_SET) {
		switch (property) {
		case BATT_VOLT_MAX:
			WRITE_ONCE(battmgr->buck_fv_applied, -1);
			break;
		case BATT_CHG_CTRL_LIM:
			WRITE_ONCE(battmgr->buck_fcc_applied, -1);
			break;
		case BATT_OPLUS_CHG_EN:
			WRITE_ONCE(battmgr->buck_enable_applied, -1);
			break;
		}
	}

	mutex_lock(&battmgr->lock);
	ret = qcom_battmgr_set_property_locked(battmgr, opcode, property, value);
	mutex_unlock(&battmgr->lock);
	return ret;
}

/* Read the raw value of a battery or USB property from the firmware */
static int qcom_battmgr_firmware_property(struct qcom_battmgr *battmgr, int opcode,
					  u32 property, u32 *value)
{
	int ret;

	mutex_lock(&battmgr->lock);
	battmgr->resp_property = U32_MAX;
	ret = qcom_battmgr_request_property(battmgr, opcode, property, 0);
	if (!ret && battmgr->resp_property != property)
		ret = -EIO;
	if (!ret)
		*value = battmgr->resp_value;
	mutex_unlock(&battmgr->lock);

	return ret;
}

/*
 * Erhai keeps the pack gauge (TI bq27z561) on AP I2C, as the stock device
 * tree says (oplus_spec,external_gauge_support). The ADSP charger firmware
 * does not read it: it reports its defaults (50 %, 4.0 V, 0 mA, 25 C) and no
 * OPLUS battery authentication, which on the stock firmware a daemon supplies
 * from the TEE. With a gauge in the device tree the battery readings come
 * from the gauge only; a failed gauge read is an error, never a fallback to
 * those defaults. The stock pack is identified by the design capacity
 * programmed in its gauge: 11410 mAh, the rated minimum of the 12140 mAh
 * typical pack (oplus,batt_capacity_mah) of the stock device tree.
 *
 * Not every Erhai has that gauge. The stock overlays with oplus,gauge-id 1
 * leave the bq27z561 out: there the firmware gauges the pack itself
 * (oplus,adsp_gauge), nothing answers at the gauge's address and the pack
 * is authenticated over a 1-wire chip this kernel has no driver for. Both
 * variants boot the same device tree, so the charge worker settles which
 * one this is, see qcom_battmgr_gauge_detect(). Until it has, and for good
 * once the gauge has answered, all of the above holds. Only when the gauge
 * is found absent do the battery readings come from the firmware, as on
 * any other board of this firmware family. They are not used to steer the
 * buck: the AP sets no float voltage and no charge current of its own, see
 * qcom_battmgr_buck_guard(). The pumps stay parked unless gaugeless_pps_ma
 * lets PD PPS in, see qcom_battmgr_gaugeless_pumps().
 */
#define ERHAI_PACK_DESIGN_UAH		11410000

/* What the firmware reports for a pack it does not gauge, see above. */
#define ERHAI_FW_DEFAULT_UV		4000000
#define ERHAI_FW_DEFAULT_CENTI_C	2500
#define ERHAI_FW_DEFAULT_CENTI_PCT	5000

/*
 * One probe a second for the first minute, then at the charge poll rate.
 * The gauge is absent once this many probes in a row were not acknowledged.
 */
#define GAUGE_DETECT_MS			1000
#define GAUGE_DETECT_FAST_POLLS		60
#define GAUGE_ABSENT_NACKS		10

/*
 * Fast charging on a board without the pack gauge. The stock system charges
 * such a board through the pumps as well, from the firmware's battery
 * readings; here it is PD PPS only, see erhai_pps.c. 0 keeps the pumps
 * parked: the firmware charges by itself from 5 V or a fixed 9 V contract.
 * Any other value lets PPS in and is the ceiling of the current taken from
 * the charger in mA (the pumps put about twice that into the pack); the
 * stock ceiling is 5000. Read on every pass of the charge worker, so it can
 * be lowered, or cleared to stop, while a charger is connected.
 *
 * Off by default. With it set the engine's own limits still apply: the
 * table by charge level and temperature, 3 A from 43 C, 4 A per pump, a
 * pack current over 11.4 A for 8 s ends the session, the cross-check and
 * the battery thermal zone. A cable without a 5 A e-marker keeps the
 * charger at 3 A.
 */
static unsigned int gaugeless_pps_ma;
module_param(gaugeless_pps_ma, uint, 0644);
MODULE_PARM_DESC(gaugeless_pps_ma,
		 "without the pack gauge: PD PPS fast charging up to this bus current in mA (0 = off, at most 5000)");

/* The battery profile the firmware runs for the stock pack of this board. */
#define ERHAI_FW_PACK_PROFILE		"Erhai_TABLET_12140MAH"

/*
 * FFC (fast float charge) on a board without the pack gauge. The stock
 * charger finishes the pack this way once a fast charging session has
 * charged its table through: 15 s with the charge output off, then the buck
 * at 4630 mV and 1500 mA, 1000 mA, until the cell reaches 4540 mV or the
 * current falls under 1300 mA, 900 mA, then its ordinary float voltage and
 * the full decision, see erhai_buck.h. Without the gauge the AP sets none of
 * that by itself (qcom_battmgr_buck_guard()) and the firmware holds the cell
 * at its pre-FFC voltage. Set, the AP runs the stock tail after a gaugeless
 * PPS session that has charged its table through, on the firmware's battery
 * readings checked against the master pump's battery ADC, see
 * qcom_battmgr_ffc_tick(). Read on every pass of the charge worker:
 * clearing it ends a running tail and gives the charge back to the
 * firmware.
 *
 * Off by default. Setting it accepts the battery profile the firmware
 * selected for the pack as the identity check of the pack, as for PPS.
 */
static bool gaugeless_ffc;
module_param(gaugeless_ffc, bool, 0644);
MODULE_PARM_DESC(gaugeless_ffc,
		 "without the pack gauge: finish the charge after PD PPS with the stock FFC");

/*
 * The stock full decision and percentage on a board without the pack
 * gauge. The firmware decides neither: it holds the cell at its float
 * voltage and its own percentage stops short of 100 (93 % at 4484 mV and
 * 43 mA measured). Set, the AP declares the pack full on every charge as
 * the stock charger does (above 4440 mV at no more than 597 mA, recharge
 * from 4390 mV, see qcom_battmgr_buck_guard()) and the battery reports the
 * stock percentage (erhai_uisoc.h) instead of the firmware's, which stays
 * in charge_contract. Only for the stock pack's battery profile, the same
 * decision as gaugeless_ffc. Clearing it gives both back to the firmware.
 */
static bool gaugeless_full;
module_param(gaugeless_full, bool, 0644);
MODULE_PARM_DESC(gaugeless_full,
		 "without the pack gauge: the stock full decision and percentage");

static struct power_supply *qcom_battmgr_gauge(struct qcom_battmgr *b)
{
	struct power_supply *psy = READ_ONCE(b->gauge);

	if (psy)
		return psy;
	psy = power_supply_get_by_reference(dev_fwnode(b->dev), "oplus,external-gauge");
	if (IS_ERR_OR_NULL(psy))
		return NULL;
	if (cmpxchg(&b->gauge, NULL, psy))
		power_supply_put(psy);
	return READ_ONCE(b->gauge);
}

/* The device tree names a pack gauge; it may still turn out not fitted. */
static bool qcom_battmgr_gauge_described(struct qcom_battmgr *b)
{
	return of_property_present(b->dev->of_node, "oplus,external-gauge");
}

/* The battery readings come from the pack gauge. */
static bool qcom_battmgr_has_gauge(struct qcom_battmgr *b)
{
	return qcom_battmgr_gauge_described(b) &&
	       READ_ONCE(b->gauge_state) != GAUGE_ABSENT;
}

/* The gauge of the device tree is not fitted: the firmware gauges the pack. */
static bool qcom_battmgr_gaugeless(struct qcom_battmgr *b)
{
	return READ_ONCE(b->gauge_state) == GAUGE_ABSENT;
}

static bool qcom_battmgr_uisoc_shown(struct qcom_battmgr *b);

static const char *qcom_battmgr_gauge_name(struct qcom_battmgr *b)
{
	static const char * const names[] = {
		[GAUGE_UNKNOWN] = "detecting",
		[GAUGE_PRESENT] = "present",
		[GAUGE_ABSENT] = "absent",
	};

	if (!qcom_battmgr_gauge_described(b))
		return "none";
	return names[READ_ONCE(b->gauge_state)];
}

static void qcom_battmgr_put_gauge(void *data)
{
	struct qcom_battmgr *b = data;
	struct power_supply *psy = xchg(&b->gauge, NULL);

	if (psy)
		power_supply_put(psy);
}

static int qcom_battmgr_gauge_read(struct qcom_battmgr *b,
				   enum power_supply_property psp, int *value)
{
	struct power_supply *psy = qcom_battmgr_gauge(b);
	union power_supply_propval val;
	int ret;

	if (!psy)
		return -ENODEV;
	ret = power_supply_get_property(psy, psp, &val);
	if (!ret)
		*value = val.intval;
	return ret;
}

/*
 * Address the gauge directly, whether or not its driver is bound: 1 if it
 * acknowledged, 0 if it did not, an error if the probe says nothing about
 * it (its bus has not probed yet, a timeout, a bus error). Only the address
 * phase matters; reading its first command register changes nothing.
 */
static int qcom_battmgr_gauge_ping(struct qcom_battmgr *b)
{
#if IS_REACHABLE(CONFIG_I2C)
	struct fwnode_handle *fwnode;
	struct i2c_client *client;
	struct i2c_msg msg[2];
	u8 reg = 0, data[2];
	int ret;

	fwnode = fwnode_find_reference(dev_fwnode(b->dev), "oplus,external-gauge", 0);
	if (IS_ERR(fwnode))
		return PTR_ERR(fwnode);
	client = i2c_find_device_by_fwnode(fwnode);
	fwnode_handle_put(fwnode);
	if (!client)
		return -ENODEV;

	msg[0] = (struct i2c_msg){ .addr = client->addr, .len = sizeof(reg), .buf = &reg };
	msg[1] = (struct i2c_msg){ .addr = client->addr, .flags = I2C_M_RD,
				   .len = sizeof(data), .buf = data };
	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	put_device(&client->dev);
	if (ret == -ENXIO)
		return 0;
	return ret < 0 ? ret : 1;
#else
	return -EOPNOTSUPP;
#endif
}

/*
 * The firmware gauges the pack itself: it answers with a battery that is
 * not the set of defaults it reports for a pack whose gauge it cannot read.
 */
static bool qcom_battmgr_firmware_gauge(struct qcom_battmgr *b)
{
	u32 uv, ua, temp, soc;

	if (qcom_battmgr_firmware_property(b, BATTMGR_BAT_PROPERTY_GET, BATT_VOLT_NOW, &uv) ||
	    qcom_battmgr_firmware_property(b, BATTMGR_BAT_PROPERTY_GET, BATT_CURR_NOW, &ua) ||
	    qcom_battmgr_firmware_property(b, BATTMGR_BAT_PROPERTY_GET, BATT_TEMP, &temp) ||
	    qcom_battmgr_firmware_property(b, BATTMGR_BAT_PROPERTY_GET, BATT_CAPACITY, &soc))
		return false;
	if (!uv || soc > 10000 || ua == BIT(31))
		return false;
	return uv != ERHAI_FW_DEFAULT_UV || ua || temp != ERHAI_FW_DEFAULT_CENTI_C ||
	       soc != ERHAI_FW_DEFAULT_CENTI_PCT;
}

/* The stock pack, by the design capacity programmed in its gauge. */
static bool qcom_battmgr_gauge_stock_pack(struct qcom_battmgr *b)
{
	int uah;

	return !qcom_battmgr_gauge_read(b, POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN, &uah) &&
	       uah == ERHAI_PACK_DESIGN_UAH;
}

/* Battery properties in the firmware's units, from the gauge. */
static int qcom_battmgr_gauge_property(struct qcom_battmgr *b, u32 property,
				       u32 *value)
{
	int val, ret;

	switch (property) {
	case BATT_VOLT_NOW:
		ret = qcom_battmgr_gauge_read(b, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
		break;
	case BATT_CURR_NOW:
		/* The firmware reports discharge as positive. */
		ret = qcom_battmgr_gauge_read(b, POWER_SUPPLY_PROP_CURRENT_NOW, &val);
		val = -val;
		break;
	case BATT_TEMP:
		/* Centi-degrees in the firmware, deci-degrees in the gauge. */
		ret = qcom_battmgr_gauge_read(b, POWER_SUPPLY_PROP_TEMP, &val);
		val *= 10;
		break;
	case BATT_CAPACITY:
		/* Hundredths of a percent in the firmware. */
		ret = qcom_battmgr_gauge_read(b, POWER_SUPPLY_PROP_CAPACITY, &val);
		val *= 100;
		break;
	case BATT_CYCLE_COUNT:
		ret = qcom_battmgr_gauge_read(b, POWER_SUPPLY_PROP_CYCLE_COUNT, &val);
		break;
	case BATT_HEALTH:
		ret = qcom_battmgr_gauge_read(b, POWER_SUPPLY_PROP_HEALTH, &val);
		break;
	case BATT_OPLUS_AUTH:
	case BATT_OPLUS_HMAC:
		ret = 0;
		val = qcom_battmgr_gauge_stock_pack(b);
		break;
	default:
		return -EOPNOTSUPP;
	}
	if (!ret)
		*value = val;
	return ret;
}

static void qcom_battmgr_buck_invalidate(struct qcom_battmgr *b);

static int qcom_battmgr_get_property(struct qcom_battmgr *battmgr, int opcode,
				     u32 property, u32 *value)
{
	int ret;

	if (opcode == BATTMGR_BAT_PROPERTY_GET && qcom_battmgr_has_gauge(battmgr)) {
		ret = qcom_battmgr_gauge_property(battmgr, property, value);
		if (ret != -EOPNOTSUPP)
			return ret;
	}
	/* Without its gauge nothing here tells the stock pack from another. */
	if (opcode == BATTMGR_BAT_PROPERTY_GET && qcom_battmgr_gaugeless(battmgr) &&
	    (property == BATT_OPLUS_AUTH || property == BATT_OPLUS_HMAC)) {
		*value = 0;
		return 0;
	}

	return qcom_battmgr_firmware_property(battmgr, opcode, property, value);
}

/*
 * Settle whether this board has the gauge of its device tree. Called by the
 * charge worker, with the firmware service up, until it is settled; returns
 * the time to the next attempt in ms, 0 once there is none.
 *
 * Present: a gauge reading succeeded, or the gauge acknowledged its address
 * while its driver has yet to bind. One answer settles it for good: a gauge
 * that fails later is an error as before, never a fallback.
 *
 * Absent: nothing has ever answered, GAUGE_ABSENT_NACKS probes at least
 * GAUGE_DETECT_MS apart all ended in an address NACK, and the firmware
 * reports a pack it gauges itself. A probe that fails any other way, such
 * as a timeout, a bus error or a bus that has yet to probe, says nothing
 * and starts the count over, and so does not turn a fitted gauge absent.
 */
static unsigned int qcom_battmgr_gauge_detect(struct qcom_battmgr *b)
{
	unsigned long next;
	unsigned int wait;
	int value, ret;

	if (b->gauge_state != GAUGE_UNKNOWN || !qcom_battmgr_gauge_described(b))
		return 0;

	wait = b->gauge_polls < GAUGE_DETECT_FAST_POLLS ? GAUGE_DETECT_MS :
							  CHARGE_CTRL_POLL_MS;
	next = b->gauge_poll_at + msecs_to_jiffies(wait);
	if (b->gauge_polls && time_before(jiffies, next))
		return jiffies_to_msecs(next - jiffies) ?: 1;
	b->gauge_poll_at = jiffies;
	b->gauge_polls++;

	if (!qcom_battmgr_gauge_read(b, POWER_SUPPLY_PROP_VOLTAGE_NOW, &value))
		ret = 1;
	else
		ret = qcom_battmgr_gauge_ping(b);
	if (ret > 0) {
		WRITE_ONCE(b->gauge_state, GAUGE_PRESENT);
		if (b->pps_parked) {
			b->pps_ops = b->pps_parked;
			b->pps_parked = NULL;
		}
		return 0;
	}
	if (ret < 0) {
		b->gauge_nacks = 0;
		return wait;
	}
	if (++b->gauge_nacks < GAUGE_ABSENT_NACKS)
		return wait;
	b->gauge_nacks = GAUGE_ABSENT_NACKS;
	if (!qcom_battmgr_firmware_gauge(b))
		return wait;

	WRITE_ONCE(b->gauge_state, GAUGE_ABSENT);
	/* The supervisor had no readings so far; the guard starts afresh. */
	b->buck = (struct erhai_buck){};
	b->buck_error = 0;
	qcom_battmgr_buck_invalidate(b);
	dev_info(b->dev, "pack gauge not fitted: battery readings from the charger firmware, AP fast charging %s\n",
		 READ_ONCE(gaugeless_pps_ma) ? "by gaugeless_pps_ma" : "off");
	queue_delayed_work(system_freezable_wq, &b->uisoc_work, 0);
	power_supply_changed(b->bat_psy);
	return 0;
}

static int qcom_battmgr_update_status(struct qcom_battmgr *battmgr)
{
	struct qcom_battmgr_update_request request = {
		.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.hdr.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
		.hdr.opcode = cpu_to_le32(BATTMGR_BAT_STATUS),
		.battery_id = cpu_to_le32(0),
	};

	return qcom_battmgr_request(battmgr, &request, sizeof(request));
}

static int qcom_battmgr_update_info(struct qcom_battmgr *battmgr)
{
	struct qcom_battmgr_update_request request = {
		.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.hdr.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
		.hdr.opcode = cpu_to_le32(BATTMGR_BAT_INFO),
		.battery_id = cpu_to_le32(0),
	};

	return qcom_battmgr_request(battmgr, &request, sizeof(request));
}

static int qcom_battmgr_update_charge_time(struct qcom_battmgr *battmgr)
{
	struct qcom_battmgr_charge_time_request request = {
		.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.hdr.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
		.hdr.opcode = cpu_to_le32(BATTMGR_BAT_CHARGE_TIME),
		.battery_id = cpu_to_le32(0),
		.percent = cpu_to_le32(100),
	};

	return qcom_battmgr_request(battmgr, &request, sizeof(request));
}

static int qcom_battmgr_update_discharge_time(struct qcom_battmgr *battmgr)
{
	struct qcom_battmgr_discharge_time_request request = {
		.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.hdr.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
		.hdr.opcode = cpu_to_le32(BATTMGR_BAT_DISCHARGE_TIME),
		.battery_id = cpu_to_le32(0),
		.rate = cpu_to_le32(0),
	};

	return qcom_battmgr_request(battmgr, &request, sizeof(request));
}

static const u8 sm8350_bat_prop_map[] = {
	[POWER_SUPPLY_PROP_STATUS] = BATT_STATUS,
	[POWER_SUPPLY_PROP_HEALTH] = BATT_HEALTH,
	[POWER_SUPPLY_PROP_PRESENT] = BATT_PRESENT,
	[POWER_SUPPLY_PROP_CHARGE_TYPE] = BATT_CHG_TYPE,
	[POWER_SUPPLY_PROP_CAPACITY] = BATT_CAPACITY,
	[POWER_SUPPLY_PROP_VOLTAGE_OCV] = BATT_VOLT_OCV,
	[POWER_SUPPLY_PROP_VOLTAGE_NOW] = BATT_VOLT_NOW,
	[POWER_SUPPLY_PROP_VOLTAGE_MAX] = BATT_VOLT_MAX,
	[POWER_SUPPLY_PROP_CURRENT_NOW] = BATT_CURR_NOW,
	[POWER_SUPPLY_PROP_TEMP] = BATT_TEMP,
	[POWER_SUPPLY_PROP_TECHNOLOGY] = BATT_TECHNOLOGY,
	[POWER_SUPPLY_PROP_CHARGE_COUNTER] =  BATT_CHG_COUNTER,
	[POWER_SUPPLY_PROP_CYCLE_COUNT] = BATT_CYCLE_COUNT,
	[POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN] =  BATT_CHG_FULL_DESIGN,
	[POWER_SUPPLY_PROP_CHARGE_FULL] = BATT_CHG_FULL,
	[POWER_SUPPLY_PROP_MODEL_NAME] = BATT_MODEL_NAME,
	[POWER_SUPPLY_PROP_TIME_TO_FULL_AVG] = BATT_TTF_AVG,
	[POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG] = BATT_TTE_AVG,
	[POWER_SUPPLY_PROP_INTERNAL_RESISTANCE] = BATT_RESISTANCE,
	[POWER_SUPPLY_PROP_STATE_OF_HEALTH] = BATT_SOH,
	[POWER_SUPPLY_PROP_POWER_NOW] = BATT_POWER_NOW,
	[POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD] = BATT_CHG_CTRL_START_THR,
	[POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD] = BATT_CHG_CTRL_END_THR,
};

static int qcom_battmgr_bat_sm8350_update(struct qcom_battmgr *battmgr,
					  enum power_supply_property psp)
{
	unsigned int prop;
	int ret;

	if (psp >= ARRAY_SIZE(sm8350_bat_prop_map))
		return -EINVAL;

	prop = sm8350_bat_prop_map[psp];

	mutex_lock(&battmgr->lock);
	ret = qcom_battmgr_request_property(battmgr, BATTMGR_BAT_PROPERTY_GET, prop, 0);
	mutex_unlock(&battmgr->lock);

	return ret;
}

/*
 * With QCOM_BATTMGR_QUIRK_CHARGE_ENABLE the thresholds live in the driver;
 * reading them must not send the firmware a request for their property ids.
 */
static bool qcom_battmgr_charge_ctrl_local(struct qcom_battmgr *battmgr,
					   enum power_supply_property psp)
{
	if (!(battmgr->quirks & QCOM_BATTMGR_QUIRK_CHARGE_ENABLE))
		return false;

	return psp == POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD ||
	       psp == POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD ||
	       psp == POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR;
}

static int qcom_battmgr_bat_sc8280xp_update(struct qcom_battmgr *battmgr,
					    enum power_supply_property psp)
{
	int ret;

	mutex_lock(&battmgr->lock);

	if (!battmgr->info.valid) {
		ret = qcom_battmgr_update_info(battmgr);
		if (ret < 0)
			goto out_unlock;
		battmgr->info.valid = true;
	}

	ret = qcom_battmgr_update_status(battmgr);
	if (ret < 0)
		goto out_unlock;

	if (psp == POWER_SUPPLY_PROP_TIME_TO_FULL_AVG) {
		ret = qcom_battmgr_update_charge_time(battmgr);
		if (ret < 0) {
			ret = -ENODATA;
			goto out_unlock;
		}
	}

	if (psp == POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG) {
		ret = qcom_battmgr_update_discharge_time(battmgr);
		if (ret < 0) {
			ret = -ENODATA;
			goto out_unlock;
		}
	}

out_unlock:
	mutex_unlock(&battmgr->lock);
	return ret;
}

/* The ADSP charge state can stay latched while the cell supplies the
 * system. Use the same signed convention as CURRENT_NOW for Erhai.
 */
static int erhai_battery_status(int status, int current_ua)
{
	return current_ua < 0 ? POWER_SUPPLY_STATUS_DISCHARGING : status;
}

/* The battery as the gauge measures it; -EOPNOTSUPP leaves psp to the
 * firmware. Readings the firmware only has as defaults are not reported.
 */
static int qcom_battmgr_gauge_bat_property(struct qcom_battmgr *b,
					   enum power_supply_property psp,
					   union power_supply_propval *val)
{
	int full, design, ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_HEALTH:
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
	case POWER_SUPPLY_PROP_TEMP:
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		return qcom_battmgr_gauge_read(b, psp, &val->intval);
	case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
		return qcom_battmgr_gauge_read(b, POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
					       &val->intval);
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		return qcom_battmgr_gauge_read(b, POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
					       &val->intval);
	case POWER_SUPPLY_PROP_STATE_OF_HEALTH:
		ret = qcom_battmgr_gauge_read(b, POWER_SUPPLY_PROP_CHARGE_FULL, &full);
		ret = ret ?: qcom_battmgr_gauge_read(b, POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
						     &design);
		if (ret)
			return ret;
		if (design <= 0)
			return -ENODATA;
		val->intval = min(100, DIV_ROUND_CLOSEST(full * 100LL, design));
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		return qcom_battmgr_gauge_read(b, POWER_SUPPLY_PROP_CHARGE_NOW, &val->intval);
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
	case POWER_SUPPLY_PROP_INTERNAL_RESISTANCE:
	case POWER_SUPPLY_PROP_POWER_NOW:
		return -ENODATA;
	default:
		return -EOPNOTSUPP;
	}
}

static int qcom_battmgr_bat_get_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	struct qcom_battmgr *battmgr = power_supply_get_drvdata(psy);
	enum qcom_battmgr_unit unit = battmgr->unit;
	u32 raw_ua;
	int ret;

	if (!battmgr->service_up)
		return -EAGAIN;

	if (qcom_battmgr_charge_ctrl_local(battmgr, psp))
		ret = 0;
	else if (battmgr->variant == QCOM_BATTMGR_SC8280XP ||
		 battmgr->variant == QCOM_BATTMGR_X1E80100)
		ret = qcom_battmgr_bat_sc8280xp_update(battmgr, psp);
	else
		ret = qcom_battmgr_bat_sm8350_update(battmgr, psp);
	if (ret < 0)
		return ret;

	if (qcom_battmgr_has_gauge(battmgr)) {
		ret = qcom_battmgr_gauge_bat_property(battmgr, psp, val);
		if (ret != -EOPNOTSUPP)
			return ret;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = battmgr->status.status;
		/*
		 * oplus_comm_check_battery_status(): full is reported once the
		 * percentage has reached 100; until then, with the AP's full
		 * decided, the pack is shown charging towards it.
		 */
		if (qcom_battmgr_uisoc_shown(battmgr) && READ_ONCE(battmgr->buck_online) &&
		    (READ_ONCE(battmgr->uisoc.full) ||
		     READ_ONCE(battmgr->buck.phase) == ERHAI_BUCK_FULL)) {
			int region = READ_ONCE(battmgr->buck.temp_region);

			val->intval = READ_ONCE(battmgr->uisoc.ui) == 100 || region == 0 || region >= 7 ?
				      POWER_SUPPLY_STATUS_FULL : POWER_SUPPLY_STATUS_CHARGING;
			break;
		}
		if (battmgr->quirks & QCOM_BATTMGR_QUIRK_OPLUS_PD) {
			/* AP full is an attachment-scoped decision. Cell relaxation
			 * can supply the system after output is disabled; it does
			 * not undo full until the controller declares recharge.
			 */
			if (READ_ONCE(battmgr->buck.phase) == ERHAI_BUCK_FULL &&
			    READ_ONCE(battmgr->buck_online)) {
				val->intval = POWER_SUPPLY_STATUS_FULL;
				break;
			}
			ret = qcom_battmgr_get_property(battmgr, BATTMGR_BAT_PROPERTY_GET,
						BATT_CURR_NOW, &raw_ua);
			if (ret)
				return ret;
			if (READ_ONCE(battmgr->buck.disable) ||
				 READ_ONCE(battmgr->charge_policy_hold) ||
				 READ_ONCE(battmgr->charge_inhibited))
				val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			else if ((s32)raw_ua < 0)
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
			/* Without the gauge full is the firmware's decision. */
			else if (READ_ONCE(battmgr->buck.phase) != ERHAI_BUCK_OFF &&
				 !qcom_battmgr_gaugeless(battmgr) &&
				 val->intval == POWER_SUPPLY_STATUS_FULL)
				val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			val->intval = erhai_battery_status(val->intval, -(s32)raw_ua);
		}
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = battmgr->info.charge_type;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = battmgr->status.health;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = battmgr->info.present;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = battmgr->info.technology;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		val->intval = battmgr->info.cycle_count;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = battmgr->info.voltage_max_design;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = battmgr->info.voltage_max;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = battmgr->status.voltage_now;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		val->intval = battmgr->status.voltage_ocv;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = battmgr->status.current_now;
		if (battmgr->quirks & QCOM_BATTMGR_QUIRK_BATT_CURRENT_INVERTED)
			val->intval = -val->intval;
		break;
	case POWER_SUPPLY_PROP_POWER_NOW:
		val->intval = battmgr->status.power_now;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		if (unit != QCOM_BATTMGR_UNIT_mAh)
			return -ENODATA;
		val->intval = battmgr->info.design_capacity;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		if (unit != QCOM_BATTMGR_UNIT_mAh)
			return -ENODATA;
		val->intval = battmgr->info.last_full_capacity;
		break;
	case POWER_SUPPLY_PROP_CHARGE_EMPTY:
		if (unit != QCOM_BATTMGR_UNIT_mAh)
			return -ENODATA;
		val->intval = battmgr->info.capacity_low;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		if (unit != QCOM_BATTMGR_UNIT_mAh)
			return -ENODATA;
		val->intval = battmgr->status.capacity;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		val->intval = battmgr->info.charge_count;
		break;
	case POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN:
		if (unit != QCOM_BATTMGR_UNIT_mWh)
			return -ENODATA;
		val->intval = battmgr->info.design_capacity;
		break;
	case POWER_SUPPLY_PROP_ENERGY_FULL:
		if (unit != QCOM_BATTMGR_UNIT_mWh)
			return -ENODATA;
		val->intval = battmgr->info.last_full_capacity;
		break;
	case POWER_SUPPLY_PROP_ENERGY_EMPTY:
		if (unit != QCOM_BATTMGR_UNIT_mWh)
			return -ENODATA;
		val->intval = battmgr->info.capacity_low;
		break;
	case POWER_SUPPLY_PROP_ENERGY_NOW:
		if (unit != QCOM_BATTMGR_UNIT_mWh)
			return -ENODATA;
		val->intval = battmgr->status.capacity;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (qcom_battmgr_uisoc_shown(battmgr)) {
			val->intval = READ_ONCE(battmgr->uisoc.ui);
			break;
		}
		if (battmgr->status.percent == (unsigned int)-1)
			return -ENODATA;
		val->intval = battmgr->status.percent;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = battmgr->status.temperature;
		break;
	case POWER_SUPPLY_PROP_INTERNAL_RESISTANCE:
		val->intval = battmgr->status.resistance;
		break;
	case POWER_SUPPLY_PROP_STATE_OF_HEALTH:
		val->intval = battmgr->status.soh_percent;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		val->intval = battmgr->status.discharge_time;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
		val->intval = battmgr->status.charge_time;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD:
		val->intval = battmgr->info.charge_ctrl_start;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		val->intval = battmgr->info.charge_ctrl_end;
		break;
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
		val->intval = READ_ONCE(battmgr->charge_bypass) ?
			      POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE :
			      POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURE_YEAR:
		val->intval = battmgr->info.year;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURE_MONTH:
		val->intval = battmgr->info.month;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURE_DAY:
		val->intval = battmgr->info.day;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = battmgr->info.model_number;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = battmgr->info.oem_info;
		break;
	case POWER_SUPPLY_PROP_SERIAL_NUMBER:
		val->strval = battmgr->info.serial_number;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/*
 * The battery temperature as the sensor of a device tree thermal zone
 * (#thermal-sensor-cells on the pmic-glink node), so that the board can cap
 * the CPU and GPU clocks for the battery's sake and power off when the pack
 * gets too hot. The firmware raises no temperature alarm: the zone polls.
 *
 * A reading the zone cannot use is reported as THERMAL_TEMP_INVALID, on which
 * the thermal core keeps polling and leaves the cooling as it is. An error
 * would instead make it retry with a growing delay and, after two minutes of
 * failures, disable the zone together with its critical trip. Readings are
 * unavailable until the firmware service is up and while it restarts.
 */
#define BATTMGR_TZ_TEMP_MIN	(-40 * MILLIDEGREE_PER_DEGREE)
#define BATTMGR_TZ_TEMP_MAX	(120 * MILLIDEGREE_PER_DEGREE)
/* More than the pack's temperature can change between two polls. */
#define BATTMGR_TZ_TEMP_STEP	(5 * MILLIDEGREE_PER_DEGREE)

static int qcom_battmgr_bat_get_temp(struct thermal_zone_device *tz, int *temp)
{
	struct qcom_battmgr *battmgr = thermal_zone_device_priv(tz);
	union power_supply_propval val;
	int ret, mc, last;

	*temp = THERMAL_TEMP_INVALID;

	ret = power_supply_get_property(battmgr->bat_psy, POWER_SUPPLY_PROP_TEMP, &val);
	if (ret) {
		/* -EAGAIN: the firmware service is not up (yet). */
		if (ret != -EAGAIN && !battmgr->bat_tz_failing)
			dev_warn(battmgr->dev, "no battery temperature for the thermal zone: %d\n",
				 ret);
		battmgr->bat_tz_failing = ret != -EAGAIN;
		return 0;
	}

	mc = val.intval * MILLIDEGREE_PER_DECIDEGREE;
	if (mc < BATTMGR_TZ_TEMP_MIN || mc > BATTMGR_TZ_TEMP_MAX) {
		if (!battmgr->bat_tz_failing)
			dev_warn(battmgr->dev, "battery temperature out of range: %d mC\n", mc);
		battmgr->bat_tz_failing = true;
		return 0;
	}
	battmgr->bat_tz_failing = false;

	/*
	 * A step the pack cannot make between two readings is passed on only
	 * when the next reading confirms it, so that one wrong reading neither
	 * powers the system off nor moves the cooling.
	 */
	last = battmgr->bat_tz_last;
	battmgr->bat_tz_last = mc;
	if (battmgr->bat_tz_known &&
	    abs(mc - battmgr->bat_tz_temp) > BATTMGR_TZ_TEMP_STEP &&
	    abs(mc - last) > BATTMGR_TZ_TEMP_STEP)
		return 0;

	battmgr->bat_tz_known = true;
	battmgr->bat_tz_temp = mc;
	*temp = mc;

	return 0;
}

static const struct thermal_zone_device_ops qcom_battmgr_bat_tz_ops = {
	.get_temp = qcom_battmgr_bat_get_temp,
};

/*
 * A device tree without a zone for the sensor is fine. A zone that fails to
 * register is reported, but the battery and the charger stay usable.
 */
static void qcom_battmgr_register_thermal(struct qcom_battmgr *battmgr)
{
	struct thermal_zone_device *tz;

	if (!IS_ENABLED(CONFIG_THERMAL_OF))
		return;

	tz = devm_thermal_of_zone_register(battmgr->dev, 0, battmgr, &qcom_battmgr_bat_tz_ops);
	if (IS_ERR(tz) && PTR_ERR(tz) != -ENODEV)
		dev_err(battmgr->dev, "battery thermal zone not registered: %pe\n", tz);
}

static int qcom_battmgr_set_charge_control(struct qcom_battmgr *battmgr,
					   u32 target_soc, u32 delta_soc)
{
	struct qcom_battmgr_charge_ctrl_request request = {
		.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.hdr.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
		.hdr.opcode = cpu_to_le32(BATTMGR_CHG_CTRL_LIMIT_EN),
		.enable = cpu_to_le32(1),
		.target_soc = cpu_to_le32(target_soc),
		.delta_soc = cpu_to_le32(delta_soc),
	};

	/* The driver enforces the thresholds, see qcom_battmgr_charge_worker() */
	if (battmgr->quirks & QCOM_BATTMGR_QUIRK_CHARGE_ENABLE)
		return 0;

	return qcom_battmgr_request(battmgr, &request, sizeof(request));
}

static int qcom_battmgr_set_charge_start_threshold(struct qcom_battmgr *battmgr, int start_soc)
{
	u32 target_soc, delta_soc;
	int ret;

	start_soc = clamp(start_soc, CHARGE_CTRL_START_THR_MIN, CHARGE_CTRL_START_THR_MAX);

	/*
	 * If the new start threshold is larger than the old end threshold,
	 * move the end threshold one step (DELTA_SOC) after the new start
	 * threshold.
	 */
	if (start_soc > battmgr->info.charge_ctrl_end) {
		target_soc = start_soc + CHARGE_CTRL_DELTA_SOC;
		target_soc = min_t(u32, target_soc, CHARGE_CTRL_END_THR_MAX);
		delta_soc = target_soc - start_soc;
		delta_soc = min_t(u32, delta_soc, CHARGE_CTRL_DELTA_SOC);
	} else {
		target_soc =  battmgr->info.charge_ctrl_end;
		delta_soc = battmgr->info.charge_ctrl_end - start_soc;
	}

	mutex_lock(&battmgr->lock);
	ret = qcom_battmgr_set_charge_control(battmgr, target_soc, delta_soc);
	mutex_unlock(&battmgr->lock);
	if (!ret) {
		battmgr->info.charge_ctrl_start = start_soc;
		battmgr->info.charge_ctrl_end = target_soc;
	}

	return 0;
}

static int qcom_battmgr_set_charge_end_threshold(struct qcom_battmgr *battmgr, int end_soc)
{
	u32 delta_soc = CHARGE_CTRL_DELTA_SOC;
	int ret;

	end_soc = clamp(end_soc, CHARGE_CTRL_END_THR_MIN, CHARGE_CTRL_END_THR_MAX);

	if (battmgr->info.charge_ctrl_start && end_soc > battmgr->info.charge_ctrl_start)
		delta_soc = end_soc - battmgr->info.charge_ctrl_start;

	mutex_lock(&battmgr->lock);
	ret = qcom_battmgr_set_charge_control(battmgr, end_soc, delta_soc);
	mutex_unlock(&battmgr->lock);
	if (!ret) {
		battmgr->info.charge_ctrl_start = end_soc - delta_soc;
		battmgr->info.charge_ctrl_end = end_soc;
	}

	return 0;
}

static int qcom_battmgr_charge_control_thresholds_init(struct qcom_battmgr *battmgr)
{
	int ret;
	u8 en, end_soc, start_soc, delta_soc;

	ret = nvmem_cell_read_u8(battmgr->dev->parent, "charge_limit_en", &en);
	if (!ret && en != 0) {
		ret = nvmem_cell_read_u8(battmgr->dev->parent, "charge_limit_end", &end_soc);
		if (ret < 0)
			return ret;

		ret = nvmem_cell_read_u8(battmgr->dev->parent, "charge_limit_delta", &delta_soc);
		if (ret < 0)
			return ret;

		if (delta_soc >= end_soc)
			return -EINVAL;

		start_soc = end_soc - delta_soc;
		end_soc = clamp(end_soc, CHARGE_CTRL_END_THR_MIN, CHARGE_CTRL_END_THR_MAX);
		start_soc = clamp(start_soc, CHARGE_CTRL_START_THR_MIN, CHARGE_CTRL_START_THR_MAX);

		battmgr->info.charge_ctrl_start = start_soc;
		battmgr->info.charge_ctrl_end = end_soc;
	}

	return 0;
}

/* Charge control is unavailable if its request means something else */
static bool qcom_battmgr_no_charge_ctrl(struct qcom_battmgr *battmgr)
{
	return (battmgr->quirks & QCOM_BATTMGR_QUIRK_NO_CHARGE_CONTROL) &&
	       !(battmgr->quirks & QCOM_BATTMGR_QUIRK_CHARGE_ENABLE);
}

static bool qcom_battmgr_charge_limited(struct qcom_battmgr *battmgr)
{
	unsigned int end = READ_ONCE(battmgr->info.charge_ctrl_end);

	return READ_ONCE(battmgr->charge_bypass) ||
	       (end && end < CHARGE_CTRL_END_THR_MAX);
}

/* The ADSP USB_ADAP_TYPE identifies the two computer-port exceptions.
 * Unknown is a wired source while APSD or the serial PHY is identifying it.
 */
static bool erhai_should_stay_awake(bool online, bool otg, u32 type)
{
	return online && !otg && type != POWER_SUPPLY_USB_TYPE_SDP &&
	       type != POWER_SUPPLY_USB_TYPE_CDP;
}

static void qcom_battmgr_charge_wake_locked(struct qcom_battmgr *battmgr, bool awake)
{
	lockdep_assert_held(&battmgr->notify_lock);
	awake = awake && (battmgr->quirks & QCOM_BATTMGR_QUIRK_CHARGE_ENABLE) &&
		battmgr->charge_ws && battmgr->service_up && !battmgr->removing;
	if (battmgr->charge_awake == awake)
		return;
	battmgr->charge_awake = awake;
	if (awake)
		__pm_stay_awake(battmgr->charge_ws);
	else
		__pm_relax(battmgr->charge_ws);
	if (battmgr->usb_psy)
		power_supply_changed(battmgr->usb_psy);
}

static void qcom_battmgr_charge_wake_remove(void *data)
{
	struct qcom_battmgr *battmgr = data;

	__pm_relax(battmgr->charge_ws);
	wakeup_source_unregister(battmgr->charge_ws);
}

static int qcom_battmgr_charge_enable(struct qcom_battmgr *battmgr, bool enable)
{
	int ret;

	enable &= !battmgr->buck.disable &&
		  !test_bit(PD_EVENT_TAIL, &battmgr->pd_events);

	ret = qcom_battmgr_set_property(battmgr, BATTMGR_BAT_PROPERTY_SET,
					BATT_OPLUS_CHG_EN, enable);

	/* Report a refusal once, not on every check */
	if (ret && !battmgr->charge_error)
		dev_warn(battmgr->dev, "failed to turn charging %s: %d\n",
			 enable ? "on" : "off", ret);
	battmgr->charge_error = !!ret;
	battmgr->buck_enable_applied = ret ? -1 : enable;

	return ret;
}

/*
 * Stop charging once the state of charge reaches the end threshold and
 * resume once it has fallen to the start threshold.  Nothing is sent while
 * no end threshold below 100 % is set. Erhai still runs its independent
 * AP buck full/recharge controller.
 */
static void qcom_battmgr_charge_limit_update(struct qcom_battmgr *battmgr)
{
	unsigned int start = READ_ONCE(battmgr->info.charge_ctrl_start);
	unsigned int end = READ_ONCE(battmgr->info.charge_ctrl_end);
	unsigned int soc;
	int ret;

	if (!qcom_battmgr_charge_limited(battmgr)) {
		battmgr->charge_policy_hold = false;
		if ((battmgr->charge_inhibited || battmgr->charge_error) &&
		    !qcom_battmgr_charge_enable(battmgr, true)) {
			battmgr->charge_inhibited = false;
			dev_info(battmgr->dev, "charge limit removed, charging resumed\n");
		}
		return;
	}

	ret = qcom_battmgr_bat_sm8350_update(battmgr, POWER_SUPPLY_PROP_CAPACITY);
	if (ret)
		return;

	soc = battmgr->status.percent;
	if (READ_ONCE(battmgr->charge_bypass)) {
		if (!battmgr->charge_inhibited || battmgr->charge_error) {
			battmgr->charge_policy_hold = true;
			if (!qcom_battmgr_charge_enable(battmgr, false)) {
				battmgr->charge_inhibited = true;
				dev_info(battmgr->dev, "bypass charging on\n");
			}
		}
		return;
	}

	if (soc >= end)
		battmgr->charge_policy_hold = true;
	else if (soc <= start)
		battmgr->charge_policy_hold = false;

	/* An unacknowledged command may have reached the firmware. Keep
	 * the hysteresis decision and reconcile it on the next poll, even
	 * when SOC has since moved between the thresholds.
	 */
	if (battmgr->charge_policy_hold) {
		if ((soc >= end || !battmgr->charge_inhibited || battmgr->charge_error) &&
		    !qcom_battmgr_charge_enable(battmgr, false) && !battmgr->charge_inhibited) {
			battmgr->charge_inhibited = true;
			dev_info(battmgr->dev, "charging stopped at %u%% (end threshold %u%%)\n",
				 soc, end);
		}
	} else if ((battmgr->charge_inhibited || battmgr->charge_error) &&
		   !qcom_battmgr_charge_enable(battmgr, true)) {
		battmgr->charge_inhibited = false;
		dev_info(battmgr->dev, "charging resumed at %u%% (start threshold %u%%)\n",
			 soc, start);
	}
}

/* Caller owns @lock until it has consumed the PDO array. */
static int qcom_battmgr_read_pdos_locked(struct qcom_battmgr *battmgr)
{
	struct qcom_battmgr_read_pdos_request req = {
		.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.hdr.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
		.hdr.opcode = cpu_to_le32(BATTMGR_OPLUS_READ_PDOS),
		.data_size = cpu_to_le32(OPLUS_READ_BUFFER_WORDS * sizeof(__le32)),
	};
	int ret;

	battmgr->num_src_pdos = 0;
	ret = __qcom_battmgr_request(battmgr, &req, sizeof(req),
				     msecs_to_jiffies(BATTMGR_SET_TIMEOUT_MS));
	if (ret)
		return ret;
	if (!battmgr->num_src_pdos || pdo_type(battmgr->src_pdos[0]) != PDO_TYPE_FIXED ||
	    pdo_fixed_voltage(battmgr->src_pdos[0]) != PD_DEFAULT_MV ||
	    !pdo_max_current(battmgr->src_pdos[0]))
		return -ENODATA;
	return 0;
}

static bool qcom_battmgr_pps_running(struct qcom_battmgr *battmgr)
{
	return battmgr->pps_ops && battmgr->pps_ops->active(battmgr->pps_data);
}

/* 24926 AP VOOC PHY votes WIRED_CHARGE_SUSPEND in direct charge.
 * The ADSP HAL maps that vote to USB ICL 0 and its release to U32_MAX.
 * The charge lock serializes it with the 5 V fallback and PD/PPS votes.
 */
static int qcom_battmgr_serial_buck(struct qcom_battmgr *b, bool suspend)
{
	int ret;

	if (suspend == b->serial_buck_suspended)
		return 0;
	/* Even on a missing ACK the firmware may have accepted a suspend. */
	b->serial_buck_suspended = true;
	ret = qcom_battmgr_set_property(b, BATTMGR_USB_PROPERTY_SET, USB_INPUT_CURR_LIMIT,
					suspend ? 0 : U32_MAX);
	if (!ret) {
		b->serial_buck_suspended = suspend;
		b->serial_icl_vote = false;
		b->aicl_done = false;
	}
	return ret;
}

static int qcom_battmgr_serial_stop(struct qcom_battmgr *b, const char *reason)
{
	if (b->serial_probe)
		dev_info(b->dev, "serial release: %s online=%d type=%u events=%lx hold=%d inhibited=%d error=%d buck=%u/%d\n",
			 reason, b->buck_online, b->charge_detected_type, b->pd_events,
			 b->charge_policy_hold, b->charge_inhibited, b->charge_error,
			 b->buck.phase, b->buck.disable);
	if (b->pps_ops) {
		int ret = b->pps_ops->serial_session(false);

		if (ret)
			return ret;
	}
	/* VOOC normal-full retains FASTCHG output-disable while input
	 * suspend is released, then common starts the 15 s FFC wait.
	 */
	if (b->buck_enable_applied != 0 &&
	    (test_bit(PD_EVENT_TAIL, &b->pd_events) || b->buck.phase == ERHAI_FFC_WAIT)) {
		int ret = qcom_battmgr_charge_enable(b, false);

		if (ret)
			return ret;
	}
	return qcom_battmgr_serial_buck(b, false);
}

static int qcom_battmgr_pps_stop(struct qcom_battmgr *battmgr, bool upgrading,
				  const char *reason)
{
	int ret = qcom_battmgr_serial_stop(battmgr, reason);

	if (ret || !battmgr->pps_ops)
		return ret;
	return battmgr->pps_ops->stop(battmgr->pps_data, upgrading);
}

/* Match only the Type-C supply belonging to this PMIC GLINK instance. */
static bool qcom_battmgr_is_ucsi_supply(struct qcom_battmgr *battmgr,
				      struct power_supply *psy)
{
	struct device *parent = psy->dev.parent;

	return parent && parent != battmgr->dev &&
	       parent->parent == battmgr->dev->parent &&
	       psy->desc->type == POWER_SUPPLY_TYPE_USB;
}

struct qcom_battmgr_pd_source {
	struct qcom_battmgr *battmgr;
	bool sink;
};

/* A GLINK PLUGIN notification has no direction. Observe the removal
 * of the UCSI Type-C partner itself, even when attach overtakes the
 * charge worker. Match only the port of this battery manager.
 */
static int qcom_battmgr_typec_port(struct device *dev, const void *unused)
{
	return dev->type && !strcmp(dev->type->name, "typec_port");
}

static void qcom_battmgr_partner_removed(struct device *dev)
{
	struct qcom_battmgr *b;
	struct device *port = dev->parent;

	if (!dev->type || strcmp(dev->type->name, "typec_partner") ||
	    !port || !port->parent)
		return;
	guard(mutex)(&erhai_bridge_lock);
	b = erhai_battmgr;
	if (b && port->parent->parent == b->dev->parent) {
		guard(spinlock_irqsave)(&b->notify_lock);
		if (b->removing)
			return;
		qcom_battmgr_charge_wake_locked(b, false);
		set_bit(PD_EVENT_DETACH, &b->pd_events);
		mod_delayed_work(system_freezable_wq, &b->charge_work, 0);
	}
}

static void qcom_battmgr_watch_partner(struct qcom_battmgr *b, struct device *ucsi)
{
	struct device *port;

	if (b->partner_events.class || !(b->quirks & QCOM_BATTMGR_QUIRK_PPS))
		return;
	port = device_find_child(ucsi, NULL, qcom_battmgr_typec_port);
	if (!port)
		return;
	b->partner_events.class = port->class;
	b->partner_events.remove_dev = qcom_battmgr_partner_removed;
	if (class_interface_register(&b->partner_events))
		b->partner_events.class = NULL;
	put_device(port);
}

static int qcom_battmgr_pd_source_match(struct power_supply *psy, void *data)
{
	struct qcom_battmgr_pd_source *src = data;
	union power_supply_propval val;

	if (!qcom_battmgr_is_ucsi_supply(src->battmgr, psy))
		return 0;
	qcom_battmgr_watch_partner(src->battmgr, psy->dev.parent);
	if (power_supply_get_property(psy, POWER_SUPPLY_PROP_ONLINE, &val) ||
	    !val.intval)
		return 0;
	if (power_supply_get_property(psy, POWER_SUPPLY_PROP_USB_TYPE, &val))
		return 0;

	src->sink = val.intval == POWER_SUPPLY_USB_TYPE_PD ||
		    val.intval == POWER_SUPPLY_USB_TYPE_PD_PPS;
	return src->sink;
}

/* On a PD charger the mandatory first fixed PDO describes whether the
 * partner supports USB communication. Reserve those data pins for USB.
 * Returns 1 for a charge-only source, 0 for a data partner, or an error
 * when the capabilities could not be read.
 */
static int qcom_battmgr_pd_charge_only(struct qcom_battmgr *b)
{
	int ret;

	guard(mutex)(&b->lock);
	ret = qcom_battmgr_read_pdos_locked(b);
	if (ret)
		return ret;
	return !(b->src_pdos[0] & PDO_FIXED_USB_COMM);
}

/* A missing BC1.2 result can describe either a brick or a data port.
 * Use the connector's UCSI-controlled role switch before claiming D+/D-.
 */
static bool qcom_battmgr_usb_idle(struct qcom_battmgr *b)
{
	struct fwnode_handle *node;
	struct usb_role_switch *sw;
	bool idle;

	device_for_each_child_node(b->dev->parent, node) {
		if (!fwnode_device_is_compatible(node, "usb-c-connector"))
			continue;
		sw = fwnode_usb_role_switch_get(node);
		fwnode_handle_put(node);
		if (IS_ERR_OR_NULL(sw))
			return false;
		idle = usb_role_switch_get_role(sw) == USB_ROLE_NONE;
		usb_role_switch_put(sw);
		return idle;
	}
	return false;
}

/* Read fresh capabilities; never infer a fixed PDO from an APDO range. */
static int qcom_battmgr_pd_current(struct qcom_battmgr *battmgr, u32 mv, u32 *ua)
{
	unsigned int i;
	u32 pdo;
	int ret;

	*ua = 0;
	mutex_lock(&battmgr->lock);
	ret = qcom_battmgr_read_pdos_locked(battmgr);
	if (ret)
		goto out;

	for (i = 0; i < battmgr->num_src_pdos; i++) {
		pdo = battmgr->src_pdos[i];
		if (pdo_type(pdo) == PDO_TYPE_FIXED &&
		    pdo_fixed_voltage(pdo) == mv) {
			*ua = min_t(u32, pdo_max_current(pdo) * 1000, PD_INPUT_MAX_UA);
			break;
		}
	}
out:
	mutex_unlock(&battmgr->lock);
	return ret;
}

static ssize_t charge_contract_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct qcom_battmgr *b = power_supply_get_drvdata(to_power_supply(dev));
	ssize_t len;

	guard(mutex)(&b->charge_lock);
	len = sysfs_emit(buf, "fixed_pd_mv=%u fixed_icl_ua=%u ",
			 b->pd_state == PD_ACTIVE && !b->qc_contract ? PD_FIXED_MV : 0,
			 b->pd_state == PD_ACTIVE ? b->pd_input_limit_ua : 0);

	len += sysfs_emit_at(buf, len, "qc_mv=%u serial_probe=%d detected_type=%u serial_source=%d serial_icl_vote=%d serial_buck_suspended=%d buck_tail=%d fixed_exit=%s ",
			     b->pd_state == PD_ACTIVE && b->qc_contract ? PD_FIXED_MV : 0,
			     b->serial_probe, b->charge_detected_type, !!(b->retry_evidence & ERHAI_SERIAL),
			     b->serial_icl_vote, b->serial_buck_suspended,
			     b->tail_owned || test_bit(PD_EVENT_TAIL, &b->pd_events), b->fixed_exit ?: "none");
	if (b->serial_icl_vote)
		len += sysfs_emit_at(buf, len, "fallback_vote_ua=%u fallback_icl_ua=%d fallback_input_ua=%d fallback_max_ua=%d fallback_age_ms=%lld ",
				     ERHAI_SERIAL_5V_UA, b->serial_icl_ua, b->serial_input_ua,
				     b->serial_max_ua, b->serial_sample_at ?
				     (long long)jiffies_to_msecs(jiffies - b->serial_sample_at) : -1LL);
	len += sysfs_emit_at(buf, len,
		"buck_phase=%u ffc_step=%u ffc_used=%d buck_fv_mv=%d buck_fcc_ua=%d "
		"buck_enable=%d buck_cutoff_mv=%d buck_cutoff_ma=%d buck_recharge_mv=%d "
		"buck_v_hits=%u buck_i_hits=%u buck_full=%d buck_error=%d buck_gauge_error=%d "
		"buck_mv=%d buck_ma=%d buck_temp=%d buck_sample_age_ms=%lld ",
		b->buck.phase, b->buck.step, b->buck.ffc_used, b->buck_fv_applied,
		b->buck_fcc_applied, b->buck_enable_applied, b->buck.cutoff_mv,
		b->buck.cutoff_ma, b->buck.recharge_mv, b->buck.voltage_count,
		b->buck.current_count, b->buck.phase == ERHAI_BUCK_FULL, b->buck_error,
		b->buck_gauge_error,
		b->buck_sample.mv, b->buck_sample.ma, b->buck_sample.temp,
		b->buck_sample_at ? (long long)jiffies_to_msecs(jiffies - b->buck_sample_at) : -1LL);
	len += sysfs_emit_at(buf, len, "gauge=%s ", qcom_battmgr_gauge_name(b));
	if (qcom_battmgr_gaugeless(b))
		len += sysfs_emit_at(buf, len,
			"gaugeless_ffc=%d ffc_gl=%s ffc_fw_mv=%d ffc_pump_mv=%d ffc_fw_fv=%d ffc_restore_fv_mv=%d ffc_gl_end=\"%s\" ",
			READ_ONCE(gaugeless_ffc),
			b->ffc.ending ? "ending" : b->ffc.active ? "active" :
			b->ffc.fault ? "fault" : "off",
			b->ffc.fw_mv, b->ffc.pump_mv, b->ffc.fw_fv, b->ffc.restore_fv_mv,
			b->ffc.ending ?: b->ffc.end ?: "none");
	if (qcom_battmgr_gaugeless(b))
		len += sysfs_emit_at(buf, len,
			"gaugeless_full=%d stock_pack=%d fw_soc=%d ui_soc=%d smooth_soc=%d ui_full=%d ui_shown=%d ",
			READ_ONCE(gaugeless_full), READ_ONCE(b->pack), READ_ONCE(b->uisoc_fw_soc),
			READ_ONCE(b->uisoc.ui), READ_ONCE(b->uisoc.smooth),
			READ_ONCE(b->uisoc.full), qcom_battmgr_uisoc_shown(b));
	if (b->pps_ops)
		return b->pps_ops->describe(b->pps_data, buf, len);
	return len + sysfs_emit_at(buf, len, "state=idle\n");
}
static DEVICE_ATTR_RO(charge_contract);

static struct attribute *erhai_charge_attrs[] = {
	&dev_attr_charge_contract.attr,
	NULL,
};
ATTRIBUTE_GROUPS(erhai_charge);

/* Record measured input, not the value merely accepted by a SET response. */
static void qcom_battmgr_pd_log(struct qcom_battmgr *battmgr, u32 type)
{
	const char *name;
	u32 uv, ua, icl;

	if (battmgr->pd_logged ||
	    qcom_battmgr_get_property(battmgr, BATTMGR_USB_PROPERTY_GET, USB_VOLT_NOW, &uv) ||
	    qcom_battmgr_get_property(battmgr, BATTMGR_USB_PROPERTY_GET, USB_CURR_NOW, &ua) ||
	    qcom_battmgr_get_property(battmgr, BATTMGR_USB_PROPERTY_GET,
				      USB_INPUT_CURR_LIMIT, &icl))
		return;

	switch (type) {
	case POWER_SUPPLY_USB_TYPE_PD:
	case POWER_SUPPLY_USB_TYPE_PD_DRP:
	case OPLUS_USB_TYPE_PD_SDP:
	case POWER_SUPPLY_USB_TYPE_PD_PPS:
		name = "PD fixed";
		break;
	case POWER_SUPPLY_USB_TYPE_SDP:
		name = "SDP";
		break;
	case POWER_SUPPLY_USB_TYPE_CDP:
		name = "CDP";
		break;
	case POWER_SUPPLY_USB_TYPE_DCP:
		name = "DCP";
		break;
	default:
		name = "USB";
		break;
	}

	dev_info(battmgr->dev, "USB contract: %s, VBUS %u mV, IBUS %u mA, ICL %u mA\n",
		 name, uv / 1000, ua / 1000, icl / 1000);
	battmgr->pd_logged = true;
}

/* Erhai fixed-PD buck profile. FCC controls battery demand independently
 * of USB ICL. The AP owns the tail FV/full/recharge decisions. Use
 * the domestic cell envelope and the firmware-reported FCC rating.
 * BATT_TEMP is in centi-C. */
static int qcom_battmgr_pd_fcc(struct qcom_battmgr *b)
{
	const struct erhai_charge_step *step;
	u32 auth, hmac, temp, soc, uv, max_fcc, want, current_fcc;
	int ret;

	if (b->buck.phase != ERHAI_BUCK_OFF && b->buck.phase != ERHAI_BUCK_NORMAL)
		return 0;
	/* The charge current stays the firmware's own without the gauge. */
	if (qcom_battmgr_gaugeless(b))
		return 0;

	ret = qcom_battmgr_get_property(b, BATTMGR_BAT_PROPERTY_GET, BATT_CHG_CTRL_LIM, &current_fcc);
	if (ret)
		return ret;
	if (!b->pd_restore_fcc) {
		/* The firmware reports a limit above its own ceiling and
		 * refuses that value on write. Save what it will accept back.
		 */
		b->pd_saved_fcc_ua = current_fcc;
		if (!qcom_battmgr_get_property(b, BATTMGR_BAT_PROPERTY_GET,
					       BATT_CHG_CTRL_LIM_MAX, &max_fcc) &&
		    max_fcc > 0 && current_fcc > max_fcc)
			b->pd_saved_fcc_ua = max_fcc;
	}
	want = b->pd_saved_fcc_ua;
	if (!b->tail_owned && !test_bit(PD_EVENT_TAIL, &b->pd_events) &&
	    !b->charge_inhibited && !b->charge_policy_hold && !b->charge_error &&
	    !qcom_battmgr_get_property(b, BATTMGR_BAT_PROPERTY_GET, BATT_TEMP, &temp) &&
	    !qcom_battmgr_get_property(b, BATTMGR_BAT_PROPERTY_GET, BATT_CAPACITY, &soc) &&
	    !qcom_battmgr_get_property(b, BATTMGR_BAT_PROPERTY_GET, BATT_VOLT_NOW, &uv) &&
	    !qcom_battmgr_get_property(b, BATTMGR_BAT_PROPERTY_GET, BATT_OPLUS_AUTH, &auth) &&
	    auth == 1 &&
	    !qcom_battmgr_get_property(b, BATTMGR_BAT_PROPERTY_GET, BATT_OPLUS_HMAC, &hmac) &&
	    hmac == 1 &&
	    !qcom_battmgr_get_property(b, BATTMGR_BAT_PROPERTY_GET, BATT_CHG_CTRL_LIM_MAX, &max_fcc) &&
	    max_fcc > 0 && max_fcc <= INT_MAX) {
		step = erhai_cell_entry(soc / 100, (s32)temp < 0 ? -1 : (s32)temp / 10, uv / 1000);
		/* The ordinary buck profile applies outside the pump table. */
		if (step)
			want = min(max_fcc, step->ma * 2000U);
	}
	if (want == current_fcc)
		return 0;
	/* A timeout does not prove that the firmware ignored the write. */
	b->pd_restore_fcc = true;
	return qcom_battmgr_set_property(b, BATTMGR_BAT_PROPERTY_SET, BATT_CHG_CTRL_LIM, want);
}

/* Restore the host's previous ICL only after asking the firmware for 5 V. */
static int qcom_battmgr_pd_stop(struct qcom_battmgr *battmgr, bool offline,
			      const char *reason)
{
	u32 vbus;
	int ret, err, fcc_err = 0;

	if (battmgr->pd_restore_fcc) {
		/* FFC now owns FCC and saved the ordinary vote for its exit. */
		if (battmgr->buck.phase == ERHAI_FFC_FAST && battmgr->buck_saved_fcc_valid)
			ret = 0;
		else
			ret = qcom_battmgr_set_property(battmgr, BATTMGR_BAT_PROPERTY_SET,
							BATT_CHG_CTRL_LIM, battmgr->pd_saved_fcc_ua);
		fcc_err = ret;
		if (!ret)
			battmgr->pd_restore_fcc = false;
	}
	if (!battmgr->pd_restore)
		return fcc_err;

	battmgr->fixed_exit = reason;
	err = qcom_battmgr_set_property(battmgr, BATTMGR_USB_PROPERTY_SET,
					USB_INPUT_CURR_LIMIT,
					min(battmgr->pd_input_limit_ua, PD_TRANSITION_UA));
	ret = qcom_battmgr_set_property(battmgr, BATTMGR_BAT_PROPERTY_SET,
					battmgr->qc_contract ? BATT_OPLUS_SET_QC : BATT_OPLUS_SET_PDO,
					PD_DEFAULT_MV);
	/* The firmware answers the 5 V request with -EREMOTEIO even when it
	 * performs it. Trust the VBUS readback instead of the reply.
	 */
	if ((!ret || ret == -EREMOTEIO) && !offline) {
		msleep(PD_SHUTDOWN_WAIT_MS);
		ret = qcom_battmgr_get_property(battmgr, BATTMGR_USB_PROPERTY_GET,
						USB_VOLT_NOW, &vbus);
		if (!ret && vbus >= PD_VBUS_RISEN_UV)
			ret = -EAGAIN;
	}
	if (!ret || offline) {
		err = qcom_battmgr_set_property(battmgr, BATTMGR_USB_PROPERTY_SET,
						USB_INPUT_CURR_LIMIT, battmgr->pd_saved_limit_ua);
		if (!err || offline)
			battmgr->pd_restore = false;
	}
	err = fcc_err ?: ret ?: err;
	battmgr->pd_state = battmgr->pd_restore || battmgr->pd_restore_fcc ? PD_FAILED : PD_IDLE;
	battmgr->pd_logged = false;
	if (err)
		dev_warn(battmgr->dev, "PD: restore 5 V (%s) failed: %d\n", reason, err);
	else
		dev_info(battmgr->dev, "PD: restored 5 V (%s)\n", reason);
	return err;
}

static void qcom_battmgr_route_delivered(struct erhai_route_retry *retry)
{
	retry->delivered = true;
	retry->recovery_step = 0;
	retry->next = jiffies + msecs_to_jiffies(CHARGE_CTRL_POLL_MS);
}

static void qcom_battmgr_note_delivery(struct qcom_battmgr *b, unsigned int route)
{
	switch (route) {
	case ERHAI_SERIAL:
		qcom_battmgr_route_delivered(&b->serial_retry_state);
		break;
	case ERHAI_UFCS:
		qcom_battmgr_route_delivered(&b->ufcs_retry_state);
		break;
	case ERHAI_PPS:
		qcom_battmgr_route_delivered(&b->pps_retry_state);
		break;
	}
}

static void qcom_battmgr_restore_route(struct qcom_battmgr *b, unsigned int route)
{
	if (route) {
		qcom_battmgr_note_delivery(b, route);
		b->restore_route = route;
	} else {
		b->retry_once |= ERHAI_SERIAL | ERHAI_UFCS;
	}
}

static void qcom_battmgr_ffc_arm(struct qcom_battmgr *b);

/* Stock recharge check: six qualified samples, COUNT_TIMELIMIT 4 s.
 * Only a completed direct session arms this latch, never SOC at attach.
 * Without the pack gauge the engine reports one only for gaugeless_ffc.
 */
static bool qcom_battmgr_tail_owned(struct qcom_battmgr *b)
{
	if (test_bit(PD_EVENT_TAIL, &b->pd_events) && qcom_battmgr_gaugeless(b)) {
		clear_bit(PD_EVENT_TAIL, &b->pd_events);
		if (!b->ffc.active)
			qcom_battmgr_ffc_arm(b);
		return b->tail_owned;
	}
	if (test_and_clear_bit(PD_EVENT_TAIL, &b->pd_events)) {
		b->tail_owned = true;
		b->recharge_samples = 0;
		b->recharge_at = jiffies;
		if (b->buck.phase == ERHAI_BUCK_OFF)
			erhai_buck_start(&b->buck, jiffies);
		erhai_buck_pump_done(&b->buck, jiffies);
	}
	/* A voltage dip during the FFC wait or its disabled idle interval
	 * is not recharge. Only check_rechg after AP full releases this latch.
	 */
	return b->tail_owned;
}

void erhai_charge_tail_complete(unsigned int route)
{
	guard(mutex)(&erhai_bridge_lock);
	if (erhai_battmgr) {
		if (route == ERHAI_SERIAL)
			WRITE_ONCE(erhai_battmgr->serial_tail_check_at,
				   jiffies + msecs_to_jiffies(3000));
		WRITE_ONCE(erhai_battmgr->tail_route, route);
		set_bit(PD_EVENT_TAIL, &erhai_battmgr->pd_events);
	}
}
EXPORT_SYMBOL_GPL(erhai_charge_tail_complete);

/* D+/D- probes share the existing charge-work polling interval. */
static unsigned long qcom_battmgr_probe_retry(unsigned int *failures)
{
	/* Repeat only on idle 5 V. Bound the exponent so an unclassified
	 * charger is revisited even without an attach/capability interrupt.
	 */
	unsigned int shift = min(*failures, 4U);

	*failures = shift + 1;
	return jiffies + msecs_to_jiffies(CHARGE_CTRL_POLL_MS << shift);
}

/* Fixed PDOs and PPS share a worker and never write the contract concurrently. */
/* Absolute retry slots following stable power: 30 s, 120 s, 480 s.
 * If the evidence arrives late, keep the spacing between later slots.
 */
static bool qcom_battmgr_retry_slot(struct erhai_route_retry *retry, unsigned long stable_since,
				    bool restoring)
{
	static const unsigned int slots[] = { 30, 120, 480 };
	unsigned int slot = retry->attempts;
	unsigned long due;

	if (retry->delivered) {
		/* Recovery of a known contract has no attachment quota. If
		 * recovery fails, space subsequent probes up to eight minutes.
		 */
		if (!restoring && time_before(jiffies, retry->next))
			return false;
		retry->next = jiffies + slots[retry->recovery_step] * HZ;
		if (retry->recovery_step + 1 < ARRAY_SIZE(slots))
			retry->recovery_step++;
		return true;
	}
	if (slot >= ARRAY_SIZE(slots))
		return false;
	due = slot ? retry->next : stable_since + slots[0] * HZ;
	if (time_before(jiffies, due))
		return false;
	if (++retry->attempts < ARRAY_SIZE(slots))
		retry->next = jiffies + (slots[slot + 1] - slots[slot]) * HZ;
	return true;
}

/* Reserve each interruption before issuing any stop command. PHY readiness,
 * cell termination and per-protocol hardware protection take precedence.
 */
static unsigned int qcom_battmgr_retry_routes(struct qcom_battmgr *b,
					      const struct erhai_probe_status *status,
					      bool ufcs_active)
{
	unsigned int eligible = b->retry_once | b->retry_evidence | status->responded;
	unsigned int routes = 0;

	if (b->serial_retry_state.delivered)
		eligible |= ERHAI_SERIAL;
	/* A missed first handshake earns one discovery slot after 30 s
	 * of confirmed fixed 9 V power. Other failures supply no evidence.
	 */
	if (b->ufcs_retry_state.delivered ||
	    (status->first_ufcs_timeout && !b->ufcs_retry_state.attempts &&
	     b->pd_state == PD_ACTIVE &&
	     time_after_eq(jiffies, b->fixed_stable_since + 30 * HZ)))
		eligible |= ERHAI_UFCS;
	eligible &= status->ready & ~status->protected;
	if (eligible & b->restore_route)
		eligible &= b->restore_route;
	if (b->restore_route == ERHAI_PPS && (status->ready & ERHAI_PPS) &&
	    b->pps_ops->available(b->pps_data))
		return 0;

	if ((eligible & ERHAI_SERIAL) && !b->serial_blocked &&
	    qcom_battmgr_retry_slot(&b->serial_retry_state, b->stable_since,
				    b->restore_route == ERHAI_SERIAL))
		routes |= ERHAI_SERIAL;
	if ((eligible & ERHAI_UFCS) && !ufcs_active &&
	    qcom_battmgr_retry_slot(&b->ufcs_retry_state, b->stable_since,
				    b->restore_route == ERHAI_UFCS))
		routes |= ERHAI_UFCS;
	b->retry_once &= ~routes;
	return routes;
}

/* Sample on the 5 V fallback path. Contract readers and the pump watchdog
 * never wait for these GLINK transactions.
 */
static int erhai_usb_readback(struct qcom_battmgr *b, unsigned int prop)
{
	u32 value;
	int ret = qcom_battmgr_get_property(b, BATTMGR_USB_PROPERTY_GET, prop, &value);

	return ret ? ret : (int)value;
}

/* The crossbar returning D+/D- to the PMIC can turn a known brick into
 * an apparent SDP. Give its 5 V buck path the board's charger vote and
 * run hardware AICL. Release the attachment-scoped vote on unplug.
 */
static int qcom_battmgr_serial_icl(struct qcom_battmgr *b, bool enable)
{
	u32 vbus;
	int ret;

	if (enable) {
		if (b->aicl_done && b->serial_icl_vote)
			goto sample;
		ret = qcom_battmgr_get_property(b, BATTMGR_USB_PROPERTY_GET, USB_VOLT_NOW, &vbus);
		if (ret || vbus < 4000000 || vbus > 5500000)
			return ret ?: -EAGAIN;
		if (!b->serial_icl_vote) {
			b->serial_sample_at = 0;
			b->serial_icl_ua = b->serial_input_ua = b->serial_max_ua = -ENODATA;
		}
		b->serial_icl_vote = true; /* A missed write ACK may have applied the vote. */
	} else if (!b->serial_icl_vote) {
		return 0;
	}
	ret = qcom_battmgr_set_property(b, BATTMGR_USB_PROPERTY_SET, USB_INPUT_CURR_LIMIT,
					enable ? ERHAI_SERIAL_5V_UA : U32_MAX);
	if (ret)
		return ret;
	if (!enable) {
		b->serial_icl_vote = false;
		b->aicl_done = false;
		return 0;
	}
	ret = qcom_battmgr_set_property(b, BATTMGR_USB_PROPERTY_SET, USB_OPLUS_RERUN_AICL, 0);
	b->aicl_done = !ret;
	if (ret)
		return ret;
sample:
	b->serial_icl_ua = erhai_usb_readback(b, USB_INPUT_CURR_LIMIT);
	b->serial_input_ua = erhai_usb_readback(b, USB_CURR_NOW);
	b->serial_max_ua = erhai_usb_readback(b, USB_CURR_MAX);
	b->serial_sample_at = jiffies;
	return 0;
}

/* All actuator writes share charge_lock with protocol arbitration and
 * user limits. A lost acknowledgement invalidates the cached command.
 */
static void qcom_battmgr_buck_invalidate(struct qcom_battmgr *b)
{
	b->buck_fv_applied = b->buck_fcc_applied = -1;
	b->buck_enable_applied = b->buck_para_applied = -1;
	b->buck_full_applied = -1;
}

static int qcom_battmgr_buck_write(struct qcom_battmgr *b, unsigned int property,
				 int value, int *applied)
{
	int ret;

	if (*applied == value)
		return 0;
	*applied = -1;
	ret = qcom_battmgr_set_property(b, BATTMGR_BAT_PROPERTY_SET, property, value);
	if (!ret)
		*applied = value;
	return ret;
}

static int qcom_battmgr_buck_pause(struct qcom_battmgr *b)
{
	int ret, fv_ret = 0;

	ret = qcom_battmgr_buck_write(b, BATT_OPLUS_CHG_EN, 0, &b->buck_enable_applied);
	if (b->buck.normal_fv_mv && b->buck.temp_region < 7 && !b->buck.fv_firmware)
		fv_ret = qcom_battmgr_buck_write(b, BATT_VOLT_MAX, b->buck.normal_fv_mv,
					       &b->buck_fv_applied);
	return ret ?: fv_ret;
}

static int qcom_battmgr_buck_apply(struct qcom_battmgr *b)
{
	struct erhai_buck *buck = &b->buck;
	bool enable = !buck->disable && !b->charge_policy_hold && !b->charge_inhibited &&
		      !READ_ONCE(b->otg_request);
	int ret, hint, para, region, ffc = 0;
	u32 fcc, max_fcc;

	if (buck->temp_region >= 7 &&
	    b->buck_sample.mv > buck->fv_mv - ERHAI_BUCK_WARM_FV_MARGIN_MV &&
	    b->buck_fv_applied != buck->fv_mv)
		enable = false;

	/* switch_ffc and FFC_IDLE use an output-disable vote, not ICL=0;
	 * the USB source continues supplying the system during both waits.
	 */
	if (!enable || b->buck_enable_applied < 0 || b->buck_fv_applied < buck->fv_mv) {
		ret = qcom_battmgr_buck_write(b, BATT_OPLUS_CHG_EN, 0,
					    &b->buck_enable_applied);
		if (ret)
			return ret;
	}
	/* The stock float voltage check keeps the PMIC power path above
	 * a warm cell. Output remains disabled while it exceeds warm FV.
	 */
	if (buck->temp_region < 7 ||
	    b->buck_sample.mv <= buck->fv_mv - ERHAI_BUCK_WARM_FV_MARGIN_MV) {
		ret = qcom_battmgr_buck_write(b, BATT_VOLT_MAX, buck->fv_mv,
					    &b->buck_fv_applied);
		if (ret)
			return ret;
	}
	if (buck->fcc_ma) {
		if (!b->buck_saved_fcc_valid) {
			ret = qcom_battmgr_get_property(b, BATTMGR_BAT_PROPERTY_GET,
						BATT_CHG_CTRL_LIM, &fcc);
			if (ret)
				return ret;
			/* As qcom_battmgr_pd_fcc(): the firmware reports a limit
			 * above its ceiling and refuses that value on write.
			 */
			if (!qcom_battmgr_get_property(b, BATTMGR_BAT_PROPERTY_GET,
						       BATT_CHG_CTRL_LIM_MAX, &max_fcc) &&
			    max_fcc > 0 && fcc > max_fcc)
				fcc = max_fcc;
			b->buck_saved_fcc_ua = b->pd_restore_fcc ? b->pd_saved_fcc_ua : fcc;
			b->buck_saved_fcc_valid = true;
		}
		ret = qcom_battmgr_buck_write(b, BATT_CHG_CTRL_LIM, buck->fcc_ma * 1000,
					    &b->buck_fcc_applied);
		if (ret)
			return ret;
	} else if (b->buck_saved_fcc_valid) {
		ret = qcom_battmgr_buck_write(b, BATT_CHG_CTRL_LIM, b->buck_saved_fcc_ua,
					    &b->buck_fcc_applied);
		if (ret)
			return ret;
		b->buck_saved_fcc_valid = false;
	}
	/* HAL get_props_from_adsp_by_buffer / set_full_para_qbg.
	 * V3P7 has a zero-width little-cold-high entry in the newer enum.
	 */
	region = buck->temp_region >= 2 ? buck->temp_region + 1 : buck->temp_region;
	if (buck->phase >= ERHAI_FFC_WAIT && buck->phase <= ERHAI_FFC_IDLE)
		ffc = buck->phase - ERHAI_FFC_WAIT + 1;
	para = (region << 3) | (ffc << 1);
	hint = qcom_battmgr_buck_write(b, BATT_OPLUS_FULL_PARA, para,
				     &b->buck_para_applied);
	ret = qcom_battmgr_buck_write(b, BATT_OPLUS_SET_FULL,
				    buck->phase == ERHAI_BUCK_FULL && buck->temp_region <= 6,
				    &b->buck_full_applied);
	b->buck_gauge_error = hint ?: ret;
	/* These report AP decisions to the gauge. The output-disable and
	 * FV/FCC actuators above remain authoritative if a hint is rejected.
	 */
	return qcom_battmgr_buck_write(b, BATT_OPLUS_CHG_EN, enable,
				      &b->buck_enable_applied);
}

/*
 * The guard's rule without the pack gauge, see qcom_battmgr_buck_guard():
 * no charging below 0 C, from 53 C, from the battery overvoltage threshold,
 * nor from 45 C above the warm float voltage until the warm recharge one.
 */
static bool qcom_battmgr_guard_withholds(const struct erhai_buck *buck, int mv, bool withheld)
{
	if (buck->temp_region < 2 || buck->temp_region > 7 || buck->over_voltage)
		return true;
	return buck->temp_region == 7 && mv > (withheld ? 4030 : 4130);
}

/*
 * Whether the firmware selected the stock pack's battery profile, checked
 * until it has said so; see qcom_battmgr_gaugeless_pack().
 */
static bool qcom_battmgr_gaugeless_pack(struct qcom_battmgr *b);

static int qcom_battmgr_stock_pack(struct qcom_battmgr *b)
{
	if (READ_ONCE(b->pack) == 1 || time_before(jiffies, READ_ONCE(b->pack_at)))
		return READ_ONCE(b->pack);
	WRITE_ONCE(b->pack, qcom_battmgr_gaugeless_pack(b));
	WRITE_ONCE(b->pack_at, jiffies + msecs_to_jiffies(CHARGE_CTRL_POLL_MS));
	return READ_ONCE(b->pack);
}

/* The master pump's battery ADC, pumps idle; @hold false turns it off. */
static int qcom_battmgr_pump_cell_mv(struct qcom_battmgr *b, bool hold)
{
	const struct erhai_pps_ops *ops = b->pps_ops ?: b->pps_parked;

	if (!ops || !ops->cell_mv || !b->pps_data)
		return hold ? -ENODEV : 0;
	return ops->cell_mv(b->pps_data, hold);
}

/*
 * End a gaugeless FFC tail and give the charge back to the firmware: output
 * off, the float voltage it had before (or the stock normal one, which it
 * holds by itself as well), its own charge current, the gauge hints as the
 * stock charger leaves them outside FFC and full, then the output as the
 * guard would have it. Until all of that is acknowledged the output stays
 * off and the charge worker tries again.
 */
static int qcom_battmgr_ffc_handback(struct qcom_battmgr *b, const char *why)
{
	struct erhai_buck *buck = &b->buck;
	int ret, region;
	bool enable;

	b->ffc.ending = why;
	ret = qcom_battmgr_buck_write(b, BATT_OPLUS_CHG_EN, 0, &b->buck_enable_applied);
	ret = ret ?: qcom_battmgr_buck_write(b, BATT_VOLT_MAX, b->ffc.restore_fv_mv,
					     &b->buck_fv_applied);
	if (!ret && b->buck_saved_fcc_valid) {
		ret = qcom_battmgr_buck_write(b, BATT_CHG_CTRL_LIM, b->buck_saved_fcc_ua,
					      &b->buck_fcc_applied);
		if (!ret)
			b->buck_saved_fcc_valid = false;
	}
	if (!ret && buck->temp_region >= 0) {
		region = buck->temp_region >= 2 ? buck->temp_region + 1 : buck->temp_region;
		b->buck_gauge_error = qcom_battmgr_buck_write(b, BATT_OPLUS_FULL_PARA, region << 3,
							      &b->buck_para_applied) ?:
				      qcom_battmgr_buck_write(b, BATT_OPLUS_SET_FULL, 0,
							      &b->buck_full_applied);
	}
	enable = !qcom_battmgr_guard_withholds(buck, b->buck_sample.mv, buck->disable) &&
		 !b->charge_policy_hold && !b->charge_inhibited;
	ret = ret ?: qcom_battmgr_buck_write(b, BATT_OPLUS_CHG_EN, enable,
					     &b->buck_enable_applied);
	if (ret) {
		dev_warn_ratelimited(b->dev, "gaugeless FFC: handing the charge back to the firmware failed: %d; retrying\n",
				     ret);
		return ret;
	}
	qcom_battmgr_pump_cell_mv(b, false);
	dev_info(b->dev, "gaugeless FFC ended (%s): firmware charge profile back, FV %d mV, FCC %d uA, output %s\n",
		 why, b->ffc.restore_fv_mv, b->buck_fcc_applied, enable ? "on" : "off");
	b->ffc.active = false;
	b->ffc.ending = NULL;
	b->ffc.end = why;
	b->tail_owned = false;
	/* The guard starts over on the next pass. */
	b->buck = (struct erhai_buck){};
	b->guard_withheld = !enable;
	power_supply_changed(b->bat_psy);
	return 0;
}

static int qcom_battmgr_buck_release(struct qcom_battmgr *b)
{
	bool enable;
	int ret;

	if (b->ffc.active)
		return qcom_battmgr_ffc_handback(b, b->ffc.ending ?: "charger removed or driver stopped");
	if (b->buck.phase == ERHAI_BUCK_OFF)
		return 0;
	if (!b->buck.normal_fv_mv || b->buck.fv_firmware) {
		ret = qcom_battmgr_buck_write(b, BATT_OPLUS_CHG_EN,
					    !b->charge_policy_hold && !b->charge_inhibited,
					    &b->buck_enable_applied);
		if (ret)
			return ret;
		/* Not full any more for the firmware's gauge either. */
		if (b->buck.fv_firmware && b->buck_full_applied != 0)
			b->buck_gauge_error = qcom_battmgr_buck_write(b, BATT_OPLUS_SET_FULL, 0,
								      &b->buck_full_applied);
		b->buck = (struct erhai_buck){};
		b->guard_withheld = false;
		return 0;
	}
	enable = b->buck.phase != ERHAI_BUCK_FULL && !b->buck.over_voltage &&
		 b->buck.temp_region > 0 && b->buck.temp_region < 8 &&
		 !b->charge_policy_hold && !b->charge_inhibited;
	/* Withdraw elevated FV before leaving AP supervision, including reboot. */
	b->buck.phase = ERHAI_BUCK_NORMAL;
	b->buck.disable = true;
	b->buck.fv_mv = b->buck.normal_fv_mv;
	b->buck.fcc_ma = 0;
	ret = qcom_battmgr_buck_apply(b);
	if (ret)
		return ret;
	enable &= b->buck_fv_applied == b->buck.normal_fv_mv;
	ret = qcom_battmgr_buck_write(b, BATT_OPLUS_CHG_EN, enable,
				    &b->buck_enable_applied);
	if (ret)
		return ret;
	b->buck = (struct erhai_buck){};
	return 0;
}

static int qcom_battmgr_buck_read(struct qcom_battmgr *b, struct erhai_buck_sample *s)
{
	u32 mv, ma, temp, soc, cycles, auth, hmac;
	int ret;

	ret = qcom_battmgr_get_property(b, BATTMGR_BAT_PROPERTY_GET, BATT_VOLT_NOW, &mv);
	ret = ret ?: qcom_battmgr_get_property(b, BATTMGR_BAT_PROPERTY_GET, BATT_CURR_NOW, &ma);
	ret = ret ?: qcom_battmgr_get_property(b, BATTMGR_BAT_PROPERTY_GET, BATT_TEMP, &temp);
	ret = ret ?: qcom_battmgr_get_property(b, BATTMGR_BAT_PROPERTY_GET, BATT_CAPACITY, &soc);
	ret = ret ?: qcom_battmgr_get_property(b, BATTMGR_BAT_PROPERTY_GET, BATT_CYCLE_COUNT,
					    &cycles);
	ret = ret ?: qcom_battmgr_get_property(b, BATTMGR_BAT_PROPERTY_GET, BATT_OPLUS_AUTH, &auth);
	ret = ret ?: qcom_battmgr_get_property(b, BATTMGR_BAT_PROPERTY_GET, BATT_OPLUS_HMAC, &hmac);
	if (ret)
		return ret;
	if (!mv || soc > 10000 || ma == BIT(31))
		return -ENODATA;
	*s = (struct erhai_buck_sample){
		.mv = mv / 1000, .ma = -(s32)ma / 1000, .temp = (s32)temp / 10,
		.soc = soc / 100, .cycles = cycles, .qualified = auth == 1 && hmac == 1,
	};
	return 0;
}

/*
 * Without the pack gauge the firmware charges by itself: from its own
 * gauge, to its own float voltage and at its own current, as it does on the
 * boards of its family that have no AP charge controller. The AP adjusts
 * none of that. It only takes the charge output away where the stock
 * charger would not charge at this rate either and the AP, steering
 * nothing, cannot charge more slowly instead: below 0 C and from 53 C, from
 * 45 C once the cell is above the warm float voltage until it has fallen to
 * the warm recharge voltage, and from the battery overvoltage threshold
 * until 200 mV below it (batt-them-thr, fv-mv and vbatt-ov-thr-mv of the
 * stock charger devicetree). Without a reading it takes it away as well.
 * After a PD PPS session and with gaugeless_ffc set, the stock tail takes
 * over until the charger is removed, see qcom_battmgr_ffc_tick().
 */
static unsigned int qcom_battmgr_buck_guard(struct qcom_battmgr *b)
{
	struct erhai_buck *buck = &b->buck;
	enum erhai_buck_phase previous;
	struct erhai_buck_sample s;
	bool withhold, disable, full_check, pumps;
	u32 uv, ua, temp, soc, cycles;
	int mv, ret, region;

	if (buck->phase == ERHAI_BUCK_OFF)
		erhai_buck_start(buck, jiffies);
	if (time_before(jiffies, buck->next_sample) && !b->buck_error)
		return jiffies_to_msecs(buck->next_sample - jiffies) ?: 1;

	ret = qcom_battmgr_get_property(b, BATTMGR_BAT_PROPERTY_GET, BATT_VOLT_NOW, &uv);
	ret = ret ?: qcom_battmgr_get_property(b, BATTMGR_BAT_PROPERTY_GET, BATT_TEMP, &temp);
	if (!ret && !uv)
		ret = -ENODATA;
	if (ret)
		goto fail;
	mv = uv / 1000;
	b->buck_sample = (struct erhai_buck_sample){ .mv = mv, .temp = (s32)temp / 10 };
	b->buck_sample_at = jiffies;
	previous = buck->phase;
	pumps = qcom_battmgr_pps_running(b);
	WRITE_ONCE(b->uisoc_fastchg, pumps);

	/*
	 * gaugeless_full: the stock full and recharge checks (erhai_buck.h,
	 * oplus_comm_check_sw_full() and _check_rechg()) on the firmware's
	 * readings. Not while the pumps charge (the stock charger checks no
	 * full during a fast session) nor while a policy holds the output.
	 * The firmware keeps its float voltage, so only the current decides.
	 */
	full_check = READ_ONCE(gaugeless_full) && !pumps && !b->charge_policy_hold &&
		     !b->charge_inhibited && qcom_battmgr_stock_pack(b) == 1;
	if (full_check) {
		ret = qcom_battmgr_get_property(b, BATTMGR_BAT_PROPERTY_GET, BATT_CURR_NOW, &ua);
		ret = ret ?: qcom_battmgr_get_property(b, BATTMGR_BAT_PROPERTY_GET, BATT_CAPACITY,
						    &soc);
		ret = ret ?: qcom_battmgr_get_property(b, BATTMGR_BAT_PROPERTY_GET,
						    BATT_CYCLE_COUNT, &cycles);
		if (!ret && (soc > 10000 || ua == BIT(31)))
			ret = -ENODATA;
		if (ret)
			goto fail;
		s = (struct erhai_buck_sample){
			.mv = mv, .ma = -(s32)ua / 1000, .temp = (s32)temp / 10,
			.soc = soc / 100, .cycles = cycles, .qualified = true,
		};
		b->buck_sample = s;
		buck->fv_firmware = true;
		erhai_buck_update(buck, &s, jiffies);
		buck->recharged = false;	/* PPS may follow by itself */
	} else {
		if (!READ_ONCE(gaugeless_full) && buck->phase == ERHAI_BUCK_FULL)
			buck->phase = ERHAI_BUCK_NORMAL;
		buck->next_sample = jiffies + msecs_to_jiffies(4000);
		erhai_buck_temperature(buck, b->buck_sample.temp);
		if (mv >= 4600)
			buck->over_voltage = true;
		else if (mv < 4400)
			buck->over_voltage = false;
	}

	withhold = qcom_battmgr_guard_withholds(buck, mv, b->guard_withheld);
	if (withhold != b->guard_withheld || b->buck_error)
		dev_info(b->dev, "charge output %s: cell %d mV, %s%d.%d C\n",
			 withhold ? "withdrawn" : "allowed", mv,
			 b->buck_sample.temp < 0 ? "-" : "",
			 abs(b->buck_sample.temp) / 10, abs(b->buck_sample.temp) % 10);
	b->guard_withheld = withhold;
	if (previous != buck->phase && (previous == ERHAI_BUCK_FULL ||
					buck->phase == ERHAI_BUCK_FULL)) {
		dev_info(b->dev, "%s: cell %d mV, %d mA, firmware %d%%\n",
			 buck->phase == ERHAI_BUCK_FULL ? "battery full" : "recharge",
			 mv, b->buck_sample.ma, b->buck_sample.soc);
		mod_delayed_work(system_freezable_wq, &b->uisoc_work, 0);
	}
	disable = withhold || buck->phase == ERHAI_BUCK_FULL;
	if (disable != buck->disable || previous != buck->phase)
		power_supply_changed(b->bat_psy);
	buck->disable = disable;
	ret = qcom_battmgr_buck_write(b, BATT_OPLUS_CHG_EN,
				      !disable && !b->charge_policy_hold && !b->charge_inhibited,
				      &b->buck_enable_applied);
	if (ret)
		goto fail;
	/*
	 * The stock gauge hints (oplus_get_props_from_adsp_by_buffer(),
	 * oplus_mms_gauge_set_batt_full_work()): fast charging, no FFC, the
	 * temperature region; full up to the normal-high region. Hints: a
	 * refusal is recorded, not acted on.
	 */
	if (READ_ONCE(b->pack) == 1 && buck->temp_region >= 0 &&
	    (READ_ONCE(gaugeless_full) || b->buck_full_applied == 1)) {
		region = buck->temp_region >= 2 ? buck->temp_region + 1 : buck->temp_region;
		b->buck_gauge_error =
			qcom_battmgr_buck_write(b, BATT_OPLUS_FULL_PARA, (region << 3) | pumps,
						&b->buck_para_applied) ?:
			qcom_battmgr_buck_write(b, BATT_OPLUS_SET_FULL,
						buck->phase == ERHAI_BUCK_FULL &&
						buck->temp_region <= 6,
						&b->buck_full_applied);
	}
	b->buck_error = 0;
	return 4000;

fail:
	if (b->buck_error != ret)
		dev_warn(b->dev, "charge guard failed: %d; withdrawing charge output\n", ret);
	b->buck_error = ret;
	buck->sw_count = buck->hw_count = buck->recharge_count = 0;
	qcom_battmgr_buck_pause(b);
	return PD_POLL_MS;
}

/* gaugeless_ffc limits, see qcom_battmgr_ffc_tick(). */
#define ERHAI_FFC_POLL_MS		1000	/* while FFC charges */
#define ERHAI_FFC_SLOW_MS		4000	/* stock COUNT_TIMELIMIT */
#define ERHAI_FFC_CEILING_MV		4570
#define ERHAI_FFC_VDIFF_MV		200
#define ERHAI_FFC_BAD_SAMPLES		4
#define ERHAI_FFC_STALE_SAMPLES		10
/* fv-mv of the normal regions: what the firmware holds the cell at itself. */
#define ERHAI_FFC_RESTORE_FV_MV		4490

static bool qcom_battmgr_gaugeless_pack(struct qcom_battmgr *b);

/*
 * A gaugeless PPS session has charged its table through; the stock charger
 * would now run FFC (oplus_comm_switch_ffc()). Take the buck over for that
 * if everything the tail depends on is there; otherwise the firmware goes on
 * charging by itself as it does without gaugeless_ffc.
 */
static void qcom_battmgr_ffc_arm(struct qcom_battmgr *b)
{
	const char *no = NULL;
	int pump = -ENODEV, mv;
	u32 fv;

	if (!READ_ONCE(gaugeless_ffc))
		return;
	if (b->ffc.fault)
		no = "a cross-check failed on this plug";
	else if (!b->buck_online || READ_ONCE(b->otg_request))
		no = "no charger";
	else if (b->charge_policy_hold || b->charge_inhibited || b->charge_error ||
		 READ_ONCE(b->charge_bypass))
		no = "charging inhibited";
	else if (b->buck.phase == ERHAI_BUCK_OFF || b->buck.disable || b->buck_error)
		no = "the charge guard withholds charging";
	/* As for PPS: the stock pack by the firmware's battery profile. */
	else if (qcom_battmgr_stock_pack(b) != 1)
		no = "the firmware reports no " ERHAI_FW_PACK_PROFILE " battery profile";
	else if ((pump = qcom_battmgr_pump_cell_mv(b, true)) < 0 && pump != -ENODATA)
		no = "the pump battery ADC cannot be read";
	if (no) {
		qcom_battmgr_pump_cell_mv(b, false);
		b->ffc.end = no;
		dev_info(b->dev, "gaugeless FFC not started: %s\n", no);
		return;
	}

	/* What to give back: the firmware's float voltage if it reads as one. */
	b->ffc.fw_fv = qcom_battmgr_get_property(b, BATTMGR_BAT_PROPERTY_GET,
						  BATT_VOLT_MAX, &fv) ? -1 : (int)fv;
	mv = b->ffc.fw_fv >= 1000000 ? b->ffc.fw_fv / 1000 : b->ffc.fw_fv;
	b->ffc.restore_fv_mv = mv >= 4400 && mv <= ERHAI_FFC_RESTORE_FV_MV + 10 ?
			       mv : ERHAI_FFC_RESTORE_FV_MV;
	b->ffc.active = true;
	b->ffc.ending = b->ffc.end = NULL;
	b->ffc.fw_mv = 0;
	b->ffc.pump_mv = pump;
	b->ffc.last_ua = U32_MAX;
	b->ffc.stale = b->ffc.vdiff = b->ffc.iover = 0;
	b->ffc.next = jiffies;
	b->tail_owned = true;
	b->buck_error = 0;
	/* The AP owns the float voltage now; the pumps are done. */
	b->buck.fv_firmware = false;
	WRITE_ONCE(b->uisoc_fastchg, false);
	/* switch_ffc: 15 s with the output off, then ffc_start_work. */
	erhai_buck_pump_done(&b->buck, jiffies);
	dev_info(b->dev, "gaugeless FFC: PPS session complete, output off for 15 s, then FFC; firmware FV %d, handed back as %d mV\n",
		 b->ffc.fw_fv, b->ffc.restore_fv_mv);
	power_supply_changed(b->bat_psy);
}

static unsigned int qcom_battmgr_ffc_stop(struct qcom_battmgr *b, const char *why, bool fault)
{
	b->ffc.fault |= fault;
	return qcom_battmgr_ffc_handback(b, why) ? PD_POLL_MS : 1;
}

/*
 * The stock tail on a board without the pack gauge (gaugeless_ffc), from
 * qcom_battmgr_ffc_arm() until the charger is removed: erhai_buck_update()
 * and qcom_battmgr_buck_apply() as with the gauge, on the firmware's battery
 * readings. They steer a float voltage 140 mV above the firmware's own, so
 * on every sample, once a second while FFC charges:
 *
 * - all of them have to be there, and the cell voltage has to agree with
 *   the master pump's battery ADC within 200 mV (pumps idle, its ADC held
 *   on); the higher of the two is the cell voltage that steers the tail;
 * - the cell voltage stays under a hard ceiling of 4570 mV, between the
 *   stock FFC step voltage (4540 mV) and the battery overvoltage threshold
 *   (4600 mV);
 * - while FFC charges, the current reading has to move and the pack current
 *   has to stay near the FFC charge current the AP set.
 *
 * Four failed voltage checks in a row (ten readings of a current that does
 * not move, eight of a current above the limit), a reading at the ceiling,
 * a failed write, the pumps in use or a restart of the charger firmware end
 * the tail for this plug. So do clearing gaugeless_ffc, charging inhibited
 * by policy (bypass, charge limit), removing the charger and suspend. Every
 * end gives the charge back to the firmware, see qcom_battmgr_ffc_handback().
 */
static unsigned int qcom_battmgr_ffc_tick(struct qcom_battmgr *b)
{
	struct erhai_buck *buck = &b->buck;
	struct erhai_buck_sample s;
	enum erhai_buck_phase previous = buck->phase;
	unsigned int step = buck->step;
	u32 uv, ua, temp, soc, cycles;
	const char *fault = NULL;
	int pump, fcc_ma, ret;
	bool fast = buck->phase == ERHAI_FFC_FAST;

	if (b->ffc.ending)
		return qcom_battmgr_ffc_handback(b, b->ffc.ending) ? PD_POLL_MS : 1;
	if (!READ_ONCE(gaugeless_ffc))
		return qcom_battmgr_ffc_stop(b, "gaugeless_ffc cleared", false);
	if (b->charge_policy_hold || b->charge_inhibited || b->charge_error ||
	    READ_ONCE(b->charge_bypass))
		return qcom_battmgr_ffc_stop(b, "charging inhibited", false);
	if (qcom_battmgr_pps_running(b)) {
		/* The session may still be handing the port back: wait with
		 * the output off, as long as the stock 15 s wait again.
		 */
		if (buck->phase != ERHAI_FFC_WAIT ||
		    time_after(jiffies, buck->wait_until + msecs_to_jiffies(15000)))
			return qcom_battmgr_ffc_stop(b, "the pumps are in use", true);
		if (qcom_battmgr_buck_write(b, BATT_OPLUS_CHG_EN, 0, &b->buck_enable_applied))
			return qcom_battmgr_ffc_stop(b, "charge control write failed", true);
		return PD_POLL_MS;
	}
	if (time_before(jiffies, b->ffc.next))
		return jiffies_to_msecs(b->ffc.next - jiffies) ?: 1;

	ret = qcom_battmgr_get_property(b, BATTMGR_BAT_PROPERTY_GET, BATT_VOLT_NOW, &uv);
	ret = ret ?: qcom_battmgr_get_property(b, BATTMGR_BAT_PROPERTY_GET, BATT_CURR_NOW, &ua);
	ret = ret ?: qcom_battmgr_get_property(b, BATTMGR_BAT_PROPERTY_GET, BATT_TEMP, &temp);
	ret = ret ?: qcom_battmgr_get_property(b, BATTMGR_BAT_PROPERTY_GET, BATT_CAPACITY, &soc);
	ret = ret ?: qcom_battmgr_get_property(b, BATTMGR_BAT_PROPERTY_GET, BATT_CYCLE_COUNT,
					    &cycles);
	if (!ret && (!uv || soc > 10000 || ua == BIT(31)))
		ret = -ENODATA;
	if (ret) {
		dev_warn(b->dev, "gaugeless FFC: battery readings unavailable: %d\n", ret);
		return qcom_battmgr_ffc_stop(b, "battery readings unavailable", true);
	}
	pump = qcom_battmgr_pump_cell_mv(b, true);
	b->ffc.fw_mv = uv / 1000;
	b->ffc.pump_mv = pump;
	b->ffc.vdiff = pump < 0 || abs(b->ffc.fw_mv - pump) > ERHAI_FFC_VDIFF_MV ?
		       b->ffc.vdiff + 1 : 0;
	b->ffc.stale = fast && ua == b->ffc.last_ua ? b->ffc.stale + 1 : 0;
	b->ffc.last_ua = ua;
	s = (struct erhai_buck_sample){
		.mv = max(b->ffc.fw_mv, pump), .ma = -(s32)ua / 1000,
		.temp = (s32)temp / 10, .soc = soc / 100, .cycles = cycles,
		/* Armed only for the stock pack's profile, see above. */
		.qualified = true,
	};
	fcc_ma = b->buck_fcc_applied > 0 ? b->buck_fcc_applied / 1000 : 0;
	b->ffc.iover = fast && fcc_ma && s.ma > fcc_ma + fcc_ma / 5 + 300 ?
		       b->ffc.iover + 1 : 0;

	if (s.mv >= ERHAI_FFC_CEILING_MV)
		fault = "cell voltage at the ceiling";
	else if (b->ffc.vdiff >= ERHAI_FFC_BAD_SAMPLES)
		fault = "cell voltage disagrees with the pump ADC";
	else if (b->ffc.stale >= ERHAI_FFC_STALE_SAMPLES)
		fault = "current reading does not change";
	else if (b->ffc.iover >= 2 * ERHAI_FFC_BAD_SAMPLES)
		fault = "charge current above the FFC limit";
	if (fault) {
		dev_warn(b->dev, "gaugeless FFC: %s: firmware %d mV %d mA, pump vbat %d mV, FCC %d mA, phase %u\n",
			 fault, b->ffc.fw_mv, s.ma, pump, fcc_ma, buck->phase);
		return qcom_battmgr_ffc_stop(b, fault, true);
	}

	b->buck_sample = s;
	b->buck_sample_at = jiffies;
	erhai_buck_update(buck, &s, jiffies);
	/* The guard's rule without the gauge: nothing below 0 C. */
	if (buck->temp_region < 2)
		buck->disable = true;
	ret = qcom_battmgr_buck_apply(b);
	if (ret) {
		dev_warn(b->dev, "gaugeless FFC: charge control write failed: %d\n", ret);
		return qcom_battmgr_ffc_stop(b, "charge control write failed", true);
	}
	b->buck_error = 0;
	if (buck->recharged) {
		/* PPS stays done for this plug; the buck recharges. */
		buck->recharged = false;
		dev_info(b->dev, "gaugeless FFC: recharge at %d mV\n", s.mv);
	}
	if (previous != buck->phase || step != buck->step) {
		dev_info(b->dev, "gaugeless FFC: phase=%u step=%u FV=%d mV FCC=%d uA enable=%d cell=%d mV (firmware %d, pump %d) %d mA soc %d%% %s%d.%d C\n",
			 buck->phase, buck->step, b->buck_fv_applied, b->buck_fcc_applied,
			 b->buck_enable_applied, s.mv, b->ffc.fw_mv, pump, s.ma, s.soc,
			 s.temp < 0 ? "-" : "", abs(s.temp) / 10, abs(s.temp) % 10);
		power_supply_changed(b->bat_psy);
	}
	b->ffc.next = jiffies + msecs_to_jiffies(buck->phase == ERHAI_FFC_FAST ?
						  ERHAI_FFC_POLL_MS : ERHAI_FFC_SLOW_MS);
	return jiffies_to_msecs(b->ffc.next - jiffies) ?: 1;
}

static unsigned int qcom_battmgr_buck_tick(struct qcom_battmgr *b)
{
	struct erhai_buck_sample sample;
	enum erhai_buck_phase previous;
	unsigned int step;
	int ret;

	if (!b->buck_online || READ_ONCE(b->otg_request))
		return 0;
	if (qcom_battmgr_gaugeless(b)) {
		qcom_battmgr_tail_owned(b); /* a PPS session may have just completed */
		return b->ffc.active ? qcom_battmgr_ffc_tick(b) : qcom_battmgr_buck_guard(b);
	}
	qcom_battmgr_tail_owned(b); /* A pump may have just published completion. */
	if (b->serial_probe || qcom_battmgr_pps_running(b) ||
	    (b->pps_ops && b->pps_ops->ufcs_active(b->pps_data)))
		return 0;
	if (b->buck.phase == ERHAI_BUCK_OFF)
		erhai_buck_start(&b->buck, jiffies);
	if (time_before(jiffies, b->buck.next_sample) && !b->buck_error)
		return jiffies_to_msecs(b->buck.next_sample - jiffies);
	ret = qcom_battmgr_buck_read(b, &sample);
	if (ret)
		goto fail;
	b->buck_sample = sample;
	b->buck_sample_at = jiffies;
	previous = b->buck.phase;
	step = b->buck.step;
	if (!b->charge_policy_hold && !b->charge_inhibited)
		erhai_buck_update(&b->buck, &sample, jiffies);
	else {
		erhai_buck_temperature(&b->buck, sample.temp);
		erhai_buck_normal_limits(&b->buck, &sample);
		if (b->buck.phase != ERHAI_BUCK_NORMAL && b->buck.phase != ERHAI_FFC_FAST)
			b->buck.disable = true;
		b->buck.next_sample = jiffies + msecs_to_jiffies(4000);
	}
	ret = qcom_battmgr_buck_apply(b);
	if (ret)
		goto fail;
	if (previous != b->buck.phase || step != b->buck.step || b->buck_error) {
		dev_info(b->dev, "buck phase=%u step=%u FV=%d mV FCC=%d uA enable=%d cell=%d mV %d mA\n",
			 b->buck.phase, b->buck.step, b->buck_fv_applied, b->buck_fcc_applied,
			 b->buck_enable_applied, sample.mv, sample.ma);
		power_supply_changed(b->bat_psy);
	}
	b->buck_error = 0;
	if (b->buck.recharged) {
		b->buck.recharged = false;
		b->tail_owned = false;
		qcom_battmgr_restore_route(b, READ_ONCE(b->tail_route));
		if (b->pps_ops)
			b->pps_ops->recharge(b->pps_data);
		b->serial_backoff = b->ufcs_backoff = false;
		b->serial_failures = b->ufcs_failures = 0;
	}
	return jiffies_to_msecs(b->buck.next_sample - jiffies) ?: 1;

fail:
	if (b->buck_error != ret)
		dev_warn(b->dev, "buck supervision failed: %d; withdrawing charge output\n", ret);
	b->buck_error = ret;
	b->buck.current_count = b->buck.sw_count = b->buck.hw_count = 0;
	b->buck.recharge_count = 0;
	/* Do not leave 4.63 V active without current gauge data or an
	 * acknowledged control sequence. Retry through the normal worker.
	 */
	qcom_battmgr_buck_pause(b);
	return PD_POLL_MS;
}

/* A SUPERVOOC reset interrupts VBUS and can unregister the UCSI partner
 * without anyone removing the cable. Stock AP VOOC PHY, FAST_NOTIFY_FULL
 * and check_chg_out_work_func: recheck the source after 3 s, below 2.5 V.
 * Use the board's CID detector (oplus_chg_8350_get_hw_detect) as well.
 * Unknown CID is not a detach; an unsupported CID falls back to the AP
 * PHY's voltage check. Stop contracts immediately in either case.
 */
static bool qcom_battmgr_keep_serial_tail(struct qcom_battmgr *b)
{
	u32 cid, uv;
	int ret;

	if (READ_ONCE(b->tail_route) != ERHAI_SERIAL ||
	    !(b->tail_owned || test_bit(PD_EVENT_TAIL, &b->pd_events)))
		return false;
	if (time_before(jiffies, READ_ONCE(b->serial_tail_check_at)))
		return true;
	ret = qcom_battmgr_get_property(b, BATTMGR_USB_PROPERTY_GET, USB_OPLUS_CID_STATUS, &cid);
	if (ret && ret != -EOPNOTSUPP)
		return true;
	if (!ret && cid)
		return true;
	if (qcom_battmgr_get_property(b, BATTMGR_USB_PROPERTY_GET, USB_VOLT_NOW, &uv))
		return true;
	return uv >= 2500000;
}

static unsigned int qcom_battmgr_pd_update(struct qcom_battmgr *battmgr)
{
	struct qcom_battmgr_pd_source src = { .battmgr = battmgr };
	struct erhai_probe_status status = {};
	bool stable, ufcs_active, tail, probe_idle, prefer_pps, dedicated, can_probe, can_probe_pumps;
	bool serial_source, detached, qc, try_qc, caps_changed, current_changed = false, bypass;
	bool gaugeless = qcom_battmgr_gaugeless(battmgr);
	u32 online, type = POWER_SUPPLY_USB_TYPE_UNKNOWN, health, vbat, vbus, ua, real_type;
	int ret;

	if (!battmgr->pps_ops && qcom_battmgr_serial_buck(battmgr, false))
		return PD_POLL_MS;
	if (test_and_clear_bit(PD_EVENT_SERVICE_RESET, &battmgr->pd_events)) {
		battmgr->pd_restore_fcc = false;
		battmgr->aicl_done = false;
		qcom_battmgr_buck_invalidate(battmgr);
		battmgr->buck.next_sample = jiffies;
		/* Not FFC on a firmware that has just restarted under it. */
		if (battmgr->ffc.active && !battmgr->ffc.ending) {
			dev_warn(battmgr->dev, "gaugeless FFC: charger firmware restarted\n");
			battmgr->ffc.fault = true;
			battmgr->ffc.ending = "charger firmware restarted";
		}
	}
	if (test_and_clear_bit(PD_EVENT_RESET, &battmgr->pd_events)) {
		qcom_battmgr_buck_invalidate(battmgr);
		if (qcom_battmgr_pps_stop(battmgr, false, "PD-reset") ||
		    qcom_battmgr_pd_stop(battmgr, false, "PD reset"))
			return PD_POLL_MS;
		battmgr->pd_state = PD_IDLE;
		battmgr->pd_logged = false;
		set_bit(PD_EVENT_CAPS, &battmgr->pd_events);
	}

	/* Consume the detach before the property transaction. A removal
	 * during it invalidates both values and is consumed on the next pass.
	 */
read_source:
	detached = test_and_clear_bit(PD_EVENT_DETACH, &battmgr->pd_events);
	if (qcom_battmgr_get_property(battmgr, BATTMGR_USB_PROPERTY_GET,
				      USB_ONLINE, &online)) {
		if (detached)
			set_bit(PD_EVENT_DETACH, &battmgr->pd_events);
		qcom_battmgr_pps_stop(battmgr, false, "USB_ONLINE-read-failed");
		return PD_POLL_MS;
	}
	if (online && qcom_battmgr_get_property(battmgr, BATTMGR_USB_PROPERTY_GET,
					       USB_ADAP_TYPE, &type))
		type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
	real_type = type;
	if (online && battmgr->pps_ops && battmgr->pps_ops->enabled() &&
	    qcom_battmgr_get_property(battmgr, BATTMGR_USB_PROPERTY_GET,
				      USB_REAL_TYPE, &real_type)) {
		/* Classification is optional. The power contract has its own readback. */
		real_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
	}
	/* Process the PHY evidence before the wake and routing decisions.
	 * Do not transfer evidence across a replacement Type-C partner.
	 */
	status = (struct erhai_probe_status){ .wired_online = online && !detached };
	if (online && !detached && battmgr->pps_ops)
		battmgr->pps_ops->probe_status(battmgr->pps_data, &status);
	/* Without the pack gauge the AP drives PD PPS only: the serial and
	 * UFCS routes are never ready and no answer of a source counts as
	 * evidence for them.
	 */
	if (gaugeless) {
		status.ready &= ERHAI_PPS;
		status.responded &= ERHAI_PPS;
		status.delivering &= ERHAI_PPS;
		status.first_ufcs_timeout = false;
		status.serial_needs_buck_off = false;
	}
	scoped_guard(spinlock_irqsave, &battmgr->notify_lock) {
		if (test_bit(PD_EVENT_DETACH, &battmgr->pd_events))
			goto read_source;
		if (online && !detached && !gaugeless) {
			battmgr->retry_evidence |= (status.responded | status.delivering) & ERHAI_SERIAL;
			if (test_and_clear_bit(PD_EVENT_SVOOC_RESPONSE, &battmgr->pd_events) ||
			    (battmgr->pps_ops && battmgr->pps_ops->enabled() && real_type == 9))
				battmgr->retry_evidence |= ERHAI_SERIAL;
		}
		serial_source = !detached && (battmgr->retry_evidence & ERHAI_SERIAL);
		battmgr->buck_online = online && !READ_ONCE(battmgr->otg_request);
		qcom_battmgr_charge_wake_locked(battmgr,
			erhai_should_stay_awake(online == 1, battmgr->otg_request,
						serial_source || (battmgr->buck.phase >= ERHAI_FFC_WAIT &&
				battmgr->buck.phase <= ERHAI_FFC_IDLE) ? POWER_SUPPLY_USB_TYPE_DCP : type));
	}
	if (detached || !online) {
		bool keep_tail;
		int err;

		qcom_battmgr_buck_invalidate(battmgr);
		ret = qcom_battmgr_pps_stop(battmgr, false, "detach-or-offline");
		if (!online && battmgr->buck.phase != ERHAI_BUCK_OFF)
			ret = qcom_battmgr_buck_pause(battmgr) ?: ret;
		/* Check after requesting PHY stop: its last sample may have
		 * announced CV while the removal notification was in flight.
		 */
		keep_tail = qcom_battmgr_keep_serial_tail(battmgr);
		if (!keep_tail) {
			err = qcom_battmgr_buck_release(battmgr);
			if (err) {
				dev_warn_ratelimited(battmgr->dev, "detach: buck release failed: %d\n", err);
				set_bit(PD_EVENT_DETACH, &battmgr->pd_events);
				return PD_POLL_MS;
			}
			if (battmgr->pps_ops)
				battmgr->pps_ops->detached(battmgr->pps_data);
			test_and_clear_bit(PD_EVENT_TAIL, &battmgr->pd_events);
			battmgr->tail_owned = false;
			WRITE_ONCE(battmgr->serial_tail_check_at, 0);
			WRITE_ONCE(battmgr->tail_route, 0);
		}
		battmgr->recharge_samples = 0;
		battmgr->ffc.fault = false;
		battmgr->stable_seen = false;
		battmgr->missing_routes = battmgr->retry_pending = 0;
		battmgr->retry_stopping = false;
		battmgr->retry_once = 0;
		battmgr->retry_evidence = keep_tail ? ERHAI_SERIAL : 0;
		serial_source = keep_tail;
		battmgr->serial_retry_state = (struct erhai_route_retry){};
		battmgr->ufcs_retry_state = (struct erhai_route_retry){};
		battmgr->pps_retry_state = (struct erhai_route_retry){};
		battmgr->fixed_retry_state = (struct erhai_route_retry){};
		battmgr->restore_route = battmgr->suspend_route = 0;
		test_and_clear_bit(PD_EVENT_SVOOC_RESPONSE, &battmgr->pd_events);
		test_and_clear_bit(PD_EVENT_UFCS_RESPONSE, &battmgr->pd_events);
		battmgr->serial_failures = battmgr->ufcs_failures = 0;
		battmgr->serial_backoff = false;
		battmgr->serial_blocked = false;
		battmgr->serial_probe = false;
		battmgr->aicl_done = false;
		battmgr->ufcs_backoff = false;
		battmgr->qc_probed = false;
		battmgr->fixed_backoff = false;
		battmgr->pd_partner_checked = false;
		battmgr->pd_charge_only = false;
		battmgr->pd_data_partner = false;
		if (ret)
			dev_warn_ratelimited(battmgr->dev, "detach: charge route stop failed: %d\n", ret);
		if (ret || (qcom_battmgr_pd_stop(battmgr, !online, "charger removed") &&
			    (battmgr->pd_restore_fcc || battmgr->pd_restore))) {
			set_bit(PD_EVENT_DETACH, &battmgr->pd_events);
			return PD_POLL_MS;
		}
		err = qcom_battmgr_serial_icl(battmgr, false);
		if (err) {
			dev_warn_ratelimited(battmgr->dev, "detach: input limit release failed: %d\n", err);
			set_bit(PD_EVENT_DETACH, &battmgr->pd_events);
			return PD_POLL_MS;
		}
		battmgr->pd_state = PD_IDLE;
		battmgr->pd_logged = false;
		set_bit(PD_EVENT_CAPS, &battmgr->pd_events);
		if (!online)
			return keep_tail ? PD_POLL_MS : 0;
	}

	power_supply_for_each_psy(&src, qcom_battmgr_pd_source_match);
	if (type != battmgr->charge_detected_type)
		battmgr->aicl_done = false;
	if (!serial_source && type != battmgr->charge_detected_type) {
		battmgr->serial_failures = battmgr->ufcs_failures = 0;
		battmgr->serial_backoff = battmgr->ufcs_backoff = false;
	}
	battmgr->charge_detected_type = type;
	if (!src.sink) {
		battmgr->pd_charge_only = false;
		battmgr->pd_data_partner = false;
		battmgr->pd_partner_checked = false;
	} else if (test_and_clear_bit(PD_EVENT_PARTNER, &battmgr->pd_events) ||
		   !battmgr->pd_partner_checked ||
		   (!battmgr->pd_charge_only && !battmgr->pd_data_partner &&
		    !qcom_battmgr_pps_running(battmgr) &&
		    time_after_eq(jiffies, battmgr->pd_classified_at + msecs_to_jiffies(CHARGE_CTRL_POLL_MS)))) {
		int charge_only = qcom_battmgr_pd_charge_only(battmgr);
		bool data_partner = charge_only == 0;

		/* A charger first read as unknown gets its request now. */
		if (charge_only > 0 && !battmgr->pd_charge_only)
			set_bit(PD_EVENT_CAPS, &battmgr->pd_events);
		/* Every capability read is traffic on the cable. A data partner
		 * (monitor, dock) is read once and then left alone, so that its
		 * DisplayPort or other mode entry is not interrupted.
		 */
		if (data_partner && !battmgr->pd_data_partner)
			dev_info(battmgr->dev, "PD: data partner, keeping the default contract\n");
		battmgr->pd_charge_only = charge_only > 0;
		battmgr->pd_data_partner = data_partner;
		battmgr->pd_partner_checked = true;
		battmgr->pd_classified_at = jiffies;
	}
	qc = (type >= 0x80 && type <= 0x82) || (real_type >= 0x80 && real_type <= 0x82);
	dedicated = src.sink ? battmgr->pd_charge_only :
		    (qc || type == POWER_SUPPLY_USB_TYPE_DCP ||
		     type == OPLUS_USB_TYPE_APPLE_BRICK_ID ||
		     ((type == POWER_SUPPLY_USB_TYPE_UNKNOWN || type == POWER_SUPPLY_USB_TYPE_C) &&
		      qcom_battmgr_usb_idle(battmgr)));
	tail = qcom_battmgr_tail_owned(battmgr);
	dedicated |= serial_source;
	can_probe = dedicated && battmgr->buck.phase != ERHAI_BUCK_FULL &&
		    !battmgr->buck.disable && !READ_ONCE(battmgr->pps_suspended) &&
		    !READ_ONCE(battmgr->otg_request) && !battmgr->charge_inhibited &&
		    !battmgr->charge_policy_hold && !battmgr->charge_error;
	can_probe_pumps = can_probe && !status.connector_blocked;
	qcom_battmgr_note_delivery(battmgr, status.delivering);
	if (status.delivering == ERHAI_SERIAL) {
		battmgr->serial_failures = 0;
		battmgr->serial_backoff = false;
	} else if (status.delivering == ERHAI_UFCS) {
		battmgr->ufcs_failures = 0;
		battmgr->ufcs_backoff = false;
	}
	if (status.delivering && status.delivering == battmgr->restore_route)
		battmgr->restore_route = 0;
	ufcs_active = battmgr->pps_ops && battmgr->pps_ops->ufcs_active(battmgr->pps_data);
	if (dedicated)
		battmgr->missing_routes |= (ERHAI_SERIAL | ERHAI_UFCS) &
					  (status.connector_blocked ? ~0U : ~status.ready);
	if (!status.connector_blocked) {
		battmgr->retry_once |= battmgr->missing_routes & status.ready;
		battmgr->missing_routes &= ~status.ready;
	}
	if (test_and_clear_bit(PD_EVENT_UFCS_RESPONSE, &battmgr->pd_events) && !gaugeless)
		battmgr->retry_evidence |= ERHAI_UFCS;
	stable = battmgr->pd_state == PD_ACTIVE || status.delivering;
	if (stable && !battmgr->stable_seen) {
		battmgr->stable_seen = true;
		battmgr->stable_since = jiffies;
	}
	if (!can_probe_pumps || tail) {
		battmgr->retry_pending = 0;
		battmgr->retry_stopping = false;
	} else if (stable && !battmgr->retry_pending && !battmgr->serial_probe) {
		battmgr->retry_pending = qcom_battmgr_retry_routes(battmgr, &status, ufcs_active);
		if (battmgr->retry_pending) {
			if (ufcs_active)
				battmgr->retry_pending |= ERHAI_UFCS;
			battmgr->retry_stopping = true;
			battmgr->retry_deadline = jiffies + msecs_to_jiffies(PD_CONFIG_TIMEOUT_MS);
			set_bit(PD_EVENT_CAPS, &battmgr->pd_events);
		}
	}
	if (battmgr->retry_stopping) {
		ret = qcom_battmgr_pps_stop(battmgr, true, "upward-retry");
		ret = ret ?: qcom_battmgr_pd_stop(battmgr, false, "qualified protocol retry");
		if (ret && time_before(jiffies, battmgr->retry_deadline))
			return PD_POLL_MS;
		battmgr->retry_stopping = false;
		if (ret)
			battmgr->retry_pending = 0;
	}
	probe_idle = !tail && can_probe_pumps && !battmgr->pd_restore && !battmgr->pd_restore_fcc &&
		     !qcom_battmgr_pps_running(battmgr) &&
		     !qcom_battmgr_get_property(battmgr, BATTMGR_USB_PROPERTY_GET, USB_VOLT_NOW, &vbus) &&
		     vbus >= 4000000 && vbus <= 5500000;
	/* Losing an ACK or a slow discharge is not another upward attempt.
	 * Allow the existing PD transition deadline for 5 V readback before
	 * taking D+/D-. On expiry resume normal fallback arbitration.
	 */
	if (battmgr->retry_pending && !battmgr->serial_probe && !probe_idle &&
	    time_before(jiffies, battmgr->retry_deadline))
		return PD_POLL_MS;
	if (battmgr->pps_ops) {
		/* A probe gets exclusive D+/D- ownership only on a dedicated
		 * source at 5 V. Its decline/timeout releases the crossbar
		 * before the next protocol (including firmware QC) can act.
		 */
		if ((status.ready & ERHAI_SERIAL) &&
		    (!battmgr->retry_pending || (battmgr->retry_pending & ERHAI_SERIAL)) &&
		    (probe_idle || (!tail && can_probe_pumps && battmgr->serial_probe)) && !battmgr->serial_blocked &&
		    ((battmgr->retry_pending & ERHAI_SERIAL) || !battmgr->serial_backoff || time_after_eq(jiffies, battmgr->serial_retry))) {
			battmgr->retry_once &= ~ERHAI_SERIAL;
			battmgr->restore_route &= ~ERHAI_SERIAL;
			ret = battmgr->pps_ops->serial_session(true);
			battmgr->serial_blocked = ret == -EPERM;
			if (!ret)
				ret = qcom_battmgr_serial_buck(battmgr, status.serial_needs_buck_off);
			if (!ret && status.serial_needs_buck_off && !status.serial_buck_ready)
				battmgr->pps_ops->serial_buck_sample(erhai_usb_readback(battmgr, USB_CURR_NOW));
			if (!ret) {
				battmgr->serial_probe = true;
				battmgr->aicl_done = false;
				return PD_POLL_MS;
			}
			dev_info(battmgr->dev, "serial setup failed: %d\n", ret);
			/* Keep the reservation until the entire retry ladder is done. */
			battmgr->serial_backoff = true;
			battmgr->serial_retry = qcom_battmgr_probe_retry(&battmgr->serial_failures);
			set_bit(PD_EVENT_CAPS, &battmgr->pd_events);
		}
		if (qcom_battmgr_serial_stop(battmgr, "serial-ineligible"))
			return PD_POLL_MS;
		battmgr->serial_probe = false;
	}
	if (IS_REACHABLE(CONFIG_OPLUS_UFCS_CLASS) && can_probe_pumps && battmgr->pps_ops &&
	    (battmgr->pps_ops->ufcs_active(battmgr->pps_data) ||
	     (probe_idle && (status.ready & ERHAI_UFCS) && !(status.protected & ERHAI_UFCS) &&
	      (!battmgr->retry_pending || (battmgr->retry_pending & ERHAI_UFCS)) &&
	      ((battmgr->retry_pending & ERHAI_UFCS) || !battmgr->ufcs_backoff ||
	       time_after_eq(jiffies, battmgr->ufcs_retry))))) {
		unsigned int delay;

		battmgr->retry_pending = 0;
		battmgr->retry_once &= ~ERHAI_UFCS;
		battmgr->restore_route &= ~ERHAI_UFCS;
		if (qcom_battmgr_serial_icl(battmgr, false))
			return PD_POLL_MS;
		delay = battmgr->pps_ops->ufcs_update(battmgr->pps_data);
		if (battmgr->pps_ops->ufcs_active(battmgr->pps_data))
			return delay ?: PD_POLL_MS;
		battmgr->ufcs_backoff = true;
		battmgr->ufcs_retry = qcom_battmgr_probe_retry(&battmgr->ufcs_failures);
		set_bit(PD_EVENT_CAPS, &battmgr->pd_events);
	}
	if (battmgr->pps_ops && battmgr->pps_ops->ufcs_active(battmgr->pps_data) &&
	    qcom_battmgr_pps_stop(battmgr, false, "UFCS-exit"))
		return PD_POLL_MS;
	/* A failed 5 V readback or a disappearing PHY cancels this reservation. */
	battmgr->retry_pending = 0;
	/* The Type-C partner's PD capabilities are stronger evidence than
	 * an incomplete BC1.2 classification. PPS also checks its APDO.
	 */
	prefer_pps = !status.connector_blocked && !tail && !battmgr->buck.disable &&
		     battmgr->buck.phase != ERHAI_BUCK_FULL &&
		     !test_bit(PD_EVENT_TAIL, &battmgr->pd_events) && !READ_ONCE(battmgr->pps_suspended) &&
		     battmgr->pps_ops && battmgr->pps_ops->enabled() &&
		     src.sink && !READ_ONCE(battmgr->otg_request);
	if (prefer_pps && !battmgr->charge_inhibited && !battmgr->charge_policy_hold &&
	    !battmgr->charge_error && battmgr->pps_ops->available(battmgr->pps_data)) {
		unsigned int delay;

		/* A live APDO is affirmative evidence. Budget retries that
		 * would break working fixed power; on idle 5 V the PPS
		 * engine handles recoverable errors with its own holdoff.
		 */
		if (battmgr->pd_state == PD_WAIT)
			goto fixed_contract;
		if (battmgr->pd_state == PD_ACTIVE &&
		    (!(status.ready & ERHAI_PPS) ||
		     !qcom_battmgr_retry_slot(&battmgr->pps_retry_state, battmgr->stable_since,
					     battmgr->restore_route == ERHAI_PPS)))
			goto fixed_contract;
		if (qcom_battmgr_serial_icl(battmgr, false) ||
		    qcom_battmgr_pd_stop(battmgr, false, "PPS handover"))
			return PD_POLL_MS;
		battmgr->pd_state = PD_IDLE;
		battmgr->restore_route &= ~ERHAI_PPS;
		delay = battmgr->pps_ops->update(battmgr->pps_data,
				test_and_clear_bit(PD_EVENT_CAPS, &battmgr->pd_events));
		if (qcom_battmgr_pps_running(battmgr))
			return delay;
		/* Cleanup completed. Re-read a fixed PDO in this same poll. */
		set_bit(PD_EVENT_CAPS, &battmgr->pd_events);
	}
	if (qcom_battmgr_pps_running(battmgr)) {
		if (qcom_battmgr_pps_stop(battmgr, false, "PPS-exit"))
			return PD_POLL_MS;
		set_bit(PD_EVENT_CAPS, &battmgr->pd_events);
	}

fixed_contract:
	/* A suspend vote can persist across a cable removal in this firmware. */
	if (battmgr->pps_input_used) {
		ret = qcom_battmgr_set_property(battmgr, BATTMGR_USB_PROPERTY_SET,
						USB_INPUT_CURR_LIMIT, U32_MAX);
		if (ret)
			return PD_POLL_MS;
		battmgr->pps_input_used = false;
	}

	/* Charge-only PD sources may also implement HVDCP. Try their
	 * fixed PDO first; after a refusal or an absent 9 V PDO let QC
	 * negotiate, and revisit the fixed PDO after its normal timeout.
	 */
	if (src.sink && dedicated && qc && battmgr->qc_contract && battmgr->pd_restore) {
		/* A fixed 9 V PDO can appear while QC is delivering. Use
		 * read-only PDO evidence before spending a disruption slot.
		 */
		if (!battmgr->fixed_backoff)
			battmgr->fixed_retry = jiffies;
		battmgr->fixed_backoff = true;
		if (battmgr->pd_state == PD_ACTIVE &&
		    time_after_eq(jiffies, battmgr->fixed_retry) &&
		    !qcom_battmgr_pd_current(battmgr, PD_FIXED_MV, &ua) && ua &&
		    qcom_battmgr_retry_slot(&battmgr->fixed_retry_state, battmgr->stable_since, false)) {
			battmgr->fixed_backoff = false;
			set_bit(PD_EVENT_CAPS, &battmgr->pd_events);
		}
	} else if (battmgr->fixed_backoff && time_after_eq(jiffies, battmgr->fixed_retry)) {
		battmgr->fixed_backoff = false;
		set_bit(PD_EVENT_CAPS, &battmgr->pd_events);
	}
	if (src.sink && dedicated && !battmgr->fixed_backoff && battmgr->pd_state != PD_ACTIVE &&
	    battmgr->pd_state != PD_WAIT && !qcom_battmgr_pd_current(battmgr, PD_FIXED_MV, &ua) && !ua) {
		battmgr->fixed_backoff = true;
		battmgr->fixed_retry = jiffies + msecs_to_jiffies(PD_CONFIG_TIMEOUT_MS);
	}
	try_qc = !src.sink || (dedicated && battmgr->fixed_backoff);
	qc = qc && try_qc;
	/* A DCP classification can precede HVDCP detection. Ask firmware
	 * to negotiate identification (BATT_SET_QC=0), never raise VBUS
	 * without its positive HVDCP response. Data ports keep D+/D-.
	 */
	if (can_probe && try_qc && !qc && !battmgr->pd_restore && (!battmgr->qc_probed ||
	    time_after_eq(jiffies, battmgr->qc_probe_at + msecs_to_jiffies(CHARGE_CTRL_POLL_MS)))) {
		battmgr->qc_probed = true;
		battmgr->qc_probe_at = jiffies;
		if (!qcom_battmgr_set_property(battmgr, BATTMGR_BAT_PROPERTY_SET,
					       BATT_OPLUS_SET_QC, 0)) {
			set_bit(PD_EVENT_CAPS, &battmgr->pd_events);
			return PD_POLL_MS;
		}
	}
	if (READ_ONCE(battmgr->otg_request) && qcom_battmgr_serial_icl(battmgr, false))
		return PD_POLL_MS;
	if ((try_qc && !qc) || READ_ONCE(battmgr->otg_request)) {
		if (qcom_battmgr_pd_stop(battmgr, false, "no high-voltage sink"))
			return PD_POLL_MS;
		qcom_battmgr_pd_log(battmgr, type);
		/* Stock ADSP implements USB_SET_RERUN_AICL. Let its PMIC
		 * regulate a weak port within the negotiated Type-C/BC limit.
		 */
		if (serial_source && !READ_ONCE(battmgr->otg_request)) {
			if (qcom_battmgr_serial_icl(battmgr, true))
				return PD_POLL_MS;
		} else if (!battmgr->aicl_done && !READ_ONCE(battmgr->otg_request) &&
		    !qcom_battmgr_set_property(battmgr, BATTMGR_USB_PROPERTY_SET,
					       USB_OPLUS_RERUN_AICL, 0))
			battmgr->aicl_done = true;
		return CHARGE_CTRL_POLL_MS;
	}
	if (!battmgr->serial_icl_vote)
		battmgr->aicl_done = false;
	if (battmgr->qc_contract != qc) {
		if (qcom_battmgr_pd_stop(battmgr, false, "source protocol changed"))
			return PD_POLL_MS;
		battmgr->pd_state = PD_IDLE;
		battmgr->qc_contract = qc;
		set_bit(PD_EVENT_CAPS, &battmgr->pd_events);
	}

	if (battmgr->pd_state == PD_FAILED) {
		if (battmgr->pd_restore_fcc || battmgr->pd_restore) {
			if (qcom_battmgr_pd_stop(battmgr, false, "pending restore"))
				return PD_POLL_MS;
			battmgr->pd_state = PD_FAILED;
		}
		if (time_before(jiffies, battmgr->pd_since + msecs_to_jiffies(PD_CONFIG_TIMEOUT_MS)))
			return PD_POLL_MS;
		battmgr->pd_state = PD_IDLE;
		set_bit(PD_EVENT_CAPS, &battmgr->pd_events);
	}

	if (qcom_battmgr_get_property(battmgr, BATTMGR_BAT_PROPERTY_GET, BATT_HEALTH, &health) ||
	    qcom_battmgr_get_property(battmgr, BATTMGR_BAT_PROPERTY_GET, BATT_VOLT_NOW, &vbat))
		return CHARGE_CTRL_POLL_MS;
	/*
	 * Bypass charging keeps the 9 V contract and its input limit: with
	 * charging off the charger still supplies the system, so the tablet
	 * draws what it needs from the source rather than from the battery.
	 */
	bypass = READ_ONCE(battmgr->charge_bypass) && !battmgr->charge_error;
	if ((!bypass && (battmgr->charge_inhibited || battmgr->charge_policy_hold ||
			 vbat >= (battmgr->pd_restore ? PD_VBAT_STOP_UV : PD_VBAT_START_UV))) ||
	    battmgr->charge_error ||
	    (health != POWER_SUPPLY_HEALTH_GOOD && health != POWER_SUPPLY_HEALTH_COOL &&
	     health != POWER_SUPPLY_HEALTH_WARM)) {
		qcom_battmgr_pd_stop(battmgr, false, "battery limit");
		set_bit(PD_EVENT_CAPS, &battmgr->pd_events);
		qcom_battmgr_pd_log(battmgr, type);
		return CHARGE_CTRL_POLL_MS;
	}

	if (src.sink && !qc && battmgr->pd_data_partner && battmgr->pd_state == PD_IDLE)
		return CHARGE_CTRL_POLL_MS;

	caps_changed = test_and_clear_bit(PD_EVENT_CAPS, &battmgr->pd_events);
	if (caps_changed) {
		if (qc) {
			/* PMIC maximum for the detected QC source, in microamps. */
			ret = qcom_battmgr_get_property(battmgr, BATTMGR_USB_PROPERTY_GET,
							USB_CURR_MAX, &ua);
			ua = ret ? 0 : min(ua, (u32)PD_INPUT_MAX_UA);
		} else {
			ret = qcom_battmgr_pd_current(battmgr, PD_FIXED_MV, &ua);
		}
		/*
		 * Without the gauge the firmware alone limits what the buck
		 * passes on to the cell: do not offer it more than stock does.
		 */
		if (qcom_battmgr_gaugeless(battmgr))
			ua = min(ua, (u32)PD_INPUT_STOCK_UA);
		if (ret || !ua) {
			dev_info(battmgr->dev, "PD: no verified 9 V fixed PDO (error %d)\n", ret);
			if (qcom_battmgr_pd_stop(battmgr, false, "source capabilities") ||
			    (serial_source && qcom_battmgr_serial_icl(battmgr, true)))
				return PD_POLL_MS;
			qcom_battmgr_pd_log(battmgr, type);
			set_bit(PD_EVENT_CAPS, &battmgr->pd_events);
			return CHARGE_CTRL_POLL_MS;
		}
		current_changed = ua != battmgr->pd_input_limit_ua;
		battmgr->pd_input_limit_ua = ua;
	}

	switch (battmgr->pd_state) {
	case PD_IDLE:
		if (qcom_battmgr_serial_icl(battmgr, false))
			return PD_POLL_MS;
		if (!caps_changed)
			return CHARGE_CTRL_POLL_MS;
		/* A PD source that also carries data (monitor, dock, hub, host),
		 * or one whose capabilities are unknown yet, keeps its default
		 * contract: renegotiating VBUS resets such a partner before it
		 * can enter DisplayPort or other modes.
		 */
		if (!qc && !dedicated)
			return CHARGE_CTRL_POLL_MS;
		ret = qcom_battmgr_get_property(battmgr, BATTMGR_USB_PROPERTY_GET,
						USB_INPUT_CURR_LIMIT, &battmgr->pd_saved_limit_ua);
		if (ret || !battmgr->pd_saved_limit_ua) {
			qcom_battmgr_pd_log(battmgr, type);
			set_bit(PD_EVENT_CAPS, &battmgr->pd_events);
			return CHARGE_CTRL_POLL_MS;
		}
		/* Also undo a timed-out write, since the firmware may have applied it. */
		battmgr->pd_restore = true;
		ret = qcom_battmgr_set_property(battmgr, BATTMGR_USB_PROPERTY_SET,
						USB_INPUT_CURR_LIMIT,
						min(battmgr->pd_input_limit_ua, PD_TRANSITION_UA));
		if (!ret)
			ret = qcom_battmgr_set_property(battmgr, BATTMGR_BAT_PROPERTY_SET,
							qc ? BATT_OPLUS_SET_QC : BATT_OPLUS_SET_PDO,
							PD_FIXED_MV);
		if (ret)
			goto failed;
		battmgr->pd_state = PD_WAIT;
		battmgr->pd_since = jiffies;
		battmgr->pd_logged = false;
		dev_info(battmgr->dev, "%s: requested 9000 mV, input limit up to %u mA\n",
			 qc ? "QC" : "PD", battmgr->pd_input_limit_ua / 1000);
		return qc ? 350 : PD_POLL_MS; /* oplus_chg_8350_set_qc_config */
	case PD_WAIT:
	case PD_ACTIVE:
		ret = qcom_battmgr_get_property(battmgr, BATTMGR_USB_PROPERTY_GET,
						USB_VOLT_NOW, &vbus);
		if (ret)
			goto failed;
		if (vbus < (qc ? 8000000 : PD_VBUS_RISEN_UV)) {
			if (battmgr->pd_state == PD_WAIT &&
			    time_before(jiffies, battmgr->pd_since +
					msecs_to_jiffies(qc ? 350 : PD_CONFIG_TIMEOUT_MS)))
				return qc ? 350 : PD_POLL_MS;
			ret = -ETIMEDOUT;
			goto failed;
		}
		if (vbus >= PD_VBUS_MAX_UV) {
			ret = -ERANGE;
			goto failed;
		}
		if (battmgr->pd_state == PD_WAIT || current_changed || READ_ONCE(battmgr->pps_suspended)) {
			ret = qcom_battmgr_set_property(battmgr, BATTMGR_USB_PROPERTY_SET,
							USB_INPUT_CURR_LIMIT,
							battmgr->pd_input_limit_ua);
			if (ret)
				goto failed;
		}
		ret = qcom_battmgr_pd_fcc(battmgr);
		if (ret)
			goto failed;
		if (!battmgr->stable_seen) {
			battmgr->stable_seen = true;
			battmgr->stable_since = jiffies;
		}
		if (battmgr->pd_state == PD_WAIT)
			battmgr->fixed_stable_since = jiffies;
		if (!qc)
			qcom_battmgr_route_delivered(&battmgr->fixed_retry_state);
		battmgr->pd_state = PD_ACTIVE;
		qcom_battmgr_pd_log(battmgr, type);
		return CHARGE_CTRL_POLL_MS;
	default:
		return CHARGE_CTRL_POLL_MS;
	}

failed:
	dev_warn(battmgr->dev, "PD: 9 V request failed: %d\n", ret);
	qcom_battmgr_pd_stop(battmgr, false, "request failed");
	battmgr->pd_since = jiffies;
	if (!qc && dedicated) {
		battmgr->fixed_backoff = true;
		battmgr->fixed_retry = jiffies + msecs_to_jiffies(PD_CONFIG_TIMEOUT_MS);
	}
	battmgr->pd_state = PD_FAILED;
	qcom_battmgr_pd_log(battmgr, type);
	return PD_POLL_MS;
}

/* Firmware owns the converter while asleep. Its FCC is distinct from the
 * fixed PDO and from the input ceiling. Capture both sides at handoff. */
static void qcom_battmgr_log_handoff(struct qcom_battmgr *b, const char *phase)
{
	u32 fcc, icl, uv, ua, counter;

	if (qcom_battmgr_get_property(b, BATTMGR_BAT_PROPERTY_GET, BATT_CHG_CTRL_LIM, &fcc) ||
	    qcom_battmgr_get_property(b, BATTMGR_USB_PROPERTY_GET, USB_INPUT_CURR_LIMIT, &icl) ||
	    qcom_battmgr_get_property(b, BATTMGR_USB_PROPERTY_GET, USB_VOLT_NOW, &uv) ||
	    qcom_battmgr_get_property(b, BATTMGR_USB_PROPERTY_GET, USB_CURR_NOW, &ua) ||
	    qcom_battmgr_get_property(b, BATTMGR_BAT_PROPERTY_GET, BATT_CHG_COUNTER, &counter))
		return;
	dev_info(b->dev, "fixed %s %s: input %u uV %u uA ICL %u uA FCC %u uA charge %u uAh\n",
		 b->qc_contract ? "QC" : "PD", phase, uv, ua, icl, fcc, counter);
}

/* Finish the existing fixed-PD transition before the charge worker freezes. */
static void qcom_battmgr_pd_suspend(struct qcom_battmgr *battmgr)
{
	struct qcom_battmgr_pd_source src = { .battmgr = battmgr };
	unsigned long deadline = jiffies + msecs_to_jiffies(PD_CONFIG_TIMEOUT_MS);
	u32 ua, vbus;
	int ret;

	if (!battmgr->service_up || !(battmgr->quirks & QCOM_BATTMGR_QUIRK_OPLUS_PD))
		return;

	qcom_battmgr_charge_limit_update(battmgr);
	set_bit(PD_EVENT_CAPS, &battmgr->pd_events);
	qcom_battmgr_pd_update(battmgr);
	while (battmgr->pd_state == PD_WAIT && time_before(jiffies, deadline)) {
		msleep(PD_POLL_MS);
		qcom_battmgr_pd_update(battmgr);
	}
	if (battmgr->pd_state == PD_ACTIVE) {
		qcom_battmgr_log_handoff(battmgr, "suspend");
		return;
	}
	if (battmgr->pd_state == PD_WAIT)
		qcom_battmgr_pd_stop(battmgr, false, "suspend timeout");

	/*
	 * PPS stop and a failed 9 V transition both request fixed 5 V. Only
	 * raise its input limit after cleanup succeeded and VBUS is at 5 V.
	 * Missing capabilities or a failed read leave the firmware fallback.
	 */
	if (battmgr->pd_restore || battmgr->pd_restore_fcc || READ_ONCE(battmgr->otg_request))
		return;
	power_supply_for_each_psy(&src, qcom_battmgr_pd_source_match);
	if (!src.sink || qcom_battmgr_pd_current(battmgr, PD_DEFAULT_MV, &ua) || !ua)
		return;
	if (qcom_battmgr_get_property(battmgr, BATTMGR_USB_PROPERTY_GET,
				      USB_VOLT_NOW, &vbus) ||
	    vbus < 4000000 || vbus > 5500000)
		return;

	ret = qcom_battmgr_set_property(battmgr, BATTMGR_USB_PROPERTY_SET,
					USB_INPUT_CURR_LIMIT, ua);
	if (ret)
		dev_warn(battmgr->dev, "PD: suspend 5 V input limit failed: %d\n", ret);
	else
		dev_info(battmgr->dev, "PD: suspend at 5 V, input limit %u mA\n", ua / 1000);
}

/*
 * The firmware cannot tell the AP that the pack is the stock one (the stock
 * system asks a 1-wire chip in the pack for that); what it does tell is the
 * battery profile it selected for the pack it found.
 */
static bool qcom_battmgr_gaugeless_pack(struct qcom_battmgr *b)
{
	int ret;

	mutex_lock(&b->lock);
	ret = qcom_battmgr_request_property(b, BATTMGR_BAT_PROPERTY_GET, BATT_MODEL_NAME, 0);
	mutex_unlock(&b->lock);
	return !ret && str_has_prefix(b->info.model_number, ERHAI_FW_PACK_PROFILE);
}

/*
 * Without the pack gauge the pumps are let in and taken out again by
 * gaugeless_pps_ma. Taking them out first ends a running session; until
 * that has completed the engine stays registered and is asked again.
 */
static void qcom_battmgr_gaugeless_pumps(struct qcom_battmgr *b)
{
	unsigned int ma = READ_ONCE(gaugeless_pps_ma);

	if (!qcom_battmgr_gaugeless(b))
		return;
	if (ma && b->pps_parked) {
		if (!qcom_battmgr_gaugeless_pack(b)) {
			dev_warn_once(b->dev, "gaugeless_pps_ma ignored: the firmware reports no %s battery profile\n",
				      ERHAI_FW_PACK_PROFILE);
			return;
		}
		b->pps_ops = b->pps_parked;
		b->pps_parked = NULL;
		dev_info(b->dev, "PD PPS fast charging on the firmware's battery readings: up to %u mA from the charger\n",
			 min(ma, 5000U));
	} else if (!ma && b->pps_ops) {
		if (qcom_battmgr_pps_stop(b, false, "gaugeless-pps-off"))
			return;
		b->pps_parked = b->pps_ops;
		b->pps_ops = NULL;
		dev_info(b->dev, "PD PPS fast charging off: the firmware charges by itself\n");
	} else {
		return;
	}
	set_bit(PD_EVENT_CAPS, &b->pd_events);
}

/* The stock percentage is what the battery reports (gaugeless_full). */
static bool qcom_battmgr_uisoc_shown(struct qcom_battmgr *b)
{
	return qcom_battmgr_gaugeless(b) && READ_ONCE(gaugeless_full) &&
	       READ_ONCE(b->uisoc.ready);
}

/*
 * The stock percentage on a board without the pack gauge (erhai_uisoc.h),
 * on its own freezable work: it runs with and without a charger, every
 * 10 s or when a step is due, and catches up after a suspend.
 */
static void qcom_battmgr_uisoc_worker(struct work_struct *work)
{
	struct qcom_battmgr *b = container_of(to_delayed_work(work), struct qcom_battmgr,
					      uisoc_work);
	struct erhai_uisoc_in in;
	unsigned long delay = 5 * HZ;
	u32 soc, uv, ua, temp, online, saved = 0;
	bool changed = false, init = false;
	int ui, ret;

	if (READ_ONCE(b->removing) || !qcom_battmgr_gaugeless(b))
		return;
	if (!READ_ONCE(gaugeless_full) || qcom_battmgr_stock_pack(b) != 1) {
		if (READ_ONCE(b->uisoc.ready)) {
			WRITE_ONCE(b->uisoc.ready, false);
			dev_info(b->dev, "percentage: the firmware's again\n");
			power_supply_changed(b->bat_psy);
		}
		delay = msecs_to_jiffies(CHARGE_CTRL_POLL_MS);
		goto out;
	}
	ret = qcom_battmgr_get_property(b, BATTMGR_BAT_PROPERTY_GET, BATT_CAPACITY, &soc);
	ret = ret ?: qcom_battmgr_get_property(b, BATTMGR_BAT_PROPERTY_GET, BATT_VOLT_NOW, &uv);
	ret = ret ?: qcom_battmgr_get_property(b, BATTMGR_BAT_PROPERTY_GET, BATT_CURR_NOW, &ua);
	ret = ret ?: qcom_battmgr_get_property(b, BATTMGR_BAT_PROPERTY_GET, BATT_TEMP, &temp);
	ret = ret ?: qcom_battmgr_get_property(b, BATTMGR_USB_PROPERTY_GET, USB_ONLINE, &online);
	if (ret || !uv || soc > 10000 || ua == BIT(31))
		goto out;	/* keep showing the last percentage */
	in = (struct erhai_uisoc_in){
		.soc = soc / 100, .vbat_mv = uv / 1000,
		/* The firmware reports discharge positive, as the stock ibat. */
		.ibat_ma = (s32)ua / 1000,
		.online = online && !READ_ONCE(b->otg_request),
		.mmi = !READ_ONCE(b->charge_policy_hold) && !READ_ONCE(b->charge_inhibited),
		.ap_full = READ_ONCE(b->buck.phase) == ERHAI_BUCK_FULL && READ_ONCE(b->buck_online),
		.fastchg = READ_ONCE(b->uisoc_fastchg),
		.warm = erhai_buck_region((s32)temp / 10) >= 7,
	};

	mutex_lock(&b->uisoc_lock);
	if (!b->uisoc.ready) {
		ret = qcom_battmgr_get_property(b, BATTMGR_BAT_PROPERTY_GET, BATT_OPLUS_RTC_SOC,
						&saved);
		erhai_uisoc_init(&b->uisoc, &in, ret ? -1 : (int)saved, jiffies);
		init = changed = true;
	} else {
		b->uisoc.sleep_s += atomic_long_xchg(&b->uisoc_sleep_s, 0);
		ui = b->uisoc.ui;
		delay = erhai_uisoc_update(&b->uisoc, &in, jiffies, &changed);
		changed |= ui != b->uisoc.ui;
	}
	ui = b->uisoc.ui;
	WRITE_ONCE(b->uisoc_fw_soc, in.soc);
	mutex_unlock(&b->uisoc_lock);

	if (init)
		dev_info(b->dev, "percentage: the stock one, %d%% (firmware %d%%, saved %d%%)\n",
			 ui, in.soc, ret ? -1 : (int)saved);
	if (!changed)
		goto out;
	dev_dbg(b->dev, "percentage %d%% full %d (firmware %d%%, smoothed %d%%)\n", ui,
		b->uisoc.full, in.soc, b->uisoc.smooth);
	/* oplus_back_ui_soc_work(): saved on every change. */
	ret = qcom_battmgr_set_property(b, BATTMGR_BAT_PROPERTY_SET, BATT_OPLUS_RTC_SOC, ui);
	if (ret && !b->uisoc_backup_failed)
		dev_warn(b->dev, "percentage: saving it for a reboot failed: %d\n", ret);
	b->uisoc_backup_failed = !!ret;
	power_supply_changed(b->bat_psy);
out:
	if (!READ_ONCE(b->removing))
		queue_delayed_work(system_freezable_wq, &b->uisoc_work, delay);
}

static void qcom_battmgr_charge_worker(struct work_struct *work)
{
	struct qcom_battmgr *battmgr = container_of(to_delayed_work(work),
						    struct qcom_battmgr, charge_work);
	unsigned int delay = 0, buck_delay, gauge_delay;

	/* Kicked again by qcom_battmgr_enable_worker() once the service is up */
	if (READ_ONCE(battmgr->removing) || READ_ONCE(battmgr->pps_suspended))
		return;

	mutex_lock(&battmgr->charge_lock);
	if (READ_ONCE(battmgr->pps_suspended)) {
		mutex_unlock(&battmgr->charge_lock);
		return;
	}
	if (!battmgr->service_up) {
		qcom_battmgr_pps_stop(battmgr, false, "service-down");
		mutex_unlock(&battmgr->charge_lock);
		return;
	}

	gauge_delay = qcom_battmgr_gauge_detect(battmgr);
	qcom_battmgr_gaugeless_pumps(battmgr);

	qcom_battmgr_charge_limit_update(battmgr);

	if (battmgr->quirks & QCOM_BATTMGR_QUIRK_OPLUS_PD)
		delay = qcom_battmgr_pd_update(battmgr);
	if (battmgr->quirks & QCOM_BATTMGR_QUIRK_OPLUS_PD) {
		buck_delay = qcom_battmgr_buck_tick(battmgr);
		if (buck_delay)
			delay = delay ? min(delay, buck_delay) : buck_delay;
	}
	if (delay && battmgr->tail_owned)
		delay = min(delay, 4000U);
	if (gauge_delay)
		delay = delay ? min(delay, gauge_delay) : gauge_delay;

	/* Also retry turning charging back on after the limit was removed */
	if (!delay && (qcom_battmgr_charge_limited(battmgr) ||
		       battmgr->charge_inhibited || battmgr->charge_error))
		delay = CHARGE_CTRL_POLL_MS;

	mutex_unlock(&battmgr->charge_lock);

	if (delay && !READ_ONCE(battmgr->removing) && !READ_ONCE(battmgr->pps_suspended))
		queue_delayed_work(system_freezable_wq, &battmgr->charge_work,
				   msecs_to_jiffies(delay));
}

/* Check the charge state now, may be called in atomic context */
static void qcom_battmgr_charge_kick(struct qcom_battmgr *battmgr)
{
	if ((battmgr->quirks & QCOM_BATTMGR_QUIRK_CHARGE_ENABLE) &&
	    !READ_ONCE(battmgr->removing))
		mod_delayed_work(system_freezable_wq, &battmgr->charge_work, 0);
}

void erhai_charge_recheck(void)
{
	guard(mutex)(&erhai_bridge_lock);
	if (erhai_battmgr)
		qcom_battmgr_charge_kick(erhai_battmgr);
}
EXPORT_SYMBOL_GPL(erhai_charge_recheck);

/* A serial IRQ has already sent the curve. If GLINK is busy the
 * existing serial-session poll will service its buck handoff.
 */
void erhai_charge_try_recheck(void)
{
	if (!mutex_trylock(&erhai_bridge_lock))
		return;
	if (erhai_battmgr)
		qcom_battmgr_charge_kick(erhai_battmgr);
	mutex_unlock(&erhai_bridge_lock);
}
EXPORT_SYMBOL_GPL(erhai_charge_try_recheck);

/*
 * A charger came or went: the firmware may have turned charging back on, so
 * check right away instead of at the next poll.  Called in atomic context.
 */
static void qcom_battmgr_usb_event(struct qcom_battmgr *battmgr)
{
	if (qcom_battmgr_charge_limited(battmgr) || battmgr->charge_inhibited || battmgr->charge_error ||
	    (battmgr->quirks & QCOM_BATTMGR_QUIRK_OPLUS_PD))
		qcom_battmgr_charge_kick(battmgr);
}

static int qcom_battmgr_psy_notify(struct notifier_block *nb,
				   unsigned long event, void *data)
{
	struct qcom_battmgr *battmgr = container_of(nb, struct qcom_battmgr, psy_nb);
	struct power_supply *psy = data;

	if (event == PSY_EVENT_PROP_CHANGED && qcom_battmgr_is_ucsi_supply(battmgr, psy)) {
		guard(spinlock_irqsave)(&battmgr->notify_lock);
		if (!battmgr->removing) {
			set_bit(PD_EVENT_CAPS, &battmgr->pd_events);
			qcom_battmgr_usb_event(battmgr);
		}
	}
	return NOTIFY_OK;
}

/* A menu suspend can bypass wakeup_count. Flush the existing charge
 * work before admitting sleep so a new attachment gets its ordinary PD
 * sample. A live charger can veto without delaying its pump watchdog.
 */
static bool qcom_battmgr_prepare_sleep(struct qcom_battmgr *b)
{
	scoped_guard(spinlock_irqsave, &b->notify_lock) {
		if (b->charge_awake && !test_bit(PD_EVENT_DETACH, &b->pd_events))
			return false;
	}

	flush_delayed_work(&b->charge_work);
	/* A second worker may already have consumed DETACH and still be in
	 * USB_ONLINE. Wait for that sample to commit before testing the lock.
	 */
	guard(mutex)(&b->charge_lock);
	guard(spinlock_irqsave)(&b->notify_lock);
	if (b->charge_awake ||
	    (b->buck_online && b->buck.phase >= ERHAI_FFC_WAIT &&
	     b->buck.phase <= ERHAI_FFC_IDLE) ||
	    (b->service_up && !b->removing && test_bit(PD_EVENT_DETACH, &b->pd_events)))
		return false;
	WRITE_ONCE(b->pps_suspended, true);
	return true;
}

/*
 * A PPS contract is not monitored while the system is suspended, so end it
 * before freezing the worker and hand over to autonomous fixed PD. The work
 * may start PPS again after resume.
 */
static int qcom_battmgr_pm_notify(struct notifier_block *nb,
				  unsigned long action, void *data)
{
	struct qcom_battmgr *battmgr = container_of(nb, struct qcom_battmgr, pm_nb);
	struct erhai_probe_status status = {};

	switch (action) {
	case PM_HIBERNATION_PREPARE:
	case PM_SUSPEND_PREPARE:
		battmgr->uisoc_suspend_at = ktime_get_boottime();
		if (!qcom_battmgr_prepare_sleep(battmgr))
			return NOTIFY_BAD;
		mutex_lock(&battmgr->charge_lock);
		if (battmgr->pps_ops)
			battmgr->pps_ops->probe_status(battmgr->pps_data, &status);
		battmgr->suspend_route = status.delivering;
		/* Nobody supervises the tail while asleep: hand it back. */
		if ((battmgr->ffc.active &&
		     qcom_battmgr_ffc_handback(battmgr, battmgr->ffc.ending ?: "suspend")) ||
		    qcom_battmgr_pps_stop(battmgr, false, "suspend")) {
			mutex_unlock(&battmgr->charge_lock);
			WRITE_ONCE(battmgr->pps_suspended, false);
			qcom_battmgr_charge_kick(battmgr);
			return NOTIFY_BAD;
		}
		qcom_battmgr_pd_suspend(battmgr);
		mutex_unlock(&battmgr->charge_lock);
		break;
	case PM_POST_HIBERNATION:
	case PM_POST_RESTORE:
	case PM_POST_SUSPEND:
		/* oplus_comm_pm_resume(): the time asleep, for the percentage. */
		if (battmgr->uisoc_suspend_at) {
			atomic_long_add(ktime_ms_delta(ktime_get_boottime(),
						       battmgr->uisoc_suspend_at) / 1000,
					&battmgr->uisoc_sleep_s);
			battmgr->uisoc_suspend_at = 0;
			mod_delayed_work(system_freezable_wq, &battmgr->uisoc_work, 0);
		}
		if (!READ_ONCE(battmgr->pps_suspended))
			break;
		scoped_guard(mutex, &battmgr->charge_lock) {
			qcom_battmgr_restore_route(battmgr, battmgr->suspend_route);
			battmgr->suspend_route = 0;
			if (battmgr->service_up && battmgr->pd_state == PD_ACTIVE)
				qcom_battmgr_log_handoff(battmgr, "resume");
		}
		WRITE_ONCE(battmgr->pps_suspended, false);
		qcom_battmgr_charge_kick(battmgr);
		break;
	}

	return NOTIFY_DONE;
}

static int qcom_battmgr_bat_is_writeable(struct power_supply *psy,
					 enum power_supply_property psp)
{
	struct qcom_battmgr *battmgr = power_supply_get_drvdata(psy);

	if (qcom_battmgr_no_charge_ctrl(battmgr))
		return 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD:
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		return 1;
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
		return !!(battmgr->quirks & QCOM_BATTMGR_QUIRK_CHARGE_ENABLE);
	default:
		return 0;
	}

	return 0;
}

/*
 * Bypass charging: with "inhibit-charge" the charge worker holds charging
 * off, so a connected charger powers the system and the battery is neither
 * charged nor, while the charger covers the load, discharged.  "auto" hands
 * charging back to the thresholds.
 */
static int qcom_battmgr_set_charge_bypass(struct qcom_battmgr *battmgr, int behaviour)
{
	if (!(battmgr->quirks & QCOM_BATTMGR_QUIRK_CHARGE_ENABLE))
		return -EOPNOTSUPP;

	switch (behaviour) {
	case POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO:
		guard(mutex)(&battmgr->charge_lock);
		if (battmgr->charge_bypass) {
			WRITE_ONCE(battmgr->charge_bypass, false);
			/* The thresholds decide again from the current charge */
			battmgr->charge_policy_hold = false;
			dev_info(battmgr->dev, "bypass charging off\n");
		}
		return 0;
	case POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE:
		WRITE_ONCE(battmgr->charge_bypass, true);
		/* Renegotiate a contract that stopped once the battery was full */
		set_bit(PD_EVENT_CAPS, &battmgr->pd_events);
		return 0;
	default:
		return -EINVAL;
	}
}

static int qcom_battmgr_bat_set_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 const union power_supply_propval *pval)
{
	struct qcom_battmgr *battmgr = power_supply_get_drvdata(psy);
	int ret;

	if (qcom_battmgr_no_charge_ctrl(battmgr))
		return -EOPNOTSUPP;

	if (!battmgr->service_up)
		return -EAGAIN;

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD:
		ret = qcom_battmgr_set_charge_start_threshold(battmgr, pval->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		ret = qcom_battmgr_set_charge_end_threshold(battmgr, pval->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
		ret = qcom_battmgr_set_charge_bypass(battmgr, pval->intval);
		break;
	default:
		return -EINVAL;
	}

	qcom_battmgr_charge_kick(battmgr);

	return ret;
}

static const enum power_supply_property sc8280xp_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_POWER_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_EMPTY,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN,
	POWER_SUPPLY_PROP_ENERGY_FULL,
	POWER_SUPPLY_PROP_ENERGY_EMPTY,
	POWER_SUPPLY_PROP_ENERGY_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_MANUFACTURE_YEAR,
	POWER_SUPPLY_PROP_MANUFACTURE_MONTH,
	POWER_SUPPLY_PROP_MANUFACTURE_DAY,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
};

static const struct power_supply_desc sc8280xp_bat_psy_desc = {
	.name = "qcom-battmgr-bat",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = sc8280xp_bat_props,
	.num_properties = ARRAY_SIZE(sc8280xp_bat_props),
	.get_property = qcom_battmgr_bat_get_property,
};

static const enum power_supply_property x1e80100_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_POWER_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_EMPTY,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN,
	POWER_SUPPLY_PROP_ENERGY_FULL,
	POWER_SUPPLY_PROP_ENERGY_EMPTY,
	POWER_SUPPLY_PROP_ENERGY_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_MANUFACTURE_YEAR,
	POWER_SUPPLY_PROP_MANUFACTURE_MONTH,
	POWER_SUPPLY_PROP_MANUFACTURE_DAY,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD,
};

static const struct power_supply_desc x1e80100_bat_psy_desc = {
	.name = "qcom-battmgr-bat",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = x1e80100_bat_props,
	.num_properties = ARRAY_SIZE(x1e80100_bat_props),
	.get_property = qcom_battmgr_bat_get_property,
	.set_property = qcom_battmgr_bat_set_property,
	.property_is_writeable = qcom_battmgr_bat_is_writeable,
};

static const enum power_supply_property sm8350_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_INTERNAL_RESISTANCE,
	POWER_SUPPLY_PROP_STATE_OF_HEALTH,
	POWER_SUPPLY_PROP_POWER_NOW,
};

static const struct power_supply_desc sm8350_bat_psy_desc = {
	.name = "qcom-battmgr-bat",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = sm8350_bat_props,
	.num_properties = ARRAY_SIZE(sm8350_bat_props),
	.get_property = qcom_battmgr_bat_get_property,
};

static const enum power_supply_property sm8550_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_INTERNAL_RESISTANCE,
	POWER_SUPPLY_PROP_STATE_OF_HEALTH,
	POWER_SUPPLY_PROP_POWER_NOW,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD,
	POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR,
};

static const struct power_supply_desc sm8550_bat_psy_desc = {
	.name = "qcom-battmgr-bat",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.charge_behaviours = BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO) |
			     BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE),
	.properties = sm8550_bat_props,
	.num_properties = ARRAY_SIZE(sm8550_bat_props),
	.get_property = qcom_battmgr_bat_get_property,
	.set_property = qcom_battmgr_bat_set_property,
	.property_is_writeable = qcom_battmgr_bat_is_writeable,
};

static int qcom_battmgr_ac_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct qcom_battmgr *battmgr = power_supply_get_drvdata(psy);
	int ret;

	if (!battmgr->service_up)
		return -EAGAIN;

	ret = qcom_battmgr_bat_sc8280xp_update(battmgr, psp);
	if (ret)
		return ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = battmgr->ac.online;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const enum power_supply_property sc8280xp_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static const struct power_supply_desc sc8280xp_ac_psy_desc = {
	.name = "qcom-battmgr-ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = sc8280xp_ac_props,
	.num_properties = ARRAY_SIZE(sc8280xp_ac_props),
	.get_property = qcom_battmgr_ac_get_property,
};

static const u8 sm8350_usb_prop_map[] = {
	[POWER_SUPPLY_PROP_ONLINE] = USB_ONLINE,
	[POWER_SUPPLY_PROP_VOLTAGE_NOW] = USB_VOLT_NOW,
	[POWER_SUPPLY_PROP_VOLTAGE_MAX] = USB_VOLT_MAX,
	[POWER_SUPPLY_PROP_CURRENT_NOW] = USB_CURR_NOW,
	[POWER_SUPPLY_PROP_CURRENT_MAX] = USB_CURR_MAX,
	[POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT] = USB_INPUT_CURR_LIMIT,
	[POWER_SUPPLY_PROP_USB_TYPE] = USB_ADAP_TYPE,
};

static int qcom_battmgr_usb_sm8350_update(struct qcom_battmgr *battmgr,
					  enum power_supply_property psp)
{
	unsigned int prop;
	int ret;

	if (psp >= ARRAY_SIZE(sm8350_usb_prop_map))
		return -EINVAL;

	prop = sm8350_usb_prop_map[psp];
	if (psp == POWER_SUPPLY_PROP_USB_TYPE &&
	    (battmgr->quirks & QCOM_BATTMGR_QUIRK_OPLUS_PD))
		prop = USB_ADAP_TYPE;

	mutex_lock(&battmgr->lock);
	ret = qcom_battmgr_request_property(battmgr, BATTMGR_USB_PROPERTY_GET, prop, 0);
	mutex_unlock(&battmgr->lock);

	return ret;
}

static int qcom_battmgr_usb_get_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	struct qcom_battmgr *battmgr = power_supply_get_drvdata(psy);
	int ret;

	if (!battmgr->service_up)
		return -EAGAIN;

	if (battmgr->variant == QCOM_BATTMGR_SC8280XP ||
	    battmgr->variant == QCOM_BATTMGR_X1E80100)
		ret = qcom_battmgr_bat_sc8280xp_update(battmgr, psp);
	else
		ret = qcom_battmgr_usb_sm8350_update(battmgr, psp);
	if (ret)
		return ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = battmgr->usb.online;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = battmgr->usb.voltage_now;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = battmgr->usb.voltage_max;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = battmgr->usb.current_now;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = battmgr->usb.current_max;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		val->intval = battmgr->usb.current_limit;
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = battmgr->usb.usb_type;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const enum power_supply_property sc8280xp_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static const struct power_supply_desc sc8280xp_usb_psy_desc = {
	.name = "qcom-battmgr-usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = sc8280xp_usb_props,
	.num_properties = ARRAY_SIZE(sc8280xp_usb_props),
	.get_property = qcom_battmgr_usb_get_property,
	.usb_types = BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN) |
		     BIT(POWER_SUPPLY_USB_TYPE_SDP)     |
		     BIT(POWER_SUPPLY_USB_TYPE_DCP)     |
		     BIT(POWER_SUPPLY_USB_TYPE_CDP)     |
		     BIT(POWER_SUPPLY_USB_TYPE_ACA)     |
		     BIT(POWER_SUPPLY_USB_TYPE_C)       |
		     BIT(POWER_SUPPLY_USB_TYPE_PD)      |
		     BIT(POWER_SUPPLY_USB_TYPE_PD_DRP)  |
		     BIT(POWER_SUPPLY_USB_TYPE_PD_PPS)  |
		     BIT(POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID),
};

static const enum power_supply_property sm8350_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_USB_TYPE,
};

static const struct power_supply_desc sm8350_usb_psy_desc = {
	.name = "qcom-battmgr-usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = sm8350_usb_props,
	.num_properties = ARRAY_SIZE(sm8350_usb_props),
	.get_property = qcom_battmgr_usb_get_property,
	.usb_types = BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN) |
		     BIT(POWER_SUPPLY_USB_TYPE_SDP)     |
		     BIT(POWER_SUPPLY_USB_TYPE_DCP)     |
		     BIT(POWER_SUPPLY_USB_TYPE_CDP)     |
		     BIT(POWER_SUPPLY_USB_TYPE_ACA)     |
		     BIT(POWER_SUPPLY_USB_TYPE_C)       |
		     BIT(POWER_SUPPLY_USB_TYPE_PD)      |
		     BIT(POWER_SUPPLY_USB_TYPE_PD_DRP)  |
		     BIT(POWER_SUPPLY_USB_TYPE_PD_PPS)  |
		     BIT(POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID),
};

static const u8 sm8350_wls_prop_map[] = {
	[POWER_SUPPLY_PROP_ONLINE] = WLS_ONLINE,
	[POWER_SUPPLY_PROP_VOLTAGE_NOW] = WLS_VOLT_NOW,
	[POWER_SUPPLY_PROP_VOLTAGE_MAX] = WLS_VOLT_MAX,
	[POWER_SUPPLY_PROP_CURRENT_NOW] = WLS_CURR_NOW,
	[POWER_SUPPLY_PROP_CURRENT_MAX] = WLS_CURR_MAX,
};

static int qcom_battmgr_wls_sm8350_update(struct qcom_battmgr *battmgr,
					  enum power_supply_property psp)
{
	unsigned int prop;
	int ret;

	if (psp >= ARRAY_SIZE(sm8350_wls_prop_map))
		return -EINVAL;

	prop = sm8350_wls_prop_map[psp];

	mutex_lock(&battmgr->lock);
	ret = qcom_battmgr_request_property(battmgr, BATTMGR_WLS_PROPERTY_GET, prop, 0);
	mutex_unlock(&battmgr->lock);

	return ret;
}

static int qcom_battmgr_wls_get_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	struct qcom_battmgr *battmgr = power_supply_get_drvdata(psy);
	int ret;

	if (!battmgr->service_up)
		return -EAGAIN;

	if (battmgr->variant == QCOM_BATTMGR_SC8280XP ||
	    battmgr->variant == QCOM_BATTMGR_X1E80100)
		ret = qcom_battmgr_bat_sc8280xp_update(battmgr, psp);
	else
		ret = qcom_battmgr_wls_sm8350_update(battmgr, psp);
	if (ret < 0)
		return ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = battmgr->wireless.online;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = battmgr->wireless.voltage_now;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = battmgr->wireless.voltage_max;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = battmgr->wireless.current_now;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = battmgr->wireless.current_max;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const enum power_supply_property sc8280xp_wls_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static const struct power_supply_desc sc8280xp_wls_psy_desc = {
	.name = "qcom-battmgr-wls",
	.type = POWER_SUPPLY_TYPE_WIRELESS,
	.properties = sc8280xp_wls_props,
	.num_properties = ARRAY_SIZE(sc8280xp_wls_props),
	.get_property = qcom_battmgr_wls_get_property,
};

static const enum power_supply_property sm8350_wls_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static const struct power_supply_desc sm8350_wls_psy_desc = {
	.name = "qcom-battmgr-wls",
	.type = POWER_SUPPLY_TYPE_WIRELESS,
	.properties = sm8350_wls_props,
	.num_properties = ARRAY_SIZE(sm8350_wls_props),
	.get_property = qcom_battmgr_wls_get_property,
};

static void qcom_battmgr_notification(struct qcom_battmgr *battmgr,
				      const struct qcom_battmgr_message *msg,
				      int len)
{
	size_t payload_len = len - sizeof(struct pmic_glink_hdr);
	unsigned int notification;

	if (payload_len != sizeof(msg->notification)) {
		dev_warn(battmgr->dev, "ignoring notification with invalid length\n");
		return;
	}

	notification = le32_to_cpu(msg->notification);
	if (!(battmgr->quirks & QCOM_BATTMGR_QUIRK_OPLUS_PD))
		notification &= 0xff;
	if ((battmgr->quirks & QCOM_BATTMGR_QUIRK_PPS) &&
	    (notification == NOTIF_OPLUS_PD_SVOOC || notification == NOTIF_OPLUS_UFCS_HANDSHAKE_OK)) {
		set_bit(notification == NOTIF_OPLUS_PD_SVOOC ? PD_EVENT_SVOOC_RESPONSE :
			PD_EVENT_UFCS_RESPONSE, &battmgr->pd_events);
		qcom_battmgr_usb_event(battmgr);
		return;
	}
	switch (notification) {
	case NOTIF_BAT_INFO:
		battmgr->info.valid = false;
		fallthrough;
	case NOTIF_BAT_STATUS:
	case NOTIF_BAT_PROPERTY:
	case NOTIF_BAT_CHARGING_STATE:
		power_supply_changed(battmgr->bat_psy);
		break;
	case NOTIF_USB_PROPERTY:
		power_supply_changed(battmgr->usb_psy);
		qcom_battmgr_usb_event(battmgr);
		break;
	case NOTIF_WLS_PROPERTY:
		power_supply_changed(battmgr->wls_psy);
		break;
	case NOTIF_OPLUS_OTG_ENABLE:
	case NOTIF_OPLUS_OTG_DISABLE:
		if (battmgr->quirks & QCOM_BATTMGR_QUIRK_OTG_VBUS) {
			WRITE_ONCE(battmgr->otg_request,
				   notification == NOTIF_OPLUS_OTG_ENABLE);
			if (battmgr->otg_request)
				qcom_battmgr_charge_wake_locked(battmgr, false);
			queue_work(system_freezable_wq, &battmgr->otg_work);
			qcom_battmgr_usb_event(battmgr);
			break;
		}
		fallthrough;
	case NOTIF_OPLUS_CID_DETECT:
	case NOTIF_OPLUS_SOURCECAP_DONE:
	case NOTIF_OPLUS_TYPEC_STATE:
	case NOTIF_OPLUS_PLUGIN:
	case NOTIF_OPLUS_APSD_DONE:
		if (battmgr->quirks & QCOM_BATTMGR_QUIRK_CHARGE_ENABLE) {
			set_bit(PD_EVENT_PARTNER, &battmgr->pd_events);
			set_bit(PD_EVENT_CAPS, &battmgr->pd_events);
			power_supply_changed(battmgr->usb_psy);
			qcom_battmgr_usb_event(battmgr);
			break;
		}
		fallthrough;
	case NOTIF_OPLUS_UFCS_DISABLE_MOS:
	case NOTIF_OPLUS_POWER_ROLE:
	case NOTIF_OPLUS_PD_HARD_RESET:
		if (battmgr->quirks & QCOM_BATTMGR_QUIRK_OPLUS_PD) {
			if (notification == NOTIF_OPLUS_PD_HARD_RESET)
				set_bit(PD_EVENT_RESET, &battmgr->pd_events);
			set_bit(PD_EVENT_CAPS, &battmgr->pd_events);
			qcom_battmgr_usb_event(battmgr);
			break;
		}
		/*
		 * UFCS charge pump and reverse charging events, for features
		 * this driver does not have; the power role reaches the OS
		 * through UCSI.
		 */
		if (battmgr->quirks & QCOM_BATTMGR_QUIRK_CHARGE_ENABLE)
			break;
		fallthrough;
	default:
		dev_err(battmgr->dev, "unknown notification: %#x\n", notification);
		break;
	}
}

static void qcom_battmgr_sc8280xp_strcpy(char *dest, const char *src)
{
	size_t len = src[0];

	/* Some firmware versions return Pascal-style strings */
	if (len < BATTMGR_STRING_LEN && len == strnlen(src + 1, BATTMGR_STRING_LEN - 1)) {
		memcpy(dest, src + 1, len);
		dest[len] = '\0';
	} else {
		strscpy(dest, src, BATTMGR_STRING_LEN);
	}
}

static unsigned int qcom_battmgr_sc8280xp_parse_technology(const char *chemistry)
{
	if ((!strncmp(chemistry, "LIO", 3)) ||
	    (!strncmp(chemistry, "OOI", 3)))
		return POWER_SUPPLY_TECHNOLOGY_LION;
	if (!strncmp(chemistry, "LIP", 3) ||
	    !strncmp(chemistry, "LiP", 3))
		return POWER_SUPPLY_TECHNOLOGY_LIPO;

	pr_err("Unknown battery technology '%s'\n", chemistry);
	return POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
}

static unsigned int qcom_battmgr_sc8280xp_convert_temp(unsigned int temperature)
{
	return DIV_ROUND_CLOSEST(temperature, 10);
}

/*
 * The charger supplies' online state comes from the battery status response,
 * and the firmware sends no notification of its own when it changes, so
 * announce the change here. A supply that is not registered yet announces its
 * state when it is.
 */
static void qcom_battmgr_sc8280xp_set_online(struct qcom_battmgr *battmgr,
					  struct power_supply *psy,
					  bool *online, bool value)
{
	guard(spinlock_irqsave)(&battmgr->notify_lock);
	if (*online == value)
		return;

	*online = value;
	if (psy && !battmgr->removing)
		power_supply_changed(psy);
}

static void qcom_battmgr_sc8280xp_callback(struct qcom_battmgr *battmgr,
					   const struct qcom_battmgr_message *resp,
					   size_t len)
{
	unsigned int opcode = le32_to_cpu(resp->hdr.opcode);
	unsigned int power_state;
	unsigned int source;
	unsigned int state;
	size_t payload_len = len - sizeof(struct pmic_glink_hdr);

	if (payload_len < sizeof(__le32)) {
		dev_warn(battmgr->dev, "invalid payload length for %#x: %zd\n",
			 opcode, len);
		return;
	}

	switch (opcode) {
	case BATTMGR_REQUEST_NOTIFICATION:
		battmgr->error = 0;
		break;
	case BATTMGR_BAT_INFO:
		/* some firmware versions report an extra __le32 at the end of the payload */
		if (payload_len != sizeof(resp->info) &&
		    payload_len != (sizeof(resp->info) + sizeof(__le32))) {
			dev_warn(battmgr->dev,
				 "invalid payload length for battery information request: %zd\n",
				 payload_len);
			battmgr->error = -ENODATA;
			return;
		}

		battmgr->unit = le32_to_cpu(resp->info.power_unit);

		battmgr->info.present = true;
		battmgr->info.design_capacity = le32_to_cpu(resp->info.design_capacity) * 1000;
		battmgr->info.last_full_capacity = le32_to_cpu(resp->info.last_full_capacity) * 1000;
		battmgr->info.voltage_max_design = le32_to_cpu(resp->info.design_voltage) * 1000;
		battmgr->info.capacity_low = le32_to_cpu(resp->info.capacity_low) * 1000;
		battmgr->info.cycle_count = le32_to_cpu(resp->info.cycle_count);
		qcom_battmgr_sc8280xp_strcpy(battmgr->info.model_number, resp->info.model_number);
		qcom_battmgr_sc8280xp_strcpy(battmgr->info.serial_number, resp->info.serial_number);
		battmgr->info.technology = qcom_battmgr_sc8280xp_parse_technology(resp->info.battery_chemistry);
		qcom_battmgr_sc8280xp_strcpy(battmgr->info.oem_info, resp->info.oem_info);
		battmgr->info.day = resp->info.day;
		battmgr->info.month = resp->info.month;
		battmgr->info.year = le16_to_cpu(resp->info.year);
		break;
	case BATTMGR_BAT_STATUS:
		if (payload_len != sizeof(resp->status)) {
			dev_warn(battmgr->dev,
				 "invalid payload length for battery status request: %zd\n",
				 payload_len);
			battmgr->error = -ENODATA;
			return;
		}

		battmgr->status.capacity = le32_to_cpu(resp->status.capacity) * 1000;
		battmgr->status.power_now = le32_to_cpu(resp->status.rate) * 1000;
		battmgr->status.voltage_now = le32_to_cpu(resp->status.battery_voltage) * 1000;
		battmgr->status.temperature = qcom_battmgr_sc8280xp_convert_temp(le32_to_cpu(resp->status.temperature));

		state = le32_to_cpu(resp->status.battery_state);
		power_state = le32_to_cpu(resp->status.power_state);
		if (state & BIT(0))
			battmgr->status.status = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (state & BIT(1))
			battmgr->status.status = POWER_SUPPLY_STATUS_CHARGING;
		else if ((power_state & BIT(0)) &&
			 battmgr->info.last_full_capacity &&
			 battmgr->status.capacity >= battmgr->info.last_full_capacity)
			battmgr->status.status = POWER_SUPPLY_STATUS_FULL;
		else
			battmgr->status.status = POWER_SUPPLY_STATUS_NOT_CHARGING;

		source = le32_to_cpu(resp->status.charging_source);
		qcom_battmgr_sc8280xp_set_online(battmgr, battmgr->ac_psy, &battmgr->ac.online,
						 source == BATTMGR_CHARGING_SOURCE_AC);
		qcom_battmgr_sc8280xp_set_online(battmgr, battmgr->usb_psy, &battmgr->usb.online,
						 source == BATTMGR_CHARGING_SOURCE_USB);
		qcom_battmgr_sc8280xp_set_online(battmgr, battmgr->wls_psy, &battmgr->wireless.online,
						 source == BATTMGR_CHARGING_SOURCE_WIRELESS);
		if (battmgr->info.last_full_capacity != 0) {
			/*
			 * 100 * battmgr->status.capacity can overflow a 32bit
			 * unsigned integer. FW readings are in m{W/A}h, which
			 * are multiplied by 1000 converting them to u{W/A}h,
			 * the format the power_supply API expects.
			 * To avoid overflow use the original value for dividend
			 * and convert the divider back to m{W/A}h, which can be
			 * done without any loss of precision.
			 */
			battmgr->status.percent =
				(100 * le32_to_cpu(resp->status.capacity)) /
				(battmgr->info.last_full_capacity / 1000);
		} else {
			/*
			 * Let the sysfs handler know no data is available at
			 * this time.
			 */
			battmgr->status.percent = (unsigned int)-1;
		}
		break;
	case BATTMGR_BAT_DISCHARGE_TIME:
		battmgr->status.discharge_time = le32_to_cpu(resp->time);
		break;
	case BATTMGR_BAT_CHARGE_TIME:
		battmgr->status.charge_time = le32_to_cpu(resp->time);
		break;
	case BATTMGR_CHG_CTRL_LIMIT_EN:
		battmgr->error = 0;
		break;
	default:
		dev_warn(battmgr->dev, "unknown message %#x\n", opcode);
		break;
	}

	complete(&battmgr->ack);
}

static void qcom_battmgr_sm8350_callback(struct qcom_battmgr *battmgr,
					 const struct qcom_battmgr_message *resp,
					 size_t len)
{
	unsigned int property;
	unsigned int opcode = le32_to_cpu(resp->hdr.opcode);
	size_t payload_len = len - sizeof(struct pmic_glink_hdr);
	unsigned int val;

	if (payload_len < sizeof(__le32)) {
		dev_warn(battmgr->dev, "invalid payload length for %#x: %zd\n",
			 opcode, len);
		return;
	}

	switch (opcode) {
	case BATTMGR_BAT_PROPERTY_GET:
		property = le32_to_cpu(resp->intval.property);
		if (property == BATT_MODEL_NAME) {
			if (payload_len != sizeof(resp->strval)) {
				dev_warn(battmgr->dev,
					 "invalid payload length for BATT_MODEL_NAME request: %zd\n",
					 payload_len);
				battmgr->error = -ENODATA;
				return;
			}
		} else {
			if (payload_len != sizeof(resp->intval)) {
				dev_warn(battmgr->dev,
					 "invalid payload length for %#x request: %zd\n",
					 property, payload_len);
				battmgr->error = -ENODATA;
				return;
			}

			battmgr->error = le32_to_cpu(resp->intval.result);
			if (battmgr->error)
				goto out_complete;

			battmgr->resp_property = property;
			battmgr->resp_value = le32_to_cpu(resp->intval.value);
		}

		switch (property) {
		case BATT_STATUS:
			battmgr->status.status = le32_to_cpu(resp->intval.value);
			break;
		case BATT_HEALTH:
			battmgr->status.health = le32_to_cpu(resp->intval.value);
			break;
		case BATT_PRESENT:
			battmgr->info.present = le32_to_cpu(resp->intval.value);
			break;
		case BATT_CHG_TYPE:
			battmgr->info.charge_type = le32_to_cpu(resp->intval.value);
			break;
		case BATT_CAPACITY:
			battmgr->status.percent = le32_to_cpu(resp->intval.value) / 100;
			break;
		case BATT_SOH:
			battmgr->status.soh_percent = le32_to_cpu(resp->intval.value);
			break;
		case BATT_VOLT_OCV:
			battmgr->status.voltage_ocv = le32_to_cpu(resp->intval.value);
			break;
		case BATT_VOLT_NOW:
			battmgr->status.voltage_now = le32_to_cpu(resp->intval.value);
			break;
		case BATT_VOLT_MAX:
			battmgr->info.voltage_max = le32_to_cpu(resp->intval.value);
			break;
		case BATT_CURR_NOW:
			battmgr->status.current_now = le32_to_cpu(resp->intval.value);
			break;
		case BATT_TEMP:
			val = le32_to_cpu(resp->intval.value);
			battmgr->status.temperature = DIV_ROUND_CLOSEST(val, 10);
			break;
		case BATT_TECHNOLOGY:
			battmgr->info.technology = le32_to_cpu(resp->intval.value);
			break;
		case BATT_CHG_COUNTER:
			battmgr->info.charge_count = le32_to_cpu(resp->intval.value);
			break;
		case BATT_CYCLE_COUNT:
			battmgr->info.cycle_count = le32_to_cpu(resp->intval.value);
			break;
		case BATT_CHG_FULL_DESIGN:
			battmgr->info.design_capacity = le32_to_cpu(resp->intval.value);
			break;
		case BATT_CHG_FULL:
			battmgr->info.last_full_capacity = le32_to_cpu(resp->intval.value);
			break;
		case BATT_MODEL_NAME:
			strscpy(battmgr->info.model_number, resp->strval.model, BATTMGR_STRING_LEN);
			break;
		case BATT_TTF_AVG:
			battmgr->status.charge_time = le32_to_cpu(resp->intval.value);
			break;
		case BATT_TTE_AVG:
			battmgr->status.discharge_time = le32_to_cpu(resp->intval.value);
			break;
		case BATT_RESISTANCE:
			battmgr->status.resistance = le32_to_cpu(resp->intval.value);
			break;
		case BATT_POWER_NOW:
			battmgr->status.power_now = le32_to_cpu(resp->intval.value);
			break;
		case BATT_CHG_CTRL_START_THR:
			battmgr->info.charge_ctrl_start = le32_to_cpu(resp->intval.value);
			break;
		case BATT_CHG_CTRL_END_THR:
			battmgr->info.charge_ctrl_end = le32_to_cpu(resp->intval.value);
			break;
		case BATT_CHG_CTRL_LIM:
		case BATT_CHG_CTRL_LIM_MAX:
		case BATT_OPLUS_AUTH:
		case BATT_OPLUS_RTC_SOC:
		case BATT_OPLUS_HMAC:
			/* Only read through qcom_battmgr_get_property() */
			break;
		default:
			dev_warn(battmgr->dev, "unknown property %#x\n", property);
			break;
		}
		break;
	case BATTMGR_USB_PROPERTY_GET:
		property = le32_to_cpu(resp->intval.property);
		if (payload_len != sizeof(resp->intval)) {
			dev_warn(battmgr->dev,
				 "invalid payload length for %#x request: %zd\n",
				 property, payload_len);
			battmgr->error = -ENODATA;
			return;
		}

		battmgr->error = le32_to_cpu(resp->intval.result);
		if (battmgr->error)
			goto out_complete;

		battmgr->resp_property = property;
		battmgr->resp_value = le32_to_cpu(resp->intval.value);

		switch (property) {
		case USB_ONLINE:
			battmgr->usb.online = le32_to_cpu(resp->intval.value);
			break;
		case USB_VOLT_NOW:
			battmgr->usb.voltage_now = le32_to_cpu(resp->intval.value);
			break;
		case USB_VOLT_MAX:
			battmgr->usb.voltage_max = le32_to_cpu(resp->intval.value);
			break;
		case USB_CURR_NOW:
			battmgr->usb.current_now = le32_to_cpu(resp->intval.value);
			break;
		case USB_CURR_MAX:
			battmgr->usb.current_max = le32_to_cpu(resp->intval.value);
			break;
		case USB_INPUT_CURR_LIMIT:
			battmgr->usb.current_limit = le32_to_cpu(resp->intval.value);
			break;
		case USB_ADAP_TYPE:
			val = le32_to_cpu(resp->intval.value);
			if (!(battmgr->quirks & QCOM_BATTMGR_QUIRK_OPLUS_PD)) {
				battmgr->usb.usb_type = val;
				break;
			}
			if (val == OPLUS_USB_TYPE_PD_SDP)
				val = POWER_SUPPLY_USB_TYPE_PD;
			else if (val >= 0x80 && val <= 0x82)
				val = POWER_SUPPLY_USB_TYPE_DCP; /* Qualcomm HVDCP */
			else if (val == OPLUS_USB_TYPE_APPLE_BRICK_ID)
				val = POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID;
			else if (val > POWER_SUPPLY_USB_TYPE_PD_PPS)
				val = POWER_SUPPLY_USB_TYPE_UNKNOWN;
			battmgr->usb.usb_type = val;
			break;
		case USB_REAL_TYPE:
		case USB_OPLUS_CID_STATUS:
		case 26: /* OPLUS serial fast-charge classification */
		case USB_OPLUS_PPS_TYPE:
		case USB_OPLUS_PPS_MAX_CURR:
			/* Only read through qcom_battmgr_get_property() */
			break;
		default:
			dev_warn(battmgr->dev, "unknown property %#x\n", property);
			break;
		}
		break;
	case BATTMGR_WLS_PROPERTY_GET:
		property = le32_to_cpu(resp->intval.property);
		if (payload_len != sizeof(resp->intval)) {
			dev_warn(battmgr->dev,
				 "invalid payload length for %#x request: %zd\n",
				 property, payload_len);
			battmgr->error = -ENODATA;
			return;
		}

		battmgr->error = le32_to_cpu(resp->intval.result);
		if (battmgr->error)
			goto out_complete;

		switch (property) {
		case WLS_ONLINE:
			battmgr->wireless.online = le32_to_cpu(resp->intval.value);
			break;
		case WLS_VOLT_NOW:
			battmgr->wireless.voltage_now = le32_to_cpu(resp->intval.value);
			break;
		case WLS_VOLT_MAX:
			battmgr->wireless.voltage_max = le32_to_cpu(resp->intval.value);
			break;
		case WLS_CURR_NOW:
			battmgr->wireless.current_now = le32_to_cpu(resp->intval.value);
			break;
		case WLS_CURR_MAX:
			battmgr->wireless.current_max = le32_to_cpu(resp->intval.value);
			break;
		default:
			dev_warn(battmgr->dev, "unknown property %#x\n", property);
			break;
		}
		break;
	case BATTMGR_REQUEST_NOTIFICATION:
	case BATTMGR_CHG_CTRL_LIMIT_EN:
		battmgr->error = 0;
		break;
	case BATTMGR_BAT_PROPERTY_SET:
	case BATTMGR_USB_PROPERTY_SET:
		if (payload_len != sizeof(resp->intval)) {
			dev_warn(battmgr->dev,
				 "invalid payload length for %#x response: %zd\n",
				 opcode, payload_len);
			battmgr->error = -ENODATA;
			break;
		}

		/* The firmware answers a write with a nonzero code if it refused it */
		if (le32_to_cpu(resp->intval.result)) {
			battmgr->error = -EREMOTEIO;
			break;
		}

		battmgr->resp_property = le32_to_cpu(resp->intval.property);
		break;
	default:
		dev_warn(battmgr->dev, "unknown message %#x\n", opcode);
		break;
	}

out_complete:
	complete(&battmgr->ack);
}

/* AP_OPCODE_PD_INFO_BUFFER carries partner identity VDOs, not a PDO reply. */
static void qcom_battmgr_pd_info(struct qcom_battmgr *battmgr,
			       const struct qcom_battmgr_pd_info *msg, size_t len)
{
	u32 size, id;
	unsigned int i;

	if (len != sizeof(*msg))
		goto invalid;

	size = le32_to_cpu(msg->data_size);
	id = le32_to_cpu(msg->msg_id);
	if (!size || size > sizeof(msg->data) || size % sizeof(__le32))
		goto invalid;
	if (id != OPLUS_PD_INFO_VDM_ID) {
		dev_dbg(battmgr->dev, "unhandled PD info id %u\n", id);
		return;
	}

	for (i = 0; i < size / sizeof(__le32); i++)
		dev_dbg(battmgr->dev, "PD identity VDO %u: %#08x\n",
			i, le32_to_cpu(msg->data[i]));
	return;

invalid:
	dev_warn_ratelimited(battmgr->dev, "invalid PD identity buffer\n");
}

static void qcom_battmgr_pdos_response(struct qcom_battmgr *battmgr,
				     const struct qcom_battmgr_read_pdos_response *msg,
				     size_t len)
{
	unsigned int i, words;
	u32 size;

	battmgr->num_src_pdos = 0;
	battmgr->error = -ENODATA;
	if (len != sizeof(*msg))
		goto out;

	size = le32_to_cpu(msg->data_size);
	if (size < 3 * sizeof(__le32) || size > sizeof(msg->data) ||
	    size % sizeof(__le32))
		goto out;

	/* Words 0/1 are PPS imax/vmax, followed by up to seven raw PDOs. */
	words = min_t(unsigned int, size / sizeof(__le32) - 2, PDO_MAX_OBJECTS);
	for (i = 0; i < words; i++)
		battmgr->src_pdos[i] = le32_to_cpu(msg->data[i + 2]);
	battmgr->num_src_pdos = words;
	battmgr->error = 0;
out:
	complete(&battmgr->ack);
}

static bool qcom_battmgr_is_charger_notification(const void *data, size_t len, void *priv)
{
	const struct qcom_battmgr_message *msg = data;
	struct qcom_battmgr *battmgr = priv;
	unsigned int notification;

	if (le32_to_cpu(msg->hdr.opcode) != BATTMGR_NOTIFICATION ||
	    len != sizeof(msg->hdr) + sizeof(msg->notification))
		return false;

	notification = le32_to_cpu(msg->notification);
	if (battmgr->quirks & QCOM_BATTMGR_QUIRK_OPLUS_PD)
		return notification == NOTIF_OPLUS_PLUGIN;

	notification &= 0xff;

	return notification == NOTIF_USB_PROPERTY || notification == NOTIF_WLS_PROPERTY;
}

static void qcom_battmgr_callback(const void *data, size_t len, void *priv)
{
	const struct pmic_glink_hdr *hdr = data;
	struct qcom_battmgr *battmgr = priv;
	unsigned int opcode = le32_to_cpu(hdr->opcode);

	if (opcode == BATTMGR_OPLUS_PD_INFO &&
	    (battmgr->quirks & QCOM_BATTMGR_QUIRK_OPLUS_PD)) {
		guard(spinlock_irqsave)(&battmgr->notify_lock);
		if (!battmgr->removing)
			qcom_battmgr_pd_info(battmgr, data, len);
	} else if (opcode == BATTMGR_NOTIFICATION) {
		guard(spinlock_irqsave)(&battmgr->notify_lock);
		if (!battmgr->removing)
			qcom_battmgr_notification(battmgr, data, len);
	} else if (opcode != READ_ONCE(battmgr->request_opcode)) {
		dev_dbg(battmgr->dev, "unsolicited message %#x\n", opcode);
	} else if (opcode == BATTMGR_OPLUS_READ_PDOS &&
		   (battmgr->quirks & QCOM_BATTMGR_QUIRK_OPLUS_PD)) {
		qcom_battmgr_pdos_response(battmgr, data, len);
	} else if (battmgr->variant == QCOM_BATTMGR_SC8280XP ||
		   battmgr->variant == QCOM_BATTMGR_X1E80100) {
		qcom_battmgr_sc8280xp_callback(battmgr, data, len);
	} else {
		qcom_battmgr_sm8350_callback(battmgr, data, len);
	}
}

/*
 * Let the firmware use the port as a USB source.  It then detects a sink on
 * the port and asks for VBUS with NOTIF_OPLUS_OTG_ENABLE.  The firmware does
 * not detect a sink while this is cleared, so it is done once the service
 * is up, before anything is plugged in; it does not turn VBUS on by itself.
 */
static void qcom_battmgr_otg_init(struct qcom_battmgr *battmgr)
{
	int ret;

	ret = qcom_battmgr_set_property(battmgr, BATTMGR_USB_PROPERTY_SET,
					USB_OPLUS_OTG_AP_ENABLE, 1);
	if (!ret)
		ret = qcom_battmgr_set_property(battmgr, BATTMGR_USB_PROPERTY_SET,
						USB_OPLUS_OTG_SWITCH, 1);
	if (ret)
		dev_err(battmgr->dev, "failed to enable USB source mode: %d\n", ret);
}

/*
 * Switch the VBUS boost to what the firmware asked for last: on while it
 * reports a sink, that is while the port is the USB source and host, and
 * off once the sink is gone.  The data role stays with the Type-C driver.
 */
static void qcom_battmgr_otg_worker(struct work_struct *work)
{
	struct qcom_battmgr *battmgr = container_of(work, struct qcom_battmgr, otg_work);
	bool on = READ_ONCE(battmgr->otg_request);
	int ret;

	guard(mutex)(&battmgr->charge_lock);
	if (!battmgr->service_up || on == battmgr->otg_vbus)
		return;

	if (on && (qcom_battmgr_pps_stop(battmgr, false, "OTG-enable") ||
		   qcom_battmgr_pd_stop(battmgr, false, "USB source role")))
		return;

	/* The stock system sets the boost current limit before the boost */
	if (on) {
		ret = qcom_battmgr_set_property(battmgr, BATTMGR_USB_PROPERTY_SET,
						USB_OPLUS_OTG_BOOST_CURRENT,
						OTG_BOOST_CURRENT_MA);
		if (ret)
			dev_warn(battmgr->dev, "failed to set the VBUS current limit: %d\n",
				 ret);
	}

	ret = qcom_battmgr_set_property(battmgr, BATTMGR_USB_PROPERTY_SET,
					USB_OPLUS_OTG_VBUS_ENABLE, on);
	if (ret) {
		dev_err(battmgr->dev, "failed to turn VBUS %s: %d\n",
			on ? "on" : "off", ret);
		return;
	}

	battmgr->otg_vbus = on;
	dev_info(battmgr->dev, "VBUS %s for a USB sink\n", on ? "on" : "off");
}

static void qcom_battmgr_enable_worker(struct work_struct *work)
{
	struct qcom_battmgr *battmgr = container_of(work, struct qcom_battmgr, enable_work);
	struct qcom_battmgr_enable_request req = {
		.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.hdr.type = cpu_to_le32(PMIC_GLINK_NOTIFY),
		.hdr.opcode = cpu_to_le32(BATTMGR_REQUEST_NOTIFICATION),
	};
	int ret;

	mutex_lock(&battmgr->lock);
	ret = qcom_battmgr_request(battmgr, &req, sizeof(req));
	mutex_unlock(&battmgr->lock);
	if (ret)
		dev_err(battmgr->dev, "failed to request power notifications\n");

	if (battmgr->quirks & QCOM_BATTMGR_QUIRK_OTG_VBUS)
		qcom_battmgr_otg_init(battmgr);

	qcom_battmgr_charge_kick(battmgr);
}

static void qcom_battmgr_pdr_notify(void *priv, int state)
{
	struct qcom_battmgr *battmgr = priv;

	guard(spinlock_irqsave)(&battmgr->notify_lock);
	if (battmgr->removing)
		return;

	if (state == SERVREG_SERVICE_STATE_UP) {
		battmgr->service_up = true;
		schedule_work(&battmgr->enable_work);
	} else {
		battmgr->service_up = false;
		battmgr->buck_online = false;
		qcom_battmgr_charge_wake_locked(battmgr, false);
		/* A restarted firmware charges again by default */
		battmgr->charge_inhibited = false;
		/* ... and starts with the boost off and without a PPS contract */
		battmgr->otg_request = false;
		battmgr->otg_vbus = false;
		set_bit(PD_EVENT_SERVICE_RESET, &battmgr->pd_events);
		set_bit(PD_EVENT_RESET, &battmgr->pd_events);
		qcom_battmgr_charge_kick(battmgr);
	}
}

static const struct of_device_id qcom_battmgr_of_variants[] = {
	{ .compatible = "qcom,glymur-pmic-glink", .data = (void *)QCOM_BATTMGR_X1E80100 },
	{ .compatible = "qcom,kaanapali-pmic-glink", .data = (void *)QCOM_BATTMGR_SM8550 },
	{ .compatible = "qcom,sc8180x-pmic-glink", .data = (void *)QCOM_BATTMGR_SC8280XP },
	{ .compatible = "qcom,sc8280xp-pmic-glink", .data = (void *)QCOM_BATTMGR_SC8280XP },
	{ .compatible = "qcom,sm8550-pmic-glink", .data = (void *)QCOM_BATTMGR_SM8550 },
	{ .compatible = "qcom,x1e80100-pmic-glink", .data = (void *)QCOM_BATTMGR_X1E80100 },
	/* Unmatched devices falls back to QCOM_BATTMGR_SM8350 */
	{}
};

/*
 * Devices whose firmware follows the protocol of their SoC but reports some
 * values with a different meaning.
 */
static const struct of_device_id qcom_battmgr_of_quirks[] = {
	{
		.compatible = "oneplus,pad2pro-pmic-glink",
		.data = (void *)(QCOM_BATTMGR_QUIRK_BATT_CURRENT_INVERTED |
				 QCOM_BATTMGR_QUIRK_NO_BATT_POWER_NOW |
				 QCOM_BATTMGR_QUIRK_NO_CHARGE_CONTROL |
				 QCOM_BATTMGR_QUIRK_CHARGE_ENABLE |
				 QCOM_BATTMGR_QUIRK_OTG_VBUS |
				 QCOM_BATTMGR_QUIRK_PPS |
				 QCOM_BATTMGR_QUIRK_OPLUS_PD),
	},
	{}
};

static bool qcom_battmgr_bat_prop_hidden(struct qcom_battmgr *battmgr,
					 enum power_supply_property psp)
{
	switch (psp) {
	/* Leave POWER_NOW out rather than report a value that is not one */
	case POWER_SUPPLY_PROP_POWER_NOW:
		return battmgr->quirks & QCOM_BATTMGR_QUIRK_NO_BATT_POWER_NOW;
	/* Never send a request that the firmware reads as another command */
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD:
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		return qcom_battmgr_no_charge_ctrl(battmgr);
	/* Only the driver-enforced charge control can hold charging off */
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
		return !(battmgr->quirks & QCOM_BATTMGR_QUIRK_CHARGE_ENABLE);
	default:
		return false;
	}
}

static const struct power_supply_desc *
qcom_battmgr_sm8350_bat_desc(struct qcom_battmgr *battmgr,
			     const struct power_supply_desc *desc)
{
	enum power_supply_property *props;
	struct power_supply_desc *copy;
	size_t i, n = 0;

	if (!(battmgr->quirks & (QCOM_BATTMGR_QUIRK_NO_BATT_POWER_NOW |
				 QCOM_BATTMGR_QUIRK_NO_CHARGE_CONTROL)))
		return desc;

	copy = devm_kmemdup(battmgr->dev, desc, sizeof(*desc), GFP_KERNEL);
	props = devm_kcalloc(battmgr->dev, desc->num_properties, sizeof(*props),
			     GFP_KERNEL);
	if (!copy || !props)
		return NULL;

	for (i = 0; i < desc->num_properties; i++)
		if (!qcom_battmgr_bat_prop_hidden(battmgr, desc->properties[i]))
			props[n++] = desc->properties[i];

	copy->properties = props;
	copy->num_properties = n;

	return copy;
}

static char *qcom_battmgr_battery[] = { "battery" };

static void qcom_battmgr_stop_notifications(void *data)
{
	struct qcom_battmgr *battmgr = data;

	guard(spinlock_irqsave)(&battmgr->notify_lock);
	battmgr->removing = true;
	qcom_battmgr_charge_wake_locked(battmgr, false);
}

static void qcom_battmgr_cancel_work(void *data)
{
	struct qcom_battmgr *battmgr = data;

	disable_delayed_work_sync(&battmgr->charge_work);
	disable_delayed_work_sync(&battmgr->uisoc_work);
	disable_work_sync(&battmgr->otg_work);
	mutex_lock(&battmgr->charge_lock);
	qcom_battmgr_pps_stop(battmgr, false, "driver-stop");
	if (battmgr->service_up) {
		qcom_battmgr_pd_stop(battmgr, false, "driver stopped");
		qcom_battmgr_buck_release(battmgr);
	}
	mutex_unlock(&battmgr->charge_lock);
}

static void qcom_battmgr_release_client(void *data)
{
	struct qcom_battmgr *battmgr = data;

	/* OPLUS callbacks and workers also use the serialized request path. */
	qcom_battmgr_stop_notifications(battmgr);
	disable_work_sync(&battmgr->enable_work);
	qcom_battmgr_cancel_work(battmgr);

	mutex_lock(&battmgr->lock);
	battmgr->client_gone = true;
	mutex_unlock(&battmgr->lock);
}

static void qcom_battmgr_unregister_psy_notifier(void *data)
{
	struct qcom_battmgr *battmgr = data;

	power_supply_unreg_notifier(&battmgr->psy_nb);
}

static void qcom_battmgr_unregister_pm_notifier(void *data)
{
	struct qcom_battmgr *battmgr = data;

	unregister_pm_notifier(&battmgr->pm_nb);
}

/* PD sources qualify their advertised current against the cable identity.
 * Use one live PPS APDO covering the complete voltage regulation window. */
int erhai_pps_apdo_current(u32 min_mv, u32 max_mv, u32 *selected)
{
	struct qcom_battmgr *b;
	unsigned int i;
	int ret, ma = 0;

	*selected = 0;
	guard(mutex)(&erhai_bridge_lock);
	b = erhai_battmgr;
	if (!b || !b->service_up)
		return -ENODEV;
	guard(mutex)(&b->lock);
	ret = qcom_battmgr_read_pdos_locked(b);
	if (ret)
		return ret;
	for (i = 0; i < b->num_src_pdos; i++) {
		u32 pdo = b->src_pdos[i];

		if (pdo_type(pdo) == PDO_TYPE_APDO && pdo_apdo_type(pdo) == APDO_TYPE_PPS &&
		    pdo_pps_apdo_min_voltage(pdo) <= min_mv &&
		    pdo_pps_apdo_max_voltage(pdo) >= max_mv &&
		    pdo_pps_apdo_max_current(pdo) > ma) {
			ma = pdo_pps_apdo_max_current(pdo);
			*selected = pdo;
		}
	}
	return min(ma, 5000);
}
EXPORT_SYMBOL_GPL(erhai_pps_apdo_current);

/*
 * What steers the pumps: negative with the pack gauge (or while it is still
 * being looked for, when the pumps are parked anyway). Without the gauge the
 * ceiling of the PPS bus current in mA, 0 when fast charging is off.
 */
int erhai_pps_gaugeless_ma(void)
{
	guard(mutex)(&erhai_bridge_lock);
	if (!erhai_battmgr || !qcom_battmgr_gaugeless(erhai_battmgr))
		return -ENODEV;
	return min(READ_ONCE(gaugeless_pps_ma), 5000U);
}
EXPORT_SYMBOL_GPL(erhai_pps_gaugeless_ma);

/*
 * Whether a gaugeless PPS session that has charged its table through hands
 * the tail to the battery manager's FFC (gaugeless_ffc), see
 * qcom_battmgr_ffc_arm(), which checks the rest.
 */
bool erhai_gaugeless_ffc(void)
{
	guard(mutex)(&erhai_bridge_lock);
	return erhai_battmgr && qcom_battmgr_gaugeless(erhai_battmgr) &&
	       READ_ONCE(gaugeless_ffc);
}
EXPORT_SYMBOL_GPL(erhai_gaugeless_ffc);

/* Property traffic from PPS uses the existing serialized request channel. */
int erhai_otg_adsp_prop_get(u32 opcode, u32 property)
{
	u32 value;
	int ret;

	guard(mutex)(&erhai_bridge_lock);
	if (!erhai_battmgr || !erhai_battmgr->service_up)
		return -ENODEV;
	if (opcode != BATTMGR_USB_PROPERTY_GET && opcode != BATTMGR_BAT_PROPERTY_GET)
		return -EINVAL;
	ret = qcom_battmgr_get_property(erhai_battmgr, opcode, property, &value);
	return ret ?: (int)value;
}
EXPORT_SYMBOL_GPL(erhai_otg_adsp_prop_get);

int erhai_otg_adsp_usb_prop(u32 property)
{
	return erhai_otg_adsp_prop_get(BATTMGR_USB_PROPERTY_GET, property);
}
EXPORT_SYMBOL_GPL(erhai_otg_adsp_usb_prop);

int erhai_otg_adsp_prop_set(u32 opcode, u32 property, u32 value)
{
	int ret;

	guard(mutex)(&erhai_bridge_lock);
	if (!erhai_battmgr || !erhai_battmgr->service_up)
		return -ENODEV;
	/* Only the PPS handshake is exported; OTG and thresholds stay local. */
	if (opcode == BATTMGR_USB_PROPERTY_SET) {
		if (property == USB_INPUT_CURR_LIMIT && !value)
			erhai_battmgr->pps_input_used = true;
		if (property == 49 && value && erhai_battmgr->pps_input_used) {
			ret = qcom_battmgr_set_property(erhai_battmgr,
					BATTMGR_USB_PROPERTY_SET, USB_INPUT_CURR_LIMIT, U32_MAX);
			if (ret)
				return ret;
			erhai_battmgr->pps_input_used = false;
		}
		if (property != 5 && property != 34 && property != 35 &&
		    property != 39 && property != 40 && property != 49)
			return -EINVAL;
	} else if (opcode == BATTMGR_BAT_PROPERTY_SET) {
		if (property == BATT_OPLUS_CHG_EN)
			return qcom_battmgr_charge_enable(erhai_battmgr,
						 !erhai_battmgr->charge_inhibited &&
						 !erhai_battmgr->charge_policy_hold);
		if (property != BATT_CHG_CTRL_LIM && property != 46 &&
		    !(property == BATT_OPLUS_SET_PDO && value == PD_DEFAULT_MV))
			return -EINVAL;
	} else {
		return -EINVAL;
	}
	return qcom_battmgr_set_property(erhai_battmgr, opcode, property, value);
}
EXPORT_SYMBOL_GPL(erhai_otg_adsp_prop_set);

bool erhai_otg_charge_inhibited(void)
{
	guard(mutex)(&erhai_bridge_lock);
	return !erhai_battmgr || erhai_battmgr->charge_inhibited ||
	       erhai_battmgr->charge_policy_hold || erhai_battmgr->charge_error ||
	       READ_ONCE(erhai_battmgr->otg_request);
}
EXPORT_SYMBOL_GPL(erhai_otg_charge_inhibited);

/* This endpoint carries pen boost acknowledgements on the battery device. */
static bool erhai_pen_wakeup_filter(const void *data, size_t len, void *priv)
{
	return false;
}

/* Allocate on the real GLINK child, and bind the consumer's lifetime to it. */
struct pmic_glink_client *
erhai_otg_glink_client_register_sibling(struct device *consumer, u32 id,
		void (*cb)(const void *, size_t, void *),
		void (*pdr)(void *, int), void *priv)
{
	struct pmic_glink_client *client;

	guard(mutex)(&erhai_bridge_lock);
	if (!erhai_battmgr)
		return ERR_PTR(-EPROBE_DEFER);
	if (id != 0x8011)
		return ERR_PTR(-EINVAL);
	if (!device_link_add(consumer, erhai_battmgr->dev,
			     DL_FLAG_AUTOREMOVE_CONSUMER))
		return ERR_PTR(-ENOMEM);
	client = devm_pmic_glink_client_alloc(erhai_battmgr->dev, id, cb, pdr, priv);
	if (!IS_ERR(client)) {
		pmic_glink_client_set_wakeup_filter(client, erhai_pen_wakeup_filter);
		pmic_glink_client_register(client);
	}
	return client;
}
EXPORT_SYMBOL_GPL(erhai_otg_glink_client_register_sibling);

int erhai_pps_register(struct device *parent, const struct erhai_pps_ops *ops, void *data)
{
	struct qcom_battmgr *battmgr = dev_get_drvdata(parent);

	guard(mutex)(&battmgr->charge_lock);
	if (battmgr->pps_ops || battmgr->pps_parked)
		return -EBUSY;
	battmgr->pps_data = data;
	/*
	 * The pumps charge the cell directly, on the pack gauge's readings.
	 * Where the device tree names that gauge they are only let in once
	 * it has answered, see qcom_battmgr_gauge_detect(). On a board that
	 * turns out not to have it they stay parked unless gaugeless_pps_ma
	 * is set, see qcom_battmgr_gaugeless_pumps().
	 */
	if (qcom_battmgr_gauge_described(battmgr) &&
	    battmgr->gauge_state != GAUGE_PRESENT)
		battmgr->pps_parked = ops;
	else
		battmgr->pps_ops = ops;
	qcom_battmgr_charge_kick(battmgr);
	return 0;
}
EXPORT_SYMBOL_GPL(erhai_pps_register);

void erhai_pps_unregister(struct device *parent)
{
	struct qcom_battmgr *battmgr = dev_get_drvdata(parent);

	guard(mutex)(&battmgr->charge_lock);
	qcom_battmgr_pps_stop(battmgr, false, "PPS-unregister");
	battmgr->pps_ops = NULL;
	battmgr->pps_parked = NULL;
	battmgr->pps_data = NULL;
	set_bit(PD_EVENT_CAPS, &battmgr->pd_events);
	qcom_battmgr_charge_kick(battmgr);
}
EXPORT_SYMBOL_GPL(erhai_pps_unregister);

static void qcom_battmgr_charging_remove(void *data)
{
	struct platform_device *pdev = data;

	struct qcom_battmgr *b = dev_get_drvdata(pdev->dev.parent);

	/* Stop discovery before removing the observer. */
	disable_delayed_work_sync(&b->charge_work);
	if (b->partner_events.class)
		class_interface_unregister(&b->partner_events);
	platform_device_unregister(pdev);
	guard(mutex)(&erhai_bridge_lock);
	erhai_battmgr = NULL;
}

static int qcom_battmgr_charging_init(struct qcom_battmgr *battmgr)
{
	struct platform_device *pdev;
	struct platform_device_info info = {
		.parent = battmgr->dev,
		.name = "erhai-pps",
		.id = PLATFORM_DEVID_NONE,
	};

	if (!(battmgr->quirks & QCOM_BATTMGR_QUIRK_PPS))
		return 0;
	mutex_lock(&erhai_bridge_lock);
	erhai_battmgr = battmgr;
	mutex_unlock(&erhai_bridge_lock);
	pdev = platform_device_register_full(&info);
	if (IS_ERR(pdev)) {
		guard(mutex)(&erhai_bridge_lock);
		erhai_battmgr = NULL;
		return PTR_ERR(pdev);
	}
	return devm_add_action_or_reset(battmgr->dev, qcom_battmgr_charging_remove, pdev);
}

static int qcom_battmgr_probe(struct auxiliary_device *adev,
			      const struct auxiliary_device_id *id)
{
	const struct power_supply_desc *psy_desc;
	struct power_supply_config psy_cfg_supply = {};
	struct power_supply_config psy_cfg = {};
	const struct of_device_id *match;
	struct qcom_battmgr *battmgr;
	struct device *dev = &adev->dev;
	int ret;

	battmgr = devm_kzalloc_obj(dev, *battmgr);
	if (!battmgr)
		return -ENOMEM;

	battmgr->dev = dev;
	ret = devm_add_action_or_reset(dev, qcom_battmgr_put_gauge, battmgr);
	if (ret)
		return ret;
	auxiliary_set_drvdata(adev, battmgr);

	psy_cfg.drv_data = battmgr;
	psy_cfg.fwnode = dev_fwnode(&adev->dev);

	psy_cfg_supply.drv_data = battmgr;
	psy_cfg_supply.fwnode = dev_fwnode(&adev->dev);
	psy_cfg_supply.supplied_to = qcom_battmgr_battery;
	psy_cfg_supply.num_supplicants = 1;

	/*
	 * The charger firmware sends a notification on every change, and
	 * power_supply_changed() reports each one as a wakeup event, which
	 * aborts suspend-to-idle within minutes. Those updates aren't a reason
	 * to wake the system.
	 */
	psy_cfg.no_wakeup_source = true;
	psy_cfg_supply.no_wakeup_source = true;

	INIT_DELAYED_WORK(&battmgr->charge_work, qcom_battmgr_charge_worker);
	INIT_DELAYED_WORK(&battmgr->uisoc_work, qcom_battmgr_uisoc_worker);
	mutex_init(&battmgr->uisoc_lock);
	battmgr->pack = -1;
	INIT_WORK(&battmgr->enable_work, qcom_battmgr_enable_worker);
	INIT_WORK(&battmgr->otg_work, qcom_battmgr_otg_worker);
	mutex_init(&battmgr->lock);
	spin_lock_init(&battmgr->notify_lock);
	mutex_init(&battmgr->charge_lock);
	qcom_battmgr_buck_invalidate(battmgr);
	init_completion(&battmgr->ack);
	battmgr->request_opcode = U32_MAX;

	if (IS_ENABLED(CONFIG_PM_SLEEP)) {
		battmgr->charge_ws = wakeup_source_register(dev, "qcom-battmgr-charge");
		if (!battmgr->charge_ws)
			return -ENOMEM;
		ret = devm_add_action_or_reset(dev, qcom_battmgr_charge_wake_remove, battmgr);
		if (ret)
			return ret;
	}

	match = of_match_device(qcom_battmgr_of_variants, dev->parent);
	if (match)
		battmgr->variant = (unsigned long)match->data;
	else
		battmgr->variant = QCOM_BATTMGR_SM8350;

	match = of_match_device(qcom_battmgr_of_quirks, dev->parent);
	if (match)
		battmgr->quirks = (unsigned long)match->data;

	ret = qcom_battmgr_charge_control_thresholds_init(battmgr);
	if (ret < 0)
		return dev_err_probe(dev, ret,
				     "failed to init battery charge control thresholds\n");

	/* Without a configured end threshold, charge to full */
	if ((battmgr->quirks & QCOM_BATTMGR_QUIRK_CHARGE_ENABLE) &&
	    !battmgr->info.charge_ctrl_end)
		battmgr->info.charge_ctrl_end = CHARGE_CTRL_END_THR_MAX;

	if (battmgr->variant == QCOM_BATTMGR_SC8280XP ||
	    battmgr->variant == QCOM_BATTMGR_X1E80100) {
		if (battmgr->variant == QCOM_BATTMGR_X1E80100)
			psy_desc = &x1e80100_bat_psy_desc;
		else
			psy_desc = &sc8280xp_bat_psy_desc;

		battmgr->bat_psy = devm_power_supply_register(dev, psy_desc, &psy_cfg);
		if (IS_ERR(battmgr->bat_psy))
			return dev_err_probe(dev, PTR_ERR(battmgr->bat_psy),
					     "failed to register battery power supply\n");

		battmgr->ac_psy = devm_power_supply_register(dev, &sc8280xp_ac_psy_desc, &psy_cfg_supply);
		if (IS_ERR(battmgr->ac_psy))
			return dev_err_probe(dev, PTR_ERR(battmgr->ac_psy),
					     "failed to register AC power supply\n");

		battmgr->usb_psy = devm_power_supply_register(dev, &sc8280xp_usb_psy_desc, &psy_cfg_supply);
		if (IS_ERR(battmgr->usb_psy))
			return dev_err_probe(dev, PTR_ERR(battmgr->usb_psy),
					     "failed to register USB power supply\n");

		battmgr->wls_psy = devm_power_supply_register(dev, &sc8280xp_wls_psy_desc, &psy_cfg_supply);
		if (IS_ERR(battmgr->wls_psy))
			return dev_err_probe(dev, PTR_ERR(battmgr->wls_psy),
					     "failed to register wireless charing power supply\n");
	} else {
		/*
		 * The SM8350-class firmware has no BATTMGR_BAT_INFO request, so
		 * @unit is never populated from the firmware. It reports charge
		 * domain values, so set it statically here - otherwise the
		 * CHARGE_* properties are rejected with -ENODATA.
		 */
		battmgr->unit = QCOM_BATTMGR_UNIT_mAh;

		if (battmgr->variant == QCOM_BATTMGR_SM8550)
			psy_desc = &sm8550_bat_psy_desc;
		else
			psy_desc = &sm8350_bat_psy_desc;

		psy_desc = qcom_battmgr_sm8350_bat_desc(battmgr, psy_desc);
		if (!psy_desc)
			return -ENOMEM;

		battmgr->bat_psy = devm_power_supply_register(dev, psy_desc, &psy_cfg);
		if (IS_ERR(battmgr->bat_psy))
			return dev_err_probe(dev, PTR_ERR(battmgr->bat_psy),
					     "failed to register battery power supply\n");

		if (battmgr->quirks & QCOM_BATTMGR_QUIRK_OPLUS_PD)
			psy_cfg_supply.attr_grp = erhai_charge_groups;
		battmgr->usb_psy = devm_power_supply_register(dev, &sm8350_usb_psy_desc, &psy_cfg_supply);
		psy_cfg_supply.attr_grp = NULL;
		if (IS_ERR(battmgr->usb_psy))
			return dev_err_probe(dev, PTR_ERR(battmgr->usb_psy),
					     "failed to register USB power supply\n");

		battmgr->wls_psy = devm_power_supply_register(dev, &sm8350_wls_psy_desc, &psy_cfg_supply);
		if (IS_ERR(battmgr->wls_psy))
			return dev_err_probe(dev, PTR_ERR(battmgr->wls_psy),
					     "failed to register wireless charing power supply\n");
	}

	battmgr->client = devm_pmic_glink_client_alloc(dev, PMIC_GLINK_OWNER_BATTMGR,
						       qcom_battmgr_callback,
						       qcom_battmgr_pdr_notify,
						       battmgr);
	if (IS_ERR(battmgr->client))
		return PTR_ERR(battmgr->client);

	/* Quiesce every requester before the client devres is released. */
	ret = devm_add_action_or_reset(dev, qcom_battmgr_release_client, battmgr);
	if (ret)
		return ret;

	if (battmgr->quirks & QCOM_BATTMGR_QUIRK_OPLUS_PD) {
		set_bit(PD_EVENT_CAPS, &battmgr->pd_events);
		battmgr->psy_nb.notifier_call = qcom_battmgr_psy_notify;
		ret = power_supply_reg_notifier(&battmgr->psy_nb);
		if (ret)
			return ret;
		ret = devm_add_action_or_reset(dev, qcom_battmgr_unregister_psy_notifier,
					       battmgr);
		if (ret)
			return ret;
	}

	if (battmgr->quirks & QCOM_BATTMGR_QUIRK_PPS) {
		battmgr->pm_nb.notifier_call = qcom_battmgr_pm_notify;
		ret = register_pm_notifier(&battmgr->pm_nb);
		if (ret)
			return ret;

		ret = devm_add_action_or_reset(dev, qcom_battmgr_unregister_pm_notifier,
					       battmgr);
		if (ret)
			return ret;
	}

	ret = qcom_battmgr_charging_init(battmgr);
	if (ret)
		return dev_err_probe(dev, ret, "cannot create charging interface\n");

	/*
	 * After the client release action, so that the zone stops reading
	 * the battery before requests to the firmware are shut off.
	 */
	qcom_battmgr_register_thermal(battmgr);

	if (battmgr->quirks & QCOM_BATTMGR_QUIRK_OPLUS_PD) {
		ret = devm_device_init_wakeup(dev);
		if (ret)
			return ret;
	} else {
		device_set_wakeup_capable(dev, true);
	}
	pmic_glink_client_set_wakeup_filter(battmgr->client,
					    qcom_battmgr_is_charger_notification);

	pmic_glink_client_register(battmgr->client);

	return 0;
}

static void qcom_battmgr_shutdown(struct auxiliary_device *adev)
{
	struct qcom_battmgr *battmgr = auxiliary_get_drvdata(adev);

	qcom_battmgr_stop_notifications(battmgr);
	cancel_work_sync(&battmgr->enable_work);
	qcom_battmgr_cancel_work(battmgr);
}

static const struct auxiliary_device_id qcom_battmgr_id_table[] = {
	{ .name = "pmic_glink.power-supply", },
	{},
};
MODULE_DEVICE_TABLE(auxiliary, qcom_battmgr_id_table);

static struct auxiliary_driver qcom_battmgr_driver = {
	.name = "pmic_glink_power_supply",
	.probe = qcom_battmgr_probe,
	.shutdown = qcom_battmgr_shutdown,
	.id_table = qcom_battmgr_id_table,
};

module_auxiliary_driver(qcom_battmgr_driver);

MODULE_DESCRIPTION("Qualcomm PMIC GLINK battery manager driver");
MODULE_LICENSE("GPL");
