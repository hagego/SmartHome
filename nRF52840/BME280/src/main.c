/* SPDX-License-Identifier: MIT */
/*
 * Copyright (c) 2026 Daniel Glaser
 *
 * BThome V2 Full-Node Sample
 * ==========================
 * Combines three sensor sources into one BThome V2 BLE advertisement:
 *
 *  1. Die temperature   — nRF52840 internal temperature sensor (always present)
 *  2. 6-axis IMU        — MPU-6050 (nRF52840-DK) or LSM6DS3/LSM6DSL (XIAO Sense)
 *                         Advertises acceleration and gyroscope magnitude.
 *  3. PIR motion sensor — GPIO binary sensor (active-high, DT alias "pir-sensor")
 *
 * The device advertises a new BThome V2 packet every ADVERT_INTERVAL_SEC seconds.
 *
 * Build (nRF52840-DK with MPU-6050 breakout):
 *   west build -b nrf52840dk/nrf52840 samples/bthome_full_node
 *
 * Build (Seeed XIAO nRF52840 Sense):
 *   west build -b seeed_xiao_ble/nrf52840 samples/bthome_full_node
 *
 * Verify with bthome-logger:
 *   uv tool install bthome-logger && bthome-logger -f "MAKE"
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/drivers/gpio.h>

#include <bthome_v2/bthome_v2.h>

#include "sensor_bme280.h"
#include "sensor_pir.h"

LOG_MODULE_REGISTER(bthome_node, LOG_LEVEL_INF);

/* ── Device handles for suspend/resume ──────────────────────────────────── */
/*
 * On the Seeed XIAO nRF52840 Sense the IMU hangs on i2c0.
 * Suspending the child device first, then the bus saves the I2C pull-up
 * current and stops the LSM6DS3TR-C sampling (~0.9 mA in normal mode).
 * On boards without these nodes the DT macros resolve to nothing and the
 * pm_suspend/resume helpers compile away to empty stubs.
 */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(lsm6ds3tr_c), okay)
#define HAS_IMU_PM 1
static const struct device *imu_dev =
	DEVICE_DT_GET(DT_NODELABEL(lsm6ds3tr_c));
#else
#define HAS_IMU_PM 0
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c0), okay)
#define HAS_I2C_PM 1
static const struct device *i2c_dev =
	DEVICE_DT_GET(DT_NODELABEL(i2c0));
#else
#define HAS_I2C_PM 0
#endif

/* ── Timing ──────────────────────────────────────────────────────────────── */
#define ADVERT_INTERVAL_SEC  30

/**
 * How long to advertise per cycle ("burst" window).
 *
 * The BLE controller holds the 16 MHz HFCLK active as long as advertising
 * is running.  Stopping advertising with bt_le_adv_stop() releases the
 * HFCLK request completely, allowing the nRF52840 to drop to ~3-8 µA
 * in System ON / CPU asleep mode.
 *
 * 3 seconds is enough for Home Assistant to receive several packets
 * (at slow interval ~1 s) before the device goes back to sleep.
 */
#define ADV_BURST_SEC         3

/* ── BLE advertising parameters ─────────────────────────────────────────── */
static const struct bt_le_adv_param adv_param =
	BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_USE_IDENTITY,
			     BT_GAP_ADV_SLOW_INT_MIN,
			     BT_GAP_ADV_SLOW_INT_MAX,
			     NULL);

/* ── BThome V2 context ───────────────────────────────────────────────────── */
static struct bthome_v2_ctx bthome;

/* ── Advertising data: Flags + BThome service data ─────────────────────── */
/* The legacy advertisement is limited to 31 bytes. The name is placed in
 * the scan response so the full BThomeENOTSU service data can fit. */
static struct bt_data ad[2];
static struct bt_data scan_rsp[1];

/* ── Packet counter (incremented each advertisement) ────────────────────── */
static uint8_t pkt_id;

/* ── LED2 (alias "led1" on nRF52840-DK) ─────────────────────────────────── */
#if DT_NODE_HAS_STATUS(DT_ALIAS(led1), okay)
static const struct gpio_dt_spec led2 =
	GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
#define HAS_LED2 1
#else
#define HAS_LED2 0
#endif

/* ── PPK2 active-indicator (P0.02 / D0) ────────────────────────────────── */
/* HIGH = CPU awake, LOW = inside k_sleep(). Connect to PPK2 DI channel. */
#if DT_NODE_HAS_STATUS(DT_ALIAS(active_indicator), okay)
static const struct gpio_dt_spec active_ind =
	GPIO_DT_SPEC_GET(DT_ALIAS(active_indicator), gpios);
#define HAS_ACTIVE_IND 1
#else
#define HAS_ACTIVE_IND 0
#endif

/* ── Encode sensor data into ad[2] ──────────────────────────────────────── */
/*
 * Does NOT call bt_le_adv_update_data() — advertising is started fresh
 * each cycle via bt_le_adv_start(), so updating while running is not needed.
 */
static void encode_advertisement(void)
{
	int ret;

#if HAS_LED2
	gpio_pin_set_dt(&led2, 1);
#endif

	bthome_v2_clear(&bthome);
	bthome_v2_add_packet_id(&bthome, pkt_id++);

	sensor_pir_update(&bthome);
	sensor_bme280_update(&bthome);

	ret = bthome_v2_encode(&bthome);
	if (ret < 0) {
		LOG_ERR("BThome encode failed: %d", ret);
		goto out;
	}

	ret = bthome_v2_get_bt_data(&bthome, &ad[1]);
	if (ret < 0) {
		LOG_ERR("bthome_v2_get_bt_data failed: %d", ret);
	}

out:
#if HAS_LED2
	k_sleep(K_MSEC(50));
	gpio_pin_set_dt(&led2, 0);
#else
	(void)ret;
#endif
}

/* ── Sensor read & advertise ─────────────────────────────────────────────── */

static void update_advertisement(void)
{
	int ret;

#if HAS_LED2
	gpio_pin_set_dt(&led2, 1);
#endif

	bthome_v2_clear(&bthome);
	bthome_v2_add_packet_id(&bthome, pkt_id++);

	sensor_pir_update(&bthome);
	sensor_bme280_update(&bthome);

	/* ── Encode & update advertising ─────────────────────────────────── */
	ret = bthome_v2_encode(&bthome);
	if (ret < 0) {
		LOG_ERR("BThome encode failed: %d", ret);
		return;
	}

	ret = bthome_v2_get_bt_data(&bthome, &ad[1]);
	if (ret < 0) {
		LOG_ERR("bthome_v2_get_bt_data failed: %d", ret);
		return;
	}

	ret = bt_le_adv_update_data(ad, ARRAY_SIZE(ad), NULL, 0);
	if (ret < 0) {
		LOG_ERR("BLE adv update failed: %d", ret);
	}

#if HAS_LED2
	k_sleep(K_MSEC(50));
	gpio_pin_set_dt(&led2, 0);
#endif
}

/* ── Main ────────────────────────────────────────────────────────────────── */

int main1(void)
{
	int ret;

	LOG_INF("BThome V2 Full-Node starting");

	/* ── Initialise LED2 ────────────────────────────────────────────── */
#if HAS_LED2
	if (gpio_is_ready_dt(&led2)) {
		gpio_pin_configure_dt(&led2, GPIO_OUTPUT_INACTIVE);
	}
#endif

	/* ── Initialise sensors ─────────────────────────────────────────── */
	sensor_pir_init();

	/* ── Initialise BThome V2 context ───────────────────────────────── */
	bthome_v2_init(&bthome, false, false);

	/* ── Build initial advertising data ────────────────────────────── */
	ad[0] = (struct bt_data)
		BT_DATA_BYTES(BT_DATA_FLAGS,
			      BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR);
	scan_rsp[0] = (struct bt_data)
		BT_DATA(BT_DATA_NAME_COMPLETE,
			CONFIG_BT_DEVICE_NAME,
			sizeof(CONFIG_BT_DEVICE_NAME) - 1);

	bthome_v2_add_packet_id(&bthome, 0);
	bthome_v2_encode(&bthome);
	bthome_v2_get_bt_data(&bthome, &ad[1]);
	bthome_v2_clear(&bthome);

	/* ── Enable Bluetooth ───────────────────────────────────────────── */
	ret = bt_enable(NULL);
	if (ret) {
		LOG_ERR("bt_enable failed: %d", ret);
		return ret;
	}
	LOG_INF("Bluetooth enabled");

	/* ── Start advertising ──────────────────────────────────────────── */
	ret = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad),
			      scan_rsp, ARRAY_SIZE(scan_rsp));
	if (ret) {
		LOG_ERR("bt_le_adv_start failed: %d", ret);
		return ret;
	}
	LOG_INF("Advertising started (interval ~%d ms)",
		BT_GAP_ADV_SLOW_INT_MIN * 625 / 1000);

	/* ── Main loop ──────────────────────────────────────────────────── */
	while (true) {
		update_advertisement();
		k_sleep(K_SECONDS(ADVERT_INTERVAL_SEC));
	}

	return 0;
}

int main(void)
{
	int ret;

	LOG_INF("BThome V2 Tutorial 2 (PM) starting");

	/* ── Initialise LED2 ────────────────────────────────────────────── */
#if HAS_LED2
	if (gpio_is_ready_dt(&led2)) {
		gpio_pin_configure_dt(&led2, GPIO_OUTPUT_INACTIVE);
	}
#endif
	/* ── Initialise active-indicator pin (P0.02 / D0) ─────────────────── */
#if HAS_ACTIVE_IND
	if (gpio_is_ready_dt(&active_ind)) {
		gpio_pin_configure_dt(&active_ind, GPIO_OUTPUT_INACTIVE);
	}
#endif
	/* ── Initialise sensors ─────────────────────────────────────────── */
	sensor_pir_init();

	/* ── Initialise BThome V2 context ───────────────────────────────── */
	bthome_v2_init(&bthome, false, false);

	/* ── Build static advertising header (Flags + Name) ────────────── */
	ad[0] = (struct bt_data)
		BT_DATA_BYTES(BT_DATA_FLAGS,
			      BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR);
	scan_rsp[0] = (struct bt_data)
		BT_DATA(BT_DATA_NAME_COMPLETE,
			CONFIG_BT_DEVICE_NAME,
			sizeof(CONFIG_BT_DEVICE_NAME) - 1);

	/* ── Enable Bluetooth ───────────────────────────────────────────── */
	ret = bt_enable(NULL);
	if (ret) {
		LOG_ERR("bt_enable failed: %d", ret);
		return ret;
	}
	LOG_INF("Bluetooth enabled");

	/* Active-indicator HIGH: system operational */
#if HAS_ACTIVE_IND
	gpio_pin_set_dt(&active_ind, 1);
#endif

	/* ── Main loop ──────────────────────────────────────────────────── */
	while (true) {
		/*
		 * ── Step 1: measure + encode ───────────────────────────── *
		 * Resume I2C/IMU first (no-op on boards without them).
		 */
#if HAS_I2C_PM
		pm_device_action_run(i2c_dev, PM_DEVICE_ACTION_RESUME);
#endif
#if HAS_IMU_PM
		pm_device_action_run(imu_dev, PM_DEVICE_ACTION_RESUME);
#endif

		encode_advertisement();

		/*
		 * ── Step 2: burst advertising ──────────────────────────── *
		 * Start advertising with fresh payload, wait ADV_BURST_SEC,
		 * then stop.  bt_disable() below is what actually releases
		 * the HFCLK request held by the BLE controller.
		 */
		ret = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), scan_rsp, ARRAY_SIZE(scan_rsp));
		if (ret) {
			LOG_ERR("bt_le_adv_start failed: %d", ret);
		}

		LOG_INF("Advertising burst %d s...", ADV_BURST_SEC);
		k_sleep(K_SECONDS(ADV_BURST_SEC));

		bt_le_adv_stop();

		/*
		 * bt_disable() is the ONLY way to make the SDC release its
		 * HFCLK request.  bt_le_adv_stop() alone is not enough —
		 * the BLE controller keeps the 16 MHz HFINT oscillator running
		 * as long as bt_enable() is active, drawing ~640 µA even with
		 * the CPU in WFI.  bt_disable() synchronously tears down the
		 * controller and releases HFCLK before returning.
		 *
		 * bt_enable(NULL) at the end of the loop re-initialises the
		 * controller for the next advertising burst (~10-50 ms).
		 */
		bt_disable();

		/*
		 * ── Step 3: deep sleep ──────────────────────────────────── *
		 * Suspend peripherals, drive indicator LOW, sleep.
		 * With HFCLK released and CPU in WFI the nRF52840 draws
		 * ~3-15 µA (LFRC oscillator keep-alive only).
		 */
#if HAS_IMU_PM
		pm_device_action_run(imu_dev, PM_DEVICE_ACTION_SUSPEND);
#endif
#if HAS_I2C_PM
		pm_device_action_run(i2c_dev, PM_DEVICE_ACTION_SUSPEND);
#endif

#if HAS_ACTIVE_IND
		gpio_pin_set_dt(&active_ind, 0);
#endif

		LOG_INF("Sleeping %d s...", ADVERT_INTERVAL_SEC - ADV_BURST_SEC);
		k_sleep(K_SECONDS(ADVERT_INTERVAL_SEC - ADV_BURST_SEC));

#if HAS_ACTIVE_IND
		gpio_pin_set_dt(&active_ind, 1);
#endif

		/* Re-enable BT controller for next cycle.
		 * NULL callback = block until ready (~10-50 ms). */
		ret = bt_enable(NULL);
		if (ret) {
			LOG_ERR("bt_enable (re-init) failed: %d", ret);
		}
	}

	return 0;
}

