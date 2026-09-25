/* SPDX-License-Identifier: MIT */

#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include "sensor_bme280.h"

/*

BME280 VCC  -> 3.3V
BME280 GND  -> GND
BME280 SDA  -> XIAO D4
BME280 SCL  -> XIAO D5

*/

LOG_MODULE_DECLARE(bthome_node, LOG_LEVEL_INF);

#if DT_HAS_COMPAT_STATUS_OKAY(bosch_bme280)
static const struct device *bme280_dev = DEVICE_DT_GET_ONE(bosch_bme280);
#else
static const struct device *bme280_dev;
#endif

static int32_t sensor_value_to_centi(const struct sensor_value *value)
{
	return value->val1 * 100 + value->val2 / 10000;
}

void sensor_bme280_update(struct bthome_v2_ctx *ctx)
{
	struct sensor_value temperature;
	struct sensor_value humidity;
	struct sensor_value pressure;
	int32_t temperature_centi = 0;
	uint32_t humidity_centi = 0;
	uint32_t pressure_chpa = 0;
	int ret;

		if (!bme280_dev ) {
		LOG_WRN("bme280_dev not initialized");
		return;
	}

	if (!bme280_dev || !device_is_ready(bme280_dev)) {
		LOG_WRN("BME280 is not ready");
		return;
	}

	ret = sensor_sample_fetch(bme280_dev);
	if (ret < 0) {
		LOG_WRN("BME280 sample failed: %d", ret);
		return;
	}

	ret = sensor_channel_get(bme280_dev, SENSOR_CHAN_AMBIENT_TEMP, &temperature);
	if (ret == 0) {
		temperature_centi = sensor_value_to_centi(&temperature);
		bthome_v2_add_temperature(ctx, (int16_t)temperature_centi);
	}

	ret = sensor_channel_get(bme280_dev, SENSOR_CHAN_HUMIDITY, &humidity);
	LOG_INF("BME280 humidity read: ret=%d val1=%d val2=%d", ret, humidity.val1, humidity.val2);
	if (ret == 0) {
		humidity_centi = (uint32_t)sensor_value_to_centi(&humidity);
		bthome_v2_add_humidity(ctx, (uint16_t)humidity_centi);
	}

	ret = sensor_channel_get(bme280_dev, SENSOR_CHAN_PRESS, &pressure);
	if (ret == 0) {
		/* Zephyr reports pressure in kPa; BThome expects 0.01 hPa. */
		pressure_chpa = (uint32_t)pressure.val1 * 1000U +
			(uint32_t)pressure.val2 / 1000U;
		bthome_v2_add_pressure(ctx, pressure_chpa);
	}

	LOG_INF("BME280: %d.%02d C, %d.%02d %%, %d.%02d hPa",
		(int)(temperature_centi / 100), (int)(temperature_centi % 100),
		(int)(humidity_centi / 100), (int)(humidity_centi % 100),
		(int)(pressure_chpa / 100), (int)(pressure_chpa % 100));
}