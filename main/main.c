#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/event_groups.h"
// #include "freertos/semphr.h"
#include <unistd.h>

#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "sdkconfig.h"

#include "constants.h"
#include "system/nvs.h"
#include "bluetooth/ble.h"
#include "bluetooth/ble_profiles.h"
#include "biodyn_systems.h"
#include "system/data_fast.h"

// APP ENTRY POINT
void app_main(void)
{
	esp_err_t err;

	ESP_LOGI(MAIN_TAG, "Starting %s", BIODYN_DEVICE_NAME);

	// Initialize persistant storage (nvs)
	if ((err = biodyn_nvs_init()))
	{
		ESP_LOGE(MAIN_TAG, "FATAL: Failed to initialize non-volatile storage in %s, error code %x", __func__, err);
		return; // Fatal
	}

	// Initialize self-testing
	if ((err = biodyn_self_test_init()))
	{
		ESP_LOGE(MAIN_TAG, "FATAL: Failed to initialize self-test module, error code %x", err);
		return; // Fatal
	}

	// Initialize subsystems
	for (int i = 0; i < n_biodyn_systems; ++i)
	{
		const biodyn_system *system = &biodyn_systems[i];
		if ((err = system->init()) != ESP_OK)
			ESP_LOGE(MAIN_TAG, "Subsystem %s failed during initialization: err code %x", system->name, err);
		system->self_test();
		if (system->has_error())
			ESP_LOGE(MAIN_TAG, "Failed to initialize subsystem %s: \"%s\"", system->name, system->get_error());
	}

	// Initialize bluetooth
	if ((err = biodyn_ble_init(LEN_OF_STATIC_ARRAY(profiles), &profiles[0])))
	{
		ESP_LOGE(MAIN_TAG, "FATAL: Failed to initialize Bluetooth in %s, err code %x", __func__, err);
		return; // Fatal
	}

	// Set up!
	ESP_LOGI(MAIN_TAG, "Finished setup");

	//test_accel_gyro_mag_imu_icm20948();
}

void test_accel_gyro_mag_imu_icm20948()
{
	imu_motion_data data = {0};
	for (int i = 0; i < 2000; ++i)
	{
		biodyn_imu_icm20948_read_accel_gyro_mag(&data);
		ESP_LOGI(MAIN_TAG, "ACCEL X: %.6f %s", data.accel_x, IMU_ACCEL_UNIT);
		ESP_LOGI(MAIN_TAG, "ACCEL Y: %.6f %s", data.accel_y, IMU_ACCEL_UNIT);
		ESP_LOGI(MAIN_TAG, "ACCEL Z: %.6f %s", data.accel_z, IMU_ACCEL_UNIT);
		ESP_LOGI(MAIN_TAG, "GYRO X: %.6f %s", data.gyro_x, IMU_GYRO_UNIT);
		ESP_LOGI(MAIN_TAG, "GYRO Y: %.6f %s", data.gyro_y, IMU_GYRO_UNIT);
		ESP_LOGI(MAIN_TAG, "GYRO Z: %.6f %s", data.gyro_z, IMU_GYRO_UNIT);
		ESP_LOGI(MAIN_TAG, "MAG X: %.6f %s", data.mag_x, IMU_MAG_UNIT);
		ESP_LOGI(MAIN_TAG, "MAG Y: %.6f %s", data.mag_y, IMU_MAG_UNIT);
		ESP_LOGI(MAIN_TAG, "MAG Z: %.6f %s", data.mag_z, IMU_MAG_UNIT);
		vTaskDelay(pdMS_TO_TICKS(100)); // Sleep for 100 ms (0.1 second)
	}
}
