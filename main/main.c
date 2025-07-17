#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include <unistd.h>

#include "esp_system.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gatt_common_api.h"
#include "esp_timer.h"

#include "sdkconfig.h"

#include "constants.h"
#include "nvs.h"
#include "ble.h"
#include "ble_profiles.h"
#include "imu_icm20948_driver.h"

// APP ENTRY POINT
void app_main(void)
{
	esp_err_t err;

	ESP_LOGI(MAIN_TAG, "Starting %s", BIODYN_DEVICE_NAME);

	// Initialize persistant storage (nvs)
	if ((err = biodyn_nvs_init()))
	{
		ESP_LOGE(MAIN_TAG, "Failed to initialize non-volatile storage in %s, error code %x", __func__, err);
		return;
	}

	// Initialize LED
	if ((err = biodyn_led_init()))
	{
		ESP_LOGE(MAIN_TAG, "Failed to initialize LED module, err = %d", err);
		return;
	}

	// Initialize IMU
	if ((err = biodyn_imu_icm20948_init()))
	{
		ESP_LOGE(MAIN_TAG, "Failed to initialize IMU module, err = %d", err);
		return;
	}

	// Initialize bluetooth
	if ((err = biodyn_ble_init(LEN_OF_STATIC_ARRAY(profiles), &profiles[0])))
	{
		ESP_LOGE(MAIN_TAG, "Failed to initialize Bluetooth in %s, err code %x", __func__, err);
		return;
	}
	uint16_t output;
	for (;;)
	{
		// vTaskDelay(pdMS_TO_TICKS(100));
		err = self_test_accel(&output);
		if (err != BIODYN_IMU_OK)
		{
			ESP_LOGE(MAIN_TAG, "ERROR READING IMU ICM20948 DATA");
			continue;
		}
		ESP_LOGI(MAIN_TAG, "GOT X ACCEL LOW AS %d", output);
	}
}