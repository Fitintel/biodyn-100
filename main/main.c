#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

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
#include "temperature.h"

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
//	if ((err = biodyn_led_init()))
//	{
//		ESP_LOGE(MAIN_TAG, "Failed to initialize LED module, err = %d", err);
//		return;
//	}

	// Initialize bluetooth
	if ((err = biodyn_ble_init(LEN_OF_STATIC_ARRAY(profiles), &profiles[0])))
	{
		ESP_LOGE(MAIN_TAG, "Failed to initialize Bluetooth in %s, err code %x", __func__, err);
		return;
	}

	// Initialize temperature
	if ((err = biodyn_temperature_init()))
	{
		ESP_LOGE(MAIN_TAG, "Failed to initialize Temperature Driver in %s, err code %x", __func__, err);
		return;
	}

	// Set up!
	ESP_LOGI(MAIN_TAG, "Finished setup");




	while(1){
		int voltage = biodyn_temperature_read_voltage_mv();
		ESP_LOGI(MAIN_TAG, "Read voltage: %d", voltage);

		vTaskDelay(pdMS_TO_TICKS(500));
	}
}
