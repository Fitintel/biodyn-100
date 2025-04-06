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
#include "nvs_flash.h"
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
#include "ble.h"

// Initializes the non-volatile storage (aka persistant)
esp_err_t biodyn_nvs_init()
{
	// Initialize NVS (non-volatile storage)
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
	{
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);
	return ret;
}

// APP ENTRY POINT
void app_main(void)
{
	esp_err_t err;

	ESP_LOGI(MAIN_TAG, "Starting %s", BIODYN_DEVICE_NAME);

	// Initialize persistant storage (nvs)
	if (err = biodyn_init_nvs())
	{
		ESP_LOGE(MAIN_TAG, "Failed to initialize non-volatile storage in %s, error code %x", __func__, err);
		return;
	}

	// Initialize bluetooth
	if (err = biodyn_ble_init())
	{
		ESP_LOGE(MAIN_TAG, "Failed to initialize Bluetooth in %s, err code %x", __func__, err);
		return;
	}

	// Set up!
	ESP_LOGI(MAIN_TAG, "Finished setup");
}
