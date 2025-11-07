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

#include "biodyn_constants.h"
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
}
