#ifndef BIODYN_NVS_H
#define BIODYN_NVS_H

#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"

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

#endif // BIODYN_NVS_H