#ifndef BIODYN_CONSTANTS_H
#define BIODYN_CONSTANTS_H

#include "esp_system.h"
#include "sdkconfig.h"

// Generic constants
#define BIODYN_DEVICE_NAME "BIODYN-100"

#define BIODYN_MANUFACTURER_NAME "FITNET"
#define BIODYN_HARDWARE_VERSION "v0"
#define BIODYN_MODEL_NUMBER "0001"
#define BIODYN_SERIAL_NUMBER "000T"
#define BIODYN_FIRMWARE_VERSION "0.0.5"
#define BIODYN_SYSTEM_ID "Alpha"

// LOG TAGS
#define MAIN_TAG "MAIN"
#define BIODYN_BLE_TAG "BLE"

// Utility defines
#define LEN_OF_STATIC_ARRAY(static_array) (sizeof(static_array) / sizeof(static_array[0]))

// Generic systems
typedef struct biodyn_system
{
	const char *name;
	esp_err_t (*init)();
	bool (*has_error)();
	const char *(*get_error)();
	esp_err_t (*self_test)();
} biodyn_system;

#endif // BIODYN_CONSTANTS_H