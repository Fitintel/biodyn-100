#ifndef FITNET_BLE_H
#define FITNET_BLE_H

#include "esp_err.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"

typedef uint16_t biodyn_ble_err_t;

// Initializes the BIODYN bluetooth low energy GATTS server and GAP protocols
// Returns non-zero value on error
// This is all a client needs to call
esp_err_t biodyn_ble_init();

// Returns any accumulated errors in the biodyn bluetooth driver
biodyn_ble_err_t biodyn_ble_check_err();

// Biodyn BLE error types
#define BIODYN_BLE_ERR_ADV_DATA_INIT 0x1 // Failed to set the advertisement data
#define BIODYN_BLE_ERR_CANT_NAME 0x2 // Failed to name device
#define BIODYN_BLE_ERR_CANT_CREATE_SERVICE 0x4 // Failed to create service
#define BIODYN_BLE_ERR_CANT_FIND_SERVICE 0x8 // Can't find a service from ID
#define BIODYN_BLE_ERR_CANT_START_SERVICE 0x10 // Can't start a service
#define BIODYN_BLE_ERR_CHARS_MISCONFIGURED 0x20 // Characteristics misconfigured
#define BIODYN_BLE_ERR_CANT_CREATE_CHAR 0x40 // Couldn't create characteristic


#endif // FITNET_BLE_H