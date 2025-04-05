#ifndef FITNET_BLE_H
#define FITNET_BLE_H

#include "esp_err.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"

// Initializes the BIODYN bluetooth low energy GATTS server and GAP protocols
// Returns non-zero value on error
esp_err_t biodyn_init_ble();

// The GATTS server event handler function 
void biodyn_gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

// The GAP event handler function
void biodyn_gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);


// Biodyn ble error types
#define BIODYN_BLE_ERR_ADV_DATA_INIT 0x1 // Failed to set the advertisement data


#endif // FITNET_BLE_H