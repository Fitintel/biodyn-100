#ifndef BIODYN_DATA_FAST_H
#define BIODYN_DATA_FAST_H

#include "esp_system.h"
#include "constants.h"


esp_err_t biodyn_data_fast_init();
esp_err_t biodyn_data_fast_self_test();
bool biodyn_data_fast_has_error();
const char *biodyn_data_fast_get_error();
void data_fast_read();

// BLE callback
void ble_data_fast_packed_imu(uint16_t *size, void *out);

const static biodyn_system biodyn_data_fast_system = {
	.name = "Data Fast",
	.init = biodyn_data_fast_init,
	.has_error = biodyn_data_fast_has_error,
	.get_error = biodyn_data_fast_get_error,
	.self_test = biodyn_data_fast_self_test,
};

#endif	// BIODYN_DATA_FAST_H