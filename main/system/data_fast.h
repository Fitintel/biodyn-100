#ifndef BIODYN_DATA_FAST_H
#define BIODYN_DATA_FAST_H

#include "esp_system.h"
#include "biodyn_constants.h"

esp_err_t biodyn_data_fast_init();
esp_err_t biodyn_data_fast_self_test();
bool biodyn_data_fast_has_error();
const char *biodyn_data_fast_get_error();
void data_fast_read();

// BLE callback
void ble_data_fast_packed(uint16_t *size, void *out);
// BLE callback
void ble_data_fast_orientation_packed(uint16_t *size, void *out);

const static biodyn_system biodyn_data_fast_system = {
	.name = "Data Fast",
	.init = biodyn_data_fast_init,
	.has_error = biodyn_data_fast_has_error,
	.get_error = biodyn_data_fast_get_error,
	.self_test = biodyn_data_fast_self_test,
};

typedef uint32_t biodyn_df_err_t;
#define BIODYN_DATAFAST_OK 0x0
#define BIODYN_DATAFAST_COULDNT_CREATE_TASK 0x1
#define BIODYN_DATAFAST_TOO_MUCH_DATA 0x2
#define BIODYN_DATAFAST_COULDNT_CREATE_MUTEX 0x4
#define BIODYN_DATAFAST_RUNNING_TOO_SLOW 0x8

#endif // BIODYN_DATA_FAST_H