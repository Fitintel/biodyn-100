#ifndef BIODYN_100_TIME_SYNC_H
#define BIODYN_100_TIME_SYNC_H

#include "constants.h"
#include "esp_system.h"

// TODO: STUB. Initialize module
esp_err_t biodyn_time_sync_init();

// TODO: STUB. Return time-sync ticker.
uint32_t biodyn_time_sync_get_ticker();

// TODO: STUB. Self-test module
esp_err_t biodyn_time_sync_self_test();

// TODO: STUB. Return whether there are errors.
bool time_sync_has_error();

// TODO: STUB. Retrieve error message
const char *time_sync_get_error();

const static biodyn_system biodyn_time_sync_system = {
	.name = "Time Sync",
	.init = biodyn_time_sync_init,
	.has_error = time_sync_has_error,
	.get_error = time_sync_get_error,
	.self_test = biodyn_time_sync_self_test,
};


#endif // BIODYN_100_TIME_SYNC_H