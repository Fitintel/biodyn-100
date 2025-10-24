#ifndef BIODYN_100_TIME_SYNC_H
#define BIODYN_100_TIME_SYNC_H

#include "constants.h"
#include "esp_system.h"

typedef uint64_t ts_ticker_t;

esp_err_t biodyn_time_sync_init();
ts_ticker_t biodyn_time_sync_get_ticker();
esp_err_t biodyn_time_sync_self_test();
bool time_sync_has_error();
const char *time_sync_get_error();

// BLE callbacks
void ble_time_sync_ticker_read(uint16_t *size, void *out);
void ble_time_sync_ticker_write(uint16_t size, void *src);
void ble_time_sync_rtt_read(uint16_t *size, void *out);
void ble_time_sync_rtt_write(uint16_t size, void *src);

const static biodyn_system biodyn_time_sync_system = {
	.name = "Time Sync",
	.init = biodyn_time_sync_init,
	.has_error = time_sync_has_error,
	.get_error = time_sync_get_error,
	.self_test = biodyn_time_sync_self_test,
};

typedef uint32_t biodyn_timesync_err_t;
#define BIODYN_TIMESYNC_OK 0x0
#define BIODYN_TIMESYNC_COULDNT_ADD_ISR 0x1
#define BIODYN_TIMESYNC_COULDNT_ENABLE_TIMER 0x2
#define BIODYN_TIMESYNC_COULDNT_START_TIMER 0x4
#define BIODYN_TIMESYNC_COULDNT_CREATE_TASK 0x8
#define BIODYN_TIMESYNC_COULDNT_CREATE_TIMER 0x10
#define BIODYN_TIMESYNC_TIMER_NOT_INCREASING 0x20
#define BIODYN_TIMESYNC_COUDLNT_READ_TIMER 0x40
#define BIODYN_TIMESYNC_TOO_LITTLE_DATA_WRIT 0x80
#define BIODYN_TIMESYNC_COUDLNT_WRITE_TIMER 0x100

#endif // BIODYN_100_TIME_SYNC_H