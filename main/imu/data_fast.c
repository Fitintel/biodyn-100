#include "imu/data_fast.h"
#include "imu/imu_icm20948_driver.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "system/time_sync.h"

#define TAG "DATA_FAST"
#define IMU_DATA_CNT 10

// Right now we have 4 bytes of padding ... PERFECT for EMG
typedef struct
{
	imu_motion_data data;
	ts_ticker_t ticker;
} timed_read;

static struct
{
	timed_read data[IMU_DATA_CNT];
	int data_cnt;
	int data_ptr;
	TaskHandle_t read_task;
	char ext_err_msg[128];
	biodyn_df_err_t ext_err;
} data_fast = {
	.data_cnt = IMU_DATA_CNT,
	.data_ptr = 0,
	.read_task = NULL,
};

static esp_err_t collect_err(biodyn_timesync_err_t err, const char *msg, esp_err_t code);

void read_task(void *usr_data)
{
	TickType_t last_wake_time = xTaskGetTickCount();
	const TickType_t period = pdMS_TO_TICKS(15);

	while (1)
	{
		data_fast_read();
		vTaskDelayUntil(&last_wake_time, period);
	}
}

esp_err_t biodyn_data_fast_init()
{
	int data_len = IMU_DATA_CNT * sizeof(timed_read);
	ESP_LOGI(TAG, "Data len is %d from %d datapoints of size %d bytes", data_len, IMU_DATA_CNT, sizeof(timed_read));
	if (data_len >= 517)
		collect_err(BIODYN_DATAFAST_TOO_MUCH_DATA, "Data len was too great, got ", data_len);

	BaseType_t res = xTaskCreate(read_task, TAG, 4096, NULL, 5, &data_fast.read_task);
	if (res != pdTRUE)
	{
		collect_err(BIODYN_DATAFAST_COULDNT_CREATE_TASK, "Couldn't create task: code", res);
		return -1;
	}

	return ESP_OK;
}

esp_err_t biodyn_data_fast_self_test()
{
	return biodyn_data_fast_has_error() ? -1 : ESP_OK;
}

bool biodyn_data_fast_has_error()
{
	return data_fast.ext_err != BIODYN_DATAFAST_OK;
}

const char *biodyn_data_fast_get_error()
{
	if (biodyn_data_fast_has_error())
		return &data_fast.ext_err_msg[0];
	return "No error";
}

void data_fast_read()
{
	uint32_t read_ticker = biodyn_time_sync_get_ticker();
	data_fast.data_ptr += 1;
	data_fast.data_ptr %= data_fast.data_cnt;
	timed_read *datapoint = &data_fast.data[data_fast.data_ptr];
	datapoint->ticker = read_ticker;
	biodyn_imu_icm20948_read_accel_gyro_mag(&datapoint->data);
}

void ble_data_fast_packed_imu(uint16_t *size, void *out)
{
	int current_size = data_fast.data_ptr;
	int old_size = data_fast.data_cnt - data_fast.data_ptr;
	timed_read *p = &data_fast.data[0];
	memcpy(out + (old_size * sizeof(timed_read)), p, current_size * sizeof(timed_read));
	memcpy(out, p, old_size * sizeof(timed_read));
	*size = IMU_DATA_CNT * sizeof(timed_read);
}

static esp_err_t collect_err(biodyn_timesync_err_t err, const char *msg, esp_err_t code)
{
	data_fast.ext_err |= err;
	snprintf(data_fast.ext_err_msg, sizeof(data_fast.ext_err_msg), "%s %x", msg, code);
	ESP_LOGE(TAG, "%s %x", msg, code);
	return code;
}