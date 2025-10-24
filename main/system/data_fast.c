#include "system/data_fast.h"
#include "imu/imu_icm20948_driver.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "system/time_sync.h"

#define TAG "DATA_FAST"
#define IMU_DATA_CNT 10

typedef struct
{
	ts_ticker_t ticker;	  // ----  8 bytes
	imu_motion_data data; // -- 36 bytes
	float emg;			  // -------------  4 bytes
} timed_read;			  // TOTAL: 48 bytes aligned on 8

static struct
{
	timed_read data[IMU_DATA_CNT];
	SemaphoreHandle_t data_mutex;
	int data_cnt;
	int data_ptr;

	TaskHandle_t read_task;

	char ext_err_msg[128];
	biodyn_df_err_t ext_err;
} data_fast = {
	.data_cnt = IMU_DATA_CNT,
	.data_ptr = 0,
	.read_task = NULL,
	.data_mutex = NULL,
	.ext_err = BIODYN_DATAFAST_OK,
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
	// Check if we can transmit our data
	int data_len = IMU_DATA_CNT * sizeof(timed_read);
	ESP_LOGI(TAG, "Data len is %d from %d datapoints of size %d bytes", data_len, IMU_DATA_CNT, sizeof(timed_read));
	if (data_len >= 517)
		collect_err(BIODYN_DATAFAST_TOO_MUCH_DATA, "Data len was too great, got ", data_len);

	// Create mutex for shared resources
	data_fast.data_mutex = xSemaphoreCreateMutex();
	if (data_fast.data_mutex == NULL)
		collect_err(BIODYN_DATAFAST_COULDNT_CREATE_MUTEX, "Couldn't create mutex: ", -1);

	// Create read task
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
	ts_ticker_t read_ticker = biodyn_time_sync_get_ticker();

	// Take mutex
	if (xSemaphoreTake(data_fast.data_mutex, portMAX_DELAY))
	{
		data_fast.data_ptr += 1;
		data_fast.data_ptr %= data_fast.data_cnt;
		timed_read *datapoint = &data_fast.data[data_fast.data_ptr];
		datapoint->ticker = read_ticker;
		datapoint->emg = 0;
		biodyn_imu_icm20948_read_accel_gyro_mag(&datapoint->data);
		xSemaphoreGive(data_fast.data_mutex); // Give mutex
	}
}

void ble_data_fast_packed(uint16_t *size, void *out)
{
	// Take mutex
	if (xSemaphoreTake(data_fast.data_mutex, portMAX_DELAY))
	{
		int current_size = data_fast.data_ptr;
		int old_size = data_fast.data_cnt - data_fast.data_ptr;
		timed_read *p = &data_fast.data[0];
		memcpy(out + (old_size * sizeof(timed_read)), p, current_size * sizeof(timed_read));
		memcpy(out, p, old_size * sizeof(timed_read));
		*size = IMU_DATA_CNT * sizeof(timed_read);
		timed_read *latest = &((timed_read *)out)[IMU_DATA_CNT - 1];
		ESP_LOGI(TAG, "x: %.2f, y: %.2f, z: %.2f at %lld", latest->data.accel_x, latest->data.accel_y, latest->data.accel_z, latest->ticker);
		xSemaphoreGive(data_fast.data_mutex); // Give mutex
	}
}

static esp_err_t collect_err(biodyn_timesync_err_t err, const char *msg, esp_err_t code)
{
	data_fast.ext_err |= err;
	snprintf(data_fast.ext_err_msg, sizeof(data_fast.ext_err_msg), "%s %x", msg, code);
	ESP_LOGE(TAG, "%s %x", msg, code);
	return code;
}