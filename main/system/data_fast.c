#include "system/data_fast.h"
#include "imu/imu_icm20948_driver.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "system/time_sync.h"

#define TAG "DATA_FAST"
#define IMU_DATA_CNT 7

// A timed IMU + EMG read
typedef struct
{
	ts_ticker_t ticker;		// 8 bytes
	imu_motion_data data;	// 36 bytes
	float emg;				// 4 bytes
	quaternion orientation; // 16 bytes
} timed_read;				// TOTAL: 64 bytes aligned on 8

// A timed orientation of the device
typedef struct
{
	ts_ticker_t ticker;
	quaternion orientation;
} timed_orientation;

static struct
{
	// Rolling data storage
	timed_read raw_data[IMU_DATA_CNT];
	SemaphoreHandle_t data_mutex;
	int data_cnt;
	int data_ptr;

	TaskHandle_t read_task;

	// Error things
	char ext_err_msg[128];
	biodyn_df_err_t ext_err;
	double max_read_delay_before_err;

	// Mahony fusion things
	float3 gyro_bias;
	quaternion curr_orientation;
	float ki;
	float kp;
} data_fast = {
	.data_cnt = IMU_DATA_CNT,
	.data_ptr = 0,
	.read_task = NULL,
	.data_mutex = NULL,
	.ext_err = BIODYN_DATAFAST_OK,
	.max_read_delay_before_err = 15,
	.gyro_bias = {0, 0, 0},
	.curr_orientation = {1, 0, 0, 0},
	.ki = 0.1f,
	.kp = 2.0f,
};

static esp_err_t collect_err(biodyn_timesync_err_t err, const char *msg, esp_err_t code);
static quaternion mahony_fusion(const imu_motion_data *data, quaternion orientation, float dt);

void read_task(void *usr_data)
{
	TickType_t last_wake_time = xTaskGetTickCount();
	// This is the fastest we can go without gptimer or increasing tick rate
	const TickType_t period = pdMS_TO_TICKS(10);

	for (;;)
	{
		ts_ticker_t tick_before = biodyn_time_sync_get_ticker();
		data_fast_read();
		vTaskDelayUntil(&last_wake_time, period);

		// Ensure we're reading fast enough
		ts_ticker_t tick_after = biodyn_time_sync_get_ticker();
		double read_time_ms = ts_ticker_t_to_ms(tick_after - tick_before);
		if (read_time_ms > data_fast.max_read_delay_before_err)
		{
			data_fast.ext_err |= BIODYN_DATAFAST_RUNNING_TOO_SLOW;
			snprintf(data_fast.ext_err_msg, sizeof(data_fast.ext_err_msg),
					 "DataFast trying to run at %.1lfms, instead is %.1lfms",
					 data_fast.max_read_delay_before_err, read_time_ms);
		}
		else
		{
			data_fast.ext_err &= ~BIODYN_DATAFAST_RUNNING_TOO_SLOW;
		}
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

// Read data from IMU, store it, and calculate orientation
void data_fast_read()
{
	ts_ticker_t read_ticker = biodyn_time_sync_get_ticker();

	// Read local before taking mutex
	imu_motion_data data;
	biodyn_imu_icm20948_read_accel_gyro_mag(&data);

	// Take mutex
	if (xSemaphoreTake(data_fast.data_mutex, portMAX_DELAY))
	{
		float elapsed = fmin(ts_ticker_t_to_ms(read_ticker - data_fast.raw_data[data_fast.data_ptr].ticker) / 1000.0f, 0.1f);
		data_fast.data_ptr += 1;
		data_fast.data_ptr %= data_fast.data_cnt;
		timed_read *datapoint = &data_fast.raw_data[data_fast.data_ptr];
		datapoint->ticker = read_ticker;
		datapoint->data = data;
		datapoint->emg = 0;
		datapoint->orientation = mahony_fusion(&data, data_fast.curr_orientation, elapsed);
		data_fast.curr_orientation = datapoint->orientation;

		xSemaphoreGive(data_fast.data_mutex); // Give mutex
	}
}

// Bluetooth callback to get packed raw data
void ble_data_fast_packed(uint16_t *size, void *out)
{
	// Take mutex
	if (xSemaphoreTake(data_fast.data_mutex, portMAX_DELAY))
	{
		int current_size = data_fast.data_ptr;
		int old_size = data_fast.data_cnt - data_fast.data_ptr;
		timed_read *p = &data_fast.raw_data[0];
		memcpy(out + (old_size * sizeof(timed_read)), p, current_size * sizeof(timed_read));
		memcpy(out, p, old_size * sizeof(timed_read));
		*size = IMU_DATA_CNT * sizeof(timed_read);
		// timed_read *latest = &((timed_read *)out)[IMU_DATA_CNT - 1];
		// ESP_LOGI(TAG, "x: %.2f, y: %.2f, z: %.2f at %lld", latest->data.accel_x, latest->data.accel_y, latest->data.accel_z, latest->ticker);
		xSemaphoreGive(data_fast.data_mutex); // Give mutex
	}
}

static quaternion mahony_fusion(const imu_motion_data *data, quaternion q, float dt_s)
{
	// Convert gyro to radians
	float3 gyro = data->gyro;
	deg_to_rad_f3(&gyro);

	// Normalize accelerometer measurement
	float3 accel = data->accel;
	normalize_f3(&accel);

	// What should gravity look like?
	float3 gravity = {
		2 * (q.x * q.z - q.w * q.y),
		2 * (q.w * q.x + q.y * q.z),
		q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z};

	// Compute error between measured and expected gravity
	float3 error = cross_f3(&accel, &gravity);

	// Only add mag correction if mag isn't going crazy
	if (len_f3(&data->mag) < 125.f)
	{
		// Normalize mag measurement
		float3 mag = data->mag;
		normalize_f3(&mag);
		// Rotate mag to earth frame
		float3 mag_earth = rotate_f3_by_quat(&mag, &q);
		// Get reference mag
		float3 mag_ref = {
			sqrtf(mag_earth.x * mag_earth.x + mag_earth.y * mag_earth.y),
			0,
			mag_earth.z};
		// Compute error between measured and expected mag
		float3 mag_error = cross_f3(&mag, &mag_ref);
		// Add to total error
		error.x += mag_error.x;
		error.y += mag_error.y;
		error.z += mag_error.z;
	}

	// Keep internal bias
	float ki = data_fast.ki;
	data_fast.gyro_bias.x += ki * error.x * dt_s;
	data_fast.gyro_bias.y += ki * error.y * dt_s;
	data_fast.gyro_bias.z += ki * error.z * dt_s;

	// Adjust gyro with error
	float kp = data_fast.kp;
	float3 gyro_corr = {
		gyro.x + kp * error.x + data_fast.gyro_bias.x,
		gyro.y + kp * error.y + data_fast.gyro_bias.y,
		gyro.z + kp * error.z + data_fast.gyro_bias.z};

	// Don't actually correct if accel is too much (>1g)
	if (len_f3(&data->accel) > 11.f)
		gyro_corr = gyro;

	// Update orientation quaternion, integrating current gyro
	quaternion dq = {
		-0.5 * (q.x * gyro_corr.x + q.y * gyro_corr.y + q.z * gyro_corr.z),
		0.5 * (q.w * gyro_corr.x + q.y * gyro_corr.z - q.z * gyro_corr.y),
		0.5 * (q.w * gyro_corr.y - q.x * gyro_corr.z + q.z * gyro_corr.x),
		0.5 * (q.w * gyro_corr.z + q.x * gyro_corr.y - q.y * gyro_corr.x)};
	quaternion q_next = {
		q.w + dq.w * dt_s,
		q.x + dq.x * dt_s,
		q.y + dq.y * dt_s,
		q.z + dq.z * dt_s};

	// Normalize quaternion
	normalize_quat(&q_next);

	return q_next;
}

static esp_err_t collect_err(biodyn_df_err_t err, const char *msg, esp_err_t code)
{
	data_fast.ext_err |= err;
	snprintf(data_fast.ext_err_msg, sizeof(data_fast.ext_err_msg), "%s %x", msg, code);
	ESP_LOGE(TAG, "%s %x", msg, code);
	return code;
}
