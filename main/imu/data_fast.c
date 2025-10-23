#include "imu/data_fast.h"
#include "imu/imu_icm20948_driver.h"
#include "string.h"
#include "system/time_sync.h"

#define IMU_DATA_CNT 5

typedef struct {
	imu_motion_data data;
	ts_ticker_t ticker;
} timed_read;

static struct
{
	timed_read data[IMU_DATA_CNT];
	int data_cnt;
	int data_ptr;
} data_fast = {
	.data_cnt = IMU_DATA_CNT,
	.data_ptr = 0,
};

esp_err_t biodyn_data_fast_init()
{
	return ESP_OK;
}

esp_err_t biodyn_data_fast_self_test()
{
	return ESP_OK;
}

bool biodyn_data_fast_has_error()
{
	return IMU_DATA_CNT * sizeof(timed_read) >= 500;
}

const char *biodyn_data_fast_get_error()
{
	if (IMU_DATA_CNT * sizeof(timed_read) >= 500)
	{
		return "Data greater than MTU";
	}
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
