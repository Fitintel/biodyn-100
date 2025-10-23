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
	imu_motion_data *p = data_fast.data;
	memcpy(out + (old_size * sizeof(imu_motion_data)), p, current_size * sizeof(imu_motion_data));
	memcpy(out, p, old_size * sizeof(imu_motion_data));
	*size = IMU_DATA_CNT * sizeof(imu_motion_data);
}
