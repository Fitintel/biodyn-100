#include "imu/data_fast.h"
#include "imu/imu_icm20948_driver.h"
#include "string.h"

#define IMU_DATA_CNT 13

static struct
{
	imu_motion_data data[IMU_DATA_CNT];
	int data_cnt;
	int data_ptr;
} data_fast = {
	.data_cnt = IMU_DATA_CNT,
	.data_ptr = 0,
};

void data_fast_read()
{
	data_fast.data_ptr += 1;
	data_fast.data_ptr %= data_fast.data_cnt;
	biodyn_imu_icm20948_read_accel_gyro_mag(&data_fast.data[data_fast.data_ptr]);
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
