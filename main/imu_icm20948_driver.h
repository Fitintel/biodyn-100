#ifndef ICM20948_DRIVER_H
#define ICM20948_DRIVER_H

#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"

#define IMU_ACCEL_UNIT "m/s^2"
#define IMU_GYRO_UNIT "dps"

typedef struct
{
	float accel_x;
	float accel_y;
	float accel_z;
	float gyro_x;
	float gyro_y;
	float gyro_z;
	float mag_x;
	float mag_y;
	float mag_z;
} imu_motion_data;

typedef enum
{
	_gyro_250dps,
	_gyro_500dps,
	_gyro_1000dps,
	_gyro_2000dps,
} gyro_range;

typedef enum
{
	_accel_2g,
	_accel_4g,
	_accel_8g,
	_accel_16g,
} accel_range;

typedef enum
{
	_b0 = 0,
	_b1 = 1 << 4,
	_b2 = 2 << 4,
	_b3 = 3 << 4,
} user_bank_range;

struct imu_float3
{
	float x;
	float y;
	float z;
};
typedef struct imu_float3 imu_float3_t;

// i2c pins for a device
struct i2c_config
{
	uint8_t mosi;
	uint8_t miso;
	uint8_t sclk;
	uint8_t cs;
};
// i2c pins for a device
typedef struct i2c_config i2c_config_t;

// BIODYN IMU driver error code type: BIODYN_IMU_ERR_
typedef uint16_t biodyn_imu_err_t;
#define BIODYN_IMU_OK 0
#define BIODYN_IMU_ERR_COULDNT_INIT_SPI_BUS 0x1
#define BIODYN_IMU_ERR_COULDNT_INIT_SPI_DEV 0x2
#define BIODYN_IMU_ERR_COULDNT_SEND_DATA 0x4
#define BIODYN_IMU_ERR_WRONG_WHOAMI 0x8
#define BIODYN_IMU_ERR_COULDNT_CONFIGURE 0x10
#define BIODYN_IMU_ERR_COULDNT_READ 0x20

#define BIODYN_IMU_ERR_INVALID_ARGUMENT 0x5

// Initializes the IMU
biodyn_imu_err_t biodyn_imu_icm20948_init();

// Performs an IMU self-test
biodyn_imu_err_t biodyn_imu_icm20948_self_test();

// Reads both accel and gyro data
biodyn_imu_err_t biodyn_imu_icm20948_read_accel_gyro(imu_motion_data *data);

// TEMPORARY test to work out accel gyro multibyte reading
biodyn_imu_err_t self_test_accel(int16_t *out);

// Reads and returns compass data
biodyn_imu_err_t biodyn_imu_icm20948_read_magnetometer(imu_float3_t *out);

// biodyn_imu_err_t biodyn_imu_icm20948_read_user_ctrl();

biodyn_imu_err_t biodyn_imu_icm20948_read_register_test(uint8_t bank, uint16_t register_address, uint8_t *out);

#endif // ICM20948_DRIVER_H