#ifndef ICM20948_DRIVER_H
#define ICM20948_DRIVER_H

#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"

#define TAG "IMU_ICM20948"

typedef struct {
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
	int16_t mag_x;
	int16_t mag_y;
	int16_t mag_z;
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

struct imu_int_16
{
	int16_t x;
	int16_t y;
	int16_t z;
};
typedef struct imu_int_16 imu_int_16_3_t; //for testing

// i2c pins for a device
struct i2c_config {
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


// Reads and returns gyro data
biodyn_imu_err_t biodyn_imu_icm20948_read_gyro(imu_float3_t *out);

// Reads and returns accelerometer data
biodyn_imu_err_t biodyn_imu_icm20948_read_accel(imu_int_16_3_t *out);

// reads and returns compass data
biodyn_imu_err_t biodyn_imu_icm20948_read_compass(imu_float3_t *out);

biodyn_imu_err_t biodyn_imu_icm20948_read_user_ctrl();

biodyn_imu_err_t biodyn_imu_icm20948_read_z_accel();
biodyn_imu_err_t biodyn_imu_icm20948_read_y_accel();
biodyn_imu_err_t biodyn_imu_icm20948_read_x_accel();
biodyn_imu_err_t biodyn_imu_icm20948_run_accel_test();


#endif // ICM20948_DRIVER_H
