#ifndef ICM20948_DRIVER_H
#define ICM20948_DRIVER_H

#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"

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
