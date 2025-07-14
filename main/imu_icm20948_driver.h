#ifndef ICM20948_DRIVER_H
#define ICM20948_DRIVER_H

#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"

#define TAG "IMU_ICM20948"

#define ACCEL_XOUT_H 0x2d
#define ACCEL_XOUT_L 0x2e
#define ACCEL_YOUT_H 0x2f
#define ACCEL_YOUT_L 0x30
#define ACCEL_ZOUT_H 0x31
#define ACCEL_ZOUT_L 0x32

#define GYRO_XOUT_H 0x33
#define GYRO_XOUT_L 0x34
#define GYRO_YOUT_H 0x35
#define GYRO_YOUT_L 0x36
#define GYRO_ZOUT_H 0x37
#define GYRO_ZOUT_L 0x38

#define WHO_AM_I 0x00
#define REG_BANK_SEL 0x7f
#define ODR_ALIGN_EN 0x09
#define PWR_MGMT_1 0x06
#define PWR_MGMT_2 0x07
#define USER_CTRL 0x03

#define ACCEL_SMPLRT_DIV_1 0x10
#define ACCEL_SMPLRT_DIV_2 0x11
#define ACCEL_CONFIG 0x14

#define GYRO_SMPLRT_DIV 0x00
#define GYRO_CONFIG_1 0x01
#define GYRO_CONFIG_2 0x02


#define READ_MSB 0x80
#define WRITE_MSB 0x00

#define ACCEL_RANGE_VALUE _accel_4g
#define GYRO_RANGE_VALUE _gyro_1000dps


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

#define BIODYN_IMU_ERR_INVALID_ARGUMENT 0x5

// Initializes the IMU
biodyn_imu_err_t biodyn_imu_icm20948_init();

// Performs an IMU self-test
biodyn_imu_err_t biodyn_imu_icm20948_self_test();

// Sets the user bank of registers
biodyn_imu_err_t select_user_bank(uint8_t bank);
// Reads the user bank of registers
biodyn_imu_err_t get_user_bank(uint8_t *bank_out);
// Writes data to a single register specified by a bank and address to the IMU
biodyn_imu_err_t write_single_register(uint8_t bank, uint16_t register_address, uint16_t write_data);
// Reads data of a single register specified by a bank and address of the IMU
biodyn_imu_err_t read_single_register(uint8_t bank, uint16_t register_address, uint16_t *out);

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
