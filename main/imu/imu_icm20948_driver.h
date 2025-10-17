#ifndef ICM20948_DRIVER_H
#define ICM20948_DRIVER_H

#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "constants.h"

#define IMU_ACCEL_UNIT "m/s^2"
#define IMU_GYRO_UNIT "dps"
#define IMU_MAG_UNIT "uT"

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

/**
 * SPI pins for a device
 * MOSI: data out (master out slave in)
 * MISO: data in (master in slave out)
 * SCLK: slave clock
 * CS: Chip/slave select
 */
struct i2c_config
{
	uint8_t mosi;
	uint8_t miso;
	uint8_t sclk;
	uint8_t cs;
};
// SPI/I2C pins for a device
typedef struct i2c_config i2c_config_t;

// BIODYN IMU driver error code type: BIODYN_IMU_ERR_
typedef esp_err_t biodyn_imu_err_t;
// NOTICE: new errors added here must also be updated in biodyn_imu_icm20948_add_error_to_subsystem() function.
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

// -----------------------------BLUETOOTH HANDLE FUNCTIONS-----------------------------
void biodyn_imu_icm20948_read_accel(uint16_t *size, void *out);

void biodyn_imu_icm20948_read_gyro(uint16_t *size, void *out);

void biodyn_imu_icm20948_read_mag(uint16_t *size, void *out);

void biodyn_imu_icm20948_read_all(uint16_t *size, void *out);
// -----------------------------BLUETOOTH HANDLE FUNCTIONS-----------------------------

biodyn_imu_err_t biodyn_imu_icm20948_read_accel_gyro_mag(imu_motion_data *data);

// Tests register reading capability
biodyn_imu_err_t biodyn_imu_icm20948_read_register_test(uint8_t bank, uint16_t register_address, uint8_t *out);

// Identifies if the IMU has returned an error
bool biodyn_imu_icm20948_has_error();

// Retrieves most recent error returned by the IMU. If no error is present, returns an empty string.
const char *biodyn_imu_icm20948_get_error();

// TODO: should subsystem have a method to get all errors? add to imu system?
// char **biodyn_imu_icm20948_get_all_errors();
const static biodyn_system biodyn_imu_system = {
	.name = "IMU",
	.init = biodyn_imu_icm20948_init,
	.self_test = biodyn_imu_icm20948_self_test,
	.has_error = biodyn_imu_icm20948_has_error,
	.get_error = biodyn_imu_icm20948_get_error,
};

// TODO: What other types of calls do we want by our app via bluetooth?
// imu motion data
// configs :TODO
// errors
// self tests
// to add to...

/** Configures the accelerometer according to the following:
 * @param accel_dlpfcfg the data low pass filter configuration according to p.64 of datasheet.
 * Must be in range [0, 7] (total 3 bits of register), value is irrelevant if @param accel_fchoice = 0
 * @param accel_fs_sel the full scale selection for the accelerometer according to the table below:
 * | 00: +-2g
 * | 01: +-4g
 * | 10: +-8g
 * | 11: +-16g
 * @param accel_fchoice enable or disable data low pass filter (1 is enable, 0 is bypass)
 */
biodyn_imu_err_t biodyn_imu_icm20948_config_accel(uint8_t accel_dlpfcfg, uint8_t accel_fs_sel, bool accel_fchoice);

/**
 * Configures the number of samples averaged in the accelerometer decimator
 * @param dec3_cfg sets the number of samples by the table below:
 * | 00: 1 sample (or 4 samples if fchoice is enabled)
 * | 01: 8 samples
 * | 10: 16 samples
 * | 11: 32 samples
 */
biodyn_imu_err_t biodyn_imu_icm20948_config_accel_sample_averaging(uint8_t dec3_cfg);
// TODO rest of functions for configuring IMU

#endif // ICM20948_DRIVER_H