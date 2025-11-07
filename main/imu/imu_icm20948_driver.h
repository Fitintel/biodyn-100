#ifndef ICM20948_DRIVER_H
#define ICM20948_DRIVER_H

#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "biodyn_constants.h"
#include "imu/linear.h"

#define IMU_ACCEL_UNIT "m/s^2"
#define IMU_GYRO_UNIT "dps"
#define IMU_MAG_UNIT "uT"

typedef struct
{
	float3 accel;
	float3 gyro;
	float3 mag;
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
	_b1 = 1,
	_b2 = 2,
	_b3 = 3,
} user_bank_range;

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
#define BIODYN_IMU_ERR_INVALID_ARGUMENT 0x40
#define BIODYN_IMU_ERR_NO_EXT_ACK 0x80
#define BIODYN_IMU_ERR_COULDNT_INIT_MAG 0x100

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
 * @param dec3_cfg takes a number [0b00, 0b11] and sets the number of samples by the table below:
 * | 00: 1 sample (or 4 samples if fchoice is enabled)
 * | 01: 8 samples
 * | 10: 16 samples
 * | 11: 32 samples
 */
biodyn_imu_err_t biodyn_imu_icm20948_config_accel_number_samples_averaged(uint8_t dec3_cfg);

/**
 * Configures the ODR_ALIGN_EN (i.e., whether all sensors are sampled together or sampled disjointly, but not exclusively)
 * These sensors include the accelerometer, gyroscope, magnetometer, and ambient temperature sensors.
 * @param enabled The enabled status of ODR_ALIGN_EN (1 = true: sensors are aligned, 0 = false: sensors are not aligned)
 * NOTICE: ODR alignment only begins (if enabled) when a sample rate is provided, when one of the following registers is written to:
 * 	GYRO_SMPLRT_DIV,
 *	ACCEL_SMPLRT_DIV_1,
 * 	ACCEL_SMPLRT_DIV_2,
 * 	I2C_MST_ODR_CONFIG.
 */
biodyn_imu_err_t biodyn_imu_icm20948_config_odr_align_en(bool enabled);

/**
 * Configures the rate at which samples are taken by the accelerometer
 * @param accel_smplrt_div takes a number [0, 2^12-1] that is the sample rate divider that decides the sampling rate as per the formula below:
 * Accel sampling rate = 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0])
 * All erroneous bits will be masked out.
 * NOTICE: setting a new sample rate while ODR_ALIGN_EN is enabled will set all sensors to the same sampling rate.
 * Disable ODR_ALIGN_EN if this is not the intented result.
 */
biodyn_imu_err_t biodyn_imu_icm20948_config_accel_sample_rate(uint16_t accel_smplrt_div);

/**
 * Configures the rate at which samples are taken by the gyroscope.
 * @param gyro_smplrt_div takes a number [0, 2^8-1] that is the sample rate divider that decides the sampling rate as per the formula below:
 * Gyro sampling rate = 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0])
 * NOTICE: setting a new sample rate while ODR_ALIGN_EN is enabled will set all sensors to the same sampling rate.
 * Disable ODR_ALIGN_EN if this is not the intented result.
 */
biodyn_imu_err_t biodyn_imu_icm20948_config_gyro_sample_averaging(uint8_t gyro_smplrt_div);

/**
 * Configures the rate at which samples are taken by the gyroscope.
 * @param gyro_smplrt_div takes a number [0, 2^8-1] that is the sample rate divider that decides the sampling rate as per the formula below:
 * Gyro sampling rate = 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0])
 * NOTICE: setting a new sample rate while ODR_ALIGN_EN is enabled will set all sensors to the same sampling rate.
 * Disable ODR_ALIGN_EN if this is not the intented result.
 */
biodyn_imu_err_t biodyn_imu_icm20948_config_gyro_sample_averaging(uint8_t gyro_smplrt_div);

// TODO rest of functions for configuring IMU
// TODO remove bias from gyro and mag

// TEMP TEST FUNCTION
biodyn_imu_err_t biodyn_imu_icm20948_test_gyro(uint8_t *out);
#endif // ICM20948_DRIVER_H