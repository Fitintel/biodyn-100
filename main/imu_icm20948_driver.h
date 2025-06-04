#ifndef ICM20948_DRIVER_H
#define ICM20948_DRIVER_H

#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"


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

// Initializes the IMU
biodyn_imu_err_t biodyn_imu_icm20948_init();


#endif // ICM20948_DRIVER_H