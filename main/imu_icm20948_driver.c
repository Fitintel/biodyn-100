#include "imu_icm20948_driver.h"

#include "hal/spi_types.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#define TAG "IMU_ICM20948"

// IMU driver data
static struct 
{
	i2c_config_t i2c;
	spi_device_handle_t handle;
} imu_data = {
	{ 
		.mosi = GPIO_NUM_10,
		.miso = GPIO_NUM_11,
		.sclk = GPIO_NUM_12,
		.cs = GPIO_NUM_13,
	},
	NULL,
};

// Initializes the IMU
biodyn_imu_err_t biodyn_imu_icm20948_init()
{
	// Create bus config
	spi_bus_config_t bus_config = {
		.miso_io_num = imu_data.i2c.miso,
		.mosi_io_num = imu_data.i2c.mosi,
		.sclk_io_num = imu_data.i2c.sclk,
		.quadwp_io_num = -1, // not used
		.quadhd_io_num = -1, // not used
		.max_transfer_sz = 4096, // TODO: extract to config
	};

	// Initialize bus
	esp_err_t err = spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "Failed to initialize SPI bus, error %d", err);
		return BIODYN_IMU_ERR_COULDNT_INIT_SPI_BUS;
	}

	// Initialize interface
	spi_device_interface_config_t spi_dev_config = {
		.clock_speed_hz = 1000 * 1000, // 1 MHz
		.mode = 0, // TODO: why?
		.spics_io_num = imu_data.i2c.cs,
		.queue_size = 7, // TODO: why?
	};
	err = spi_bus_add_device(SPI2_HOST, &spi_dev_config, &imu_data.handle);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "Failed to initialize SPI device, error %d", err);
		return BIODYN_IMU_ERR_COULDNT_INIT_SPI_DEV;
	}

	// TODO: implement

	ESP_LOGI(TAG, "Initialized IMU");

	// Ok.
	return BIODYN_IMU_OK;
}

// Self-test function
biodyn_imu_err_t biodyn_imu_icm20948_self_test()
{
	// Send who am i message

	uint8_t tx_data = 0x0; // WHO_AM_I
	uint8_t rx_data = 0x0; // Recieved

	spi_transaction_t trans = {
		.length = 8,
		.tx_buffer = &tx_data,
		.rx_buffer = &rx_data,
	};

	esp_err_t err = spi_device_transmit(imu_data.handle, &trans);
	if (err != ESP_OK)
		return BIODYN_IMU_ERR_COULDNT_SEND_DATA;
	
	// Check that WHOAMI value is the same
	if (rx_data != 0xEA)
		return BIODYN_IMU_ERR_WRONG_WHOAMI;

	// TODO: Use chip's actual self-test now
	
	return BIODYN_IMU_OK;
}

// Reads and returns gyro data
biodyn_imu_err_t biodyn_imu_icm20948_read_gyro(imu_float3_t *out)
{
	// TODO: implement!

	return BIODYN_IMU_OK;
}

// Reads and returns accelerometer data
biodyn_imu_err_t biodyn_imu_icm20948_read_accel(imu_float3_t *out)
{
	// TODO: implement!

	return BIODYN_IMU_OK;
}

// reads and returns compass data
biodyn_imu_err_t biodyn_imu_icm20948_read_compass(imu_float3_t *out)
{
	// TODO: implement!

	return BIODYN_IMU_OK;
}
