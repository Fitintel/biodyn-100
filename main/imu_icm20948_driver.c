#include "imu_icm20948_driver.h"

#include "hal/spi_types.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#define TAG "IMU_ICM20948"

#define ACCEL_XOUT_H 0x2d
#define ACCEL_XOUT_L 0x2e
#define ACCEL_YOUT_H 0x2f
#define ACCEL_YOUT_L 0x30
#define ACCEL_ZOUT_H 0x31
#define ACCEL_ZOUT_L 0x32

//typedef struct {
//	int16_t accel_x;
//	int16_t accel_y;
//	int16_t accel_z;
//} imu_accel_data;




// IMU driver data
static struct 
{
	i2c_config_t i2c;
	spi_device_handle_t handle;
} imu_data = {
	{ 
		.cs = GPIO_NUM_14,
		.miso = GPIO_NUM_13,
		.sclk = GPIO_NUM_12,
		.mosi = GPIO_NUM_11,
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

	biodyn_imu_icm20948_self_test();

	ESP_LOGI(TAG, "Initialized IMU");

	// Ok.
	return BIODYN_IMU_OK;
}

// read accel data function
biodyn_imu_err_t biodyn_imu_icm20948_read_accel(imu_int_16_3_t *out)
{
	ESP_LOGI(TAG, "Reading accel data");
	uint8_t tx_data[7];
	tx_data[0] = ACCEL_XOUT_H | 0x80;
	uint8_t rx_data[7];

	spi_transaction_t trans = {
			.length = (8 * 7),
			.rxlength = (8 * 7),
			.tx_buffer = &tx_data,
			.rx_buffer = &rx_data,
	};

	esp_err_t err = spi_device_transmit(imu_data.handle, &trans);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "Failed to transmit data over SPI (reading accelerometer)");
		return BIODYN_IMU_ERR_COULDNT_SEND_DATA;
	}

	out -> x = ((int16_t) rx_data[1] << 8) + rx_data[2];
	out -> y = ((int16_t) rx_data[3] << 8) + rx_data[4];
	out -> z = ((int16_t) rx_data[5] << 8) + rx_data[6];
	ESP_LOGI(TAG, "READ: accel_x = %x, accel_y = %x, accel_z = %x", out->x, out->y, out->z);
	return BIODYN_IMU_OK;
}


biodyn_imu_err_t biodyn_imu_icm20948_read_user_ctrl(){
	ESP_LOGI(TAG, "Reading user_ctrl");
		uint8_t tx_data[2];
		tx_data[0] = 0x03 | 0x80;
		uint8_t rx_data[2];

		spi_transaction_t trans = {
				.length = (8 * 1),
				.rxlength = (8 * 1),
				.tx_buffer = &tx_data,
				.rx_buffer = &rx_data,
		};

		esp_err_t err = spi_device_transmit(imu_data.handle, &trans);
		if (err != ESP_OK)
		{
			ESP_LOGE(TAG, "Failed to transmit data over SPI (reading accelerometer)");
			return BIODYN_IMU_ERR_COULDNT_SEND_DATA;
		}

		ESP_LOGI(TAG, "Read user_ctrl as %x", rx_data[1]);

		return BIODYN_IMU_OK;
}

biodyn_imu_err_t biodyn_imu_icm20948_run_accel_test(){
	return biodyn_imu_icm20948_read_x_accel() | biodyn_imu_icm20948_read_y_accel() | biodyn_imu_icm20948_read_z_accel();
}

biodyn_imu_err_t biodyn_imu_icm20948_read_x_accel(){
	ESP_LOGI(TAG, "Reading x accel");
		uint8_t tx_data[3];
		tx_data[0] = ACCEL_XOUT_H | 0x80;
		uint8_t rx_data[3];

		spi_transaction_t trans = {
				.length = (8 * 2),
				.rxlength = (8 * 2),
				.tx_buffer = &tx_data,
				.rx_buffer = &rx_data,
		};

		esp_err_t err = spi_device_transmit(imu_data.handle, &trans);
		if (err != ESP_OK)
		{
			ESP_LOGE(TAG, "Failed to transmit data over SPI (reading accelerometer)");
			return BIODYN_IMU_ERR_COULDNT_SEND_DATA;
		}

		ESP_LOGI(TAG, "Read x accel rx_data[0] as %x", rx_data[0]);
		ESP_LOGI(TAG, "Read x accel rx_data[1] as %x", rx_data[1]);
		ESP_LOGI(TAG, "Read x accel rx_data[2] as %x", rx_data[2]);
		ESP_LOGI(TAG, "Read x accel rx_data total as %x", rx_data[1] << 8 | rx_data[2]);


		return BIODYN_IMU_OK;
}

biodyn_imu_err_t biodyn_imu_icm20948_read_y_accel(){
	ESP_LOGI(TAG, "Reading y accel");
		uint8_t tx_data[3];
		tx_data[0] = ACCEL_YOUT_H | 0x80;
		uint8_t rx_data[3];

		spi_transaction_t trans = {
				.length = (8 * 2),
				.rxlength = (8 * 2),
				.tx_buffer = &tx_data,
				.rx_buffer = &rx_data,
		};

		esp_err_t err = spi_device_transmit(imu_data.handle, &trans);
		if (err != ESP_OK)
		{
			ESP_LOGE(TAG, "Failed to transmit data over SPI (reading accelerometer)");
			return BIODYN_IMU_ERR_COULDNT_SEND_DATA;
		}

		ESP_LOGI(TAG, "Read y accel rx_data[0] as %x", rx_data[0]);
		ESP_LOGI(TAG, "Read y accel rx_data[1] as %x", rx_data[1]);
		ESP_LOGI(TAG, "Read y accel rx_data[2] as %x", rx_data[2]);
		ESP_LOGI(TAG, "Read y accel rx_data total as %x", rx_data[1] << 8 | rx_data[2]);


		return BIODYN_IMU_OK;
}

biodyn_imu_err_t biodyn_imu_icm20948_read_z_accel(){
	ESP_LOGI(TAG, "Reading z accel");
		uint8_t tx_data[3];
		tx_data[0] = ACCEL_ZOUT_H | 0x80;
		uint8_t rx_data[3];

		spi_transaction_t trans = {
				.length = (8 * 2),
				.rxlength = (8 * 2),
				.tx_buffer = &tx_data,
				.rx_buffer = &rx_data,
		};

		esp_err_t err = spi_device_transmit(imu_data.handle, &trans);
		if (err != ESP_OK)
		{
			ESP_LOGE(TAG, "Failed to transmit data over SPI (reading accelerometer)");
			return BIODYN_IMU_ERR_COULDNT_SEND_DATA;
		}

		ESP_LOGI(TAG, "Read z accel rx_data[0] as %x", rx_data[0]);
		ESP_LOGI(TAG, "Read z accel rx_data[1] as %x", rx_data[1]);
//		ESP_LOGI(TAG, "Read z accel rx_data[2] as %x", rx_data[2]);
		ESP_LOGI(TAG, "Read z accel rx_data total as %x", rx_data[0] << 8 | rx_data[1]);


		return BIODYN_IMU_OK;
}


// Self-test function
biodyn_imu_err_t biodyn_imu_icm20948_self_test()
{
	ESP_LOGI(TAG, "Running self test");

	// Send who am i message

	uint8_t tx_data[2] = { 0x80, 0x0 }; // {read, WHO_AM_I}
	uint8_t rx_data[2] = { 0x0, 0x0 }; // Recieved

	spi_transaction_t trans = {
		.length = 16,
		.rxlength = 16,
		.tx_buffer = &tx_data,
		.rx_buffer = &rx_data,
	};

	esp_err_t err = spi_device_transmit(imu_data.handle, &trans);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "Failed to transmit data over SPI");
		return BIODYN_IMU_ERR_COULDNT_SEND_DATA;
	}
	
	// Check that WHOAMI value is the same
	if (rx_data[1] != 0xEA)
	{
		ESP_LOGE(TAG, "Got wrong WHOAMI response: %x", rx_data[1]);
		return BIODYN_IMU_ERR_WRONG_WHOAMI;
	}
	ESP_LOGI(TAG, "Got correct WHOAMI response: %x", rx_data[1]);
	ESP_LOGI(TAG, "Got rx_data[0]: %x", rx_data[0]);

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
//biodyn_imu_err_t biodyn_imu_icm20948_read_accel(imu_float3_t *out)
//{
//	// TODO: implement!
//
//	return BIODYN_IMU_OK;
//}

// reads and returns compass data
biodyn_imu_err_t biodyn_imu_icm20948_read_compass(imu_float3_t *out)
{
	// TODO: implement!

	return BIODYN_IMU_OK;
}
