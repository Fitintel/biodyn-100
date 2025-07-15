#include "imu_icm20948_driver.h"

#include "hal/spi_types.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

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

// Sets the user bank of registers
biodyn_imu_err_t select_user_bank(uint8_t bank);
// Reads the user bank of registers
biodyn_imu_err_t get_user_bank(uint8_t *bank_out);
// Writes data to a single register specified by a bank and address to the IMU
biodyn_imu_err_t write_single_register(uint8_t bank, uint16_t register_address, uint8_t write_data);
// Reads data of a single register specified by a bank and address of the IMU
biodyn_imu_err_t read_single_register(uint8_t bank, uint16_t register_address, uint8_t *out);

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
		.mode = 3,					   // TODO: why? - CPOL is high and CPHA is is rising edge sampling
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

	// Configure the chip with USER_CTL
	// Set to SPI mode only with no DMP: 00010000 -> 0x10
	uint8_t user_ctl = 0x10;
	write_single_register(0, USER_CTRL, user_ctl);
	// vTaskDelay(pdMS_TO_TICKS(50)); // Let settle
	uint8_t user_ctl_out = 0;
	read_single_register(0, USER_CTRL, &user_ctl_out);
	if (user_ctl != user_ctl_out)
	{
		ESP_LOGE(TAG, "Failed to write user control: Wrote %x, got %x", user_ctl, user_ctl_out);
		return BIODYN_IMU_ERR_COULDNT_CONFIGURE;
	}

	// Ok.
	ESP_LOGI(TAG, "Initialized IMU");
	return BIODYN_IMU_OK;
}

biodyn_imu_err_t select_user_bank(uint8_t bank)
{
	// Check if bank is valid: range of [0, 3]
	if (bank > 3)
		return BIODYN_IMU_ERR_INVALID_ARGUMENT;

	// only bits [5:4] are for user bank select, rest are reserved
	// 0x30 -> 00110000 (see datasheet page 67)
	uint8_t tx_data[2] = {REG_BANK_SEL | WRITE_MSB,
						  (bank << 4) & 0x30};

	// length 2 (bytes) = max{rx_data length, tx_data length}
	spi_transaction_t trans = {
		.length = 8 * 2,
		.tx_buffer = tx_data,
		.rx_buffer = NULL,
	};

	// SPI TRANSACTION
	esp_err_t err = spi_device_transmit(imu_data.handle, &trans);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "Failed to transmit data over SPI (selecting_user_bank function with bank value %d)", bank);
		return BIODYN_IMU_ERR_COULDNT_SEND_DATA;
	}

	// Successful write, all clear
	return BIODYN_IMU_OK;
}

biodyn_imu_err_t get_user_bank(uint8_t *bank_out)
{
	// Pointer must be valid
	if (!bank_out)
		return BIODYN_IMU_ERR_INVALID_ARGUMENT;
	// Send address of bank with MSB as the read bit (1)
	uint8_t tx_data[2] = {REG_BANK_SEL | READ_MSB, 0x00};
	// Receiving byte array (must be > than minimum non trivial length of tx_data)
	uint8_t rx_data[2] = {0};

	// length 2 (bytes) = max{rx_data length, tx_data length}
	spi_transaction_t trans = {
		.length = 8 * 2,
		.tx_buffer = tx_data,
		.rx_buffer = rx_data,
	};

	// SPI TRANSACTION
	esp_err_t err = spi_device_transmit(imu_data.handle, &trans);
	if (err != ESP_OK)
		return err;

	// rx[0] is dummy garbage
	// rx_data[1] contains read data
	*bank_out = (rx_data[1] >> 4) & 0x03; // shift back out and mask for only bits [5:4]

	// Successful write, all clear
	return BIODYN_IMU_OK;
}

biodyn_imu_err_t read_single_register(uint8_t bank, uint16_t register_address, uint8_t *out)
{
	// Ensure valid pointer
	if (!out)
		return BIODYN_IMU_ERR_INVALID_ARGUMENT;

	// Identify user bank before selecting register details
	select_user_bank(bank);
	// Send user inputted register addres with MSB as the read bit (1)
	uint8_t tx_data[2] = {register_address | READ_MSB, 0x00};
	// Empty receiving byte array
	uint8_t rx_data[2] = {0};

	// length 2 (bytes) = max{rx_data length, tx_data length}
	spi_transaction_t trans = {
		.length = 8 * 2,
		.tx_buffer = tx_data,
		.rx_buffer = rx_data,
	};

	// SPI TRANSACTION
	esp_err_t err = spi_device_transmit(imu_data.handle, &trans);
	if (err != ESP_OK)
		return err;

	// rx_data[1] contains read  and rx[0] is dummy garbage
	*out = rx_data[1];

	// Successful write, all clear
	return BIODYN_IMU_OK;
}

biodyn_imu_err_t write_single_register(uint8_t bank, uint16_t register_address, uint8_t write_data)
{
	// Select user bank to write to
	select_user_bank(bank);

	// Use input register address OR with WRITE_MSB (0), with write data as second argument
	uint8_t tx_data[2] = {register_address | WRITE_MSB, write_data};
	// Receiving data array
	int8_t rx_data[2] = {0, 0};

	// length 2 (bytes) = max{rx_data length, tx_data length}
	spi_transaction_t trans = {
		.length = 8 * 2,
		.tx_buffer = tx_data,
		.rx_buffer = rx_data,
	};
	// SPI TRANSACTION
	esp_err_t err = spi_device_transmit(imu_data.handle, &trans);
	if (err != ESP_OK)
		return err;

	// Successful write, all clear
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

	ESP_LOGI(TAG, "GOT OUTPUT BUFFER: %x, %x, %x, %x, %x, %x, %x",
			 rx_data[0], rx_data[1], rx_data[2], rx_data[3], rx_data[4], rx_data[5], rx_data[6]);

	out->x = ((int16_t)rx_data[1] << 8) + rx_data[2];
	out->y = ((int16_t)rx_data[3] << 8) + rx_data[4];
	out->z = ((int16_t)rx_data[5] << 8) + rx_data[6];
	ESP_LOGI(TAG, "READ: accel_x = %x, accel_y = %x, accel_z = %x", out->x, out->y, out->z);
	return BIODYN_IMU_OK;
}

biodyn_imu_err_t self_test_whoami()
{
	uint8_t tx_data[2] = {READ_MSB | IMU_WHOAMI, 0x0}; // {read, WHO_AM_I}
	uint8_t rx_data[2] = {0x0, 0x0};				   // Recieved

	spi_transaction_t trans = {
		.length = 8 * 2,
		.rxlength = 8 * 2,
		.tx_buffer = &tx_data,
		.rx_buffer = &rx_data,
	};

	esp_err_t err = spi_device_transmit(imu_data.handle, &trans);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "Failed to transmit data over SPI");
		return BIODYN_IMU_ERR_COULDNT_SEND_DATA;
	}

	if (rx_data[1] != 0xEA)
	{
		ESP_LOGE(TAG, "Got wrong WHOAMI response: %x", rx_data[1]);
		return BIODYN_IMU_ERR_WRONG_WHOAMI;
	}

	ESP_LOGI(TAG, "\tGot correct WHOAMI response: %x", rx_data[1]);

	return BIODYN_IMU_OK;
}
biodyn_imu_err_t self_test_user_banks()
{
	biodyn_imu_err_t err = 0;

	// Write non-zero
	uint8_t write_bank_value = 2;
	if ((err = select_user_bank(write_bank_value)))
		return err;

	// Verify written as non-zero
	uint8_t initial_bank_value = 0;
	if ((err = get_user_bank(&initial_bank_value)))
		return err;
	if (initial_bank_value != write_bank_value)
	{
		ESP_LOGE(TAG, "Failed to write IMU user bank: Tried to write %d got %d",
				 write_bank_value, initial_bank_value);
		return BIODYN_IMU_ERR_COULDNT_SEND_DATA;
	}

	// Write 0 - restore default
	write_bank_value = 0;
	if ((err = select_user_bank(write_bank_value)))
		return err;

	ESP_LOGI(TAG, "\tUser banks OK");
	return BIODYN_IMU_OK;
}

// Self-test function
biodyn_imu_err_t biodyn_imu_icm20948_self_test()
{
	ESP_LOGI(TAG, "Running self test");

	biodyn_imu_err_t err;

	// Check that WHOAMI value is the same
	if ((err = self_test_whoami()))
	{
		ESP_LOGE(TAG, "Self test failed - whoami (%x)", err);
	}
	// Check that user bank selection works
	if ((err = self_test_user_banks()))
	{
		ESP_LOGE(TAG, "Self test failed - user banks (%x)", err);
	}

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
// biodyn_imu_err_t biodyn_imu_icm20948_read_accel(imu_float3_t *out)
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
