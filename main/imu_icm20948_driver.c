#include "imu_icm20948_driver.h"
#include "imu_icm20948_consts.h"

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
static biodyn_imu_err_t biodyn_imu_icm20948_set_user_bank(uint8_t bank);
// Reads the user bank of registers
static biodyn_imu_err_t biodyn_imu_icm20948_get_user_bank(uint8_t *bank_out);
// Writes data to a single register specified by a bank and address to the IMU
static biodyn_imu_err_t biodyn_imu_icm20948_write_reg(uint8_t bank, uint16_t register_address, uint8_t write_data);
// Reads data of a single register specified by a bank and address of the IMU
static biodyn_imu_err_t biodyn_imu_icm20948_read_reg(uint8_t bank, uint16_t register_address, uint8_t *out);

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
		.mode = 0,					   // CPOL is high and CPHA is is rising edge sampling,
									   // compatible with mode 0 and mode 3
		.spics_io_num = imu_data.i2c.cs,
		.queue_size = 7, // amount of queueable requested SPI transactions.
	};
	err = spi_bus_add_device(SPI2_HOST, &spi_dev_config, &imu_data.handle);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "Failed to initialize SPI device, error %d", err);
		return BIODYN_IMU_ERR_COULDNT_INIT_SPI_DEV;
	}

	// TODO: implement
	// INITIALIZATION PROCEDURE
	// Reset IMU
	biodyn_imu_icm20948_write_reg(_b0, PWR_MGMT_1, 0xc1);

	// Exit from sleep and select clock 37
	biodyn_imu_icm20948_write_reg(_b0, PWR_MGMT_1, 0x01);

	// Align output data rate
	biodyn_imu_icm20948_write_reg(_b2, ODR_ALIGN_EN, 0x00);

	// Gyroscope config with sample rate divider = 0
	biodyn_imu_icm20948_write_reg(_b2, GYRO_SMPLRT_DIV, 0x00);

	// Gyroscope config with range set and digital filter enabled
	biodyn_imu_icm20948_write_reg(_b2, GYRO_CONFIG_1, ((GYRO_RANGE_VALUE << 1) | 0x01));

	// Accelerometer config with sample rate divider = 0
	biodyn_imu_icm20948_write_reg(_b2, ACCEL_SMPLRT_DIV_1, 0x00);
	biodyn_imu_icm20948_write_reg(_b2, ACCEL_SMPLRT_DIV_2, 0x00);

	// Acceleromter config with range set and digital filter enabled
	biodyn_imu_icm20948_write_reg(_b2, ACCEL_CONFIG, ((ACCEL_RANGE_VALUE << 1) | 0x01));

	// Serial interface in SPI mode only
	uint8_t user_ctrl_data;
	biodyn_imu_icm20948_read_reg(_b0, USER_CTRL, &user_ctrl_data);
	user_ctrl_data |= 0x10;
	biodyn_imu_icm20948_write_reg(_b2, USER_CTRL, user_ctrl_data);

	// Set bank 0 to get readings
	// biodyn_imu_icm20948_set_user_bank(_b0);

	// Delay to wait for power up
	vTaskDelay(pdMS_TO_TICKS(100));

	// Wake up all sensors
	biodyn_imu_icm20948_write_reg(_b0, PWR_MGMT_2, 0x00);

	// Turn off low power mode
	biodyn_imu_icm20948_write_reg(_b0, LP_CONFIG, 0x40);
	// ERROR: Retry turning off sleep mode of icm20948
	biodyn_imu_icm20948_write_reg(_b0, PWR_MGMT_1, 0x01);

	// Self test to ensure proper functionality
	biodyn_imu_icm20948_self_test();

	// Successful init, all clear
	ESP_LOGI(TAG, "Initialized IMU");
	return BIODYN_IMU_OK;
}

biodyn_imu_err_t biodyn_imu_icm20948_set_user_bank(uint8_t bank)
{
	// Check if bank is valid: range of [0, 3]
	if (bank > _b3)
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
biodyn_imu_err_t biodyn_imu_icm20948_get_user_bank(uint8_t *bank_out)
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

biodyn_imu_err_t biodyn_imu_icm20948_read_reg(uint8_t bank, uint16_t register_address, uint8_t *out)
{
	// Ensure valid pointer
	if (!out)
		return BIODYN_IMU_ERR_INVALID_ARGUMENT;

	// Identify user bank before selecting register details
	biodyn_imu_icm20948_set_user_bank(bank);
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
biodyn_imu_err_t biodyn_imu_icm20948_write_reg(uint8_t bank, uint16_t register_address, uint8_t write_data)
{
	// Select user bank to write to
	biodyn_imu_icm20948_set_user_bank(bank);

	// Use input register address OR with WRITE_MSB (0), with write data as second argument
	uint8_t tx_data[2] = {register_address | WRITE_MSB, write_data};

	// length 2 (bytes) = max{rx_data length, tx_data length}
	spi_transaction_t trans = {
		.length = 8 * 2,
		.tx_buffer = tx_data,
		.rx_buffer = NULL,
	};
	// SPI TRANSACTION
	esp_err_t err = spi_device_transmit(imu_data.handle, &trans);
	if (err != ESP_OK)
		return err;

	// Successful write, all clear
	return BIODYN_IMU_OK;
}

// Self-test functions
static biodyn_imu_err_t self_test_whoami()
{
	uint8_t whoami;
	biodyn_imu_icm20948_read_reg(_b0, 0x00, &whoami);

	if (whoami != 0xEA)
	{
		ESP_LOGE(TAG, "Got wrong WHOAMI response: %x", whoami);
		return BIODYN_IMU_ERR_WRONG_WHOAMI;
	}

	ESP_LOGI(TAG, "\tWHOAMI (%x) OK", whoami);

	return BIODYN_IMU_OK;
}
static biodyn_imu_err_t self_test_user_banks()
{
	biodyn_imu_err_t err = 0;

	// Write non-zero
	uint8_t write_bank_value = 2;
	if ((err = biodyn_imu_icm20948_set_user_bank(write_bank_value)))
		return err;

	// Verify written as non-zero
	uint8_t initial_bank_value = 0;
	if ((err = biodyn_imu_icm20948_get_user_bank(&initial_bank_value)))
		return err;
	if (initial_bank_value != write_bank_value)
	{
		ESP_LOGE(TAG, "Failed to write IMU user bank: Tried to write %d got %d",
				 write_bank_value, initial_bank_value);
		return BIODYN_IMU_ERR_COULDNT_SEND_DATA;
	}

	// Write 0 - restore default
	write_bank_value = 0;
	if ((err = biodyn_imu_icm20948_set_user_bank(write_bank_value)))
		return err;

	ESP_LOGI(TAG, "\tUser banks OK");
	return BIODYN_IMU_OK;
}
static biodyn_imu_err_t self_test_gyro()
{
	biodyn_imu_err_t err;

	// Get previous gyro2 config
	uint8_t gyro2_cfg = 0;
	if ((err = biodyn_imu_icm20948_read_reg(2, GYRO_CONFIG_2, &gyro2_cfg)))
	{
		ESP_LOGE(TAG, "Failed to read GYRO_CONFIG_2: %x", err);
		return err;
	}
	ESP_LOGI(TAG, "Got GYRO_CONFIG_2: %x", gyro2_cfg);

	// TODO: Add self-test

	// TODO: Write new gyro2 config with self-test

	// TODO: Read gyro self-test values

	// TODO: Write gyro2 config without self-test

	return BIODYN_IMU_OK;
}
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
	// Run gyro self-test
	if ((err = self_test_gyro()))
	{
		ESP_LOGE(TAG, "Self test failed - gyro (%x)", err);
	}
	// TODO: Run accel self-test
	// TODO: Run mag self-test

	return BIODYN_IMU_OK;
}

biodyn_imu_err_t self_test_accel(uint16_t *out)
{
	uint8_t low;
	uint8_t high;
	biodyn_imu_icm20948_read_reg(_b0, ACCEL_XOUT_L, &low);
	biodyn_imu_icm20948_read_reg(_b0, ACCEL_XOUT_H, &high);
	*out = ((uint16_t)high << 8) | low;

	return BIODYN_IMU_OK;
}

biodyn_imu_err_t biodyn_imu_icm20948_read_accel_gyro(imu_motion_data *data)
{
	return BIODYN_IMU_OK;
}
// Reads and returns compass data
biodyn_imu_err_t biodyn_imu_icm20948_read_compass(imu_float3_t *out)
{
	// TODO: implement!

	return BIODYN_IMU_OK;
}

// TEST FUNCTION
// Read from a bank and register of the IMU
// TO BE REMOVED ONCE TESTING IS DONE
biodyn_imu_err_t biodyn_imu_icm20948_read_register_test(uint8_t bank, uint16_t register_address, uint8_t *out)
{
	return biodyn_imu_icm20948_read_reg(bank, register_address, out);
}