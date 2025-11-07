#include "imu_icm20948_driver.h"
#include "imu_icm20948_consts.h"

#include "hal/spi_types.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "math.h"
#include "string.h"
#include "esp_mac.h"
#include "esp_system.h"

static uint8_t accel_range_value = _accel_2g;
static uint8_t gyro_range_value = _gyro_1000dps;

static uint16_t accel_sensitivity_scale_factor = 0;
static float gyro_sensitivity_scale_factor = -1;

// TODO reorganize with config
#define MAG_SENSITIVITY_SCALE_FACTOR 4900 / (1 << 14)

// TODO ORGANIZE CODE
// TODO consider error trace system? simple push pop stack? ANSWER: not a priority... yet.

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

// IMU error list
// Collects up to 3 errors before overwriting
// 3 is arbitrarilty and can be further worked upon: TODO
char biodyn_imu_icm20948_errors[3][128] = {0};

// Dictates whether an error exists in the IMU
static bool biodyn_imu_icm20948_in_error = false;

// An index to add errors to the IMU's error logs
uint8_t biodyn_imu_icm20948_error_index = 0;

// TODO: add/remove necessary function prototypes with reorganization

// Sets the user bank of registers
static biodyn_imu_err_t set_user_bank(uint8_t bank);
// Reads the user bank of registers
static biodyn_imu_err_t get_user_bank(uint8_t *bank_out);
// Writes data to a single register specified by a bank and address to the IMU
static biodyn_imu_err_t write_reg(uint8_t bank, uint16_t register_address, uint8_t write_data);
// Reads data of a single register specified by a bank and address of the IMU
static biodyn_imu_err_t read_reg(uint8_t bank, uint16_t register_address, uint8_t *out);
// Initializes magnetometer (AK09916)
static biodyn_imu_err_t init_mag();
// Write a byte to the magnetometer attached to the IMU
static biodyn_imu_err_t mag_write_reg(uint8_t reg, uint8_t data);
// Read multiple bytes from the magnetometer attached to the IMU
static biodyn_imu_err_t mag_read_reg(uint8_t reg, uint8_t len);
// Adds an error returned by the IMU to it's subsystem
static void collect_err(uint8_t error, char *optional_attached_message);

// Adds errors to the IMU driver's subsystem error collection. A complete list of biodyn IMU errors can be found below and in imu_icm20948_driver.h
// For internal (nested) function calls of the driver's own functions, it is not necessary to manually add the error return (duplicate).
/**
 * BIODYN_IMU_OK 0
 * BIODYN_IMU_ERR_COULDNT_INIT_SPI_BUS 0x1
 * BIODYN_IMU_ERR_COULDNT_INIT_SPI_DEV 0x2
 * BIODYN_IMU_ERR_COULDNT_SEND_DATA 0x4
 * BIODYN_IMU_ERR_WRONG_WHOAMI 0x8
 * BIODYN_IMU_ERR_COULDNT_CONFIGURE 0x10
 * BIODYN_IMU_ERR_COULDNT_READ 0x20
 * BIODYN_IMU_ERR_INVALID_ARGUMENT 0x5
 */
static void collect_err(uint8_t error, char *optional_attached_message)
{
	biodyn_imu_icm20948_in_error = true;
	char error_msg[128]; // Enough space for message + suffix

	switch (error)
	{
	case BIODYN_IMU_OK:
		snprintf(error_msg, sizeof(error_msg), "BIODYN_IMU_OK");
		break;
	case BIODYN_IMU_ERR_COULDNT_INIT_SPI_BUS:
		snprintf(error_msg, sizeof(error_msg), "IMU_COULDNT_INIT_SPI_BUS");
		break;
	case BIODYN_IMU_ERR_COULDNT_INIT_SPI_DEV:
		snprintf(error_msg, sizeof(error_msg), "IMU_COULDNT_INIT_SPI_DEV");
		break;
	case BIODYN_IMU_ERR_COULDNT_SEND_DATA:
		snprintf(error_msg, sizeof(error_msg), "IMU_COULDNT_SEND_DATA");
		break;
	case BIODYN_IMU_ERR_WRONG_WHOAMI:
		snprintf(error_msg, sizeof(error_msg), "IMU_WRONG_WHOAMI");
		break;
	case BIODYN_IMU_ERR_COULDNT_CONFIGURE:
		snprintf(error_msg, sizeof(error_msg), "IMU_COULDNT_CONFIGURE");
		break;
	case BIODYN_IMU_ERR_COULDNT_READ:
		snprintf(error_msg, sizeof(error_msg), "IMU_COULDNT_READ");
		break;
	case BIODYN_IMU_ERR_INVALID_ARGUMENT:
		snprintf(error_msg, sizeof(error_msg), "IMU_INVALID_ARGUMENT");
		break;
	default:
		// Assume it is an esp error then,
		// This esp method from esp_err.h also handles unknown errors for us
		// Only overlap is BIODYN_IMU_OK and ESP_OK, therefore sufficient
		snprintf(error_msg, sizeof(error_msg), "%s", esp_err_to_name(error));
		break;
	}

	strncat(error_msg, "\n", sizeof(error_msg) - strlen(error_msg) - 1);

	if (optional_attached_message != NULL)
	{
		strncat(error_msg, optional_attached_message, sizeof(error_msg) - strlen(error_msg) - 1);
	}
	strncpy(biodyn_imu_icm20948_errors[biodyn_imu_icm20948_error_index],
			error_msg,
			sizeof(biodyn_imu_icm20948_errors[biodyn_imu_icm20948_error_index]) - 1);

	biodyn_imu_icm20948_error_index = (biodyn_imu_icm20948_error_index + 1) % 3;
	return;
}

// Tests register reading capability
biodyn_imu_err_t read_register_test(uint8_t bank, uint16_t register_address, uint8_t *out)
{
	// Ensure valid pointer
	if (!out)
	{
		collect_err(BIODYN_IMU_ERR_INVALID_ARGUMENT, "READ_REGISTER_TEST: invalid out pointer");
		return BIODYN_IMU_ERR_INVALID_ARGUMENT;
	}

	// Identify user bank before selecting register details
	set_user_bank(bank);
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
	{
		collect_err(err, "READ_REGISTER_TEST: failed spi transaction");
		return err;
	}

	// rx_data[1] contains read  and rx[0] is dummy garbage
	*out = rx_data[1];

	// Successful write, all clear
	return BIODYN_IMU_OK;
}

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
		collect_err(err, "SPI_BUS_INITIALIZE: failed spi bus initialization");
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
		collect_err(err, "SPI_BUS_INITIALIZE: failed spi bus initialization");
		ESP_LOGE(TAG, "Failed to initialize SPI device, error %d", err);
		return BIODYN_IMU_ERR_COULDNT_INIT_SPI_DEV;
	}

	// INITIALIZATION PROCEDURE

	// Initialize configuration data
	accel_sensitivity_scale_factor = (1 << (uint16_t)(14 - accel_range_value));
	gyro_sensitivity_scale_factor = 16.4 * (1 << (3 - gyro_range_value));
	ESP_LOGI(TAG, "\tPlanar sensitivity factor: %d", accel_sensitivity_scale_factor);
	ESP_LOGI(TAG, "\tGyro sensitivity factor: %f", gyro_sensitivity_scale_factor);

	// Reset IMU, wakes chip from sleep mode, turns off low power feature,
	// disables temperature sensor, and sets clock source to auto-select.
	err = write_reg(_b0, PWR_MGMT_1, 0x80);
	vTaskDelay(pdMS_TO_TICKS(100));
	uint8_t temp = 0;
	read_reg(_b0, PWR_MGMT_1, &temp);
	ESP_LOGI(TAG, "READ PWR_MGMT_1 as %d", temp);
	// Exit from sleep and select clock 37
	// DEBUG: doesn't this sleep the chip again?
	err |= write_reg(_b0, PWR_MGMT_1, 0x01);
	vTaskDelay(pdMS_TO_TICKS(10));
	read_reg(_b0, PWR_MGMT_1, &temp);
	ESP_LOGI(TAG, "READ PWR_MGMT_1 as %d", temp);

	// Turn off low power mode
	// This shouldn't matter because lp_config is disabled in pwr_mgmt1 by turning off low power feature
	err |= write_reg(_b0, LP_CONFIG, 0x00);
	err |= write_reg(_b0, PWR_MGMT_2, 0x00);
	vTaskDelay(pdMS_TO_TICKS(1));

	// ERROR: Retry turning off sleep mode of icm20948
	// TEST: what actually changes sleep
	// err |= write_reg(_b0, PWR_MGMT_1, 0x41);

	// Serial interface in SPI mode only
	uint8_t user_ctrl_data;
	err |= read_reg(_b0, USER_CTRL, &user_ctrl_data);
	user_ctrl_data |= 0x10;
	err |= write_reg(_b0, USER_CTRL, user_ctrl_data);

	// DEBUG TEST: RE-ENABLE I2C MASTER

	// uint8_t uc = 0;
	// read_reg(_b0, USER_CTRL, &uc);
	// uc |= 0x02; // I2C_MST_RESET
	// err |= write_reg(_b0, USER_CTRL, uc);
	// vTaskDelay(pdMS_TO_TICKS(10));

	// uc &= ~0x02; // Clear reset bit
	// err |= write_reg(_b0, USER_CTRL, uc);
	// vTaskDelay(pdMS_TO_TICKS(10));

	// read_reg(_b0, USER_CTRL, &uc);
	// uc &= ~0x10; // CLEAR I2C_IF_DIS
	// err |= write_reg(_b0, USER_CTRL, uc);
	// ESP_LOGI(TAG, "USER_CTRL = 0x%02X (I2C_IF_DIS cleared)", uc);

	// vTaskDelay(pdMS_TO_TICKS(10));
	// read_reg(_b0, USER_CTRL, &uc);
	// uc |= 0x20; // I2C_MST_EN
	// err |= write_reg(_b0, USER_CTRL, uc);

	// // TEST, see what user ctrl is now
	// read_reg(_b0, USER_CTRL, &uc);
	// ESP_LOGI(TAG, "Finally, user_ctrl is: %x", uc);

	// Align output data rate
	// ODR start-time alignements starts when any of the following is written to:
	/**
	 * GYRO_SMPLRT_DIV,
	 * ACCEL_SMPLRT_DIV_1,
	 * ACCEL_SMPLRT_DIV_2,
	 * I2C_MST_ODR_CONFIG
	 */
	// err |= write_reg(_b2, ODR_ALIGN_EN, 0x01);

	// Gyroscope config with sample rate divider = 0
	err |= write_reg(_b2, GYRO_SMPLRT_DIV, 0x00);

	// DEBUG TEST NECESSITY OF THIS CODE

	/**
	// Gyroscope config with range set and digital filter enabled
	// err |= write_reg(_b2, GYRO_CONFIG_1, (gyro_range_value | (1 << 2)));
	// uint8_t gyro_dlpfcfg = 0b111;
	uint8_t base_accel_gyro_cfg = 0b00111001;
	uint8_t gyro_cfg = base_accel_gyro_cfg | gyro_range_value << 1;
	// Testing bank switching
	err |= write_reg(_b2, GYRO_CONFIG_1, (gyro_cfg));
	if (err == BIODYN_IMU_ERR_COULDNT_SEND_DATA)
	{
		ESP_LOGE(TAG, "Failed on write to Gyro Config 1");
	}
	// BANK TEST FAILED, REWRITE

	// Accelerometer config with sample rate divider = 0
	err |= write_reg(_b2, ACCEL_SMPLRT_DIV_1, 0x00);
	err |= write_reg(_b2, ACCEL_SMPLRT_DIV_2, 0x00);

	// Acceleromter config with range set and digital filter enabled
	// uint8_t accel_dlpfcfg = 0b111;
	uint8_t accel_cfg = base_accel_gyro_cfg | accel_range_value << 1;
	err |= write_reg(_b2, ACCEL_CONFIG, (accel_cfg));
	if (err == BIODYN_IMU_ERR_COULDNT_SEND_DATA)
	{
		ESP_LOGE(TAG, "Failed on write to Accel Config");
	}

	// DEBUG: to read gyro and accel config
	uint8_t gyro_cfg_1_output = 0;
	uint8_t accel_cfg_output = 0;
	read_reg(_b2, GYRO_CONFIG_1, &gyro_cfg_1_output);
	read_reg(_b2, ACCEL_CONFIG, &accel_cfg_output);
	ESP_LOGI(TAG, "Gyro Config 1 output: %d", gyro_cfg_1_output);
	ESP_LOGI(TAG, "Accel Config output: %d", accel_cfg_output);
	ESP_LOGI(TAG, "gyro_cfg: %d", gyro_cfg);
	ESP_LOGI(TAG, "accel_cfg: %d", accel_cfg);
	set_user_bank(_b0);

	*/
	write_reg(_b2, GYRO_CONFIG_1, 0x00);
	write_reg(_b2, GYRO_CONFIG_1, gyro_range_value << 1);

	write_reg(_b2, ACCEL_CONFIG, 0x00);
	write_reg(_b2, ACCEL_CONFIG, accel_range_value << 1);

	// ERROR CHECKPOINT *INIT1*
	// Check err status to report any error if found
	// Possible mismatch due to &'ing errors to error codes,
	// Resolved only upon further scrutiny?
	// Due to low occurence over tested code, this is ok? TODO
	if (err != BIODYN_IMU_OK)
	{
		collect_err(err, "ICM_20948_INIT: error collected before checkpoint: INIT1. Multiple errors possible.");
		return err;
	}
	// Clear FIFO
	// err |= write_reg(_b0, FIFO_RST, 0x1F); // reset all FIFOs
	// vTaskDelay(pdMS_TO_TICKS(10));
	// err |= write_reg(_b0, FIFO_RST, 0x00);

	// Set bank 0 to get readings
	// set_user_bank(_b0);

	// Delay to wait for power up
	vTaskDelay(pdMS_TO_TICKS(100));

	// Wake up all sensors
	// err |= write_reg(_b0, PWR_MGMT_2, 0x00);

	if (err != BIODYN_IMU_OK)
	{
		collect_err(err, "ICM_20948_INIT: error collected before checkpoint: INIT2. Multiple errors possible.");
		return err;
	}

	// Initialize magnetometer
	err |= init_mag();

	if (err != ESP_OK)
	{
		collect_err(err, "ICM_20948_INIT: error in initializing and starting magnetometer. Multiple errors possible");
		ESP_LOGE(TAG, "Failed to initialize and start AK09916 (magnetometer) device, error %d", err);
		return BIODYN_IMU_ERR_COULDNT_INIT_SPI_DEV;
	}

	// Self test to ensure proper functionality
	err |= biodyn_imu_icm20948_self_test();
	if (err != ESP_OK)
	{
		collect_err(err, "ICM_20948_INIT: error in completing self-test. Further investigation into self-test part functions likely required.");
		ESP_LOGE(TAG, "Failed self_test for icm20948, investiage into composite function parts of self_test, error: %d", err);
		return BIODYN_IMU_ERR_COULDNT_INIT_SPI_DEV;
	}

	/**
	 * LOGS: Debug logs to check on the initialized status of the imu
	 * TODO: to be deleted
	 */

	// Successful init, all clear
	ESP_LOGI(TAG, "Initialized IMU");
	return BIODYN_IMU_OK;
}

/** Sets the user bank to @param bank
 * 	Range of [0, 3]
 */
biodyn_imu_err_t set_user_bank(uint8_t bank)
{
	// Check if bank is valid: range of [0, 3]
	if (bank > _b3)
	{
		char error_msg[100];
		sprintf(error_msg, "SET_USER_BANK: invalid argument %d for parameter *bank*", bank);
		collect_err(BIODYN_IMU_ERR_INVALID_ARGUMENT, error_msg);
		return BIODYN_IMU_ERR_INVALID_ARGUMENT;
	}

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
		collect_err(err, "SET_USER_BANK: Failed SPI transaction");
		ESP_LOGE(TAG, "Failed to transmit data over SPI (selecting_user_bank function with bank value %d)", bank);
		return BIODYN_IMU_ERR_COULDNT_SEND_DATA;
	}
	uint8_t temp = 0;
	get_user_bank(&temp);
	if (temp != bank)
	{
		ESP_LOGE(TAG, "FAILED TO WRITE BANK");
		collect_err(err, "SET_USER_BANK: Failed write to set bank");
		return BIODYN_IMU_ERR_COULDNT_SEND_DATA;
	}

	// Successful write, all clear

	return BIODYN_IMU_OK;
}

/** Gets the user bank, with output at @param bank_out
 * 	Range of [0, 3]
 */
biodyn_imu_err_t get_user_bank(uint8_t *bank_out)
{
	// Pointer must be valid
	if (!bank_out)
	{
		collect_err(BIODYN_IMU_ERR_INVALID_ARGUMENT, "GET_USER_BANK: null pointing argument *bank_out*");
		return BIODYN_IMU_ERR_INVALID_ARGUMENT;
	}
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
	{
		collect_err(BIODYN_IMU_ERR_INVALID_ARGUMENT, "GET_USER_BANK: Failed SPI transaction");
		return err;
	}

	// rx[0] is dummy garbage
	// rx_data[1] contains read data
	*bank_out = (rx_data[1] >> 4) & 0x03; // shift back out and mask for only bits [5:4]

	// Successful write, all clear
	return BIODYN_IMU_OK;
}

/**
 * Reads a register from the icm20948 according to the arguments
 * @param bank the bank the register to read is in
 * @param register_address the local address of the register within the bank
 * @param out the output stored read result from the icm20948
 */
biodyn_imu_err_t read_reg(uint8_t bank, uint16_t register_address, uint8_t *out)
{
	// Ensure valid pointer
	if (!out)
	{
		collect_err(BIODYN_IMU_ERR_INVALID_ARGUMENT, "READ_REG: null pointing argument *out*");
		return BIODYN_IMU_ERR_INVALID_ARGUMENT;
	}
	// Identify user bank before selecting register details
	set_user_bank(bank);
	// Send user inputted register addres with MSB as the read bit (1)
	uint8_t tx_data[2] = {register_address | READ_MSB, 0x00};
	// Empty receiving byte array
	uint8_t rx_data[2] = {0, 0};

	// length 2 (bytes) = max{rx_data length, tx_data length}
	spi_transaction_t trans = {
		.length = 8 * 2,
		.tx_buffer = tx_data,
		.rx_buffer = rx_data,
	};

	// SPI TRANSACTION
	esp_err_t err = spi_device_transmit(imu_data.handle, &trans);
	if (err != ESP_OK)
	{
		collect_err(BIODYN_IMU_ERR_INVALID_ARGUMENT, "READ_REG: Failed SPI transaction");
		return err;
	}

	// rx_data[1] contains read  and rx[0] is dummy garbage
	*out = rx_data[1];

	// Successful write, all clear
	return BIODYN_IMU_OK;
}

/**
 * A multibyte read of the icm20948
 * @param length the amount of registers to read
 * Acceptable length between 1 and the amount of registers remaining in the bank after @param register_address
 * See @fn iodyn_imu_icm20948_read_reg for further details on use
 */
biodyn_imu_err_t biodyn_imu_icm20948_multibyte_read_reg(uint8_t bank, uint16_t register_address, uint8_t *out, uint8_t length)
{
	// Invalid pointer throws an error
	if (!out || length == 0)
	{
		collect_err(BIODYN_IMU_ERR_INVALID_ARGUMENT, "MULTIBYTE_READ_REG: argument for parameter *out* must be non-zero");
		return BIODYN_IMU_ERR_INVALID_ARGUMENT;
	}
	set_user_bank(bank);
	uint8_t *tx_data = malloc(sizeof(uint8_t) * (length + 1));
	tx_data[0] = register_address | READ_MSB;
	uint8_t *rx_data = malloc(sizeof(uint8_t) * (length + 1));

	// length 2 (bytes) = max{rx_data length, tx_data length}
	spi_transaction_t trans = {
		.length = 8 * (length + 1),
		.tx_buffer = tx_data,
		.rx_buffer = rx_data,
	};

	// SPI TRANSACTION
	esp_err_t err = spi_device_transmit(imu_data.handle, &trans);
	if (err != ESP_OK)
	{
		collect_err(BIODYN_IMU_ERR_INVALID_ARGUMENT, "MULTIBYTE_READ_REG: Failed SPI transaction");
		free(tx_data);
		free(rx_data);
		return err;
	}

	// rx_data[1] contains read  and rx[0] is dummy garbage
	// out = rx_data;
	memcpy(out, rx_data + 1, length);
	free(tx_data);
	free(rx_data);
	// Successful write, all clear
	return BIODYN_IMU_OK;
}

/**
 * Writes to a register of the icm20948 according to the arguments
 * @param bank the bank the register to write to is in
 * @param register_address the local address of the register within the bank
 * @param write_data the data to write to the imu
 */
biodyn_imu_err_t write_reg(uint8_t bank, uint16_t register_address, uint8_t write_data)
{
	// Select user bank to write to
	set_user_bank(bank);

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
	{
		collect_err(BIODYN_IMU_ERR_INVALID_ARGUMENT, "WRITE_REG: Failed SPI transaction");
		return err;
	}

	// Successful write, all clear
	return BIODYN_IMU_OK;
}

/**
 * Self-test on the whoami of the IMU.
 * Checks the whoami register of the IMU which should always be readable.
 */
static biodyn_imu_err_t self_test_whoami()
{
	uint8_t whoami;
	read_reg(_b0, 0x00, &whoami);

	if (whoami != 0xEA)
	{
		ESP_LOGE(TAG, "Got wrong WHOAMI response: %x", whoami);
		collect_err(BIODYN_IMU_ERR_WRONG_WHOAMI, "SELF_TEST_WHOAMI: Incorrect whoami response");
		return BIODYN_IMU_ERR_WRONG_WHOAMI;
	}

	ESP_LOGI(TAG, "\tWHOAMI (%x) OK", whoami);

	return BIODYN_IMU_OK;
}

/**
 * Self-test the user banks of the IMU.
 * Checks whether it is possible to swap between banks, and ensures these changes are accurately reflected in the internal state of the IMU.
 */
static biodyn_imu_err_t self_test_user_banks()
{
	biodyn_imu_err_t err = 0;

	// Write non-zero
	uint8_t write_bank_value = 2;
	if ((err = set_user_bank(write_bank_value)))
	{
		collect_err(err, "SELF_TEST_USER_BANKS: Failed self-test");
		return err;
	}

	// Verify written as non-zero
	uint8_t initial_bank_value = 0;
	if ((err = get_user_bank(&initial_bank_value)))
	{
		collect_err(err, "SELF_TEST_USER_BANKS: Failed self-test");
		return err;
	}
	if (initial_bank_value != write_bank_value)
	{
		collect_err(BIODYN_IMU_ERR_COULDNT_SEND_DATA, "SELF_TEST_USER_BANKS: Mismatch between written (expected) bank value and received (actual)");
		ESP_LOGE(TAG, "Failed to write IMU user bank: Tried to write %d got %d",
				 write_bank_value, initial_bank_value);
		return BIODYN_IMU_ERR_COULDNT_SEND_DATA;
	}

	// Write 0 - restore default
	write_bank_value = 0;
	if ((err = set_user_bank(write_bank_value)))
	{
		collect_err(err, "SELF_TEST_USER_BANKS: Failed self-test");
		return err;
	}
	ESP_LOGI(TAG, "\tUser banks OK");
	return BIODYN_IMU_OK;
}

// BETTER IDEA: SELF_TEST TOGETHER (SAME MECHANISMS FOR BOTH)?

static biodyn_imu_err_t self_test_accel_gyro()
{
	// SELF-TEST RESPONSE = SENSOR OUTPUT WITH SELF-TEST ENABLED – SENSOR OUTPUT WITHOUT SELF-TEST ENABLED (p. 24)
	// Sensor output without self_test_enabled
	imu_motion_data imd_st_off = {0};
	imu_motion_data imd_st_on = {0};

	biodyn_imu_icm20948_read_accel_gyro_mag(&imd_st_off);

	// Start self_test actualizers
	// Use accel_config2 and gyro_config2

	// ACCEL SELF_TEST START
	uint8_t temp = 0;
	read_reg(_b2, ACCEL_CONFIG_2, &temp);
	// [7:6] reserved, [5:3] xyz self-test enables, [2:0] accel sample decimator (see function ...accel_number_samples_averaged)
	temp |= (0b111 << 5);
	write_reg(_b2, ACCEL_CONFIG_2, temp);

	// GYRO SELF_TEST START
	temp = 0;
	read_reg(_b2, GYRO_CONFIG_2, &temp);
	// [7:6] reserved, [5:3] xyz self-test enables, [2:0] accel sample decimator (see function ...accel_number_samples_averaged)
	temp |= (0b111 << 5);
	write_reg(_b2, GYRO_CONFIG_2, temp);
	vTaskDelay(pdMS_TO_TICKS(50));

	// Sensor output with self_test_enabled
	biodyn_imu_icm20948_read_accel_gyro_mag(&imd_st_on);

	// Subtract sensor ouput st_off froms st_on
	imu_motion_data deltas = {0};
	deltas.accel.x = imd_st_on.accel.x - imd_st_off.accel.x;
	deltas.accel.y = imd_st_on.accel.y - imd_st_off.accel.y;
	deltas.accel.z = imd_st_on.accel.z - imd_st_off.accel.z;
	deltas.gyro.x = imd_st_on.gyro.x - imd_st_off.gyro.x;
	deltas.gyro.y = imd_st_on.gyro.y - imd_st_off.gyro.y;
	deltas.gyro.z = imd_st_on.gyro.z - imd_st_off.gyro.z;

	// Obtain factory self_test results from icm20948 self_test registers to compare to.
	uint8_t trimax = 0;
	uint8_t trimay = 0;
	uint8_t trimaz = 0;
	uint8_t trimgx = 0;
	uint8_t trimgy = 0;
	uint8_t trimgz = 0;

	read_reg(_b1, SELF_TEST_X_ACCEL, &trimax);
	read_reg(_b1, SELF_TEST_Y_ACCEL, &trimay);
	read_reg(_b1, SELF_TEST_Z_ACCEL, &trimaz);
	read_reg(_b1, SELF_TEST_X_GYRO, &trimgx);
	read_reg(_b1, SELF_TEST_Y_GYRO, &trimgy);
	read_reg(_b1, SELF_TEST_Z_GYRO, &trimgz);

	// Compare trims to with a range limiter to deltas
	// Compare within magnitude by 50% range (might need to be adjusted)
	float RL = 0.5;
	float RH = 1.5;
	/** DEBUG*/
	ESP_LOGI(TAG, "\n st_off ax: %f\n st_off ay: %f\n st_off az: %f\n st_off gx: %f\n st_off gy: %f\n st_off gz: %f", imd_st_off.accel.x, imd_st_off.accel.y, imd_st_off.accel.z, imd_st_off.gyro.x, imd_st_off.gyro.y, imd_st_off.gyro.z);
	ESP_LOGI(TAG, "\n st_on ax: %f\n st_on ay: %f\n st_on az: %f\n st_on gx: %f\n st_on gy: %f\n st_on gz: %f", imd_st_on.accel.x, imd_st_on.accel.y, imd_st_on.accel.z, imd_st_on.gyro.x, imd_st_on.gyro.y, imd_st_on.gyro.z);

	ESP_LOGI(TAG, "\n delta ax: %f\n delta ay: %f\n delta az: %f\n delta gx: %f\n delta gy: %f\n delta gz: %f", deltas.accel.x, deltas.accel.y, deltas.accel.z, deltas.gyro.x, deltas.gyro.y, deltas.gyro.z);
	ESP_LOGI(TAG, "\n trimax: %d\ntrimay: %d\n trimaz: %d\n trimgx: %d\n trimgy: %d\n trimgz: %d", trimax, trimay, trimaz, trimgx, trimgy, trimgz);
	/** DEBUG */

	// CHECK if deltas-trim is between rangel*trim and rangeh*trim
	// return error on fails
	ESP_LOGI(TAG, "checking against trims");
	if (RL * trimax < deltas.accel.x / (1 << accel_range_value) && RH * trimax > deltas.accel.x / (1 << accel_range_value))
	{
		ESP_LOGI(TAG, "Self-test succesful for accel_gyro on ax");
	}
	else
	{
		ESP_LOGE(TAG, "self test failed on delta ax with ax = %f and trimax = %d", deltas.accel.x, trimax);
		collect_err(BIODYN_IMU_ERR_COULDNT_CONFIGURE, "SELF_TEST_ACCEL_GYRO: failed self-test on ax");
		return BIODYN_IMU_ERR_COULDNT_CONFIGURE;
	}
	if (RL * trimay < deltas.accel.y / (1 << accel_range_value) && RH * trimay > deltas.accel.y / (1 << accel_range_value))
	{
		ESP_LOGI(TAG, "Self-test succesful for accel_gyro on ay");
	}
	else
	{
		ESP_LOGE(TAG, "self test failed on delta ay");

		collect_err(BIODYN_IMU_ERR_COULDNT_CONFIGURE, "SELF_TEST_ACCEL_GYRO: failed self-test on ay");
		return BIODYN_IMU_ERR_COULDNT_CONFIGURE;
	}
	if (RL * trimaz < deltas.accel.z / (1 << accel_range_value) && RH * trimaz > deltas.accel.z / (1 << accel_range_value))
	{
		ESP_LOGI(TAG, "Self-test succesful for accel_gyro on az");
	}
	else
	{
		ESP_LOGE(TAG, "self test failed on delta az");
		collect_err(BIODYN_IMU_ERR_COULDNT_CONFIGURE, "SELF_TEST_ACCEL_GYRO: failed self-test on az");
		return BIODYN_IMU_ERR_COULDNT_CONFIGURE;
	}
	if (RL * trimgx < deltas.gyro.x / (1 << gyro_range_value) && RH * trimgx > deltas.gyro.x / (1 << gyro_range_value))
	{
		ESP_LOGI(TAG, "Self-test succesful for accel_gyro on gx");
	}
	else
	{
		uint8_t and1 = RL * trimgx < deltas.gyro.x / (1 << gyro_range_value);
		uint8_t and2 = RH * trimgx > deltas.gyro.x / (1 << gyro_range_value);
		ESP_LOGE(TAG, "failed with delta_gx = %f and trimgx = %d", deltas.gyro.x, trimgx);
		ESP_LOGE(TAG, "and1 = %d; and2 = %d", and1, and2);
		collect_err(BIODYN_IMU_ERR_COULDNT_CONFIGURE, "SELF_TEST_ACCEL_GYRO: failed self-test on gx");
		return BIODYN_IMU_ERR_COULDNT_CONFIGURE;
	}
	if (RL * trimgy < deltas.gyro.y / (1 << gyro_range_value) && RH * trimgy > deltas.gyro.y / (1 << gyro_range_value))
	{
		ESP_LOGI(TAG, "Self-test succesful for accel_gyro on gy");
	}
	else
	{
		uint8_t and1 = RL * trimgy < deltas.gyro.y / (1 << gyro_range_value);
		uint8_t and2 = RH * trimgy > deltas.gyro.y / (1 << gyro_range_value);
		ESP_LOGE(TAG, "failed with delta_gx = %f and trimgx = %d", deltas.gyro.y, trimgy);
		ESP_LOGE(TAG, "and1 = %d; and2 = %d", and1, and2);
		collect_err(BIODYN_IMU_ERR_COULDNT_CONFIGURE, "SELF_TEST_ACCEL_GYRO: failed self-test on gy");
		return BIODYN_IMU_ERR_COULDNT_CONFIGURE;
	}
	if (RL * trimgz < deltas.gyro.z / (1 << gyro_range_value) && RH * trimgz > deltas.gyro.z / (1 << gyro_range_value))
	{
		ESP_LOGI(TAG, "Self-test succesful for accel_gyro on gz");
	}
	else
	{
		uint8_t and1 = RL * trimgz < deltas.gyro.z / (1 << gyro_range_value);
		uint8_t and2 = RH * trimgz > deltas.gyro.z / (1 << gyro_range_value);
		ESP_LOGE(TAG, "failed with delta_gx = %f and trimgx = %d", deltas.gyro.z, trimgy);
		ESP_LOGE(TAG, "and1 = %d; and2 = %d", and1, and2);

		collect_err(BIODYN_IMU_ERR_COULDNT_CONFIGURE, "SELF_TEST_ACCEL_GYRO: failed self-test on gz");
		return BIODYN_IMU_ERR_COULDNT_CONFIGURE;
	}
	ESP_LOGI(TAG, "Finished checking against trims");

	// Clear self_test bits
	// ACCEL SELF_TEST END
	temp = 0;
	read_reg(_b2, ACCEL_CONFIG_2, &temp);
	// [7:6] reserved, [5:3] xyz self-test enables, [2:0] accel sample decimator (see function ...accel_number_samples_averaged)
	temp &= ~(0b111 << 5);
	write_reg(_b2, ACCEL_CONFIG_2, temp);

	// GYRO SELF_TEST END
	temp = 0;
	read_reg(_b2, GYRO_CONFIG_2, &temp);
	// [7:6] reserved, [5:3] xyz self-test enables, [2:0] accel sample decimator (see function ...accel_number_samples_averaged)
	temp &= ~(0b111 << 5);
	write_reg(_b2, GYRO_CONFIG_2, temp);

	return BIODYN_IMU_OK;
}

/**
 * Self-test the magnetometer of the IMU.
 * Uses the built-in structure of the AK09916 to perform self-tests on it's condition.
 */
static biodyn_imu_err_t self_test_mag()
{
	// Start by confirming who am I
	uint8_t ak09916_wia_output = 0;
	mag_read_reg(AK09916_WIA, 1);
	read_reg(_b0, EXT_SLV_SENS_DATA_00, &ak09916_wia_output);
	ESP_LOGI(TAG, "AK09916 WHO AM I: expected: 0x09, actual: %x", ak09916_wia_output);

	// test again to check reset time and ability of ext slv sens data 00
	mag_read_reg(AK09916_WIA, 1);
	read_reg(_b0, EXT_SLV_SENS_DATA_00, &ak09916_wia_output);
	ESP_LOGI(TAG, "AK09916 WHO AM I: expected: 0x09, actual: %x", ak09916_wia_output);

	// Check I2C master status for errors (NACK, etc.)
	uint8_t mst_status = 0;
	read_reg(_b0, I2C_MST_STATUS, &mst_status);
	ESP_LOGI(TAG, "I2C_MST_STATUS: 0x%02X", mst_status);

	// Key bits:
	// Bit 7: PASS_THROUGH (should be 0)
	// Bit 6: I2C_SLV4_DONE
	// Bit 5: I2C_LOST_ARB
	// Bit 4: I2C_SLV0_NACK  ← VERY IMPORTANT
	// Bit 3: I2C_SLV1_NACK
	// ...
	if (mst_status & 0x10)
	{
		ESP_LOGE(TAG, "I2C_SLV0_NACK: AK09916 not responding!");
	}
	if (mst_status & 0x20)
	{
		ESP_LOGE(TAG, "I2C_LOST_ARB: Bus arbitration lost!");
	}

	// Start self-test on ak09916
	// Read CNTL_2 to adjust for self-test
	mag_read_reg(AK09916_CONTROL2, 0x10);
	vTaskDelay(pdMS_TO_TICKS(1));

	// Trigger a single read by reading STATUS1 to get data ready (DRDY)
	// mag_read_reg(AK09916_STATUS1, 1);

	// Read output from external slave sensor data on icm20948
	uint8_t temp;
	read_reg(_b0, EXT_SLV_SENS_DATA_00, &temp);

	if (temp & 0x01)
	{
		return BIODYN_IMU_OK;
		// Read all 8 bytes of mag data (optional, for validation) ---
		uint8_t mag_data[8];
		mag_read_reg(AK09916_HXL, 8);

		// Wait for data to appear in ICM buffer
		vTaskDelay(pdMS_TO_TICKS(1));

		// Read from ICM-20948 slave buffer
		for (int i = 0; i < 8; i++)
		{
			read_reg(_b0, EXT_SLV_SENS_DATA_00 + i, &mag_data[i]);
		}

		// Check that data is non-zero (self-test field active) ---
		bool non_zero = false;
		for (int i = 0; i < 6; i++)
		{
			if (mag_data[i] != 0)
			{
				non_zero = true;
				break;
			}
		}

		if (!non_zero)
		{
			ESP_LOGE(TAG, "SELF_TEST_MAG: All mag bytes zero in self-test");
		}

		ESP_LOGI(TAG, "SELF_TEST_MAG: Passed (non-zero response)");
		// Exit self-test mode
		mag_write_reg(AK09916_CONTROL2, 0x08); // Continuous mode 4 (100 Hz)
		vTaskDelay(pdMS_TO_TICKS(1));
	}
	ESP_LOGE(TAG, "SELF_TEST_MAG: Failed self-test with read value test actual: %x, expected: non-zero", temp);
	collect_err(BIODYN_IMU_ERR_COULDNT_CONFIGURE, "SELF_TEST_MAG: Failed self-test");

	// Exit self-test mode
	mag_write_reg(AK09916_CONTROL2, 0x08); // Continuous mode 4 (100 Hz)
	vTaskDelay(pdMS_TO_TICKS(1));

	return BIODYN_IMU_ERR_COULDNT_CONFIGURE;
}

/**
 * Self-test on the entire IMU.
 * Consists of multiple separate self-tests.
 */
biodyn_imu_err_t biodyn_imu_icm20948_self_test()
{
	ESP_LOGI(TAG, "Running self test");

	biodyn_imu_err_t err;

	// Check that WHOAMI value is the same
	if ((err = self_test_whoami()))
	{
		ESP_LOGE(TAG, "Self test failed - whoami (%x)", err);
		return err;
	}
	ESP_LOGI(TAG, "Correct WHOAMI");

	// Check that user bank selection works
	if ((err = self_test_user_banks()))
	{
		ESP_LOGE(TAG, "Self test failed - user banks (%x)", err);
		return err;
	}

	// Run accel and gyro self-test
	// TODO: REPAIR
	// if ((err = self_test_accel_gyro()))
	// {
	// 	ESP_LOGE(TAG, "Self test failed - accel and gyro (%x)", err);
	// }

	// Run mag self-test
	// if ((err = self_test_mag()))
	// {
	// 	ESP_LOGE(TAG, "Self test failed - mag (%x)", err);
	// 	return err;
	// }

	return BIODYN_IMU_OK;
}

// -----------------------------BLUETOOTH HANDLE FUNCTIONS-----------------------------
/**
 * Reads the accelerometer data of the IMU with bluetooth compatability
 * Returns the converted, scaled, and signed output that is then converted into a binary stream for external (pointer)
 * @param size the amount of bytes to be read from out
 * @param out the output data from the IMU read of the accelerometer registers
 * out must already have space greater than (sizeof(float) * 3)
 * FYI sizeof(float) = 32 (bits). Therefore, 32 * 3 = 96
 */
void biodyn_imu_icm20948_read_accel(uint16_t *size, void *out)
{
	uint8_t read_out_length = 6;
	uint8_t *read_out = malloc(sizeof(uint8_t) * read_out_length);

	biodyn_imu_icm20948_multibyte_read_reg(_b0, ACCEL_XOUT_H, read_out, read_out_length);

	// Byte shifting for full high and low register with proper endianness
	int16_t raw_ax = (int16_t)((read_out[0] << 8) | read_out[1]);
	int16_t raw_ay = (int16_t)((read_out[2] << 8) | read_out[3]);
	int16_t raw_az = (int16_t)((read_out[4] << 8) | read_out[5]);

	float *fout = (float *)out;
	*size = 3 * sizeof(float);
	fout[0] = ((float)raw_ax / accel_sensitivity_scale_factor) * EARTH_GRAVITY;
	fout[1] = ((float)raw_ay / accel_sensitivity_scale_factor) * EARTH_GRAVITY;
	fout[2] = ((float)raw_az / accel_sensitivity_scale_factor) * EARTH_GRAVITY;

	// ESP_LOGI(TAG, "accel factor should be 16384 was %d", accel_sensitivity_scale_factor);
	free(read_out);
}

/**
 * Reads the gyroscope data of the IMU with bluetooth compatability
 * Returns the converted, scaled, and signed output that is then converted into a binary stream for external (pointer)
 * @param size the amount of bytes to be read from out
 * @param out the output data from the IMU read of the gyroscope registers
 * out must already have space greater than (sizeof(float) * 3)
 * FYI sizeof(float) = 32 (bits). Therefore, 32 * 3 = 96
 */
void biodyn_imu_icm20948_read_gyro(uint16_t *size, void *out)
{
	uint8_t read_out_length = 6;
	uint8_t *read_out = malloc(sizeof(uint8_t) * read_out_length);

	biodyn_imu_icm20948_multibyte_read_reg(_b0, GYRO_XOUT_H, read_out, read_out_length);

	// Byte shifting for full high and low register with proper endianness
	int16_t raw_ax = (int16_t)((read_out[0] << 8) | read_out[1]);
	int16_t raw_ay = (int16_t)((read_out[2] << 8) | read_out[3]);
	int16_t raw_az = (int16_t)((read_out[4] << 8) | read_out[5]);

	float *fout = (float *)out;
	*size = 3 * sizeof(float);
	fout[0] = ((float)raw_ax / gyro_sensitivity_scale_factor);
	fout[1] = ((float)raw_ay / gyro_sensitivity_scale_factor);
	fout[2] = ((float)raw_az / gyro_sensitivity_scale_factor);

	// ESP_LOGI(TAG, "accel factor should be 16384 was %d", accel_sensitivity_scale_factor);
	free(read_out);
}

// TODO: FIX MAG WITH NO DEBUG FIXES LIKE ST 2
/**
 * Reads the magnetometer data of the IMU with bluetooth compatability
 * Returns the converted, scaled, and signed output that is then converted into a binary stream for external (pointer)
 * @param size the amount of bytes to be read from out
 * @param out the output data from the IMU read of the magnetometer registers
 * out must already have space greater than (sizeof(float) * 3)
 * FYI sizeof(float) = 32 (bits). Therefore, 32 * 3 = 96
 */
void biodyn_imu_icm20948_read_mag(uint16_t *size, void *out)
{
	// Cannot break burst mode, therefore read from I2C cycle
	imu_motion_data imd;
	biodyn_imu_icm20948_read_accel_gyro_mag(&imd);

	float *fout = (float *)out;
	*size = 3 * sizeof(float);
	fout[0] = imd.mag.x;
	fout[1] = imd.mag.y;
	fout[2] = imd.mag.z;
}

/**
 * Reads the accelerometer, gyroscope, and magnetometer data of the IMU with bluetooth compatability
 * Returns the converted, scaled, and signed output that is then converted into a binary stream for external (pointer)
 * @param size the amount of bytes to be read from out
 * @param out the output data from the IMU of all relevant registers
 * Out should already have sufficient memory space for the data (sizeof(float) * 9)
 * FYI sizeof(float) = 32 (bits). Therefore, 32 * 9 = 288
 */
void biodyn_imu_icm20948_read_all(uint16_t *size, void *out)
{
	imu_motion_data imd = {0};
	biodyn_imu_icm20948_read_accel_gyro_mag(&imd);
	memcpy(out, &imd, sizeof(imu_motion_data));
	*size = sizeof(imu_motion_data);
}

// -----------------------------BLUETOOTH HANDLE FUNCTIONS-----------------------------

/**
 * Reads accelerometer, gyroscope, and magnetometer data in one read (18 bytes total).
 * Returns an imu_motion_data type with the output data of the icm20948 and ak09916 for their respective measurements.
 * See imu_motion_data type in @file imu_icm20948_driver.h
 * @param data the output data from the read of the IMU
 * Must be of type imu_motion_data
 */
biodyn_imu_err_t biodyn_imu_icm20948_read_accel_gyro_mag(imu_motion_data *data)
{
	// 6(accel) + 2(temp) + 6(gyro) + 9(mag: ST1, HXL... HZH, ST2)
	uint8_t out_length = 23;
	// uint8_t *out = malloc(sizeof(uint8_t) * out_length);
	uint8_t out[23];

	biodyn_imu_icm20948_multibyte_read_reg(_b0, ACCEL_XOUT_H, out, out_length);

	// Byte shifting for full high and low register with proper endianness
	int16_t raw_ax = ((int16_t)out[0] << 8) | out[1];
	int16_t raw_ay = ((int16_t)out[2] << 8) | out[3];
	int16_t raw_az = ((int16_t)out[4] << 8) | out[5];
	int16_t raw_gx = ((int16_t)out[6] << 8) | out[7];
	int16_t raw_gy = ((int16_t)out[8] << 8) | out[9];
	int16_t raw_gz = ((int16_t)out[10] << 8) | out[11];
	// Gap of two bytes between accel + gyro and mag for temperature registers
	// Gap of 1 byte for ST1
	// Magnetometer is low byte first, then high byte (little-endian)
	int16_t raw_mx = ((int16_t)out[16] << 8) | out[15];
	int16_t raw_my = ((int16_t)out[18] << 8) | out[17];
	int16_t raw_mz = ((int16_t)out[20] << 8) | out[19];
	// ESP_LOGI("TAG", "Raw Gyro: %d, %d, %d", raw_gx, raw_gy, raw_gz);
	// 21 is reserved,
	// 22 is ST2

	data->accel.x = ((float)raw_ax / accel_sensitivity_scale_factor) * EARTH_GRAVITY;
	data->accel.y = ((float)raw_ay / accel_sensitivity_scale_factor) * EARTH_GRAVITY;
	data->accel.z = ((float)raw_az / accel_sensitivity_scale_factor) * EARTH_GRAVITY;

	data->gyro.x = (float)raw_gx / gyro_sensitivity_scale_factor;
	data->gyro.y = (float)raw_gy / gyro_sensitivity_scale_factor;
	data->gyro.z = (float)raw_gz / gyro_sensitivity_scale_factor;

	data->mag.x = (float)raw_mx * MAG_SENSITIVITY_SCALE_FACTOR;
	data->mag.y = (float)raw_my * MAG_SENSITIVITY_SCALE_FACTOR;
	data->mag.z = (float)raw_mz * MAG_SENSITIVITY_SCALE_FACTOR;

	uint8_t st1 = out[14];
	// ESP_LOGI(TAG, "Read ST1 as %02x", st1);
	uint8_t st2 = out[22];
	if (st2 & 0x08)
	{
		ESP_LOGI(TAG, "Mag overflow detected!");
	}
	// DEBUG:
	// ESP_LOGI(TAG, "Raw Mag: %d, %d, %d | ST2: 0x%02X", raw_mx, raw_my, raw_mz, st2);

	// free(out);
	return BIODYN_IMU_OK;
}

/**
 * Writes to a register in the magnetometer of the IMU.
 * Requires sufficient delay due to I2C interactions between icm20948 and ak09916.
 * @param register_address the register address to write to within the ak09916 register bank
 * @param data the data to write within the requested register
 * Take care to ensure the register you write to is an ak09916 register and not an icm20948 register.
 */
static biodyn_imu_err_t mag_write_reg(uint8_t register_address, uint8_t data)
{
	// Set slave0 to be the built in magnetometer
	// Or first bit with 0 in order to indicate write
	write_reg(_b3, I2C_SLV0_ADDR, 0x00 | AK09916_ADDR);
	// Set the register to write to
	write_reg(_b3, I2C_SLV0_REG, register_address);
	// Set the data to write
	write_reg(_b3, I2C_SLV0_DO, data);

	// Enable and single data write
	// TODO: what does this mean?
	write_reg(_b3, I2C_SLV0_CTRL, 0x80 | 0x01);
	// Delay to allow I2C transaction
	vTaskDelay(pdMS_TO_TICKS(50));
	// TODO: check if delay is necessary
	return BIODYN_IMU_OK;
}

/**
 * Reads a register in the magnetometer of the IMU.
 * Requires sufficient delay due to I2C interactions between icm20948 and ak09916.
 * @param register_address the register address to read from within the ak09916 register bank
 * @param len the amount of data to read (i.e. you may multibyte read from this function, up to the limits of the amount of ext_slv_sens_data registers available)
 * Take care to ensure the register you write to is an ak09916 register and not an icm20948 register.
 */
static biodyn_imu_err_t mag_read_reg(uint8_t register_address, uint8_t len)
{
	// Set slave0 to be the built in magnetometer
	// Or first bit with 1 in order to indicate read
	write_reg(_b3, I2C_SLV0_ADDR, 0x80 | AK09916_ADDR);
	// Set the register to read from
	write_reg(_b3, I2C_SLV0_REG, register_address);

	// Enable and single data write
	// TODO: see mag_write_reg function
	// Bits [3:0] in I2C_SLV0_CTRL are the len to read (if capped)
	write_reg(_b3, I2C_SLV0_CTRL, 0x80 | len);
	// Delay to allow I2C transaction
	vTaskDelay(pdMS_TO_TICKS(50));

	return BIODYN_IMU_OK;
}

/**
 * Initializes the magnetometer (i.e. the ak09916) within the IMU (via the icm20948).
 * Prerequisitely, the icm20948 must be somewhat initialized first.
 */
static biodyn_imu_err_t init_mag()
{
	write_reg(_b2, ODR_ALIGN_EN, 0x01);
	vTaskDelay(pdMS_TO_TICKS(5));

	uint8_t temp = 0;
	read_reg(_b0, USER_CTRL, &temp);
	temp |= 0x02;
	write_reg(_b0, USER_CTRL, temp);
	vTaskDelay(pdMS_TO_TICKS(100));

	read_reg(_b0, USER_CTRL, &temp);
	temp |= (1 << 5);
	write_reg(_b0, USER_CTRL, temp);
	vTaskDelay(pdMS_TO_TICKS(10));

	write_reg(_b3, I2C_MST_CTRL, 0x07);
	vTaskDelay(pdMS_TO_TICKS(10));

	write_reg(_b0, LP_CONFIG, (1 << 6));
	vTaskDelay(pdMS_TO_TICKS(10));

	write_reg(_b3, I2C_MST_ODR_CONFIG, 0x03);
	vTaskDelay(pdMS_TO_TICKS(10));

	write_reg(_b3, I2C_SLV0_ADDR, AK09916_ADDR);
	write_reg(_b3, I2C_SLV0_REG, AK09916_CONTROL3);
	write_reg(_b3, I2C_SLV0_DO, 0x01);
	write_reg(_b3, I2C_SLV0_CTRL, 0x80 | 0x01); // 1 byte, EN
	vTaskDelay(pdMS_TO_TICKS(50));

	write_reg(_b3, I2C_SLV0_ADDR, AK09916_ADDR);
	write_reg(_b3, I2C_SLV0_REG, AK09916_CONTROL2);
	write_reg(_b3, I2C_SLV0_DO, (1 << 3));
	write_reg(_b3, I2C_SLV0_CTRL, 0x80 | 0x01); // 1 byte, EN
	vTaskDelay(pdMS_TO_TICKS(50));

	write_reg(_b3, I2C_SLV0_ADDR, AK09916_ADDR | 0x80);
	write_reg(_b3, I2C_SLV0_REG, 0x10);
	write_reg(_b3, I2C_SLV0_CTRL, 0x80 | 9);
	vTaskDelay(pdMS_TO_TICKS(100));

	uint8_t status = 0, err = BIODYN_IMU_ERR_COULDNT_READ;
	err = read_reg(_b0, WHO_AM_I, &status);
	if (status != 0xea)
	{
		ESP_LOGE(TAG, "Failed to read whoami after mag init");
		collect_err(BIODYN_IMU_ERR_WRONG_WHOAMI, "Failed to read WIA after mag init");
		return BIODYN_IMU_ERR_WRONG_WHOAMI;
	}

	return err;
	/**
	uint8_t temp = 0;

	ESP_LOGI(TAG, "Initializing AK09916 magnetometer...");

	// === 1. Enable I2C Master ===
	read_reg(_b0, USER_CTRL, &temp);
	temp |= 0x20; // I2C_MST_EN
	write_reg(_b0, USER_CTRL, temp);
	vTaskDelay(pdMS_TO_TICKS(10));

	// === 2. Set I2C Clock ===
	write_reg(_b3, I2C_MST_CTRL, 0x07); // 345.6 kHz
	vTaskDelay(pdMS_TO_TICKS(1));

	// === 3. Clear delay for SLV0 ===
	write_reg(_b3, I2C_MST_DELAY_CTRL, 0x00);
	vTaskDelay(pdMS_TO_TICKS(1));

	write_reg(_b0, I2C_MST_ODR_CONFIG, 0x03); // odr to 137Hz

	// === 4. TEST WIA (single byte) ===
	ESP_LOGI(TAG, "Reading AK09916 WIA...");
	// Test, force nack by using address 0xFF (out of bounds) instead of AK09916_ADDRESS
	write_reg(_b3, I2C_SLV0_ADDR, 0x80 | 0xff);
	write_reg(_b3, I2C_SLV0_REG, AK09916_WIA);
	write_reg(_b3, I2C_SLV0_CTRL, 0x80 | 0x01); // 1 byte, EN

	// === WAIT FOR DONE ===
	uint8_t mst_status = 0;
	int retries = 0;
	do
	{
		vTaskDelay(pdMS_TO_TICKS(1));
		read_reg(_b0, I2C_MST_STATUS, &mst_status);
		if (mst_status & 0x40)
			break; // I2C_SLV0_DONE
		retries++;
	} while (retries < 20);

	if (!(mst_status & 0x40))
	{
		ESP_LOGE(TAG, "WIA: I2C_SLV0_DONE timeout! STATUS=0x%02X", mst_status);
		return BIODYN_IMU_ERR_COULDNT_READ;
	}
	if (mst_status & 0x10)
	{
		ESP_LOGE(TAG, "WIA: I2C_SLV0_NACK! AK09916 not responding.");
		return BIODYN_IMU_ERR_COULDNT_READ;
	}

	// === READ WIA ===
	uint8_t wia = 0;
	read_reg(_b0, EXT_SLV_SENS_DATA_00, &wia);
	ESP_LOGI(TAG, "AK09916 WIA: 0x%02X (expected 0x09)", wia);
	if (wia != 0x09)
	{
		ESP_LOGE(TAG, "WIA failed! Got 0x%02X", wia);
		return BIODYN_IMU_ERR_COULDNT_READ;
	}

	// === 5. NOW REPROGRAM FOR BURST READ (8 bytes) ===
	ESP_LOGI(TAG, "Reprogramming I2C_SLV0 for burst read (8 bytes from HXL)...");

	write_reg(_b3, I2C_SLV0_ADDR, 0x80 | AK09916_ADDRESS);
	write_reg(_b3, I2C_SLV0_REG, AK09916_HXL);
	write_reg(_b3, I2C_SLV0_CTRL, 0x80 | 0x08); // 8 bytes, EN

	vTaskDelay(pdMS_TO_TICKS(10));

	// === 6. Start continuous mode on AK09916 ===
	mag_write_reg(AK09916_CONTROL2, 0x08); // 100 Hz
	vTaskDelay(pdMS_TO_TICKS(10));

	ESP_LOGI(TAG, "AK09916 initialized and burst read active!");
	return BIODYN_IMU_OK;
	*/
}
/**
 * Returns whether the IMU is currently in a state of error
 */
bool biodyn_imu_icm20948_has_error()
{
	return biodyn_imu_icm20948_in_error;
}
/**
 * Returns the most recent error of the IMU.
 * Returns a blank string if there are no errors.
 */
const char *biodyn_imu_icm20948_get_error()
{
	// Go back one index by adding limit - 1  = 2 (since limit is 3)
	return biodyn_imu_icm20948_errors[(biodyn_imu_icm20948_error_index + 2) % 3];
}

// TODO return copy of pointer data or the pointer itself? see also above get_error function
// char **biodyn_imu_icm20948_get_all_errors()
// {
// 	return biodyn_imu_icm20948_errors;
// }

// TODO
biodyn_imu_err_t biodyn_imu_icm20948_config_accel(uint8_t accel_dlpfcfg, uint8_t accel_fs_sel, bool accel_fchoice)
{
	return BIODYN_IMU_OK;
}
biodyn_imu_err_t biodyn_imu_icm20948_config_accel_sample_averaging(uint8_t dec3_cfg)
{
	return BIODYN_IMU_OK;
}

biodyn_imu_err_t biodyn_imu_icm20948_test_gyro(uint8_t *out)
{
	biodyn_imu_err_t err = 0x00;
	if ((err = read_reg(_b0, GYRO_XOUT_H, out)))
	{
		return err;
	}
	return BIODYN_IMU_OK;
}