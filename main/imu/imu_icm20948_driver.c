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
static biodyn_imu_err_t biodyn_imu_icm20948_set_user_bank(uint8_t bank);
// Reads the user bank of registers
static biodyn_imu_err_t biodyn_imu_icm20948_get_user_bank(uint8_t *bank_out);
// Writes data to a single register specified by a bank and address to the IMU
static biodyn_imu_err_t biodyn_imu_icm20948_write_reg(uint8_t bank, uint16_t register_address, uint8_t write_data);
// Reads data of a single register specified by a bank and address of the IMU
static biodyn_imu_err_t biodyn_imu_icm20948_read_reg(uint8_t bank, uint16_t register_address, uint8_t *out);
// Initializes magnetometer (AK09916)
static biodyn_imu_err_t biodyn_imu_icm20948_init_magnetomter();
// Write a byte to the magnetometer attached to the IMU
static biodyn_imu_err_t biodyn_imu_ak09916_write_reg(uint8_t reg, uint8_t data);
// Read multiple bytes from the magnetometer attached to the IMU
static biodyn_imu_err_t biodyn_imu_ak09916_read_reg(uint8_t reg, uint8_t len);
// Adds an error returned by the IMU to it's subsystem
static void biodyn_imu_icm20948_add_error_to_subsystem(uint8_t error, char *optional_attached_message);

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
static void biodyn_imu_icm20948_add_error_to_subsystem(uint8_t error, char *optional_attached_message)
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
biodyn_imu_err_t biodyn_imu_icm20948_read_register_test(uint8_t bank, uint16_t register_address, uint8_t *out)
{
	// Ensure valid pointer
	if (!out)
	{
		biodyn_imu_icm20948_add_error_to_subsystem(BIODYN_IMU_ERR_INVALID_ARGUMENT, "READ_REGISTER_TEST: invalid out pointer");
		return BIODYN_IMU_ERR_INVALID_ARGUMENT;
	}

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
	{
		biodyn_imu_icm20948_add_error_to_subsystem(err, "READ_REGISTER_TEST: failed spi transaction");
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
		biodyn_imu_icm20948_add_error_to_subsystem(err, "SPI_BUS_INITIALIZE: failed spi bus initialization");
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
		biodyn_imu_icm20948_add_error_to_subsystem(err, "SPI_BUS_INITIALIZE: failed spi bus initialization");
		ESP_LOGE(TAG, "Failed to initialize SPI device, error %d", err);
		return BIODYN_IMU_ERR_COULDNT_INIT_SPI_DEV;
	}

	// INITIALIZATION PROCEDURE

	// Initialize configuration data
	accel_sensitivity_scale_factor = (1 << (uint16_t) (14 - accel_range_value));
	gyro_sensitivity_scale_factor = 16.4 * (1 << (3 - gyro_range_value));
	ESP_LOGI(TAG, "\tPlanar sensitivity factor: %d", accel_sensitivity_scale_factor);
	ESP_LOGI(TAG, "\tGyro sensitivity factor: %f", gyro_sensitivity_scale_factor);

	// Reset IMU
	// Start error collection,
	err = biodyn_imu_icm20948_write_reg(_b0, PWR_MGMT_1, 0x81);

	// Exit from sleep and select clock 37
	err |= biodyn_imu_icm20948_write_reg(_b0, PWR_MGMT_1, 0x01);

	// Turn off low power mode
	err |= biodyn_imu_icm20948_write_reg(_b0, LP_CONFIG, 0x40);
	// ERROR: Retry turning off sleep mode of icm20948
	err |= biodyn_imu_icm20948_write_reg(_b0, PWR_MGMT_1, 0x01);

	// Align output data rate
	err |= biodyn_imu_icm20948_write_reg(_b2, ODR_ALIGN_EN, 0x00);

	// Gyroscope config with sample rate divider = 0
	err |= biodyn_imu_icm20948_write_reg(_b2, GYRO_SMPLRT_DIV, 0x00);

	// Gyroscope config with range set and digital filter enabled
	err |= biodyn_imu_icm20948_write_reg(_b2, GYRO_CONFIG_1, ((gyro_range_value << 1) | 0x01));

	// Accelerometer config with sample rate divider = 0
	err |= biodyn_imu_icm20948_write_reg(_b2, ACCEL_SMPLRT_DIV_1, 0x00);
	err |= biodyn_imu_icm20948_write_reg(_b2, ACCEL_SMPLRT_DIV_2, 0x00);

	// Acceleromter config with range set and digital filter enabled
	err |= biodyn_imu_icm20948_write_reg(_b2, ACCEL_CONFIG, ((accel_range_value << 1) | 0x01));

	// ERROR CHECKPOINT *INIT1*
	// Check err status to report any error if found
	// Possible mismatch due to &'ing errors to error codes,
	// Resolved only upon further scrutiny?
	// Due to low occurence over tested code, this is ok? TODO
	if (err != BIODYN_IMU_OK)
	{
		biodyn_imu_icm20948_add_error_to_subsystem(err, "ICM_20948_INIT: error collected before checkpoint: INIT1. Multiple errors possible.");
		return err;
	}

	// Serial interface in SPI mode only
	uint8_t user_ctrl_data;
	err |= biodyn_imu_icm20948_read_reg(_b0, USER_CTRL, &user_ctrl_data);
	user_ctrl_data |= 0x10;
	err |= biodyn_imu_icm20948_write_reg(_b2, USER_CTRL, user_ctrl_data);

	// Set bank 0 to get readings
	// biodyn_imu_icm20948_set_user_bank(_b0);

	// Delay to wait for power up
	vTaskDelay(pdMS_TO_TICKS(100));

	// Wake up all sensors
	err |= biodyn_imu_icm20948_write_reg(_b0, PWR_MGMT_2, 0x00);

	// Turn off low power mode
	err |= biodyn_imu_icm20948_write_reg(_b0, LP_CONFIG, 0x40);
	// ERROR: Retry turning off sleep mode of icm20948
	err |= biodyn_imu_icm20948_write_reg(_b0, PWR_MGMT_1, 0x01);

	// ERROR CHECKPOINT *INIT2*
	// Check err status to report any error if found
	// Possible mismatch due to &'ing errors to error codes,
	// Resolved only upon further scrutiny?
	// Due to low occurence over tested code, this is ok? TODO
	if (err != BIODYN_IMU_OK)
	{
		biodyn_imu_icm20948_add_error_to_subsystem(err, "ICM_20948_INIT: error collected before checkpoint: INIT2. Multiple errors possible.");
		return err;
	}

	// Initialize magnetometer
	err |= biodyn_imu_icm20948_init_magnetomter();

	// Read the magnetometer data from HXL to HZH
	// i.e., the values of the magnetometer for x,y,z seperated into 2 bytes each
	err |= biodyn_imu_ak09916_read_reg(AK09916_HXL, 8);
	if (err != ESP_OK)
	{
		biodyn_imu_icm20948_add_error_to_subsystem(err, "ICM_20948_INIT: error in initializing and starting magnetometer. Multiple errors possible");
		ESP_LOGE(TAG, "Failed to initialize and start AK09916 (magnetometer) device, error %d", err);
		return BIODYN_IMU_ERR_COULDNT_INIT_SPI_DEV;
	}
	// 8 (not 6) byte read necessary: must sample the status and control register
	// in order to refresh readings (take from new sample).

	// Self test to ensure proper functionality
	err |= biodyn_imu_icm20948_self_test();
	if (err != ESP_OK)
	{
		biodyn_imu_icm20948_add_error_to_subsystem(err, "ICM_20948_INIT: error in completing self-test. Further investigation into self-test part functions likely required.");
		ESP_LOGE(TAG, "Failed to initialize and start AK09916 (magnetometer) device, error %d", err);
		return BIODYN_IMU_ERR_COULDNT_INIT_SPI_DEV;
	}
	// TODO: write self_tests for magnetometer

	// Initialize the magnetometer

	// Successful init, all clear
	ESP_LOGI(TAG, "Initialized IMU");
	return BIODYN_IMU_OK;
}

/** Sets the user bank to @param bank
 * 	Range of [0, 3]
 */
biodyn_imu_err_t biodyn_imu_icm20948_set_user_bank(uint8_t bank)
{
	// Check if bank is valid: range of [0, 3]
	if (bank > _b3)
	{
		char error_msg[100];
		sprintf(error_msg, "SET_USER_BANK: invalid argument %d for parameter *bank*", bank);
		biodyn_imu_icm20948_add_error_to_subsystem(BIODYN_IMU_ERR_INVALID_ARGUMENT, error_msg);
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
		biodyn_imu_icm20948_add_error_to_subsystem(err, "SET_USER_BANK: Failed SPI transaction");
		ESP_LOGE(TAG, "Failed to transmit data over SPI (selecting_user_bank function with bank value %d)", bank);
		return BIODYN_IMU_ERR_COULDNT_SEND_DATA;
	}

	// Successful write, all clear
	return BIODYN_IMU_OK;
}

/** Gets the user bank, with output at @param bank_out
 * 	Range of [0, 3]
 */
biodyn_imu_err_t biodyn_imu_icm20948_get_user_bank(uint8_t *bank_out)
{
	// Pointer must be valid
	if (!bank_out)
	{
		biodyn_imu_icm20948_add_error_to_subsystem(BIODYN_IMU_ERR_INVALID_ARGUMENT, "GET_USER_BANK: null pointing argument *bank_out*");
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
		biodyn_imu_icm20948_add_error_to_subsystem(BIODYN_IMU_ERR_INVALID_ARGUMENT, "GET_USER_BANK: Failed SPI transaction");
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
biodyn_imu_err_t biodyn_imu_icm20948_read_reg(uint8_t bank, uint16_t register_address, uint8_t *out)
{
	// Ensure valid pointer
	if (!out)
	{
		biodyn_imu_icm20948_add_error_to_subsystem(BIODYN_IMU_ERR_INVALID_ARGUMENT, "READ_REG: null pointing argument *out*");
		return BIODYN_IMU_ERR_INVALID_ARGUMENT;
	}
	// Identify user bank before selecting register details
	biodyn_imu_icm20948_set_user_bank(bank);
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
		biodyn_imu_icm20948_add_error_to_subsystem(BIODYN_IMU_ERR_INVALID_ARGUMENT, "READ_REG: Failed SPI transaction");
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
		biodyn_imu_icm20948_add_error_to_subsystem(BIODYN_IMU_ERR_INVALID_ARGUMENT, "MULTIBYTE_READ_REG: argument for parameter *out* must be non-zero");
		return BIODYN_IMU_ERR_INVALID_ARGUMENT;
	}
	biodyn_imu_icm20948_set_user_bank(bank);
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
		biodyn_imu_icm20948_add_error_to_subsystem(BIODYN_IMU_ERR_INVALID_ARGUMENT, "MULTIBYTE_READ_REG: Failed SPI transaction");
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
	{
		biodyn_imu_icm20948_add_error_to_subsystem(BIODYN_IMU_ERR_INVALID_ARGUMENT, "WRITE_REG: Failed SPI transaction");
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
	biodyn_imu_icm20948_read_reg(_b0, 0x00, &whoami);

	if (whoami != 0xEA)
	{
		ESP_LOGE(TAG, "Got wrong WHOAMI response: %x", whoami);
		biodyn_imu_icm20948_add_error_to_subsystem(BIODYN_IMU_ERR_WRONG_WHOAMI, "SELF_TEST_WHOAMI: Incorrect whoami response");
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
	if ((err = biodyn_imu_icm20948_set_user_bank(write_bank_value)))
	{
		biodyn_imu_icm20948_add_error_to_subsystem(err, "SELF_TEST_USER_BANKS: Failed self-test");
		return err;
	}

	// Verify written as non-zero
	uint8_t initial_bank_value = 0;
	if ((err = biodyn_imu_icm20948_get_user_bank(&initial_bank_value)))
	{
		biodyn_imu_icm20948_add_error_to_subsystem(err, "SELF_TEST_USER_BANKS: Failed self-test");
		return err;
	}
	if (initial_bank_value != write_bank_value)
	{
		biodyn_imu_icm20948_add_error_to_subsystem(BIODYN_IMU_ERR_COULDNT_SEND_DATA, "SELF_TEST_USER_BANKS: Mismatch between written (expected) bank value and received (actual)");
		ESP_LOGE(TAG, "Failed to write IMU user bank: Tried to write %d got %d",
				 write_bank_value, initial_bank_value);
		return BIODYN_IMU_ERR_COULDNT_SEND_DATA;
	}

	// Write 0 - restore default
	write_bank_value = 0;
	if ((err = biodyn_imu_icm20948_set_user_bank(write_bank_value)))
	{
		biodyn_imu_icm20948_add_error_to_subsystem(err, "SELF_TEST_USER_BANKS: Failed self-test");
		return err;
	}
	ESP_LOGI(TAG, "\tUser banks OK");
	return BIODYN_IMU_OK;
}

// TODO: this doesn't look like a proper self-test, check it out?
// TODO: also check gyro self-test
static biodyn_imu_err_t self_test_accel()
{
	uint8_t low;
	uint8_t high;
	int16_t out = 0;
	biodyn_imu_icm20948_read_reg(_b0, ACCEL_XOUT_L, &low);
	biodyn_imu_icm20948_read_reg(_b0, ACCEL_XOUT_H, &high);

	out = (((int16_t)high) << 8) | low;
	out *= (9.81 / 4096);

	return BIODYN_IMU_OK;
}

/**
 * Self-test the gyroscope of the IMU.
 * Checks the built-in self-test in the IMU, which identifies if it's gyroscope is working.
 */
static biodyn_imu_err_t self_test_gyro()
{
	biodyn_imu_err_t err;

	// Get previous gyro2 config
	uint8_t gyro2_cfg = 0;
	if ((err = biodyn_imu_icm20948_read_reg(2, GYRO_CONFIG_2, &gyro2_cfg)))
	{
		biodyn_imu_icm20948_add_error_to_subsystem(err, "SELF_TEST_GYRO: Failed self-test");
		ESP_LOGE(TAG, "Failed to read GYRO_CONFIG_2: %x", err);
		return err;
	}
	ESP_LOGI(TAG, "Got GYRO_CONFIG_2: %x", gyro2_cfg);

	// TODO: Add self-test

	// TODO: Write new gyro2 config with self-test

	// TODO: Read gyro self-test values

	// TODO: Write gyro2 config without self-test

	// TODO: Add error logging for above features

	return BIODYN_IMU_OK;
}

/**
 * Self-test the magnetometer of the IMU.
 * Uses the built-in structure of the AK09916 to perform self-tests on it's condition.
 */
static biodyn_imu_err_t self_test_mag()
{
	// Start self-test on ak09916
	// Read CNTL_2 to adjust for self-test
	biodyn_imu_ak09916_read_reg(AK09916_CONTROL2, 1);

	// Read output from external slave sensor data on icm20948
	uint8_t temp;
	biodyn_imu_icm20948_read_reg(_b0, EXT_SLV_SENS_DATA_00, &temp);

	// Or with self-test mode
	temp |= 0b10000;
	biodyn_imu_ak09916_write_reg(AK09916_CONTROL2, temp);

	// Now in self-test mode, check response
	// Check by checking DRDY (data ready) from status1

	biodyn_imu_ak09916_read_reg(AK09916_STATUS1, 1);
	biodyn_imu_icm20948_read_reg(_b0, EXT_SLV_SENS_DATA_00, &temp);
	if (temp & 0b11111111)
		return BIODYN_IMU_OK;
	biodyn_imu_icm20948_add_error_to_subsystem(BIODYN_IMU_ERR_COULDNT_CONFIGURE, "SELF_TEST_MAG: Failed self-test");
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
	// Check that user bank selection works
	if ((err = self_test_user_banks()))
	{
		ESP_LOGE(TAG, "Self test failed - user banks (%x)", err);
		return err;
	}
	// Run gyro self-test
	if ((err = self_test_gyro()))
	{
		ESP_LOGE(TAG, "Self test failed - gyro (%x)", err);
	}
	// Run accel self-test
	if ((err = self_test_accel()))
	{
		ESP_LOGE(TAG, "Self test failed - accel (%x)", err);
	}

	// Run mag self-test
	if ((err = self_test_mag()))
	{
		ESP_LOGE(TAG, "Self test failed - mag (%x)", err);
	}

	return BIODYN_IMU_OK;
}

// -----------------------------BLUETOOTH HANDLE FUNCTIONS-----------------------------
/**
 * Reads the accelerometer data of the IMU with bluetooth compatability
 * Returns the converted, scaled, and signed output that is then converted into a binary stream for external (pointer)
 * @param size the amount of bytes to be read from out
 * @param out the output data from the IMU read of the accelerometer registers
 */
void biodyn_imu_icm20948_read_accel(uint16_t *size, void *out)
{
	// ASSUMES imu_motion_data has gyro xyz in that order
	imu_motion_data imd = {0};
	biodyn_imu_icm20948_read_accel_gyro_mag(&imd);
	memcpy(out, &(imd.accel_x), sizeof(float) * 3);
	*size = sizeof(float) * 3;
}

/**
 * Reads the gyroscope data of the IMU with bluetooth compatability
 * Returns the converted, scaled, and signed output that is then converted into a binary stream for external (pointer)
 * @param size the amount of bytes to be read from out
 * @param out the output data from the IMU read of the gyroscope registers
 */
void biodyn_imu_icm20948_read_gyro(uint16_t *size, void *out)
{
	// ASSUMES imu_motion_data has gyro xyz in that order
	imu_motion_data imd = {0};
	biodyn_imu_icm20948_read_accel_gyro_mag(&imd);
	memcpy(out, &(imd.gyro_x), sizeof(float) * 3);
	*size = sizeof(float) * 3;
}

/**
 * Reads the magnetometer data of the IMU with bluetooth compatability
 * Returns the converted, scaled, and signed output that is then converted into a binary stream for external (pointer)
 * @param size the amount of bytes to be read from out
 * @param out the output data from the IMU read of the magnetometer registers
 */
void biodyn_imu_icm20948_read_mag(uint16_t *size, void *out)
{
	// ASSUMES imu_motion_data has mag xyz in that order
	imu_motion_data imd = {0};
	biodyn_imu_icm20948_read_accel_gyro_mag(&imd);
	memcpy(out, &(imd.mag_x), sizeof(float) * 3);
	*size = sizeof(float) * 3;
}

/**
 * Reads the accelerometer, gyroscope, and magnetometer data of the IMU with bluetooth compatability
 * Returns the converted, scaled, and signed output that is then converted into a binary stream for external (pointer)
 * @param size the amount of bytes to be read from out
 * @param out the output data from the IMU of all relevant registers
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
	uint8_t out_length = 9 * sizeof(uint16_t) + 2 * sizeof(uint8_t);
	uint8_t *out = malloc(sizeof(uint8_t) * out_length);

	biodyn_imu_icm20948_multibyte_read_reg(_b0, ACCEL_XOUT_H, out, out_length);

	// Byte shifting for full high and low register with proper endianness
	int16_t raw_ax = ((uint16_t) out[0] << 8) | out[1];
	int16_t raw_ay = ((uint16_t) out[2] << 8) | out[3];
	int16_t raw_az = ((uint16_t) out[4] << 8) | out[5];
	int16_t raw_gx = ((uint16_t) out[6] << 8) | out[7];
	int16_t raw_gy = ((uint16_t) out[8] << 8) | out[9];
	int16_t raw_gz = ((uint16_t) out[10] << 8) | out[11];
	// gap of two bytes between accel + gyro and mag for temperature registers
	int16_t raw_mx = ((uint16_t) out[15] << 8) | out[14];
	int16_t raw_my = ((uint16_t) out[17] << 8) | out[16];
	int16_t raw_mz = ((uint16_t) out[19] << 8) | out[18];
	// ESP_LOGI("TAG", "Raw Accel: %d, %d, %d", raw_ax, raw_ay, raw_az);

	data->accel_x = ((float)raw_ax / accel_sensitivity_scale_factor) * EARTH_GRAVITY;
	data->accel_y = ((float)raw_ay / accel_sensitivity_scale_factor) * EARTH_GRAVITY;
	data->accel_z = ((float)raw_az / accel_sensitivity_scale_factor) * EARTH_GRAVITY;

	data->gyro_x = (float)raw_gx / gyro_sensitivity_scale_factor;
	data->gyro_y = (float)raw_gy / gyro_sensitivity_scale_factor;
	data->gyro_z = (float)raw_gz / gyro_sensitivity_scale_factor;

	data->gyro_x = (float)raw_mx * MAG_SENSITIVITY_SCALE_FACTOR;
	data->gyro_y = (float)raw_my * MAG_SENSITIVITY_SCALE_FACTOR;
	data->gyro_z = (float)raw_mz * MAG_SENSITIVITY_SCALE_FACTOR;

	// ESP_LOGI(TAG, "accel factor should be 16384 was %d", accel_sensitivity_scale_factor);
	free(out);
	return BIODYN_IMU_OK;
}

/**
 * Writes to a register in the magnetometer of the IMU.
 * Requires sufficient delay due to I2C interactions between icm20948 and ak09916.
 * @param register_address the register address to write to within the ak09916 register bank
 * @param data the data to write within the requested register
 * Take care to ensure the register you write to is an ak09916 register and not an icm20948 register.
 */
static biodyn_imu_err_t biodyn_imu_ak09916_write_reg(uint8_t register_address, uint8_t data)
{
	// Set slave0 to be the built in magnetometer
	// Or first bit with 0 in order to indicate write
	biodyn_imu_icm20948_write_reg(_b3, I2C_SLV0_ADDR, 0x00 | AK09916_ADDRESS);
	// Set the register to write to
	biodyn_imu_icm20948_write_reg(_b3, I2C_SLV0_REG, register_address);
	// Set the data to write
	biodyn_imu_icm20948_write_reg(_b3, I2C_SLV0_DO, data);

	// Enable and single data write
	// TODO: what does this mean?
	biodyn_imu_icm20948_write_reg(_b3, I2C_SLV0_CTRL, 0x80 | 0x01);
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
static biodyn_imu_err_t biodyn_imu_ak09916_read_reg(uint8_t register_address, uint8_t len)
{
	// Set slave0 to be the built in magnetometer
	// Or first bit with 1 in order to indicate read
	biodyn_imu_icm20948_write_reg(_b3, I2C_SLV0_ADDR, 0x80 | AK09916_ADDRESS);
	// Set the register to read from
	biodyn_imu_icm20948_write_reg(_b3, I2C_SLV0_REG, register_address);

	// Enable and single data write
	// TODO: see biodyn_imu_ak09916_write_reg function
	// Bits [3:0] in I2C_SLV0_CTRL are the len to read (if capped)
	biodyn_imu_icm20948_write_reg(_b3, I2C_SLV0_CTRL, 0x80 | len);
	// Delay to allow I2C transaction
	vTaskDelay(pdMS_TO_TICKS(50));

	return BIODYN_IMU_OK;
}

/**
 * Initializes the magnetometer (i.e. the ak09916) within the IMU (via the icm20948).
 * Prerequisitely, the icm20948 must be somewhat initialized first.
 */
static biodyn_imu_err_t biodyn_imu_icm20948_init_magnetomter()
{
	uint8_t temp;
	biodyn_imu_icm20948_read_reg(_b0, USER_CTRL, &temp);
	temp |= 0x02;
	// Resets the I2C master module by writing 1 to the bit
	// this bit set should auto clear after one clock cycle
	// (internally at 20Mhz)
	biodyn_imu_icm20948_write_reg(_b0, USER_CTRL, temp);
	// Wait for bit to auto clear
	vTaskDelay(pdMS_TO_TICKS(100));

	// Enable I2C by writing 1 to bit 5 in USER_CTRL
	// TODO: Do I need to read again here
	// Yes, the I2C reset can change bit 4 and 5, so new read is necessary
	biodyn_imu_icm20948_read_reg(_b0, USER_CTRL, &temp);
	temp |= 0x20;
	// Write 1 to bit 5 in USER_CTRL
	biodyn_imu_icm20948_write_reg(_b0, USER_CTRL, temp);

	// Documentation states (p. 81):
	/*
	 To achieve a targeted clock
	frequency of 400 kHz, MAX, it is recommended to set I2C_MST_CLK = 7 (345.6 kHz / 46.67% duty cycle).
	*/
	temp = 0x07;
	biodyn_imu_icm20948_write_reg(_b3, I2C_MST_CTRL, temp);

	// Set for a custom rate to be used for sampling the magnetometer
	temp = 0x40;
	biodyn_imu_icm20948_write_reg(_b0, LP_CONFIG, temp);
	// Set data rate as 136Hz
	// Formula: 1.1 kHz/(2^((odr_config[3:0])) )
	temp = 0x03;
	biodyn_imu_icm20948_write_reg(_b3, I2C_MST_ODR_CONFIG, temp);

	// Reset magnetometer
	biodyn_imu_ak09916_write_reg(AK09916_CONTROL3, 0x01);
	// Delay to allow reset
	vTaskDelay(pdMS_TO_TICKS(100));

	// Start continuous mode:
	// // Roughly sampling constantly at 100khz (Standard-mode p. 15)
	biodyn_imu_ak09916_write_reg(AK09916_CONTROL2, 0x08);

	// Magnetometer ready for use!
	return BIODYN_IMU_OK;
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