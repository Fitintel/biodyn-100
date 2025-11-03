#include "imu_icm20948_driver.h"
#include "imu_icm20948_consts.h"

#include "hal/spi_types.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "math.h"
#include "freertos/task.h"
#include "string.h"
#include "esp_mac.h"
#include "esp_system.h"

static uint8_t accel_range_value = _accel_2g;
static uint8_t gyro_range_value = _gyro_1000dps;
static uint16_t accel_sensitivity_scale_factor = 0;
static float gyro_sensitivity_scale_factor = -1;

// Checks and returns IMU error provided by expr
#define IMU_ERR_CHECK(expr)                         \
	{                                               \
		biodyn_imu_err_t __imuerror;                \
		if ((__imuerror = (expr)) != BIODYN_IMU_OK) \
		{                                           \
			return __imuerror;                      \
		}                                           \
	}

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
static biodyn_imu_err_t set_user_bank(user_bank_range bank);
// Reads the user bank of registers
static biodyn_imu_err_t get_user_bank(uint8_t *bank_out);
// Writes data to a single register specified by a bank and address to the IMU
static biodyn_imu_err_t write_reg(user_bank_range bank, uint16_t register_address, uint8_t write_data);
// Reads data of a single register specified by a bank and address of the IMU
static biodyn_imu_err_t read_reg(user_bank_range bank, uint16_t register_address, uint8_t *out);
// Write a byte to the magnetometer attached to the IMU
static biodyn_imu_err_t biodyn_imu_ak09916_write_reg(uint8_t reg, uint8_t data);
// Read multiple bytes from the magnetometer attached to the IMU
static biodyn_imu_err_t biodyn_imu_ak09916_read_reg(uint8_t reg, uint8_t len);
// Self-tests the whoami
biodyn_imu_err_t self_test_whoami();
// Collect IMU errors
biodyn_imu_err_t collect_err(biodyn_imu_err_t error, char *optional_attached_message);

// Initializes the SPI bus
biodyn_imu_err_t init_spi()
{
	// Create bus config
	spi_bus_config_t bus_config = {
		.miso_io_num = imu_data.i2c.miso,
		.mosi_io_num = imu_data.i2c.mosi,
		.sclk_io_num = imu_data.i2c.sclk,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
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
		.mode = 0,					   // Compatible with mode 0 and mode 3
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

	return BIODYN_IMU_OK;
}

// Wake IMU and perform basic configuration
biodyn_imu_err_t wake_imu()
{
	// Reset chip
	IMU_ERR_CHECK(write_reg(_b0, PWR_MGMT_1, 0x80));

	// Wait for reset
	vTaskDelay(pdMS_TO_TICKS(100));

	// Check chip WHOAMI
	IMU_ERR_CHECK(self_test_whoami());

	// Select best available clock (CLKSEL = 1) and turn sleep off
	IMU_ERR_CHECK(write_reg(_b0, PWR_MGMT_1, 0x01));

	// Enable accel and gyro
	IMU_ERR_CHECK(write_reg(_b0, PWR_MGMT_2, 0x00));

	// Disable all low power (duty cycling) modes for accel, gyro, mag
	IMU_ERR_CHECK(write_reg(_b0, LP_CONFIG, 0x00));

	return BIODYN_IMU_OK;
}

// Sets IMU to SPI-only mode and configures the internal I2C for magnetometer
biodyn_imu_err_t imu_config_i2c()
{
	// SPI-only and turn on internal I2C master
	uint8_t uc = 0;
	IMU_ERR_CHECK(read_reg(_b0, USER_CTRL, &uc));
	uc |= 0x20; // I2C_MST_EN = 1 -> enable internal I2C for mag
	uc |= 0x10; // I2C_IF_DIS = 1 -> put in SPI-only mode
	IMU_ERR_CHECK(write_reg(_b0, USER_CTRL, uc));

	// Single-master mode and 345 kHz internal I2C clock
	IMU_ERR_CHECK(write_reg(_b3, I2C_MST_CTRL, 0x07));

	// Disable I2C access of mag from external chips
	IMU_ERR_CHECK(write_reg(_b0, INT_PIN_CFG, 0x00));

	// Align I2C mag reads with the timing of accel,gyro reads so our data
	// has consistent timing.
	// We do not need I2C_MST_DELAY_CTRL or I2C_MST_DELAY_CTRL.
	IMU_ERR_CHECK(write_reg(_b2, ODR_ALIGN_EN, 0x01));

	// Reset I2C master
	// IMU_ERR_CHECK(write_reg(_b0, USER_CTRL, (uc | 0x02) & ~0x20));
	// vTaskDelay(pdMS_TO_TICKS(10));
	// IMU_ERR_CHECK(write_reg(_b0, USER_CTRL, uc | 0x20)); // Re-enable I2C master
	// vTaskDelay(pdMS_TO_TICKS(10));

	vTaskDelay(pdMS_TO_TICKS(10));
	return BIODYN_IMU_OK;
}

biodyn_imu_err_t write_mag(uint8_t reg, uint8_t value)
{
	// Set addr in IMU which is the internal I2C addr to write to
	IMU_ERR_CHECK(write_reg(_b3, I2C_SLV4_ADDR, AK09916_ADDRESS_WRITE));
	// Set the value to write
	IMU_ERR_CHECK(write_reg(_b3, I2C_SLV4_DO, value));
	// Set the I2C register of the given addr to write to
	IMU_ERR_CHECK(write_reg(_b3, I2C_SLV4_REG, reg));
	// Enable this slave
	IMU_ERR_CHECK(write_reg(_b3, I2C_SLV4_CTRL, 0x80));

	// Wait for write to complete by checking SLV4_CTRL EN bit
	uint8_t st;
	TickType_t t0 = xTaskGetTickCount();
	do
	{
		IMU_ERR_CHECK(read_reg(_b3, I2C_SLV4_CTRL, &st));
		if ((xTaskGetTickCount() - t0) > pdMS_TO_TICKS(150))
			return collect_err(BIODYN_IMU_ERR_NO_EXT_ACK, "Mag write timed out");
	} while ((st & 0x80) != 0);
	return BIODYN_IMU_OK;
}

biodyn_imu_err_t read_mag(uint8_t reg, uint8_t *out)
{
	// Set addr in IMU which is the internal I2C addr to write to
	IMU_ERR_CHECK(write_reg(_b3, I2C_SLV4_ADDR, AK09916_ADDRESS_READ));
	// Set the value to write
	// Set the I2C register of the given addr to write to
	IMU_ERR_CHECK(write_reg(_b3, I2C_SLV4_REG, reg));
	// Enable this slave
	IMU_ERR_CHECK(write_reg(_b3, I2C_SLV4_CTRL, 0x81));

	// Wait for write to complete by checking SLV4_CTRL EN bit
	uint8_t st;
	TickType_t t0 = xTaskGetTickCount();
	do
	{
		IMU_ERR_CHECK(read_reg(_b3, I2C_SLV4_CTRL, &st));
		if ((xTaskGetTickCount() - t0) > pdMS_TO_TICKS(150))
			return collect_err(BIODYN_IMU_ERR_NO_EXT_ACK, "Mag read timed out");
	} while ((st & 0x80) != 0);

	// Put data in out
	IMU_ERR_CHECK(read_reg(_b3, I2C_SLV4_DI, out));
	return BIODYN_IMU_OK;
}

// Configures the magnetometer
biodyn_imu_err_t init_mag()
{
	uint8_t val;
	{
		ESP_LOGI(TAG, "Register configuration: ");
		IMU_ERR_CHECK(read_reg(_b0, LP_CONFIG, &val));
		ESP_LOGI(TAG, "\tLP_CONFIG: 0x%02x", val);
		IMU_ERR_CHECK(read_reg(_b0, USER_CTRL, &val));
		ESP_LOGI(TAG, "\tUSER_CTRL: 0x%02x", val);
		IMU_ERR_CHECK(read_reg(_b3, I2C_MST_CTRL, &val));
		ESP_LOGI(TAG, "\tI2C_MST_CTRL: 0x%02x", val);
		IMU_ERR_CHECK(read_reg(_b0, INT_PIN_CFG, &val));
		ESP_LOGI(TAG, "\tINT_PIN_CFG: 0x%02x", val);
		IMU_ERR_CHECK(read_reg(_b0, PWR_MGMT_1, &val));
		ESP_LOGI(TAG, "\tPWR_MGMT_1: 0x%02x", val);
		IMU_ERR_CHECK(read_reg(_b0, PWR_MGMT_2, &val));
		ESP_LOGI(TAG, "\tPWR_MGMT_2: 0x%02x", val);
		IMU_ERR_CHECK(read_reg(_b0, ODR_ALIGN_EN, &val));
		ESP_LOGI(TAG, "\tODR_ALIGN_EN: 0x%02x", val);
	}

	// We're going to reset the mag by writing 0x01 to CNTL3
	write_mag(AK09916_CONTROL3, 0x01);
	vTaskDelay(pdMS_TO_TICKS(120));

	// Verify the AK09916 WHOAMI (WIA2)
	uint8_t mag_whoami = 0;
	IMU_ERR_CHECK(read_mag(AK09916_WHOAMI, &mag_whoami));
	// Check if this is what we expect
	if (mag_whoami != 0x09)
	{
		ESP_LOGE(TAG, "Wrong magnetometer WHOAMI: 0x%02x", mag_whoami);
		return collect_err(BIODYN_IMU_ERR_WRONG_WHOAMI, "Incorrect magnetometer WHOAMI");
	}

	// Second step is to enable continuous reading
	// -> Set starting reg address for next op to CNTL2 register
	// IMU_ERROR_CHECK(write_reg(_b3, I2C_SLV0_REG, AK09916_CONTROL2));

	return BIODYN_IMU_OK;
}

// Initializes the IMU
biodyn_imu_err_t biodyn_imu_icm20948_init()
{
	// Initialize SPI
	IMU_ERR_CHECK(init_spi());
	ESP_LOGI(TAG, "\tSPI initialized");

	// Wake up the IMU
	IMU_ERR_CHECK(wake_imu());
	ESP_LOGI(TAG, "\tIMU woke up");

	// Configure SPI mode and imu internal I2C
	IMU_ERR_CHECK(imu_config_i2c());
	ESP_LOGI(TAG, "\tConfigured IMU I2C and SPI");

	// Init magnetometer
	biodyn_imu_err_t err = BIODYN_IMU_OK;
	int tries = 0;
	while ((err = init_mag()) != BIODYN_IMU_OK)
	{
		if (tries > 10)
			return collect_err(BIODYN_IMU_ERR_COULDNT_INIT_MAG, "Failed to initialize magnetometer after multiple tries");
		vTaskDelay(pdMS_TO_TICKS(100));
		++tries;
	}
	ESP_LOGI(TAG, "\tInitialized magnetometer");

	// Initialize configuration data
	accel_sensitivity_scale_factor = (1 << (uint16_t)(14 - accel_range_value));
	gyro_sensitivity_scale_factor = 16.4 * (1 << (3 - gyro_range_value));
	ESP_LOGI(TAG, "\tPlanar sensitivity factor: %d", accel_sensitivity_scale_factor);
	ESP_LOGI(TAG, "\tGyro sensitivity factor: %f", gyro_sensitivity_scale_factor);

	ESP_LOGI(TAG, "Initialized IMU");

	return BIODYN_IMU_OK;
}

/** Sets the user bank to @param bank
 * 	Range of [0, 3]
 */
biodyn_imu_err_t set_user_bank(user_bank_range bank)
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
biodyn_imu_err_t read_reg(user_bank_range bank, uint16_t register_address, uint8_t *out)
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
biodyn_imu_err_t biodyn_imu_icm20948_multibyte_read_reg(user_bank_range bank, uint16_t register_address, uint8_t *out, uint8_t length)
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
biodyn_imu_err_t write_reg(user_bank_range bank, uint16_t register_address, uint8_t write_data)
{
	// Select user bank to write to
	set_user_bank(bank);

	// Use input register address OR with WRITE_MSB (0), with write data as second argument
	uint8_t tx_data[2] = {register_address | WRITE_MSB, write_data};
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
biodyn_imu_err_t self_test_whoami()
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
	deltas.accel_x = imd_st_on.accel_x - imd_st_off.accel_x;
	deltas.accel_y = imd_st_on.accel_y - imd_st_off.accel_y;
	deltas.accel_z = imd_st_on.accel_z - imd_st_off.accel_z;
	deltas.gyro_x = imd_st_on.gyro_x - imd_st_off.gyro_x;
	deltas.gyro_y = imd_st_on.gyro_y - imd_st_off.gyro_y;
	deltas.gyro_z = imd_st_on.gyro_z - imd_st_off.gyro_z;

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
	ESP_LOGI(TAG, "\n st_off ax: %f\n st_off ay: %f\n st_off az: %f\n st_off gx: %f\n st_off gy: %f\n st_off gz: %f", imd_st_off.accel_x, imd_st_off.accel_y, imd_st_off.accel_z, imd_st_off.gyro_x, imd_st_off.gyro_y, imd_st_off.gyro_z);
	ESP_LOGI(TAG, "\n st_on ax: %f\n st_on ay: %f\n st_on az: %f\n st_on gx: %f\n st_on gy: %f\n st_on gz: %f", imd_st_on.accel_x, imd_st_on.accel_y, imd_st_on.accel_z, imd_st_on.gyro_x, imd_st_on.gyro_y, imd_st_on.gyro_z);

	ESP_LOGI(TAG, "\n delta ax: %f\n delta ay: %f\n delta az: %f\n delta gx: %f\n delta gy: %f\n delta gz: %f", deltas.accel_x, deltas.accel_y, deltas.accel_z, deltas.gyro_x, deltas.gyro_y, deltas.gyro_z);
	ESP_LOGI(TAG, "\n trimax: %d\ntrimay: %d\n trimaz: %d\n trimgx: %d\n trimgy: %d\n trimgz: %d", trimax, trimay, trimaz, trimgx, trimgy, trimgz);
	/** DEBUG */

	// CHECK if deltas-trim is between rangel*trim and rangeh*trim
	// return error on fails
	ESP_LOGI(TAG, "checking against trims");
	if (RL * trimax < deltas.accel_x / (1 << accel_range_value) && RH * trimax > deltas.accel_x / (1 << accel_range_value))
	{
		ESP_LOGI(TAG, "Self-test succesful for accel_gyro on ax");
	}
	else
	{
		ESP_LOGE(TAG, "self test failed on delta ax with ax = %f and trimax = %d", deltas.accel_x, trimax);
		collect_err(BIODYN_IMU_ERR_COULDNT_CONFIGURE, "SELF_TEST_ACCEL_GYRO: failed self-test on ax");
		return BIODYN_IMU_ERR_COULDNT_CONFIGURE;
	}
	if (RL * trimay < deltas.accel_y / (1 << accel_range_value) && RH * trimay > deltas.accel_y / (1 << accel_range_value))
	{
		ESP_LOGI(TAG, "Self-test succesful for accel_gyro on ay");
	}
	else
	{
		ESP_LOGE(TAG, "self test failed on delta ay");

		collect_err(BIODYN_IMU_ERR_COULDNT_CONFIGURE, "SELF_TEST_ACCEL_GYRO: failed self-test on ay");
		return BIODYN_IMU_ERR_COULDNT_CONFIGURE;
	}
	if (RL * trimaz < deltas.accel_z / (1 << accel_range_value) && RH * trimaz > deltas.accel_z / (1 << accel_range_value))
	{
		ESP_LOGI(TAG, "Self-test succesful for accel_gyro on az");
	}
	else
	{
		ESP_LOGE(TAG, "self test failed on delta az");
		collect_err(BIODYN_IMU_ERR_COULDNT_CONFIGURE, "SELF_TEST_ACCEL_GYRO: failed self-test on az");
		return BIODYN_IMU_ERR_COULDNT_CONFIGURE;
	}
	if (RL * trimgx < deltas.gyro_x / (1 << gyro_range_value) && RH * trimgx > deltas.gyro_x / (1 << gyro_range_value))
	{
		ESP_LOGI(TAG, "Self-test succesful for accel_gyro on gx");
	}
	else
	{
		uint8_t and1 = RL * trimgx < deltas.gyro_x / (1 << gyro_range_value);
		uint8_t and2 = RH * trimgx > deltas.gyro_x / (1 << gyro_range_value);
		ESP_LOGE(TAG, "failed with delta_gx = %f and trimgx = %d", deltas.gyro_x, trimgx);
		ESP_LOGE(TAG, "and1 = %d; and2 = %d", and1, and2);
		collect_err(BIODYN_IMU_ERR_COULDNT_CONFIGURE, "SELF_TEST_ACCEL_GYRO: failed self-test on gx");
		return BIODYN_IMU_ERR_COULDNT_CONFIGURE;
	}
	if (RL * trimgy < deltas.gyro_y / (1 << gyro_range_value) && RH * trimgy > deltas.gyro_y / (1 << gyro_range_value))
	{
		ESP_LOGI(TAG, "Self-test succesful for accel_gyro on gy");
	}
	else
	{
		uint8_t and1 = RL * trimgy < deltas.gyro_y / (1 << gyro_range_value);
		uint8_t and2 = RH * trimgy > deltas.gyro_y / (1 << gyro_range_value);
		ESP_LOGE(TAG, "failed with delta_gx = %f and trimgx = %d", deltas.gyro_y, trimgy);
		ESP_LOGE(TAG, "and1 = %d; and2 = %d", and1, and2);
		collect_err(BIODYN_IMU_ERR_COULDNT_CONFIGURE, "SELF_TEST_ACCEL_GYRO: failed self-test on gy");
		return BIODYN_IMU_ERR_COULDNT_CONFIGURE;
	}
	if (RL * trimgz < deltas.gyro_z / (1 << gyro_range_value) && RH * trimgz > deltas.gyro_z / (1 << gyro_range_value))
	{
		ESP_LOGI(TAG, "Self-test succesful for accel_gyro on gz");
	}
	else
	{
		uint8_t and1 = RL * trimgz < deltas.gyro_z / (1 << gyro_range_value);
		uint8_t and2 = RH * trimgz > deltas.gyro_z / (1 << gyro_range_value);
		ESP_LOGE(TAG, "failed with delta_gx = %f and trimgx = %d", deltas.gyro_z, trimgy);
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

// TESTING accel_gyro replacement
/*static biodyn_imu_err_t self_test_accel()
{
	// SELF-TEST RESPONSE = SENSOR OUTPUT WITH SELF-TEST ENABLED – SENSOR OUTPUT WITHOUT SELF-TEST ENABLED (p. 24)
	// Sensor output without self_test_enabled
	uint8_t st_off_xh = 0;

	// biodyn_imu_icm20948_read_reg(_b0, ACCEL_XOUT_H, )

	// Use accel_config_2 to start self test on all axes
	uint8_t temp = 0;
	biodyn_imu_icm20948_read_reg(_b2, ACCEL_CONFIG_2, &temp);
	// [7:5] reserved, [4:2] xyz self-test enables, [1:0] accel sample decimator (see function ...accel_number_samples_averaged)
	temp |= 0b00011100;
	biodyn_imu_icm20948_write_reg(_b2, ACCEL_CONFIG_2, temp);
	// SELF_TEST ACCEL STARTED
	return BIODYN_IMU_OK;
}


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
*/

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
		ESP_LOGE(TAG, "\tSelf test failed - whoami (%x)", err);
		return err;
	}
	ESP_LOGI(TAG, "\tCorrect WHOAMI");

	// Check that user bank selection works
	if ((err = self_test_user_banks()))
	{
		ESP_LOGE(TAG, "\tSelf test failed - user banks (%x)", err);
		return err;
	}
	ESP_LOGI(TAG, "\tUser banks working");
	// // Run gyro self-test
	// if ((err = self_test_gyro()))
	// {
	// 	ESP_LOGE(TAG, "Self test failed - gyro (%x)", err);
	// }
	// // Run accel self-test
	// if ((err = self_test_accel()))
	// {
	// 	ESP_LOGE(TAG, "Self test failed - accel (%x)", err);
	// }

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
	uint8_t read_out_length = 6;
	uint8_t *read_out = malloc(sizeof(uint8_t) * read_out_length);

	biodyn_imu_icm20948_multibyte_read_reg(_b0, MAG_XOUT_H, read_out, read_out_length);

	// Byte shifting for full high and low register with proper endianness
	int16_t raw_ax = (int16_t)((read_out[0] << 8) | read_out[1]);
	int16_t raw_ay = (int16_t)((read_out[2] << 8) | read_out[3]);
	int16_t raw_az = (int16_t)((read_out[4] << 8) | read_out[5]);

	float *fout = (float *)out;
	*size = 3 * sizeof(float);
	fout[0] = ((float)raw_ax * MAG_SENSITIVITY_SCALE_FACTOR);
	fout[1] = ((float)raw_ay * MAG_SENSITIVITY_SCALE_FACTOR);
	fout[2] = ((float)raw_az * MAG_SENSITIVITY_SCALE_FACTOR);

	// TEST: read status2 register of magnetometer as required in p. 79 after each measurement
	biodyn_imu_ak09916_read_reg(AK09916_STATUS2, 1);

	// ESP_LOGI(TAG, "accel factor should be 16384 was %d", accel_sensitivity_scale_factor);
	free(read_out);
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
	uint8_t out_length = 9 * sizeof(uint16_t) + 2 * sizeof(uint8_t);
	uint8_t out[32];

	biodyn_imu_icm20948_multibyte_read_reg(_b0, ACCEL_XOUT_H, out, out_length);

	// Byte shifting for full high and low register with proper endianness
	int16_t raw_ax = ((uint16_t)out[0] << 8) | out[1];
	int16_t raw_ay = ((uint16_t)out[2] << 8) | out[3];
	int16_t raw_az = ((uint16_t)out[4] << 8) | out[5];
	int16_t raw_gx = ((uint16_t)out[6] << 8) | out[7];
	int16_t raw_gy = ((uint16_t)out[8] << 8) | out[9];
	int16_t raw_gz = ((uint16_t)out[10] << 8) | out[11];
	// gap of two bytes between accel + gyro and mag for temperature registers
	int16_t raw_mx = ((uint16_t)out[15] << 8) | out[14];
	int16_t raw_my = ((uint16_t)out[17] << 8) | out[16];
	int16_t raw_mz = ((uint16_t)out[19] << 8) | out[18];
	// ESP_LOGI("TAG", "Raw Accel: %d, %d, %d", raw_ax, raw_ay, raw_az);
	// ESP_LOGI("TAG", "Raw gyro: %d, %d, %d", raw_gx, raw_gy, raw_gz);
	// ESP_LOGI(TAG, "Sens factor %f", gyro_sensitivity_scale_factor);

	data->accel_x = ((float)raw_ax / accel_sensitivity_scale_factor) * EARTH_GRAVITY;
	data->accel_y = ((float)raw_ay / accel_sensitivity_scale_factor) * EARTH_GRAVITY;
	data->accel_z = ((float)raw_az / accel_sensitivity_scale_factor) * EARTH_GRAVITY;

	data->gyro_x = (float)raw_gx / gyro_sensitivity_scale_factor;
	data->gyro_y = (float)raw_gy / gyro_sensitivity_scale_factor;
	data->gyro_z = (float)raw_gz / gyro_sensitivity_scale_factor;

	data->mag_x = (float)raw_mx * MAG_SENSITIVITY_SCALE_FACTOR;
	data->mag_y = (float)raw_my * MAG_SENSITIVITY_SCALE_FACTOR;
	data->mag_z = (float)raw_mz * MAG_SENSITIVITY_SCALE_FACTOR;

	// TEST: read status2 register of magnetometer as required in p. 79 after each measurement
	// biodyn_imu_ak09916_read_reg(AK09916_STATUS2, 1);

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
	write_reg(_b3, I2C_SLV0_ADDR, 0x00 | AK09916_ADDRESS);
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
static biodyn_imu_err_t biodyn_imu_ak09916_read_reg(uint8_t register_address, uint8_t len)
{
	// Set slave0 to be the built in magnetometer
	// Or first bit with 1 in order to indicate read
	write_reg(_b3, I2C_SLV0_ADDR, 0x80 | AK09916_ADDRESS);
	// Set the register to read from
	write_reg(_b3, I2C_SLV0_REG, register_address);

	// Enable and single data write
	// TODO: see biodyn_imu_ak09916_write_reg function
	// Bits [3:0] in I2C_SLV0_CTRL are the len to read (if capped)
	write_reg(_b3, I2C_SLV0_CTRL, 0x80 | len);
	// Delay to allow I2C transaction
	vTaskDelay(pdMS_TO_TICKS(50));

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

biodyn_imu_err_t collect_err(biodyn_imu_err_t error, char *optional_attached_message)
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
	case BIODYN_IMU_ERR_NO_EXT_ACK:
		snprintf(error_msg, sizeof(error_msg), "IMU_EXT_NOT_ACKD");
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
		strncat(error_msg, optional_attached_message, sizeof(error_msg) - strlen(error_msg) - 1);

	strncpy(biodyn_imu_icm20948_errors[biodyn_imu_icm20948_error_index],
			error_msg,
			sizeof(biodyn_imu_icm20948_errors[biodyn_imu_icm20948_error_index]) - 1);

	biodyn_imu_icm20948_error_index = (biodyn_imu_icm20948_error_index + 1) % 3;
	return error;
}
