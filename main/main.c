#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/event_groups.h"
// #include "freertos/semphr.h"
#include <unistd.h>

#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "sdkconfig.h"

#include "constants.h"
#include "system/nvs.h"
#include "bluetooth/ble.h"
#include "bluetooth/ble_profiles.h"
#include "accel/imu_icm20948_driver.h"

void test_accel_imu_icm20948();
void test_all_registers_imu_icm20948();
void test_accel_gyro_imu_icm20948();


// APP ENTRY POINT
void app_main(void)
{
	esp_err_t err;

	ESP_LOGI(MAIN_TAG, "Starting %s", BIODYN_DEVICE_NAME);

	// Initialize persistant storage (nvs)
	if ((err = biodyn_nvs_init()))
	{
		ESP_LOGE(MAIN_TAG, "Failed to initialize non-volatile storage in %s, error code %x", __func__, err);
		return;
	}

	// Initialize LED
	if ((err = biodyn_led_init()))
	{
		ESP_LOGE(MAIN_TAG, "Failed to initialize LED module, err = %d", err);
		return;
	}

	// Initialize IMU
	if ((err = biodyn_imu_icm20948_init()))
	{
		ESP_LOGE(MAIN_TAG, "Failed to initialize IMU module, err = %d", err);
		return;
	}

	// Initialize bluetooth
	if ((err = biodyn_ble_init(LEN_OF_STATIC_ARRAY(profiles), &profiles[0])))
	{
		ESP_LOGE(MAIN_TAG, "Failed to initialize Bluetooth in %s, err code %x", __func__, err);
		return;
	}
  
  	// Set up!
	ESP_LOGI(MAIN_TAG, "Finished setup");
  
// 	test_all_registers_imu_icm20948();
// 	test_accel_imu_icm20948();
// 	test_accel_gyro_imu_icm20948();
}

void test_accel_imu_icm20948()
{
	esp_err_t err;
	int16_t output;

	// vTaskDelay(pdMS_TO_TICKS(100));
	err = self_test_accel(&output);
	if (err != BIODYN_IMU_OK)
	{
		ESP_LOGE(MAIN_TAG, "ERROR READING IMU ICM20948 DATA");
	}
	else
	{
		ESP_LOGI(MAIN_TAG, "GOT X ACCEL LOW AS %d", output);
	}

	for (int i = 0; i < 20000; ++i)
	{
		// vTaskDelay(pdMS_TO_TICKS(100));
		err = self_test_accel(&output);
		if (err != BIODYN_IMU_OK)
		{
			ESP_LOGE(MAIN_TAG, "ERROR READING IMU ICM20948 DATA");
			continue;
		}
		ESP_LOGI(MAIN_TAG, "GOT X ACCEL AS %d %s", output, IMU_ACCEL_UNIT);
		vTaskDelay(pdMS_TO_TICKS(100)); // Sleep for 1000 ms (1 second)
	}
	return;
}

// Test for all addresses to see readable registers by non-zero values
void test_all_registers_imu_icm20948()
{
	uint8_t out;
	// BANK 0
	for (uint8_t address = 0x00; address < 0x7f; ++address)
	{
		if (biodyn_imu_icm20948_read_register_test(_b0, address, &out))
		{
			ESP_LOGE(MAIN_TAG, "FAILED to read bank %d, register %x", _b0 / 16, address);
			continue;
		}
		ESP_LOGI(MAIN_TAG, "TESTED bank %d, address %x with read value: %x", _b0 / 16, address, out);
	}
	// BANK 1
	for (uint8_t address = 0x02; address < 0x28; ++address)
	{
		if (biodyn_imu_icm20948_read_register_test(_b1, address, &out))
		{
			ESP_LOGE(MAIN_TAG, "FAILED to read bank %d, register %x", _b1 / 16, address);
			continue;
		}
		ESP_LOGI(MAIN_TAG, "TESTED bank %d, address %x with read value: %x", _b1 / 16, address, out);
	}
	// BANK 2
	for (uint8_t address = 0x00; address < 0x15; ++address)
	{
		if (biodyn_imu_icm20948_read_register_test(_b2, address, &out))
		{
			ESP_LOGE(MAIN_TAG, "FAILED to read bank %d, register %x", _b2 / 16, address);
			continue;
		}
		ESP_LOGI(MAIN_TAG, "TESTED bank %d, address %x with read value: %x", _b2 / 16, address, out);
	}
	// Below bank 3 not used, not checked
	// BANK 3
	// for (uint8_t address = 0x00; address < 0x17; ++address)
	// {
	// 	if (biodyn_imu_icm20948_read_register_test(_b3, address, &out))
	// 	{
	// 		ESP_LOGE(MAIN_TAG, "FAILED to read bank %d, register %x", _b3, address);
	// 		continue;
	// 	}
	// 	ESP_LOGI(MAIN_TAG, "TESTED bank %d, address %x with read value: %x", _b3, address, out);
	// }

	return;
}

void test_accel_gyro_imu_icm20948()
{
	imu_motion_data data = {0};
	for (int i = 0; i < 2000; ++i)
	{
		biodyn_imu_icm20948_read_accel_gyro(&data);
		ESP_LOGI(MAIN_TAG, "ACCEL X: %.3f %s", data.accel_x, IMU_ACCEL_UNIT);
		ESP_LOGI(MAIN_TAG, "ACCEL Y: %.3f %s", data.accel_y, IMU_ACCEL_UNIT);
		ESP_LOGI(MAIN_TAG, "ACCEL Z: %.3f %s", data.accel_z, IMU_ACCEL_UNIT);
		ESP_LOGI(MAIN_TAG, "GYRO X: %.3f %s", data.gyro_x, IMU_GYRO_UNIT);
		ESP_LOGI(MAIN_TAG, "GYRO Y: %.3f %s", data.gyro_y, IMU_GYRO_UNIT);
		ESP_LOGI(MAIN_TAG, "GYRO Z: %.3f %s", data.gyro_z, IMU_GYRO_UNIT);
		vTaskDelay(pdMS_TO_TICKS(100)); // Sleep for 100 ms (0.1 second)
	}
}