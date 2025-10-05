#ifndef BIODYN_TEST_BLE_SERVICE
#define BIODYN_TEST_BLE_SERVICE

#include "constants.h"
#include "bluetooth/ble.h"
#include "system/led.h"
#include "imu/imu_icm20948_driver.h"
#include "string.h"

void imu_icm20948_get_state(uint16_t *len, void *dst)
{
	imu_motion_data data = {0};
	biodyn_imu_icm20948_read_accel_gyro_mag(&data);
	*len = sizeof(imu_motion_data);
	// Since imu_motion_data is a struct solely of floats, it can (allegedly) be directly copied for output
	memcpy(dst, &data, *len);
	// copying from address of imu data for its length (len)
}

const static struct biodyn_ble_characteristic sensor_test_chars[] = {
	{
		.name = "LED Control",
		.uuid = BIODYN_BLE_UUID_16(0x1235),
		.permissions = BIODYN_PERM_READ_WRITE,
		.properties = BIODYN_PROP_READ_WRITE,
		.set_data = led_set_state,
		.get_data = led_get_state,
	},
	{
		.name = "IMU Control",
		.uuid = BIODYN_BLE_UUID_16(0x1236),
		.permissions = BIODYN_PERM_READ,
		.properties = BIODYN_PROP_READ,
		.get_data = imu_icm20948_get_state,
	}};

const static struct biodyn_ble_service test_service = {
	.name = "Test Sensor Service",
	.service_id = {
		.is_primary = true,
		.id = {
			.inst_id = 0,
			.uuid = {
				.uuid = {
					.uuid128 = {
						0xFB,
						0x34,
						0x9B,
						0x5F,
						0x80,
						0x00,
						0x00,
						0x80,
						0x00,
						0x10,
						0x00,
						0x00,
						0xFF,
						0xFF,
						0x00,
						0x00,
					},
				},
				.len = ESP_UUID_LEN_128,
			},
		},
	},
	.n_characteristics = LEN_OF_STATIC_ARRAY(sensor_test_chars),
	.characteristics = &sensor_test_chars[0],
};

#endif // BIODYN_TEST_BLE_SERVICE