#ifndef BIODYN_BLE_PROFILES_H
#define BIODYN_BLE_PROFILES_H

#include "constants.h"
#include "ble.h"
#include "led.h"
#include "imu_icm20948_driver.h"

static struct biodyn_ble_characteristic device_info_chars[] = {
	{
		.name = "Manufacturer Name String",
		.uuid = BIODYN_BLE_UUID_16(0x2A29),
		.permissions = ESP_GATT_PERM_READ,
		.properties = ESP_GATT_CHAR_PROP_BIT_READ,
		.initial_value = (void *)BIODYN_MANUFACTURER_NAME,
		.intial_value_size = sizeof(BIODYN_MANUFACTURER_NAME),
	},
	{
		.name = "Firmware Revision String",
		.uuid = BIODYN_BLE_UUID_16(0x2A26),
		.permissions = ESP_GATT_PERM_READ,
		.properties = ESP_GATT_CHAR_PROP_BIT_READ,
		.initial_value = (void *)BIODYN_FIRMWARE_VERSION,
		.intial_value_size = sizeof(BIODYN_FIRMWARE_VERSION),
	},
};
static struct biodyn_ble_service device_services[] = {
	{
		.name = "Device Information Service",
		.service_id = BIODYN_BLE_SERVICE_ID_16(0x180A),
		.n_characteristics = LEN_OF_STATIC_ARRAY(device_info_chars),
		.characteristics = &device_info_chars[0],
	},
};

void get_data_test(uint16_t *len, void *dst)
{
	const char *my_data = "Testing Testing Hello 123";
	*len = strlen(my_data) * sizeof(char);
	memcpy(dst, my_data, *len);
}
void set_data_test(uint16_t len, void *src)
{
	char buf[517];
	memcpy(buf, src, len);
	buf[len] = '\0';
	ESP_LOGI("PROFILES", "Tried to write \"%s\"", buf);
}
void imu_icm20948_get_state(uint16_t *len, void *dst)
{
	imu_motion_data data = {0};
	biodyn_imu_icm20948_read_accel_gyro(&data);
	*len = sizeof(imu_motion_data);
	// Since imu_motion_data is a struct solely of floats, it can (allegedly) be directly copied for output
	memcpy(dst, &data, *len);
	// copying from address of imu data for its length (len)
}

static struct biodyn_ble_characteristic sensor_test_chars[] = {
	{
		.name = "Test Read Function",
	 .uuid = BIODYN_BLE_UUID_16(0x1234),
	 .permissions = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
	 .properties = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE,
	 .get_data = get_data_test,
	 .set_data = set_data_test},
	{
	
		.name = "LED Control",
		.uuid = BIODYN_BLE_UUID_16(0x1235),
		.permissions = ESP_GATT_PERM_WRITE,
		.properties = ESP_GATT_CHAR_PROP_BIT_WRITE,
		.set_data = led_set_state,
	},
	{
		.name = "LED State",
		.uuid = BIODYN_BLE_UUID_16(0x1245),
		.permissions = ESP_GATT_PERM_READ,
		.properties = ESP_GATT_CHAR_PROP_BIT_READ,
		.get_data = led_get_state,
	},
	{
		.name = "IMU Control",
		.uuid = BIODYN_BLE_UUID_16(0x1236),
		.permissions = ESP_GATT_PERM_READ,
		.properties = ESP_GATT_CHAR_PROP_BIT_READ,
		.get_data = imu_icm20948_get_state,
	}};
static struct biodyn_ble_service sensor_services[] = {
	{
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
	},
};

// ALL THE PROFILES
static struct biodyn_ble_profile profiles[] = {
	{
		.name = "Device Profile",
		.n_services = LEN_OF_STATIC_ARRAY(device_services),
		.services = &device_services[0],
	},
	{
		.name = "Sensor Profile",
		.n_services = LEN_OF_STATIC_ARRAY(sensor_services),
		.services = &sensor_services[0],
	},
};

#endif // BIODYN_BLE_PROFILES_H