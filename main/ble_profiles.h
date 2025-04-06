#ifndef BIODYN_BLE_PROFILES_H
#define BIODYN_BLE_PROFILES_H

#include "constants.h"
#include "ble.h"

static struct biodyn_ble_characteristic device_info_chars[] = {
	{
		.name = "Manufacturer Name String",
		.uuid =	BIODYN_BLE_UUID_16(0x2A29),
		.permissions = ESP_GATT_PERM_READ,
		.properties = ESP_GATT_CHAR_PROP_BIT_READ,
		.initial_value = (void *) BIODYN_MANUFACTURER_NAME,
		.intial_value_size = sizeof(BIODYN_MANUFACTURER_NAME),
	},
	{
		.name = "Firmware Revision String",
		.uuid = BIODYN_BLE_UUID_16(0x2A26),
		.permissions = ESP_GATT_PERM_READ,
		.properties = ESP_GATT_CHAR_PROP_BIT_READ,
		.initial_value = (void *) BIODYN_FIRMWARE_VERSION,
		.intial_value_size = sizeof(BIODYN_FIRMWARE_VERSION)
	}
};
static struct biodyn_ble_service device_services[] = {
	{
		.name = "Device Information Service",
		.service_id = BIODYN_BLE_SERVICE_ID_16(0x180A),
		.n_characteristics = 1,
		.characteristics = &device_info_chars[0],
	}
};
#define N_DEVICE_SERVICES (sizeof(device_services) / sizeof(struct biodyn_ble_service))
static struct biodyn_ble_profile profiles[] = {
	{
		.name = "Device Profile",
		.n_services = N_DEVICE_SERVICES,
		.services = &device_services[0],
	}
};
#define N_PROFILES (sizeof(profiles) / sizeof(struct biodyn_ble_profile))


#endif // BIODYN_BLE_PROFILES_H