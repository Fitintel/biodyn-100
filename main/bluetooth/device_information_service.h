#ifndef BIODYN_DEVICE_INFORMATION_SERVICE_H
#define BIODYN_DEVICE_INFORMATION_SERVICE_H

#include "constants.h"
#include "bluetooth/ble.h"

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

#endif // BIODYN_DEVICE_INFORMATION_SERVICE_H