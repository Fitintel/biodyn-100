#ifndef BIODYN_DEVICE_INFORMATION_SERVICE_H
#define BIODYN_DEVICE_INFORMATION_SERVICE_H

#include "constants.h"
#include "bluetooth/ble.h"

const static struct biodyn_ble_characteristic device_info_chars[] = {
	{
		.name = "Manufacturer Name String",
		.uuid = BIODYN_BLE_UUID_16(0x2A29),
		.permissions = BIODYN_PERM_READ,
		.properties = BIODYN_PROP_READ,
		.initial_value = (void *)BIODYN_MANUFACTURER_NAME,
		.intial_value_size = sizeof(BIODYN_MANUFACTURER_NAME),
	},
	{
		.name = "Model Number String",
		.uuid = BIODYN_BLE_UUID_16(0x2A24),
		.permissions = BIODYN_PERM_READ,
		.properties = BIODYN_PROP_READ,
		.initial_value = (void *)BIODYN_MODEL_NUMBER,
		.intial_value_size = sizeof(BIODYN_MODEL_NUMBER),
	},
	{
		.name = "Serial Number String",
		.uuid = BIODYN_BLE_UUID_16(0x2A25),
		.permissions = BIODYN_PERM_READ,
		.properties = BIODYN_PROP_READ,
		.initial_value = (void *)BIODYN_SERIAL_NUMBER,
		.intial_value_size = sizeof(BIODYN_SERIAL_NUMBER),
	},
	{
		.name = "Hardware Revision String",
		.uuid = BIODYN_BLE_UUID_16(0x2A27),
		.permissions = BIODYN_PERM_READ,
		.properties = BIODYN_PROP_READ,
		.initial_value = (void *)BIODYN_HARDWARE_VERSION,
		.intial_value_size = sizeof(BIODYN_HARDWARE_VERSION),
	},
	{
		.name = "Firmware Revision String",
		.uuid = BIODYN_BLE_UUID_16(0x2A26),
		.permissions = BIODYN_PERM_READ,
		.properties = BIODYN_PROP_READ,
		.initial_value = (void *)BIODYN_FIRMWARE_VERSION,
		.intial_value_size = sizeof(BIODYN_FIRMWARE_VERSION),
	},
	{
		.name = "System ID String",
		.uuid = BIODYN_BLE_UUID_16(0x2A23),
		.permissions = BIODYN_PERM_READ,
		.properties = BIODYN_PROP_READ,
		.initial_value = (void *)BIODYN_SYSTEM_ID,
		.intial_value_size = sizeof(BIODYN_SYSTEM_ID),
	}};

const static struct biodyn_ble_service device_information_service = {
	.name = "Device Information Service",
	.service_id = BIODYN_BLE_SERVICE_ID_16(0x180A),
	.n_characteristics = LEN_OF_STATIC_ARRAY(device_info_chars),
	.characteristics = &device_info_chars[0],
};

#endif // BIODYN_DEVICE_INFORMATION_SERVICE_H