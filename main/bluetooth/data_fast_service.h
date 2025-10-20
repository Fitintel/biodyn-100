#ifndef BIODYN_DATA_FAST_BLE_SERVICE
#define BIODYN_DATA_FAST_BLE_SERVICE

#include "constants.h"
#include "bluetooth/ble.h"
#include "imu/data_fast.h"

const static struct biodyn_ble_characteristic data_fast_chars[] = {
	// {
	// 	.name = "Packed Planar Accel Data",
	// 	.uuid = BIODYN_BLE_UUID_16(0x4150),
	// 	.permissions = BIODYN_PERM_READ,
	// 	.properties = BIODYN_PROP_READ,
	// 	// TODO: Implement packed planar accel read
	// 	// .get_data = ,
	// },
	// {
	// 	.name = "Packed Gyro Accel Data",
	// 	.uuid = BIODYN_BLE_UUID_16(0x4151),
	// 	.permissions = BIODYN_PERM_READ,
	// 	.properties = BIODYN_PROP_READ,
	// 	// TODO: Implement packed gyro accel read
	// 	// .get_data = ,
	// },
	// {
	// 	.name = "Packed Magnetometer Data",
	// 	.uuid = BIODYN_BLE_UUID_16(0x4152),
	// 	.permissions = BIODYN_PERM_READ,
	// 	.properties = BIODYN_PROP_READ,
	// 	// TODO: Implement packed magnetometer read
	// 	// .get_data = ,
	// },
	{
		.name = "Packed IMU Data",
		.uuid = BIODYN_BLE_UUID_16(0x4153),
		.permissions = BIODYN_PERM_READ,
		.properties = BIODYN_PROP_READ,
		.get_data = ble_data_fast_packed_imu,
	},
	// {
	// 	.name = "Packed EMG Value Data",
	// 	.uuid = BIODYN_BLE_UUID_16(0x4154),
	// 	.permissions = BIODYN_PERM_READ,
	// 	.properties = BIODYN_PROP_READ,
	// 	// TODO: Implement packed emg value read
	// 	// .get_data = ,
	// },
	// {
	// 	.name = "Packed 10ms EMG Average Data",
	// 	.uuid = BIODYN_BLE_UUID_16(0x4155),
	// 	.permissions = BIODYN_PERM_READ,
	// 	.properties = BIODYN_PROP_READ,
	// 	// TODO: Implement packed emg 10ms average read
	// 	// .get_data = ,
	// },
	{
		.name = "Packed Collective Data",
		.uuid = BIODYN_BLE_UUID_16(0x4160),
		.permissions = BIODYN_PERM_READ,
		.properties = BIODYN_PROP_READ,
		// TODO: Implement packed collective data read
		// .get_data = ,
	},
};

const static struct biodyn_ble_service data_fast_service = {
	.name = "Data Fast Service",
	.service_id = BIODYN_BLE_SERVICE_ID_16(0x1432),
	.n_characteristics = LEN_OF_STATIC_ARRAY(data_fast_chars),
	.characteristics = &data_fast_chars[0],
};

#endif // BIODYN_DATA_FAST_BLE_SERVICE