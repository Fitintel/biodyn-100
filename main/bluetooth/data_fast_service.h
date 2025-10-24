#ifndef BIODYN_DATA_FAST_BLE_SERVICE
#define BIODYN_DATA_FAST_BLE_SERVICE

#include "constants.h"
#include "bluetooth/ble.h"
#include "imu/data_fast.h"
#include "system/time_sync.h"

const static struct biodyn_ble_characteristic data_fast_chars[] = {
	{
		.name = "Packed IMU Data",
		.uuid = BIODYN_BLE_UUID_16(0x4153),
		.permissions = BIODYN_PERM_READ,
		.properties = BIODYN_PROP_READ,
		.get_data = ble_data_fast_packed_imu,
	},
	{
		.name = "Packed Collective Data",
		.uuid = BIODYN_BLE_UUID_16(0x4155),
		.permissions = BIODYN_PERM_READ,
		.properties = BIODYN_PROP_READ,
		// TODO: Implement packed collective data read
		// .get_data = ,
	},
	{
		.name = "Heartbeat",
		.uuid = BIODYN_BLE_UUID_16(0x4157),
		.permissions = BIODYN_PERM_READ_WRITE,
		.properties = BIODYN_PROP_READ_WRITE,
		// TODO: Implement time sync hearbeat read/write
		.set_data = ble_time_sync_ticker_write,
		.get_data = ble_time_sync_ticker_read,
	},
	{
		.name = "Round Trip Time",
		.uuid = BIODYN_BLE_UUID_16(0x4158),
		.permissions = BIODYN_PERM_READ_WRITE,
		.properties = BIODYN_PROP_READ_WRITE,
		// TODO: Implement time sync round trip time (RTT) read/write
		.set_data = ble_time_sync_rtt_write,
		.get_data = ble_time_sync_rtt_read,
	},
};

const static struct biodyn_ble_service data_fast_service = {
	.name = "Data Fast Service",
	.service_id = BIODYN_BLE_SERVICE_ID_16(0x1432),
	.n_characteristics = LEN_OF_STATIC_ARRAY(data_fast_chars),
	.characteristics = &data_fast_chars[0],
};

#endif // BIODYN_DATA_FAST_BLE_SERVICE