#ifndef BIODYN_DATA_FAST_BLE_SERVICE
#define BIODYN_DATA_FAST_BLE_SERVICE

#include "biodyn_constants.h"
#include "bluetooth/ble.h"
#include "system/data_fast.h"
#include "system/time_sync.h"

const static struct biodyn_ble_characteristic data_fast_chars[] = {
	{
		.name = "Packed Collective Data",
		.uuid = BIODYN_BLE_UUID_16(0x4153),
		.permissions = BIODYN_PERM_READ,
		.properties = BIODYN_PROP_READ,
		.get_data = ble_data_fast_packed,
	},
	{
		.name = "Heartbeat",
		.uuid = BIODYN_BLE_UUID_16(0x4157),
		.permissions = BIODYN_PERM_READ_WRITE,
		.properties = BIODYN_PROP_READ_WRITE,
		.set_data = ble_time_sync_ticker_write,
		.get_data = ble_time_sync_ticker_read,
	},
	{
		.name = "Round Trip Time",
		.uuid = BIODYN_BLE_UUID_16(0x4158),
		.permissions = BIODYN_PERM_READ_WRITE,
		.properties = BIODYN_PROP_READ_WRITE,
		.set_data = ble_time_sync_rtt_write,
		.get_data = ble_time_sync_rtt_read,
	},
	{
		.name = "Orientation",
		.uuid = BIODYN_BLE_UUID_16(0x4159),
		.permissions = BIODYN_PERM_READ,
		.properties = BIODYN_PROP_READ,
		.get_data = ble_data_fast_orientation_packed,
	}};

const static struct biodyn_ble_service data_fast_service = {
	.name = "Data Fast Service",
	.service_id = BIODYN_BLE_SERVICE_ID_16(0x1432),
	.n_characteristics = LEN_OF_STATIC_ARRAY(data_fast_chars),
	.characteristics = &data_fast_chars[0],
};

#endif // BIODYN_DATA_FAST_BLE_SERVICE