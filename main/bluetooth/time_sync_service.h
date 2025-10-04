#ifndef BIODYN_TIME_SYNC_BLE_SERVICE
#define BIODYN_TIME_SYNC_BLE_SERVICE

#include "constants.h"
#include "bluetooth/ble.h"

const static struct biodyn_ble_characteristic time_sync_chars[] = {
	{
		.name = "Heartbeat",
		.uuid = BIODYN_BLE_UUID_16(0xA550),
		.permissions = BIODYN_PERM_READ_WRITE,
		.properties = BIODYN_PROP_READ_WRITE,
		// TODO: Implement time sync hearbeat read/write
		// .set_data = ,
		// .get_data = ,
	},
	{
		.name = "Round Trip Time",
		.uuid = BIODYN_BLE_UUID_16(0xA55F),
		.permissions = BIODYN_PERM_READ_WRITE,
		.properties = BIODYN_PROP_READ_WRITE,
		// TODO: Implement time sync round trip time (RTT) read/write
		// .set_data = ,
		// .get_data = ,
	},
};

const static struct biodyn_ble_service time_sync_service = {
	.name = "Time Sync Service",
	.service_id = BIODYN_BLE_SERVICE_ID_16(0xA532),
	.n_characteristics = LEN_OF_STATIC_ARRAY(time_sync_chars),
	.characteristics = &time_sync_chars[0],
};

#endif // BIODYN_TIME_SYNC_BLE_SERVICE