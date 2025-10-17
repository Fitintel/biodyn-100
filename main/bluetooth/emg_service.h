#ifndef BIODYN_EMG_BLE_SERVICE_H
#define BIODYN_EMG_BLE_SERVICE_H

#include "constants.h"
#include "bluetooth/ble.h"

const static struct biodyn_ble_characteristic emg_service_chars[] = {
	{
		.name = "EMG Value",
		.uuid = BIODYN_BLE_UUID_16(0xB150),
		.permissions = BIODYN_PERM_READ,
		.properties = BIODYN_PROP_READ,
		// TODO: Implement data fetching for emg value char 
		// .get_data = 
	},
	// {
	// 	.name = "EMG 10ms Average",
	// 	.uuid = BIODYN_BLE_UUID_16(0xB151),
	// 	.permissions = BIODYN_PERM_READ,
	// 	.properties = BIODYN_PROP_READ,
	// 	// TODO: Implement data fetching for emg 10ms average 
	// 	// .get_data = 
	// },
	// {
	// 	.name = "EMG 100ms Average",
	// 	.uuid = BIODYN_BLE_UUID_16(0xB152),
	// 	.permissions = BIODYN_PERM_READ,
	// 	.properties = BIODYN_PROP_READ,
	// 	// TODO: Implement data fetching for emg 100ms average 
	// 	// .get_data = 
	// },
	// {
	// 	.name = "EMG 1000ms Average",
	// 	.uuid = BIODYN_BLE_UUID_16(0xB153),
	// 	.permissions = BIODYN_PERM_READ,
	// 	.properties = BIODYN_PROP_READ,
	// 	// TODO: Implement data fetching for emg 1000ms average 
	// 	// .get_data = 
	// },
};

const static struct biodyn_ble_service emg_service = {
	.name = "EMG Service",
	.service_id = BIODYN_BLE_SERVICE_ID_16(0xB132),
	.n_characteristics = LEN_OF_STATIC_ARRAY(emg_service_chars),
	.characteristics = &emg_service_chars[0],
};

#endif // BIODYN_EMG_BLE_SERVICE_H