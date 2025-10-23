#ifndef BIODYN_BLE_PROFILES_H
#define BIODYN_BLE_PROFILES_H

#include "constants.h"
#include "bluetooth/ble.h"
#include "bluetooth/device_information_service.h"
#include "bluetooth/self_test_service.h"
#include "bluetooth/data_fast_service.h"

const static struct biodyn_ble_service general_profile_services[] = {
	device_information_service,
};

const static struct biodyn_ble_service biodyn_profile_services[] = {
	self_test_service,
	data_fast_service,
};

// ALL THE PROFILES
static struct biodyn_ble_profile profiles[] = {
	{
		.name = "General Profile",
		.n_services = LEN_OF_STATIC_ARRAY(general_profile_services),
		.services = &general_profile_services[0],
	},
	{
		.name = "BIODYN Profile",
		.n_services = LEN_OF_STATIC_ARRAY(biodyn_profile_services),
		.services = &biodyn_profile_services[0],
	},
};

#endif // BIODYN_BLE_PROFILES_H