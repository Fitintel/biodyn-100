#ifndef BIODYN_BLE_PROFILES_H
#define BIODYN_BLE_PROFILES_H

#include "constants.h"
#include "bluetooth/ble.h"
#include "bluetooth/device_information_service.h"
#include "bluetooth/test_service.h"


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