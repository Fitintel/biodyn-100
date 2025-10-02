#ifndef BIODYN_BLE_PROFILES_H
#define BIODYN_BLE_PROFILES_H

#include "constants.h"
#include "bluetooth/ble.h"
#include "bluetooth/device_information_service.h"
#include "bluetooth/test_service.h"
#include "bluetooth/self_test_service.h"
#include "bluetooth/imu_service.h"
#include "bluetooth/emg_service.h"
#include "bluetooth/time_sync_service.h"
#include "bluetooth/data_fast_service.h"

const static struct biodyn_ble_service general_profile_services[] = {
	device_information_service,
};

const static struct biodyn_ble_service validation_profile_services[] = {
	self_test_service,
	test_service,
};

const static struct biodyn_ble_service sensor_profile_services[] = {
	imu_service,
	emg_service,
};

const static struct biodyn_ble_service streaming_profile_services[] = {
	data_fast_service,
	time_sync_service,
};

// ALL THE PROFILES
static struct biodyn_ble_profile profiles[] = {
	{
		.name = "General Profile",
		.n_services = LEN_OF_STATIC_ARRAY(general_profile_services),
		.services = &general_profile_services[0],
	},
	{
		.name = "Validation Profile",
		.n_services = LEN_OF_STATIC_ARRAY(validation_profile_services),
		.services = &validation_profile_services[0],
	},
	{
		.name = "Sensor Profile",
		.n_services = LEN_OF_STATIC_ARRAY(sensor_profile_services),
		.services = &sensor_profile_services[0],
	},
	{
		.name = "Streaming Profile",
		.n_services = LEN_OF_STATIC_ARRAY(streaming_profile_services),
		.services = &streaming_profile_services[0],
	}
};

#endif // BIODYN_BLE_PROFILES_H