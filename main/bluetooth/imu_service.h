#ifndef BIODYN_IMU_BLE_SERVICE_H
#define BIODYN_IMU_BLE_SERVICE_H

#include "constants.h"
#include "bluetooth/ble.h"

const static struct biodyn_ble_characteristic imu_service_chars[] = {
	{
		.name = "Planar Acceleration",
		.uuid = BIODYN_BLE_UUID_16(0xC350),
		.permissions = BIODYN_PERM_READ,
		.properties = BIODYN_PROP_READ,
		// TODO: Implement data fetching for planar accel char
		// .get_data = 
	},
	{
		.name = "Gyro Acceleration",
		.uuid = BIODYN_BLE_UUID_16(0xC351),
		.permissions = BIODYN_PERM_READ,
		.properties = BIODYN_PROP_READ,
		// TODO: Implement data fetching for gyro accel char
		// .get_data = 
	},
	{
		.name = "Magnetometer",
		.uuid = BIODYN_BLE_UUID_16(0xC352),
		.permissions = BIODYN_PERM_READ,
		.properties = BIODYN_PROP_READ,
		// TODO: Implement data fetching for magnetometer char
		// .get_data = 
	},
};

const static struct biodyn_ble_service imu_service = {
	.name = "IMU Service",
	.service_id = BIODYN_BLE_SERVICE_ID_16(0x3C32),
	.n_characteristics = LEN_OF_STATIC_ARRAY(imu_service_chars),
	.characteristics = &imu_service_chars[0],
};

#endif // BIODYN_IMU_BLE_SERVICE_H