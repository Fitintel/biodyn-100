#ifndef BIODYN_IMU_BLE_SERVICE_H
#define BIODYN_IMU_BLE_SERVICE_H

#include "constants.h"
#include "bluetooth/ble.h"
#include "imu/imu_icm20948_driver.h"

const static struct biodyn_ble_characteristic imu_service_chars[] = {
	{
		.name = "Planar Acceleration",
		.uuid = BIODYN_BLE_UUID_16(0xC350),
		.permissions = BIODYN_PERM_READ,
		.properties = BIODYN_PROP_READ,
		// TODO: Implement data fetching for planar accel char
		.get_data = biodyn_imu_icm20948_read_accel,
	},
	{
		.name = "Gyro Velocity",
		.uuid = BIODYN_BLE_UUID_16(0xC351),
		.permissions = BIODYN_PERM_READ,
		.properties = BIODYN_PROP_READ,
		// TODO: Implement data fetching for gyro velocity char
		.get_data = biodyn_imu_icm20948_read_gyro,
	},
	// {
	// 	.name = "Magnetometer",
	// 	.uuid = BIODYN_BLE_UUID_16(0xC352),
	// 	.permissions = BIODYN_PERM_READ,
	// 	.properties = BIODYN_PROP_READ,
	// 	// TODO: Implement data fetching for magnetometer char
	// 	.get_data = biodyn_imu_icm20948_read_mag,
	// },
	// {
	// 	.name = "All IMU Data",
	// 	.uuid = BIODYN_BLE_UUID_16(0xC353),
	// 	.permissions = BIODYN_PERM_READ,
	// 	.properties = BIODYN_PROP_READ,
	// 	// TODO: Implement POTENTIAL data fetching for all imu data chars
	// 	.get_data = biodyn_imu_icm20948_read_all,
	// }
	// TODO add config characteristics for IMU
};

const static struct biodyn_ble_service imu_service = {
	.name = "IMU Service",
	.service_id = BIODYN_BLE_SERVICE_ID_16(0x3C32),
	.n_characteristics = LEN_OF_STATIC_ARRAY(imu_service_chars),
	.characteristics = &imu_service_chars[0],
};

#endif // BIODYN_IMU_BLE_SERVICE_H