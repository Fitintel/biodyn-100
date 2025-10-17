#ifndef BIODYN_SYSTEM_COLLECTION
#define BIODYN_SYSTEM_COLLECTION

#include "constants.h"
#include "system/time_sync.h"
#include "system/led.h"
#include "imu/imu_icm20948_driver.h"

// Order is order of initialization
const static biodyn_system biodyn_systems[] = {
	biodyn_led_system,
	// biodyn_imu_system,
	// biodyn_time_sync_system,
};
const static int n_biodyn_systems = LEN_OF_STATIC_ARRAY(biodyn_systems);

#endif // BIODYN_SYSTEM_COLLECTION