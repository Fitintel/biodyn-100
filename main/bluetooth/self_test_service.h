#ifndef BIODYN_SELF_TEST_BLE_SERVICE
#define BIODYN_SELF_TEST_BLE_SERVICE

#include "biodyn_constants.h"
#include "bluetooth/ble.h"
#include "system/led.h"
#include "system/self_test.h"

const static struct biodyn_ble_characteristic self_test_chars[] = {
	{
		.name = "Self Test State",
		.uuid = BIODYN_BLE_UUID_16(0x1A10),
		.permissions = BIODYN_PERM_READ_WRITE,
		.properties = BIODYN_PROP_READ_WRITE,
		.set_data = self_test_set_state,
		.get_data = self_test_get_state,
	},
	{
		.name = "Self Test Message String",
		.uuid = BIODYN_BLE_UUID_16(0x1A11),
		.permissions = BIODYN_PERM_READ,
		.properties = BIODYN_PROP_READ,
		.get_data = self_test_get_err_msg,
	},
	{
		.name = "LED Control",
		.uuid = BIODYN_BLE_UUID_16(0x1A12),
		.permissions = BIODYN_PERM_READ_WRITE,
		.properties = BIODYN_PROP_READ_WRITE,
		.set_data = led_set_state,
		.get_data = led_get_state,
	},
};

const static struct biodyn_ble_service self_test_service = {
	.name = "Self Test Service",
	.service_id = {
		.is_primary = true,
		.id = {
			.inst_id = 0,
			.uuid = BIODYN_BLE_UUID_16(0xA912),
		},
	},
	.n_characteristics = LEN_OF_STATIC_ARRAY(self_test_chars),
	.characteristics = &self_test_chars[0],
};

#endif // BIODYN_SELF_TEST_BLE_SERVICE
