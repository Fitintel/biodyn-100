#ifndef BIODYN_SELF_TEST_BLE_SERVICE
#define BIODYN_SELF_TEST_BLE_SERVICE

#include "constants.h"
#include "bluetooth/ble.h"
#include "system/self_test.h"

static struct biodyn_ble_characteristic self_test_chars[] = {
	{
		.name = "Self Test State",
		.uuid = BIODYN_BLE_UUID_16(0x1A10),
		.permissions = BIODYN_PERM_READ_WRITE,
		.properties = BIODYN_PROP_READ_WRITE,
		.set_data = self_test_set_state,
		.get_data = self_test_get_state,
	}
};

static struct biodyn_ble_service self_test_services[] = {
	{
		.name = "Self Test Service",
		.service_id = {
			.is_primary = true,
			.id = {
				.inst_id = 0,
				.uuid = {
					.uuid = {
						.uuid128 = {
							0xFB,
							0x34,
							0x9B,
							0x5F,
							0x80,
							0x00,
							0x00,
							0x80,
							0x00,
							0x10,
							0x00,
							0xAB,
							0xFF,
							0xFF,
							0xCD,
							0x00,
						},
					},
					.len = ESP_UUID_LEN_128,
				},
			},
		},
		.n_characteristics = LEN_OF_STATIC_ARRAY(self_test_chars),
		.characteristics = &self_test_chars[0],
	},
};
#include "system/self_test.h"

#endif // BIODYN_SELF_TEST_BLE_SERVICE
