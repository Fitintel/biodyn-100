#ifndef BLE_HELPERS_H
#define BLE_HELPERS_H

#include <string.h>

#include "esp_system.h"
#include "esp_log.h"
#include "esp_gatt_defs.h"

#include "constants.h"

// Returns true if the two service IDs are equal, false otherwise
static bool service_ids_equal(esp_gatt_srvc_id_t *id1, esp_gatt_srvc_id_t *id2)
{
	if (id1->id.inst_id == id2->id.inst_id && id1->id.uuid.len == id2->id.uuid.len)
		switch (id1->id.uuid.len)
		{
		case ESP_UUID_LEN_16:
			if (id1->id.uuid.uuid.uuid16 == id2->id.uuid.uuid.uuid16)
				return true;
			break;
		case ESP_UUID_LEN_32:
			if (id1->id.uuid.uuid.uuid32 == id2->id.uuid.uuid.uuid32)
				return true;
			break;
		case ESP_UUID_LEN_128:
			if (memcmp(id1->id.uuid.uuid.uuid128, id2->id.uuid.uuid.uuid128, 16) == 0)
				return true;
			break;
		default:
			ESP_LOGE(GATTS_TAG, "Service ID UUID length was a non-regular value: %d", id1->id.uuid.len);
			return false;
		}
	return false;
}

#endif // BLE_HELPERS_H