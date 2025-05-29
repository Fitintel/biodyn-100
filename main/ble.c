
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gatt_common_api.h"
#include "esp_timer.h"
#include "sdkconfig.h"

#include "ble.h"
#include "constants.h"

/* ----------------------------
		Data definitions
   ---------------------------- */

// Our app id
#define BIODYN_GATTS_APP_ID 0

// Our MTU
#define BIODYN_BLE_MTU 517

struct biodyn_ble_profile_data;
struct biodyn_ble_service_data;
struct biodyn_ble_characteristic_data;
struct biodyn_ble_char_descr_data;

// Service creation state.
// First it isn't created, then it requests, then it gets started.
enum biodyn_ble_service_state
{
	biodyn_ble_service_not_created,
	biodyn_ble_service_requested_creation,
	biodyn_ble_service_started,
	biodyn_ble_service_failed,
};

// Characteristic creation state.
// First it isn't created, then it requests addition to a service, then it gets added.
enum biodyn_ble_characteristic_state
{
	biodyn_ble_characteristic_not_created,
	biodyn_ble_characteristic_requested_creation,
	biodyn_ble_characteristic_created,
	biodyn_ble_characteristic_failed,
};

// This is our general bluetooth driver data.
static struct
{
	bool has_connection;					  // Do we have an active connection to another device?
	uint16_t connection_id;					  // The connection ID for the current connection
	biodyn_ble_err_t error;					  // 0 if ok, otherwise has a BIODYN_BLE_ERR_ set
	uint16_t n_profiles;					  // The number of profiles we have
	struct biodyn_ble_profile_data *profiles; // The BLE profiles we have
	uint8_t manufacturer_data[4];			  // Our manufacturer data for adv packets
	uint16_t advertisement_uuid_size;		  // How big the UUIDs are that we're putting in adv packets (16,32,128) 0 if unset
} biodyn_ble_data = {false, 0, 0, 0, NULL, {0xF1, 0x72, 0xE7, 0x00}, 0};

// Returns any accumulated errors in the biodyn bluetooth driver
biodyn_ble_err_t biodyn_ble_get_err()
{
	return biodyn_ble_data.error;
}

// Represents the general components for a profile
struct biodyn_ble_profile_data
{
	const char *name;						  // The name of the profile
	size_t n_services;						  // The number of services this profile offers
	struct biodyn_ble_service_data *services; // A list of the services under this profile
	esp_gatt_if_t gatts_if;					  // The GATTS identifier for our server. Intiially set to ESP_GATT_IF_NONE.
};

// Represents the general components for a service
struct biodyn_ble_service_data
{
	const char *name;										// The name of the service
	struct biodyn_ble_characteristic_data *characteristics; // The characteristics for this service
	uint16_t n_characteristics;								// The number of characteristics in this service

	// The number of handles required by this service.
	// Handles include: service declaration, characteristic declaration, characteristic value,
	//  				characteristic descriptiors, etc.
	// This is calculated from the provided schematic.
	uint16_t n_handles;

	enum biodyn_ble_service_state service_state; // The service state
	uint16_t service_handle;					 // The service handle. This is assiged when created, no need to be initialized.
	esp_gatt_srvc_id_t service_id;				 // The GATT service identifier
	struct biodyn_ble_profile_data *profile;	 // This service's parent profile
	bool advertise;								 // Whether to advertise this service or not
};

// Represents a characteristic under a service
struct biodyn_ble_characteristic_data
{
	const char *name;	// The name of the characteristic
	esp_bt_uuid_t uuid; // The UUID for the characteristic

	// The associated permissions. What is required for the properties to be accessed.
	// Ex. normal read, requires encryption, requires authentication
	esp_gatt_perm_t permissions;

	// Characteristic properties: what can be done with this characteristic
	// Ex. this characteristic can be read, written to, notify the client, etc
	esp_gatt_char_prop_t properties;

	enum biodyn_ble_characteristic_state characteristic_state; // The state of creation
	uint16_t n_descriptors;									   // Number of descriptors on this characteristic
	struct biodyn_ble_char_descr_data *descriptors;			   // Descriptor array for this characteristic
	uint16_t attribute_handle;								   // Handle from the GATTS pointing to our characteristic's value (ie. attribute)
	struct biodyn_ble_service_data *service;				   // This characteristic's parent service
	esp_attr_value_t initial_value;							   // The initial value associated with this descriptor
	esp_attr_control_t response_type;						   // Auto-response with GATT or app-controlled
	void (*get_data_fn)(uint16_t *size, void *dst);			   // Function called when a read is done and we aren't doing GATT auto-response
	void (*set_data_fn)(uint16_t size, void *src);			   // Function called when a write is done and we aren't doing GATT auto-response

	// TODO: How do we provide a nice interface to register characteristic info and callbacks?
};

// Represents a desriptor for a characteristic. Ie. CCCD.
struct biodyn_ble_char_descr_data
{
	uint16_t descriptor_handle;	 // The GATTS handle for this descriptor
	esp_bt_uuid_t uuid;			 // The descriptor UUID
	esp_gatt_perm_t permissions; // The permissions given for this descriptor (ie. read, write)
	esp_attr_value_t value;		 // The value associated with this descriptor
};

// The number of ble profiles the BIODYN has
#define NUM_BLE_PROFILES (sizeof(biodyn_ble_profiles) / sizeof(struct biodyn_ble_profile))

// Returns true if the two Bluetooth UUIDs are equal, false otherwise
static bool bt_uuids_equal(esp_bt_uuid_t *id1, esp_bt_uuid_t *id2)
{
	if (id1->len != id2->len)
		return false;

	switch (id1->len)
	{
	case ESP_UUID_LEN_16:
		if (id1->uuid.uuid16 == id2->uuid.uuid16)
			return true;
		break;
	case ESP_UUID_LEN_32:
		if (id1->uuid.uuid32 == id2->uuid.uuid32)
			return true;
		break;
	case ESP_UUID_LEN_128:
		if (memcmp(id1->uuid.uuid128, id2->uuid.uuid128, 16) == 0)
			return true;
		break;
	default:
		ESP_LOGE(BIODYN_BLE_TAG, "Service ID UUID length was a non-regular value: %d", id1->len);
		return false;
	}
	return false;
}

// Returns true if the two service IDs are equal, false otherwise
static bool service_ids_equal(esp_gatt_srvc_id_t *id1, esp_gatt_srvc_id_t *id2)
{
	if (id1->id.inst_id != id2->id.inst_id)
		return false;
	return bt_uuids_equal(&id1->id.uuid, &id2->id.uuid);
}

// Returns the service data for the service with the given ID in all profiles.
// Returns NULL if not found.
struct biodyn_ble_service_data *get_service_from_id(esp_gatt_srvc_id_t *service_id)
{
	// For each profile
	for (int i = 0; i < biodyn_ble_data.n_profiles; ++i)
		// For each service
		for (int j = 0; j < biodyn_ble_data.profiles[i].n_services; ++j)
		{
			struct biodyn_ble_service_data *service = &biodyn_ble_data.profiles[i].services[j];
			// Check if the service ids are equal
			if (service_ids_equal(&service->service_id, service_id))
				return service;
		}

	// We didn't find it ... :(
	return NULL;
}

// Returns the service data for the service with the given handle. Null otherwise.
struct biodyn_ble_service_data *get_service_from_handle(uint16_t service_handle)
{
	// Each profile
	for (int i = 0; i < biodyn_ble_data.n_profiles; ++i)
		// For each service
		for (int j = 0; j < biodyn_ble_data.profiles[i].n_services; ++j)
		{
			struct biodyn_ble_service_data *service = &biodyn_ble_data.profiles[i].services[j];
			// Check if the service handles are equal
			if (service->service_handle == service_handle)
				return service;
		}
	return NULL;
}

// Returns the characteristic data for the characteristic with the given ID under the service with the given handle
// Returns NULL if not found
struct biodyn_ble_characteristic_data *get_characteristic_from_id(uint16_t service_handle, esp_bt_uuid_t *char_id)
{
	// Get the service with the given handle
	struct biodyn_ble_service_data *sd = get_service_from_handle(service_handle);
	if (!sd)
		return NULL;

	// Search the characteristics of this service for matching UUID
	for (int i = 0; i < sd->n_characteristics; ++i)
	{
		if (bt_uuids_equal(&sd->characteristics[i].uuid, char_id))
			return &sd->characteristics[i];
	}

	// Not found
	return NULL;
}

// Returns the characteristic data for the characteristic with the given handle
struct biodyn_ble_characteristic_data *get_characteristic_from_handle(uint16_t handle)
{
	// Search everything
	for (int i = 0; i < biodyn_ble_data.n_profiles; ++i)
	{
		struct biodyn_ble_profile_data *profile = &biodyn_ble_data.profiles[i];
		for (int j = 0; j < profile->n_services; ++j)
		{
			struct biodyn_ble_service_data *service = &profile->services[j];
			for (int k = 0; k < service->n_characteristics; ++k)
			{
				struct biodyn_ble_characteristic_data *chr = &service->characteristics[k];
				if (chr->attribute_handle == handle)
					return chr;
			}
		}
	}
	// We didn't find it
	return NULL;
}

// Then data length in bytes for manufacturer data included in BLE advertisement
// This *CANNOT* be larger than 31 bytes
#define BIODYN_MANUFACTURER_DATA_LEN sizeof(biodyn_ble_data.manufacturer_data)

// BLE advertisement data packet configuration.
// This configures the structure of our packets.
esp_ble_adv_data_t biodyn_ble_advertisement_data = {
	.set_scan_rsp = false,										  // This is not a scan response packet
	.include_name = true,										  // Include device name in advertisement
	.include_txpower = true,									  // Include TX power in advertisement
	.min_interval = 6,											  // Preffered connection minimum time interval
	.max_interval = 12,											  // Preferred connection maximum time interval
	.appearance = 0x090718,										  // Information about the device type (device class): ie. wearable etc
	.manufacturer_len = BIODYN_MANUFACTURER_DATA_LEN,			  // See definition
	.p_manufacturer_data = &biodyn_ble_data.manufacturer_data[0], // Pointer to manufacturer data

	// TODO: We can advertise service data without a connection made.
	// This could be useful for sharing some metrics between FITNET nodes directly
	// since our BLE only has single-connection capability.
	.service_data_len = 0,
	.p_service_data = NULL,

	// This says what services we have in the adv packet. It is not crucial that this is
	// here but it can give devices info on services provided. In context of the
	// BIOHUB it doesn't matter all that much. TODO: What services should we advertise?
	.service_uuid_len = 0,	// We aren't advertising any services yet
	.p_service_uuid = NULL, // ^

	// General discoverable mode + low-energy-bluetooth only device
	.flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// BLE advertisement parameters. This configures how we advertise, not what.
static esp_ble_adv_params_t biodyn_ble_advertisement_params = {
	// This is how frequently we send advertisment packets
	.adv_int_min = 32, // Time = 32 * 0.625 ms = 20 ms
	.adv_int_max = 64, // Time = 64 * 0.625 ms = 40 ms

	.adv_type = ADV_TYPE_IND,								// We're doing general advertisement
	.own_addr_type = BLE_ADDR_TYPE_PUBLIC,					// We have a public non-random BLE address
	.channel_map = ADV_CHNL_ALL,							// Advertise on all BLE channels
	.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY, // Allow any device to scan and connect
};

/* ----------------------------
	BLE Setup & Callback Functions
   ---------------------------- */

// Requests creation of our services (within our profiles)
// Done during gatt server setup.
esp_err_t biodyn_ble_init_profiles(esp_gatt_if_t gatts_if)
{
	esp_err_t err = 0;

	// For each of the biodyn GATT profiles
	for (int i = 0; i < biodyn_ble_data.n_profiles; ++i)
	{
		struct biodyn_ble_profile_data *profile = &biodyn_ble_data.profiles[i];
		profile->gatts_if = gatts_if;

		ESP_LOGI(BIODYN_BLE_TAG, "Creating profile \"%s\" with %d services", profile->name, profile->n_services);

		// For each of this profile's services
		for (int j = 0; j < profile->n_services; ++j)
		{
			struct biodyn_ble_service_data *service = &profile->services[j];

			// Create the service
			esp_err_t create_service_err;
			if ((create_service_err = esp_ble_gatts_create_service(gatts_if, &service->service_id, service->n_handles)))
			{
				// Failed creation
				ESP_LOGE(BIODYN_BLE_TAG, "Failed to create service: %s returned error %x", service->name, create_service_err);
				biodyn_ble_data.error |= BIODYN_BLE_ERR_CANT_CREATE_SERVICE;
				service->service_state = biodyn_ble_service_failed;
			}
			else
			{
				// Requested creation
				// We're not clearing associated error because it may have happened during this loop!
				ESP_LOGI(BIODYN_BLE_TAG, "\tCreating service \"%s\" with %d characteristics and %d handles",
						 service->name, service->n_characteristics, service->n_handles);
				service->service_state = biodyn_ble_service_requested_creation;
			}

			if (!err)
				err = create_service_err;
		}
	}

	return err;
}

// Configures and initializes our GATT server instance
// Called when recieving ESP_GATTS_REG_EVT signalling our server is ready to set up.
esp_err_t biodyn_ble_init_gatts(esp_gatt_if_t gatts_if)
{
	esp_err_t err;
	// Set the device name
	if ((err = esp_ble_gap_set_device_name(BIODYN_DEVICE_NAME)))
	{
		ESP_LOGE(BIODYN_BLE_TAG, "Failed to set device name to %s, error code = %x", BIODYN_DEVICE_NAME, err);
		biodyn_ble_data.error |= BIODYN_BLE_ERR_CANT_NAME;
	}
	else
	{
		ESP_LOGI(BIODYN_BLE_TAG, "Set device name to %s", BIODYN_DEVICE_NAME);
		biodyn_ble_data.error &= ~BIODYN_BLE_ERR_CANT_NAME;
	}

	// Create our services for our profiles
	if ((err = biodyn_ble_init_profiles(gatts_if)))
		ESP_LOGE(BIODYN_BLE_TAG, "Failed to create one or more BLE services!");

	// Configure advertisement data
	if ((err = esp_ble_gap_config_adv_data(&biodyn_ble_advertisement_data)))
	{
		ESP_LOGE(BIODYN_BLE_TAG, "Failed to configure advertisement packet data, error code = %x", err);
		biodyn_ble_data.error |= BIODYN_BLE_ERR_ADV_DATA_INIT;
	}
	else
	{
		ESP_LOGI(BIODYN_BLE_TAG, "Set advertisement data (%d service UUIDs in packet, len %d)",
				 biodyn_ble_data.advertisement_uuid_size == 0 ? 0 : (biodyn_ble_advertisement_data.service_uuid_len / biodyn_ble_data.advertisement_uuid_size),
				 biodyn_ble_data.advertisement_uuid_size);
		biodyn_ble_data.error &= ~BIODYN_BLE_ERR_ADV_DATA_INIT;
	}

	return err;
}

// Configures and initializes a service which has requested creation.
// Finds the service contained in the creation parameters.
// Called when recieving ESP_GATTS_CREATE_EVT signalling our service is ready to configure and start.
esp_err_t biodyn_ble_init_service(struct gatts_create_evt_param *cep)
{
	esp_err_t err = 0;

	// Find the created service among the ones we have
	struct biodyn_ble_service_data *service = get_service_from_id(&cep->service_id);
	if (!service)
	{
		ESP_LOGE(BIODYN_BLE_TAG, "Could not find service!");
		biodyn_ble_data.error |= BIODYN_BLE_ERR_CANT_FIND_SERVICE;
		return ESP_ERR_INVALID_STATE;
	}

	// We have our service, let's start it
	service->service_handle = cep->service_handle;
	if ((err = esp_ble_gatts_start_service(service->service_handle)))
	{
		ESP_LOGE(BIODYN_BLE_TAG, "Failed to start service \"%s\", error code %d", service->name, err);
		biodyn_ble_data.error |= BIODYN_BLE_ERR_CANT_START_SERVICE;
		service->service_state = biodyn_ble_service_failed;
		return err;
	}

	// If we have no characteristics to add, but we think we do, there's a BIG problem
	if (!service->characteristics && service->n_characteristics != 0)
	{
		ESP_LOGE(BIODYN_BLE_TAG, "Reported characteristic count is wrong (reported %d) for service \"%s\"",
				 service->n_characteristics, service->name);
		biodyn_ble_data.error |= BIODYN_BLE_ERR_CHARS_MISCONFIGURED;
		service->service_state = biodyn_ble_service_failed;
		return ESP_ERR_INVALID_STATE;
	}

	// Ok, we started it successfully
	service->service_state = biodyn_ble_service_started;

	ESP_LOGI(BIODYN_BLE_TAG, "Started service \"%s\" successfully, adding %d characteristic(s)",
			 service->name, service->n_characteristics);

	// Time to add characteristics
	for (int i = 0; i < service->n_characteristics; ++i)
	{
		struct biodyn_ble_characteristic_data *characteristic = &service->characteristics[i];
		// Create the characteristic
		esp_err_t add_char_ret = esp_ble_gatts_add_char(service->service_handle,
														&characteristic->uuid,
														characteristic->permissions,
														characteristic->properties,
														characteristic->initial_value.attr_max_len == 0
															? NULL
															: &characteristic->initial_value,
														&characteristic->response_type);
		if (add_char_ret)
		{
			// Failed to add characteristic
			ESP_LOGE(BIODYN_BLE_TAG, "Failed to add characteristic \"%s\" to service \"%s\", error code %x",
					 characteristic->name, service->name, add_char_ret);
			err = add_char_ret;
			characteristic->characteristic_state = biodyn_ble_characteristic_failed;
			biodyn_ble_data.error |= BIODYN_BLE_ERR_CANT_CREATE_CHAR;
		}
		else
		{
			// We requested it to be added
			characteristic->characteristic_state = biodyn_ble_characteristic_requested_creation;
		}
	}

	// Done!
	return err;
}

// Configures and initializes a characteristic which has requested creation.
// Finds the characteristic & service in creation params.
// Called when recieving ESP_GATTS_ADD_CHAR_EVT signalling the characteristic is ready to config + start.
esp_err_t biodyn_ble_init_characterisic(struct gatts_add_char_evt_param *acep)
{
	// Get the characteristic
	struct biodyn_ble_characteristic_data *chr = get_characteristic_from_id(acep->service_handle, &acep->char_uuid);
	if (!chr)
	{
		ESP_LOGE(BIODYN_BLE_TAG, "Could not find characteristic!");
		biodyn_ble_data.error |= BIODYN_BLE_ERR_CANT_FIND_CHAR;
		return ESP_ERR_INVALID_STATE;
	}

	// Configure the characteristic
	chr->attribute_handle = acep->attr_handle;

	// Request character descriptors
	for (int i = 0; i < chr->n_descriptors; ++i)
	{
		struct biodyn_ble_char_descr_data *descriptor = &chr->descriptors[i];
		esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(chr->service->service_handle,
															   &descriptor->uuid,
															   descriptor->permissions,
															   &descriptor->value, NULL);
		if (add_descr_ret)
		{
			ESP_LOGE(BIODYN_BLE_TAG, "Failed to add descriptor to \"%s\" - \"%s\", error code =%x",
					 chr->service->name,
					 chr->name,
					 add_descr_ret);
			biodyn_ble_data.error |= BIODYN_BLE_ERR_CANT_CREATE_DESCR;
			chr->characteristic_state = biodyn_ble_characteristic_failed; // Descriptors are part of characteristics
			return add_descr_ret;
		}
	}

	// Ok
	return 0;
}

// Handles a ble gatts read event
void biodyn_ble_handle_read(struct gatts_read_evt_param *re)
{
	// Get the characteristic trying to be read
	struct biodyn_ble_characteristic_data *chr = get_characteristic_from_handle(re->handle);
	if (!chr)
	{
		ESP_LOGE(BIODYN_BLE_TAG, "Couldn't find characteristic with handle %d", re->handle);
		biodyn_ble_data.error |= BIODYN_BLE_ERR_CANT_FIND_CHAR;
		return;
	}

	// If it's auto-handled we ignore it
	if (chr->response_type.auto_rsp == ESP_GATT_AUTO_RSP)
		return;

	ESP_LOGI(BIODYN_BLE_TAG, "Read request for characteristic \"%s\" in service \"%s\"", chr->name, chr->service->name);

	// Get the data from the characteristic
	if (!chr->get_data_fn)
	{
		ESP_LOGE(BIODYN_BLE_TAG, "Attempted to read characteristic \"%s\" but there was no get_data function", chr->name);
		biodyn_ble_data.error |= BIODYN_BLE_ERR_CHARS_MISCONFIGURED;
		return;
	}
	uint16_t data_len = 0;
	uint8_t buf[BIODYN_BLE_MTU];
	chr->get_data_fn(&data_len, &buf);

	if (data_len > BIODYN_BLE_MTU)
	{
		ESP_LOGE(BIODYN_BLE_TAG, "Attempted to read too much data (%d) when max is %d", data_len, BIODYN_BLE_MTU);
		biodyn_ble_data.error |= BIODYN_BLE_ERR_TOO_MUCH_DATA;
	}

	// Form a response
	esp_gatt_rsp_t resp;
	resp.attr_value.handle = re->handle;
	resp.attr_value.len = data_len;
	memcpy(resp.attr_value.value, &buf, data_len);

	// Send the response
	esp_ble_gatts_send_response(chr->service->profile->gatts_if, re->conn_id, re->trans_id, ESP_GATT_OK, &resp);
}

// Handles a ble gatts write event
void biodyn_ble_handle_write(struct gatts_write_evt_param *wr)
{
	// Get the characteristic trying to be read
	struct biodyn_ble_characteristic_data *chr = get_characteristic_from_handle(wr->handle);
	if (!chr)
	{
		ESP_LOGE(BIODYN_BLE_TAG, "Couldn't find characteristic with handle %d", wr->handle);
		biodyn_ble_data.error |= BIODYN_BLE_ERR_CANT_FIND_CHAR;
		return;
	}

	// If it's auto-handled we ignore it
	if (chr->response_type.auto_rsp == ESP_GATT_AUTO_RSP)
		return;

	ESP_LOGI(BIODYN_BLE_TAG, "Write request for characteristic \"%s\" in service \"%s\"", chr->name, chr->service->name);

	// Get the data from the characteristic
	if (!chr->set_data_fn)
	{
		ESP_LOGE(BIODYN_BLE_TAG, "Attempted to write characteristic \"%s\" but there was no set_data function", chr->name);
		biodyn_ble_data.error |= BIODYN_BLE_ERR_CHARS_MISCONFIGURED;
		return;
	}
	if (wr->len > BIODYN_BLE_MTU)
	{
		ESP_LOGE(BIODYN_BLE_TAG, "Attempted to write too much data (%d) when max is %d", wr->len, BIODYN_BLE_MTU);
		biodyn_ble_data.error |= BIODYN_BLE_ERR_TOO_MUCH_DATA;
		return;
	}
	chr->set_data_fn(wr->len, (void *)wr->value);

	// Form a response
	esp_gatt_rsp_t resp;
	resp.attr_value.handle = wr->handle;

	// Send the response
	esp_ble_gatts_send_response(chr->service->profile->gatts_if, wr->conn_id, wr->trans_id, ESP_GATT_OK, &resp);
}

// The global gatts server event handler function
void biodyn_gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
	switch (event)
	{
	case ESP_GATTS_REG_EVT: // Registering our server, let's provide some info.
#ifdef LOG_GATTS
		ESP_LOGD(BIODYN_BLE_TAG, "ESP_GATTS_REG_EVT: status %d, app id %d", param->reg.status, param->reg.app_id);
#endif // LOG_GATTS
		biodyn_ble_init_gatts(gatts_if);
		break;
	case ESP_GATTS_READ_EVT:
	{
#ifdef LOG_GATTS
		ESP_LOGI(BIODYN_BLE_TAG, "ESP_GATTS_READ_EVT, conn_id %d, trans_id %" PRIu32 ", handle %d", param->read.conn_id, param->read.trans_id, param->read.handle);
#endif // LOG_GATTS
		biodyn_ble_handle_read(&param->read);
		break;
	}
	case ESP_GATTS_WRITE_EVT:
	{
#ifdef LOG_GATTS
		ESP_LOGI(BIODYN_BLE_TAG, "ESP_GATTS_WRITE_EVT, conn_id %d, trans_id %" PRIu32 ", handle %d", param->read.conn_id, param->read.trans_id, param->read.handle);
#endif // LOG_GATTS
		biodyn_ble_handle_write(&param->write);
		break;
	}
	case ESP_GATTS_EXEC_WRITE_EVT:
#ifdef LOG_GATTS
		ESP_LOGI(BIODYN_BLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT, conn_id %d, trans_id %" PRIu32 ", handle %d", param->read.conn_id, param->read.trans_id, param->read.handle);
#endif // LOG_GATTS
	   // TODO: Implement execute write event
		break;
	case ESP_GATTS_MTU_EVT: // We have a connection: MTU has been negotiated
#ifdef LOG_GATTS
		ESP_LOGI(BIODYN_BLE_TAG, "ESP_GATTS_MTU_EVT: Connection made, MTU negotiated as %d", param->mtu.mtu);
#endif // LOG_GATTS
		biodyn_ble_data.has_connection = true;
		break;
	case ESP_GATTS_CREATE_EVT: // Our service is created, time to add characteristics and start the service
#ifdef LOG_GATTS
		ESP_LOGI(BIODYN_BLE_TAG, "ESP_GATTS_CREATE_EVT, status %d,  service_handle %d", param->create.status, param->create.service_handle);
#endif // LOG_GATTS
		biodyn_ble_init_service(&param->create);
		break;
	case ESP_GATTS_ADD_CHAR_EVT:
	{
#ifdef LOG_GATTS
		ESP_LOGI(BIODYN_BLE_TAG, "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d",
				 param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
#endif // LOG_GATTS
		biodyn_ble_init_characterisic(&param->add_char);
		break;
	}
	case ESP_GATTS_ADD_CHAR_DESCR_EVT: // Called when a descriptor attribute handle is ready
#ifdef LOG_GATTS
		ESP_LOGI(BIODYN_BLE_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d",
				 param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
#endif // LOG_GATTS
		struct biodyn_ble_characteristic_data *chr = get_characteristic_from_id(param->add_char_descr.service_handle,
																				&param->add_char_descr.descr_uuid);
		if (!chr)
		{
			ESP_LOGE(BIODYN_BLE_TAG, "Could not find characteristic!"); // TODO: Search for service?
			biodyn_ble_data.error |= BIODYN_BLE_ERR_CANT_FIND_CHAR;
		}
		// Set the handle
		chr->attribute_handle = param->add_char_descr.attr_handle;
		break;
	case ESP_GATTS_START_EVT:
#ifdef LOG_GATTS
		ESP_LOGI(BIODYN_BLE_TAG, "SERVICE_START_EVT, status %d, service_handle %d",
				 param->start.status, param->start.service_handle);
#endif // LOG_GATTS
		break;
	case ESP_GATTS_CONNECT_EVT:
	{
#ifdef LOG_GATTS
		ESP_LOGI(BIODYN_BLE_TAG, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:",
				 param->connect.conn_id,
				 param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
				 param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
#endif // LOG_GATTS
		biodyn_ble_data.connection_id = param->connect.conn_id;
		break;
	}
	case ESP_GATTS_DISCONNECT_EVT: // We got disconnected from :(
		ESP_LOGI(BIODYN_BLE_TAG, "Device disconnected");
		biodyn_ble_data.has_connection = false;
		// Start advertising again
		esp_ble_gap_start_advertising(&biodyn_ble_advertisement_params);
		break;
	case ESP_GATTS_DELETE_EVT:
	case ESP_GATTS_STOP_EVT:
	case ESP_GATTS_UNREG_EVT:
	case ESP_GATTS_ADD_INCL_SRVC_EVT:
	case ESP_GATTS_CONF_EVT:
	case ESP_GATTS_OPEN_EVT:
	case ESP_GATTS_CANCEL_OPEN_EVT:
	case ESP_GATTS_CLOSE_EVT:
	case ESP_GATTS_LISTEN_EVT:
	case ESP_GATTS_CONGEST_EVT:
	default:
		break;
	}
}

// The GAP event handler function
void biodyn_gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
	switch (event)
	{
	case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
		// Start advertising
		esp_ble_gap_start_advertising(&biodyn_ble_advertisement_params);
		ESP_LOGI(BIODYN_BLE_TAG, "Started advertising!");
		break;
	case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
		break;
	case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
		// Advertisement start is done, did we do it??
		if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
		{
			ESP_LOGE(BIODYN_BLE_TAG, "Advertising start failed");
			biodyn_ble_data.error |= BIODYN_BLE_ERR_CANT_ADVERTISE;
		}
		break;
	case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
		if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
		{
			ESP_LOGE(BIODYN_BLE_TAG, "Advertising stop failed");
			biodyn_ble_data.error |= BIODYN_BLE_ERR_CANT_STOP_ADVERTISE;
		}
		else
			ESP_LOGI(BIODYN_BLE_TAG, "Advertising stopped successfully");
		break;
	case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
		ESP_LOGI(BIODYN_BLE_TAG, "Updated connection parameters: status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
				 param->update_conn_params.status,
				 param->update_conn_params.min_int,
				 param->update_conn_params.max_int,
				 param->update_conn_params.conn_int,
				 param->update_conn_params.latency,
				 param->update_conn_params.timeout);
		break;
	default:
		break;
	}
}

// Loads the given profile schematics to the driver.
// This function assumes that there isn't already a schematic loaded.
void biodyn_ble_load_schematic(uint16_t n_profiles, struct biodyn_ble_profile *profiles_schematic_list)
{
	biodyn_ble_data.n_profiles = n_profiles;
	biodyn_ble_data.profiles = malloc(sizeof(struct biodyn_ble_profile_data) * n_profiles);

	// For each profile
	for (int i = 0; i < n_profiles; ++i)
	{
		struct biodyn_ble_profile_data *profile = &biodyn_ble_data.profiles[i];
		struct biodyn_ble_profile *profile_schematic = &profiles_schematic_list[i];
		profile->name = profile_schematic->name;
		profile->n_services = profile_schematic->n_services;
		profile->services = malloc(sizeof(struct biodyn_ble_service_data) * profile_schematic->n_services);
		profile->gatts_if = ESP_GATT_IF_NONE;

		// For each service
		for (int j = 0; j < profile_schematic->n_services; ++j)
		{
			struct biodyn_ble_service_data *service = &profile->services[j];
			struct biodyn_ble_service *service_schematic = &profile_schematic->services[j];
			service->name = service_schematic->name;
			service->n_characteristics = service_schematic->n_characteristics;
			service->characteristics = malloc(sizeof(struct biodyn_ble_characteristic_data) * service->n_characteristics);
			service->service_state = biodyn_ble_service_not_created;
			service->service_handle = 0; // We haven't been assigned one yet
			service->service_id = service_schematic->service_id;
			service->n_handles = 1;							   // Start with 1 for the service itself
			service->profile = profile;						   // Set parent
			service->advertise = service_schematic->advertise; // Set advertise or not

			// Add to advertisement data if we want to
			if (service->advertise)
			{
				if (service->service_id.id.uuid.len != biodyn_ble_data.advertisement_uuid_size && biodyn_ble_data.advertisement_uuid_size != 0)
				{
					ESP_LOGE(BIODYN_BLE_TAG, "Only one length of UUID can be advertised. Tried to use %d when %d is already used.",
							 service->service_id.id.uuid.len, biodyn_ble_data.advertisement_uuid_size);
					biodyn_ble_data.error |= BIODYN_BLE_ERR_CANT_ADV_SERVICE;
				}
				else
				{
					uint16_t uuid_len = service->service_id.id.uuid.len;
					uint16_t n_uuids = biodyn_ble_advertisement_data.service_uuid_len / uuid_len;
					// Set the adv uuid len
					biodyn_ble_data.advertisement_uuid_size = uuid_len;

					// Add UUID to list
					uint8_t *uuids = (uint8_t *)malloc(sizeof(uint8_t) * uuid_len * (n_uuids + 1));
					memcpy(uuids, biodyn_ble_advertisement_data.p_service_uuid, sizeof(uint8_t) * uuid_len * n_uuids);
					if (uuid_len == ESP_UUID_LEN_128)
						memcpy(uuids + (n_uuids * uuid_len), service->service_id.id.uuid.uuid.uuid128, ESP_UUID_LEN_128);
					else if (uuid_len == ESP_UUID_LEN_32)
						memcpy(uuids + (n_uuids * uuid_len), service->service_id.id.uuid.uuid.uuid128, ESP_UUID_LEN_32);
					else if (uuid_len == ESP_UUID_LEN_16)
						memcpy(uuids + (n_uuids * uuid_len), service->service_id.id.uuid.uuid.uuid128, ESP_UUID_LEN_16);
					else
					{
						ESP_LOGE(BIODYN_BLE_TAG, "Invalid UUID len: %d", uuid_len);
						biodyn_ble_data.error = BIODYN_BLE_ERR_CANT_ADV_SERVICE;
					}

					// Update length
					biodyn_ble_advertisement_data.service_uuid_len = (n_uuids + 1) * uuid_len * sizeof(uint8_t);

					// Free old data and set new
					if (n_uuids != 0)
						free(biodyn_ble_advertisement_data.p_service_uuid);
					biodyn_ble_advertisement_data.p_service_uuid = uuids;
				}
			}

			// For each characteristic
			for (int k = 0; k < service_schematic->n_characteristics; ++k)
			{
				struct biodyn_ble_characteristic_data *chr = &service->characteristics[k];
				struct biodyn_ble_characteristic *chr_schematic = &service_schematic->characteristics[k];
				chr->name = chr_schematic->name;
				chr->uuid = chr_schematic->uuid;
				chr->permissions = chr_schematic->permissions;
				chr->properties = chr_schematic->properties;
				chr->characteristic_state = biodyn_ble_characteristic_not_created;
				// Add 1 for the characteristic, 1 for the value, and 1 for each descriptor
				service->n_handles += 2 + chr_schematic->n_descriptors;
				chr->service = service; // Set parent
				// Set the initial value
				chr->initial_value.attr_len = chr_schematic->intial_value_size;
				chr->initial_value.attr_max_len = chr_schematic->intial_value_size;
				chr->initial_value.attr_value = chr_schematic->initial_value;
				chr->response_type.auto_rsp = chr_schematic->intial_value_size == 0 ? ESP_GATT_RSP_BY_APP : ESP_GATT_AUTO_RSP;
				chr->get_data_fn = chr_schematic->get_data;
				chr->set_data_fn = chr_schematic->set_data;
				// Setup descriptors
				chr->n_descriptors = chr_schematic->n_descriptors;
				chr->descriptors = malloc(sizeof(struct biodyn_ble_char_descr_data) * chr->n_descriptors);

				for (int l = 0; l < chr->n_descriptors; ++l)
				{
					struct biodyn_ble_char_descr_data *descr = &chr->descriptors[l];
					struct biodyn_ble_char_descriptor *descr_schematic = &chr_schematic->descriptors[l];
					descr->uuid = descr_schematic->uuid;
					descr->permissions = descr_schematic->permissions;
					descr->value = descr_schematic->value;
					descr->descriptor_handle = 0; // We dont have one yet

					// Add a handle if our descriptor has a value
					if (descr->value.attr_max_len != 0)
						chr->n_descriptors++;
				}
			}
		}
	}
}

// Clears & frees assocated memory of the current schematic loaded
void biodyn_ble_clear_schematic()
{
	ESP_LOGI(BIODYN_BLE_TAG, "Freeing current schematic: profiles, services, and characteristics.");
	// For each profile
	for (int i = 0; i < biodyn_ble_data.n_profiles; ++i)
	{
		struct biodyn_ble_profile_data *profile = &biodyn_ble_data.profiles[i];

		// For each service
		for (int j = 0; j < profile->n_services; ++j)
		{
			struct biodyn_ble_service_data *service = &profile->services[j];

			// For each characteristic
			for (int k = 0; j < service->n_characteristics; ++k)
			{
				struct biodyn_ble_characteristic_data *characteristic = &service->characteristics[k];

				free(characteristic->descriptors);
			}
			free(service->characteristics);
		}
		free(profile->services);
	}
	free(biodyn_ble_data.profiles);
	biodyn_ble_data.n_profiles = 0;
	biodyn_ble_data.profiles = NULL;

	ESP_LOGI(BIODYN_BLE_TAG, "Current schematic freed: profiles, services, and characteristics");
}

// Initializes the BIODYN bluetooth low energy GATTS server and GAP protocols.
// Returns non-zero value on error. This is all a client needs to call.
esp_err_t biodyn_ble_init(uint16_t n_profiles, struct biodyn_ble_profile *profiles)
{
	esp_err_t ret = 0;

	// Release resources associated with classic bluetooth - we're using BLE
	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

	// Set up bluetooth controller config as default
	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	ret = esp_bt_controller_init(&bt_cfg);
	if (ret)
	{
		ESP_LOGE(BIODYN_BLE_TAG, "BT controller init failed in %s", __func__);
		return ret;
	}

	// Set bluetooth controller mode as BLE (not classic)
	ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
	if (ret)
	{
		ESP_LOGE(BIODYN_BLE_TAG, "BT controller enable failed in %s", __func__);
		return ret;
	}

	// Initialize bluedroid
	ret = esp_bluedroid_init();
	if (ret)
	{
		ESP_LOGE(BIODYN_BLE_TAG, "Bluetooth init failed in %s", __func__);
		return ret;
	}
	ret = esp_bluedroid_enable();
	if (ret)
	{
		ESP_LOGE(BIODYN_BLE_TAG, "Bluetooth init failed in %s", __func__);
		return ret;
	}

	// Load our BLE profile/service/characteristic schematic to the driver
	biodyn_ble_load_schematic(n_profiles, profiles);

	// Register the generic attribute profile server (GATTS) callback
	ret = esp_ble_gatts_register_callback(biodyn_gatts_event_handler);
	if (ret)
	{
		ESP_LOGE(BIODYN_BLE_TAG, "gatts register error, error code = %x", ret);
		return ret;
	}
	// Register the generic access profile (GAP) callback
	ret = esp_ble_gap_register_callback(biodyn_gap_event_handler);
	if (ret)
	{
		ESP_LOGE(BIODYN_BLE_TAG, "gap register error, error code = %x", ret);
		return ret;
	}
	// Register our application identifiers
	ret = esp_ble_gatts_app_register(BIODYN_GATTS_APP_ID);
	if (ret)
	{
		ESP_LOGE(BIODYN_BLE_TAG, "gatts app register error, error code = %x", ret);
		return ret;
	}

	// Set the maximum transmission unit (MTU) size
	ret = esp_ble_gatt_set_local_mtu(517);
	if (ret)
	{
		ESP_LOGE(BIODYN_BLE_TAG, "Failed to set BLE maximum transmission unit: error code %x", ret);
		return ret;
	}

	return ret;
}
