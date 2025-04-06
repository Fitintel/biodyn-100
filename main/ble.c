
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
#include "ble_utils.h"
#include "constants.h"

/* ----------------------------
		Data definitions
   ---------------------------- */

// Our app id
#define BIODYN_GATTS_APP_ID 0

// This is our general bluetooth driver data.
static struct
{
	bool has_connection;	// Do we have an active connection to another device?
	biodyn_ble_err_t error; // 0 if ok, otherwise has a BIODYN_BLE_ERR_ set
} biodyn_ble_data = {false, 0};

// Returns any accumulated errors in the biodyn bluetooth driver
biodyn_ble_err_t biodyn_ble_check_err()
{
	return biodyn_ble_data.error;
}

// Represents a characteristic under a service
struct biodyn_ble_characteristic
{
	const char *name;	// The name of the characteristic
	esp_bt_uuid_t uuid; // The UUID for the characteristic

	// The associated permissions. What is required for the properties to be accessed.
	// Ex. normal read, requires encryption, requires authentication
	esp_gatt_perm_t permissions;

	// Characteristic properties: what can be done with this characteristic
	// Ex. this characteristic can be read, written to, notify the client, etc
	esp_gatt_char_prop_t properties;

	// TODO: How do we provide a nice interface to register characteristic info and callbacks?
};

// Represents the general components for a service
struct biodyn_ble_service
{
	const char *name;								   // The name of the service
	esp_gatt_srvc_id_t service_id;					   // The GATT service identifier
	struct biodyn_ble_characteristic *characteristics; // The characteristics for this service
	uint16_t n_characteristics;						   // The number of characteristics in this service

	// The number of handles required by this service
	// Handles include: service declaration, characteristic declaration, characteristic value,
	//  				characteristic descriptiors, etc.
	uint16_t n_handles;

	uint16_t service_handle; // The service handle. This is assiged when created, no need to be initialized.
};

// Represents the general components for a profile
struct biodyn_ble_profile
{
	const char *name;					 // The name of the profile
	struct biodyn_ble_service *services; // A list of the services under this profile
	size_t n_services;					 // The number of services this profile offers
	uint16_t gatts_if;					 // The GATTS identifier for our server
};
#define BIODYN_GATTS_APP_ID 0

// Profile identifiers
#define BIODYN_DEVICE_PROFILE_ID 0 // TODO: Implement me!
#define BIODYN_SENSOR_PROFILE_ID 1 // TODO: Implement me!

// This is a list of our profiles. See the Biodyn design document.
static struct biodyn_ble_profile biodyn_ble_profiles[] = {
	[BIODYN_DEVICE_PROFILE_ID] = {
		.name = "Device Profile",
		.n_services = 0,			  // TODO: Add services
		.services = NULL,			  // TODO: Add services
		.gatts_if = ESP_GATT_IF_NONE, // We don't know yet
	},
};
// The number of ble profiles the BIODYN has
#define NUM_BLE_PROFILES (sizeof(biodyn_ble_profiles) / sizeof(struct biodyn_ble_profile))

// Returns the service data for the service with the given ID in all profiles.
// Returns NULL if not found.
struct biodyn_ble_service *get_service_from_id(esp_gatt_srvc_id_t *service_id)
{
	// For each profile
	for (int i = 0; i < NUM_BLE_PROFILES; ++i)
		// For each service
		for (int j = 0; j < biodyn_ble_profiles[i].n_services; ++j)
		{
			// Make sure we have services for this profile
			if (biodyn_ble_profiles[i].services == NULL)
				continue;

			struct biodyn_ble_service *service = &biodyn_ble_profiles[i].services[j];

			// Check if the service ids are equal
			if (service_ids_equal(&service->service_id, service_id))
				return service;
		}

	// We didn't find it ... :(
	return NULL;
}

// Manufacturer data included in the BLE advertisement
static uint8_t manufacturer_data = 0; // TODO: What should we put here?
// Then data length in bytes for manufacturer data included in BLE advertisement
// This *CANNOT* be larger than 31 bytes
#define BIODYN_MANUFACTURER_DATA_LEN sizeof(manufacturer_data)

// BLE advertisement data packet configuration.
// This configures the structure of our packets.
esp_ble_adv_data_t ble_advertisement_data = {
	.set_scan_rsp = false,							  // This is not a scan response packet
	.include_name = true,							  // Include device name in advertisement
	.include_txpower = true,						  // Include TX power in advertisement
	.min_interval = 6,								  // Preffered connection minimum time interval
	.max_interval = 12,								  // Preferred connection maximum time interval
	.appearance = 0x090718,							  // Information about the device type (device class): ie. wearable etc
	.manufacturer_len = BIODYN_MANUFACTURER_DATA_LEN, // See definition
	.p_manufacturer_data = &manufacturer_data,		  // Pointer to manufacturer data

	// TODO: We can advertise service data without a connection made.
	// This could be useful for sharing some metrics between FITNET nodes directly
	// since our BLE only has single-connection capability.
	.service_data_len = 0,
	.p_service_data = NULL,

	// This says what services we have. It is not crucial that this is
	// here but it can give devices info on services provided. In context of the
	// BIOHUB it doesn't matter all that much. TODO: What services should we advertise?
	.service_uuid_len = 0,	// We aren't advertising any services yet
	.p_service_uuid = NULL, // ^

	// General discoverable mode + low-energy bluetooth only device
	.flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// BLE advertisement parameters. This configures how we advertise, not what.
static esp_ble_adv_params_t ble_advertisement_params = {
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
	esp_err_t err;

	// For each of the biodyn GATT profiles
	for (int i = 0; i < NUM_BLE_PROFILES; ++i)
	{
		struct biodyn_ble_profile *profile = &biodyn_ble_profiles[i];
		profile->gatts_if = gatts_if;

		ESP_LOGI(GATTS_TAG, "Creating profile \"%s\" with %d services", profile->name, profile->n_services);

		// For each of this profile's services
		for (int j = 0; j < profile->n_services; ++j)
		{
			struct biodyn_ble_service *service = &profile->services[j];

			// Create the service
			esp_err_t create_service_err;
			if (create_service_err = esp_ble_gatts_create_service(gatts_if, &service->service_id, service->n_handles))
			{
				ESP_LOGE(GATTS_TAG, "Failed to create service: %s returned error %x", service->name, create_service_err);
				biodyn_ble_data.error |= BIODYN_BLE_ERR_CANT_CREATE_SERVICE;
			}
			else // We're not clearing associated error because it may have happened during this loop!
				ESP_LOGI(GATTS_TAG, "\tCreating service \"%s\" with %d characteristics and %d handles",
						 service->name, service->n_characteristics, service->n_handles);

			if (!err)
				err = create_service_err;
		}
	}

	return err;
}

// Configures and initializes our GATT server instance
// Called when recieving ESP_GATTS_REG_EVT signalling our server is ready to set up.
esp_err_t biodyn_ble_gatts_init(esp_gatt_if_t gatts_if)
{
	esp_err_t err;
	// Set the device name
	if (err = esp_ble_gap_set_device_name(BIODYN_DEVICE_NAME))
	{
		ESP_LOGE(GATTS_TAG, "Failed to set device name to %s, error code = %x", BIODYN_DEVICE_NAME, err);
		biodyn_ble_data.error |= BIODYN_BLE_ERR_CANT_NAME;
	}
	else
	{
		ESP_LOGI(GATTS_TAG, "Set device name to %s", BIODYN_DEVICE_NAME);
		biodyn_ble_data.error &= ~BIODYN_BLE_ERR_CANT_NAME;
	}

	// Configure advertisement data
	if (err = esp_ble_gap_config_adv_data(&ble_advertisement_data))
	{
		ESP_LOGE(GATTS_TAG, "Failed to configure advertisement packet data, error code = %x", err);
		biodyn_ble_data.error |= BIODYN_BLE_ERR_ADV_DATA_INIT;
	}
	else
	{
		ESP_LOGD(GATTS_TAG, "Set advertisement data");
		biodyn_ble_data.error &= ~BIODYN_BLE_ERR_ADV_DATA_INIT;
	}

	// Create our services for our profiles
	if (err = biodyn_ble_init_profiles(gatts_if))
		ESP_LOGE(GATTS_TAG, "Failed to create one or more BLE services!");
	return err;
}

// Configures and initializes a service which has requested creation.
// Finds the service contained in the creation parameters.
// Called when recieving ESP_GATTS_CREATE_EVT signalling our service is ready to configure and start.
esp_err_t biodyn_ble_init_service(struct gatts_create_evt_param *cep)
{
	esp_err_t err = 0;

	// Find the created service among the ones we have
	struct biodyn_ble_service *service = get_service_from_id(&cep->service_id);
	if (!service)
	{
		ESP_LOGE(GATTS_TAG, "Could not find service associated with id %x", cep->service_id);
		biodyn_ble_data.error |= BIODYN_BLE_ERR_CANT_FIND_SERVICE;
		return ESP_ERR_INVALID_STATE;
	}

	// We have our service, let's start it
	service->service_handle = cep->service_handle;
	if (err = esp_ble_gatts_start_service(service->service_handle))
	{
		ESP_LOGE(GATTS_TAG, "Failed to create service \"%s\" (id %x), error code %d", cep->service_id, service->name, err);
		biodyn_ble_data.error |= BIODYN_BLE_ERR_CANT_START_SERVICE;
		return err;
	}

	// If we have no characteristics to add, but we think we do, there's a BIG problem
	if (!service->characteristics && service->n_characteristics != 0)
	{
		ESP_LOGE(GATTS_TAG, "Reported characteristic count is wrong (reported %d) for service \"%s\" (id %x)",
				 service->n_characteristics, service->name, service->service_id);
		biodyn_ble_data.error |= BIODYN_BLE_ERR_CHARS_MISCONFIGURED;
		return ESP_ERR_INVALID_STATE;
	}

	// Time to add characteristics
	for (int i = 0; i < service->n_characteristics; ++i)
	{
		struct biodyn_ble_characteristic *characteristic = &service->characteristics[i];
		// Create the characteristic with no initial value
		esp_err_t add_char_ret = esp_ble_gatts_add_char(service->service_handle, &characteristic->uuid,
														characteristic->permissions,
														characteristic->properties,
														NULL, NULL);
		if (add_char_ret)
		{
			ESP_LOGE(GATTS_TAG, "Failed to add characteristic \"%s\" to service \"%s\", error code %x",
					 characteristic->name, service->name, add_char_ret);
			err = add_char_ret;
			biodyn_ble_data.error |= BIODYN_BLE_ERR_CANT_CREATE_CHAR;
		}
	}

	// Done!
	return err;
}

// The global gatts server event handler function
void biodyn_gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
	esp_err_t err;
	switch (event)
	{
	case ESP_GATTS_REG_EVT: // Registering our server, let's provide some info.
		ESP_LOGD(GATTS_TAG, "ESP_GATTS_REG_EVT: status %d, app id %d", param->reg.status, param->reg.app_id);
		biodyn_ble_gatts_init(gatts_if);
		break;
	case ESP_GATTS_READ_EVT:
	{
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_READ_EVT, conn_id %d, trans_id %" PRIu32 ", handle %d", param->read.conn_id, param->read.trans_id, param->read.handle);
		// TODO: Implement read event
		break;
	}
	case ESP_GATTS_WRITE_EVT:
	{
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_WRITE_EVT, conn_id %d, trans_id %" PRIu32 ", handle %d", param->read.conn_id, param->read.trans_id, param->read.handle);
		// TODO: Implement write event
		break;
	}
	case ESP_GATTS_EXEC_WRITE_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_EXEC_WRITE_EVT, conn_id %d, trans_id %" PRIu32 ", handle %d", param->read.conn_id, param->read.trans_id, param->read.handle);
		// TODO: Implement execute write event
		break;
	case ESP_GATTS_MTU_EVT: // We have a connection: MTU has been negotiated
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_MTU_EVT: Connection made, MTU negotiated as %d", param->mtu.mtu);
		biodyn_ble_data.has_connection = true;
		break;
	case ESP_GATTS_UNREG_EVT:
		break;
	case ESP_GATTS_CREATE_EVT: // Our service is created, time to add characteristics and start the service
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_CREATE_EVT, status %d,  service_handle %d", param->create.status, param->create.service_handle);
		biodyn_ble_init_service(&param->create);
		break;
	case ESP_GATTS_ADD_INCL_SRVC_EVT:
		break;
	case ESP_GATTS_ADD_CHAR_EVT:
	{
		uint16_t length = 0;
		const uint8_t *prf_char;

		ESP_LOGI(GATTS_TAG, "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d",
				 param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
		gl_profile_tab[PROFILE_A_APP_ID].char_handle = param->add_char.attr_handle;
		gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
		gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
		esp_err_t get_attr_ret = esp_ble_gatts_get_attr_value(param->add_char.attr_handle, &length, &prf_char);
		if (get_attr_ret == ESP_FAIL)
		{
			ESP_LOGE(GATTS_TAG, "ILLEGAL HANDLE");
		}

		ESP_LOGI(GATTS_TAG, "the gatts demo char length = %x", length);
		for (int i = 0; i < length; i++)
		{
			ESP_LOGI(GATTS_TAG, "prf_char[%x] =%x", i, prf_char[i]);
		}
		esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].descr_uuid,
															   ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
		if (add_descr_ret)
		{
			ESP_LOGE(GATTS_TAG, "add char descr failed, error code =%x", add_descr_ret);
		}
		break;
	}
	case ESP_GATTS_ADD_CHAR_DESCR_EVT:

		gl_profile_tab[PROFILE_A_APP_ID].descr_handle = param->add_char_descr.attr_handle;
		ESP_LOGI(GATTS_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d",
				 param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
		break;
	case ESP_GATTS_DELETE_EVT:
		break;
	case ESP_GATTS_START_EVT:
		ESP_LOGI(GATTS_TAG, "SERVICE_START_EVT, status %d, service_handle %d",
				 param->start.status, param->start.service_handle);
		break;
	case ESP_GATTS_STOP_EVT:
		break;
	case ESP_GATTS_CONNECT_EVT:
	{
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:",
				 param->connect.conn_id,
				 param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
				 param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
		gl_profile_tab[PROFILE_A_APP_ID].conn_id = param->connect.conn_id;
		break;
	}
	case ESP_GATTS_DISCONNECT_EVT: // We got disconnected from :(
		ESP_LOGI(GATTS_TAG, "Device disconnected");
		biodyn_ble_data.has_connection = false;
		// Start advertising again
		esp_ble_gap_start_advertising(&ble_advertisement_params);
		break;
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
	// TODO: Implement
	switch (event)
	{
	case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
		adv_config_done &= (~adv_config_flag);
		if (adv_config_done == 0)
		{
			esp_ble_gap_start_advertising(&adv_params);
		}
		break;
	case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
		adv_config_done &= (~scan_rsp_config_flag);
		if (adv_config_done == 0)
		{
			esp_ble_gap_start_advertising(&adv_params);
		}
		break;
	case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
		// advertising start complete event to indicate advertising start successfully or failed
		if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
		{
			ESP_LOGE(GATTS_TAG, "Advertising start failed");
		}
		break;
	case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
		if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
		{
			ESP_LOGE(GATTS_TAG, "Advertising stop failed");
		}
		else
		{
			ESP_LOGI(GATTS_TAG, "Stop adv successfully");
		}
		break;
	case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
		ESP_LOGI(GATTS_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
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

// Initializes the BIODYN bluetooth low energy GATTS server and GAP protocols.
// Returns non-zero value on error. This is all a client needs to call.
esp_err_t biodyn_ble_init()
{
	esp_err_t ret = 0;

	// Release resources associated with classic bluetooth - we're using BLE
	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

	// Set up bluetooth controller config as default
	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	ret = esp_bt_controller_init(&bt_cfg);
	if (ret)
	{
		ESP_LOGE(GATTS_TAG, "BT controller init failed in %s", __func__);
		return ret;
	}

	// Set bluetooth controller mode as BLE (not classic)
	ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
	if (ret)
	{
		ESP_LOGE(GATTS_TAG, "BT controller enable failed in %s", __func__);
		return ret;
	}

	// Initialize bluedroid
	ret = esp_bluedroid_init();
	if (ret)
	{
		ESP_LOGE(GATTS_TAG, "Bluetooth init failed in %s", __func__);
		return ret;
	}
	ret = esp_bluedroid_enable();
	if (ret)
	{
		ESP_LOGE(GATTS_TAG, "Bluetooth init failed in %s", __func__);
		return ret;
	}

	// Register the generic attribute profile server (GATTS) callback
	ret = esp_ble_gatts_register_callback(biodyn_gatts_event_handler);
	if (ret)
	{
		ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
		return ret;
	}
	// Register the generic access profile (GAP) callback
	ret = esp_ble_gap_register_callback(biodyn_gap_event_handler);
	if (ret)
	{
		ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
		return ret;
	}
	// Register our application identifiers
	ret = esp_ble_gatts_app_register(BIODYN_GATTS_APP_ID);
	if (ret)
	{
		ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
		return ret;
	}

	// Set the maximum transmission unit (MTU) size
	ret = esp_ble_gatt_set_local_mtu(517);
	if (ret)
	{
		ESP_LOGE(GATTS_TAG, "Failed to set BLE maximum transmission unit: error code %x", ret);
		return ret;
	}

	return ret;
}
