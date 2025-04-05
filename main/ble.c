
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

// Profile identifiers (also known as 'applications')
#define BIODYN_DEVICE_PROFILE_ID 0 // TODO: Implement me!
#define BIODYN_SENSOR_PROFILE_ID 1 // TODO: Implement me!

// Manufacturer data included in the BLE advertisement
static uint8_t manufacturer_data = 0; // TODO: What should we put here?
// Then data length in bytes for manufacturer data included in BLE advertisement
// This *CANNOT* be larger than 31 bytes
#define BIODYN_MANUFACTURER_DATA_LEN sizeof(manufacturer_data)


// BLE advertisement data configuration
esp_ble_adv_data_t ble_advertisement_data = {
	.set_scan_rsp = false, // This is not a scan response packet 
	.include_name = true, // Include device name in advertisement
	.include_txpower = true, // Include TX power in advertisement
	.min_interval = 6, // Preffered connection minimum time interval
	.max_interval = 12, // Preferred connection maximum time interval
	.appearance = 0x090718, // Information about the device type (device class): ie. wearable etc
	.manufacturer_len = BIODYN_MANUFACTURER_DATA_LEN, // See definition
	.p_manufacturer_data = &manufacturer_data, // Pointer to manufacturer data

	// TODO: We can advertise service data without a connection made.
	// This could be useful for sharing some metrics between FITNET nodes directly
	// since our BLE only has single-connection capability.
	.service_data_len = 0,
	.p_service_data = NULL,

	// This is our service advertisement data. It is not crucial that this is
	// here but it can give devices info on services provided. In context of the
	// BIOHUB it doesn't matter all that much. TODO: What services should we advertise?
	.service_uuid_len = 0, // We aren't providing any service data yet 
	.p_service_uuid = NULL, // ^

	// General discoverable mode + low-energy bluetooth only device
	.flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT), 
};

// Initializes the BIODYN bluetooth low energy GATTS server and GAP protocols
// Returns non-zero value on error
esp_err_t biodyn_init_ble()
{
	esp_err_t ret;

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

	return 0;
}


// The GATTS server callback function
void biodyn_gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
	switch (event)
	{
	case ESP_GATTS_REG_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_REG_EVT, status %d, app_id %d", param->reg.status, param->reg.app_id);

		// Set the device name
		esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(BIODYN_DEVICE_NAME);
		if (set_dev_name_ret)
		{
			ESP_LOGE(GATTS_TAG, "Failed to se device name to %s, error code = %x", BIODYN_DEVICE_NAME, set_dev_name_ret);
		}
		// Configure advertisement data
		esp_err_t ret = esp_ble_gap_config_adv_data(&ble_advertisement_data);
		if (ret)
		{
			ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
		}
		adv_config_done |= adv_config_flag;
		// config scan response data
		ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
		if (ret)
		{
			ESP_LOGE(GATTS_TAG, "config scan response data failed, error code = %x", ret);
		}
		adv_config_done |= scan_rsp_config_flag;
		esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_A_APP_ID].service_id, GATTS_NUM_HANDLE_TEST_A);
		break;
	case ESP_GATTS_READ_EVT:
	{
		ESP_LOGI(GATTS_TAG, "GATT_READ_EVT, conn_id %d, trans_id %" PRIu32 ", handle %d", param->read.conn_id, param->read.trans_id, param->read.handle);
		esp_gatt_rsp_t rsp;
		memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
		rsp.attr_value.handle = param->read.handle;
		rsp.attr_value.len = 4;
		rsp.attr_value.value[0] = 0xde;
		rsp.attr_value.value[1] = 0xed;
		rsp.attr_value.value[2] = 0xbe;
		rsp.attr_value.value[3] = 0xef;
		esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
									ESP_GATT_OK, &rsp);
		break;
	}
	case ESP_GATTS_WRITE_EVT:
	{
		example_write_event_env(gatts_if, &a_prepare_write_env, param);
		break;
	}
	case ESP_GATTS_EXEC_WRITE_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
		esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
		example_exec_write_event_env(&a_prepare_write_env, param);
		break;
	case ESP_GATTS_MTU_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
		is_connect = true;
		break;
	case ESP_GATTS_UNREG_EVT:
		break;
	case ESP_GATTS_CREATE_EVT:
		ESP_LOGI(GATTS_TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d", param->create.status, param->create.service_handle);
		gl_profile_tab[PROFILE_A_APP_ID].service_handle = param->create.service_handle;
		gl_profile_tab[PROFILE_A_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
		gl_profile_tab[PROFILE_A_APP_ID].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_TEST_A;

		esp_ble_gatts_start_service(gl_profile_tab[PROFILE_A_APP_ID].service_handle);
		a_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
		esp_err_t add_char_ret = esp_ble_gatts_add_char(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].char_uuid,
														ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
														a_property,
														&gatts_demo_char1_val, NULL);
		if (add_char_ret)
		{
			ESP_LOGE(GATTS_TAG, "add char failed, error code =%x", add_char_ret);
		}
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
	case ESP_GATTS_DISCONNECT_EVT:
		is_connect = false;
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_DISCONNECT_EVT");
		esp_ble_gap_start_advertising(&adv_params);
		break;
	case ESP_GATTS_CONF_EVT:
		break;
	case ESP_GATTS_OPEN_EVT:
	case ESP_GATTS_CANCEL_OPEN_EVT:
	case ESP_GATTS_CLOSE_EVT:
	case ESP_GATTS_LISTEN_EVT:
		break;
	case ESP_GATTS_CONGEST_EVT:
		break;
	default:
		break;
	}
}


void biodyn_gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
	// TODO: Implement
}