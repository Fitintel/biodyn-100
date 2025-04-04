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

#include "constants.h"
#include "ble.h"

/**********************************************************
 * Thread/Task reference
 **********************************************************/
#ifdef CONFIG_BLUEDROID_PINNED_TO_CORE
#define BLUETOOTH_TASK_PINNED_TO_CORE (CONFIG_BLUEDROID_PINNED_TO_CORE < CONFIG_FREERTOS_NUMBER_OF_CORES ? CONFIG_BLUEDROID_PINNED_TO_CORE : tskNO_AFFINITY)
#else
#define BLUETOOTH_TASK_PINNED_TO_CORE (0)
#endif

#define SECOND_TO_USECOND 1000000

static bool is_connect = false;
/// Declare the static functior
static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

#define GATTS_SERVICE_UUID_TEST_A 0x00FF
#define GATTS_CHAR_UUID_TEST_A 0xFF01
#define GATTS_DESCR_UUID_TEST_A 0x3333
#define GATTS_NUM_HANDLE_TEST_A 4

#define GATTS_SERVICE_UUID_TEST_B 0x00EE
#define GATTS_CHAR_UUID_TEST_B 0xEE01
#define GATTS_DESCR_UUID_TEST_B 0x2222
#define GATTS_NUM_HANDLE_TEST_B 4

#define DEVICE_NAME "BIODYN-100"

#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40

#define PREPARE_BUF_MAX_SIZE 1024

static uint8_t char1_str[] = {0x11, 0x22, 0x33};
static esp_gatt_char_prop_t a_property = 0;

static esp_attr_value_t gatts_demo_char1_val =
	{
		.attr_max_len = GATTS_DEMO_CHAR_VAL_LEN_MAX,
		.attr_len = sizeof(char1_str),
		.attr_value = char1_str,
};

static uint8_t adv_config_done = 0;
#define adv_config_flag (1 << 0)
#define scan_rsp_config_flag (1 << 1)

static uint8_t adv_service_uuid128[32] = {
	/* LSB <--------------------------------------------------------------------------------> MSB */
	// first uuid, 16bit, [12],[13] is the value
	0xfb,
	0x34,
	0x9b,
	0x5f,
	0x80,
	0x00,
	0x00,
	0x80,
	0x00,
	0x10,
	0x00,
	0x00,
	0xEE,
	0x00,
	0x00,
	0x00,
	// second uuid, 32bit, [12], [13], [14], [15] is the value
	0xfb,
	0x34,
	0x9b,
	0x5f,
	0x80,
	0x00,
	0x00,
	0x80,
	0x00,
	0x10,
	0x00,
	0x00,
	0xFF,
	0x00,
	0x00,
	0x00,
};

// The length of adv data must be less than 31 bytes
// TODO: Use this to differentiate device type
#define MANUFACTURER_DATA_LEN 17
static uint8_t manufacturer_data[MANUFACTURER_DATA_LEN] = {0x12, 0x23, 0x45, 0x56};

// Advertising data
static esp_ble_adv_data_t adv_data = {
	.set_scan_rsp = false,
	.include_name = true,
	.include_txpower = true,
	.min_interval = 0x0006, // slave connection min interval, Time = min_interval * 1.25 msec
	.max_interval = 0x000C, // slave connection max interval, Time = max_interval * 1.25 msec
	.appearance = 0x00,
	.manufacturer_len = MANUFACTURER_DATA_LEN,
	.p_manufacturer_data = &manufacturer_data[0],
	.service_data_len = 0,
	.p_service_data = NULL,
	.service_uuid_len = 32,
	.p_service_uuid = adv_service_uuid128,
	.flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
	.set_scan_rsp = true,
	.include_name = true,
	.include_txpower = true,
	.min_interval = 0x0006,
	.max_interval = 0x000C,
	.appearance = 0x00,
	.manufacturer_len = MANUFACTURER_DATA_LEN,
	.p_manufacturer_data = &manufacturer_data[0],
	.service_data_len = 0,
	.p_service_data = NULL,
	.service_uuid_len = 32,
	.p_service_uuid = adv_service_uuid128,
	.flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
	.adv_int_min = 0x20,
	.adv_int_max = 0x40,
	.adv_type = ADV_TYPE_IND,
	.own_addr_type = BLE_ADDR_TYPE_PUBLIC,
	//.peer_addr            =
	//.peer_addr_type       =
	.channel_map = ADV_CHNL_ALL,
	.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

#define PROFILE_NUM 1
#define PROFILE_A_APP_ID 0

struct gatts_profile_inst
{
	esp_gatts_cb_t gatts_cb;
	uint16_t gatts_if;
	uint16_t app_id;
	uint16_t conn_id;
	uint16_t service_handle;
	esp_gatt_srvc_id_t service_id;
	uint16_t char_handle;
	esp_bt_uuid_t char_uuid;
	esp_gatt_perm_t perm;
	esp_gatt_char_prop_t property;
	uint16_t descr_handle;
	esp_bt_uuid_t descr_uuid;
};

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
	[PROFILE_A_APP_ID] = {
		.gatts_cb = gatts_profile_a_event_handler,
		.gatts_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
	},
};

typedef struct
{
	uint8_t *prepare_buf;
	int prepare_len;
} prepare_type_env_t;

static prepare_type_env_t a_prepare_write_env;

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);

static uint8_t check_sum(uint8_t *addr, uint16_t count)
{
	uint32_t sum = 0;

	if (addr == NULL || count == 0)
	{
		return 0;
	}

	for (int i = 0; i < count; i++)
	{
		sum = sum + addr[i];
	}

	while (sum >> 8)
	{
		sum = (sum & 0xff) + (sum >> 8);
	}

	return (uint8_t)~sum;
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
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

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
	esp_gatt_status_t status = ESP_GATT_OK;
	if (param->write.need_rsp)
	{
		if (param->write.is_prep)
		{
			if (param->write.offset > PREPARE_BUF_MAX_SIZE)
			{
				status = ESP_GATT_INVALID_OFFSET;
			}
			else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE)
			{
				status = ESP_GATT_INVALID_ATTR_LEN;
			}

			if (status == ESP_GATT_OK && prepare_write_env->prepare_buf == NULL)
			{
				prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
				prepare_write_env->prepare_len = 0;
				if (prepare_write_env->prepare_buf == NULL)
				{
					ESP_LOGE(GATTS_TAG, "Gatt_server prep no mem");
					status = ESP_GATT_NO_RESOURCES;
				}
			}

			esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
			if (gatt_rsp)
			{
				gatt_rsp->attr_value.len = param->write.len;
				gatt_rsp->attr_value.handle = param->write.handle;
				gatt_rsp->attr_value.offset = param->write.offset;
				gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
				memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
				esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);

				if (response_err != ESP_OK)
				{
					ESP_LOGE(GATTS_TAG, "Send response error\n");
				}
				free(gatt_rsp);
			}
			else
			{
				ESP_LOGE(GATTS_TAG, "malloc failed, no resource to send response error\n");
				status = ESP_GATT_NO_RESOURCES;
			}

			if (status != ESP_GATT_OK)
			{
				return;
			}
			memcpy(prepare_write_env->prepare_buf + param->write.offset,
				   param->write.value,
				   param->write.len);
			prepare_write_env->prepare_len += param->write.len;
		}
		else
		{
			esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
		}
	}
}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
	if (param->exec_write.exec_write_flag != ESP_GATT_PREP_WRITE_EXEC)
	{
		ESP_LOGI(GATTS_TAG, "ESP_GATT_PREP_WRITE_CANCEL");
	}
	if (prepare_write_env->prepare_buf)
	{
		free(prepare_write_env->prepare_buf);
		prepare_write_env->prepare_buf = NULL;
	}
	prepare_write_env->prepare_len = 0;
}

static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
}


// Initializes the non-volatile storage (aka persistant)
esp_err_t biodyn_init_nvs()
{
	esp_err_t ret;
	// Initialize NVS (non-volatile storage)
	ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
	{
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);
	return ret;
}


// APP ENTRY POINT
void app_main(void)
{
	esp_err_t ret;

	// Initialize persistant storage (nvs)
	if (biodyn_init_nvs())
	{
		ESP_LOGE(MAIN_TAG, "Failed to initialize non-volatile storage in %s", __func__);
		return;
	}

	// Initialize bluetooth
	if (biodyn_init_ble())
	{
		ESP_LOGE(MAIN_TAG, "Failed to initialize Bluetooth in %s", __func__);
		return;
	}

	// Set up!
}
