#ifndef FITNET_BLE_H
#define FITNET_BLE_H

#include "esp_system.h"
#include "esp_err.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"

struct biodyn_ble_profile;
struct biodyn_ble_service;
struct biodyn_ble_characteristic;
struct biodyn_ble_char_descriptor;

// Initializes the BIODYN BLE GATTS server and GAP protocols
esp_err_t biodyn_ble_init(uint16_t n_profiles, struct biodyn_ble_profile *profiles);

// BIODYN BLE Driver error code type: BIODYN_BLE_ERR_
typedef uint16_t biodyn_ble_err_t;

// BIODYN BLE error types
#define BIODYN_BLE_ERR_ADV_DATA_INIT 0x1		 // Failed to set the advertisement data
#define BIODYN_BLE_ERR_CANT_NAME 0x2			 // Failed to name device
#define BIODYN_BLE_ERR_CANT_CREATE_SERVICE 0x4	 // Failed to create service
#define BIODYN_BLE_ERR_CANT_FIND_SERVICE 0x8	 // Can't find a service from ID
#define BIODYN_BLE_ERR_CANT_START_SERVICE 0x10	 // Can't start a service
#define BIODYN_BLE_ERR_CHARS_MISCONFIGURED 0x20	 // Characteristics misconfigured
#define BIODYN_BLE_ERR_CANT_CREATE_CHAR 0x40	 // Couldn't create characteristic
#define BIODYN_BLE_ERR_CANT_FIND_CHAR 0x80		 // Couldn't find characteristic
#define BIODYN_BLE_ERR_CANT_CREATE_DESCR 0x100	 // Couldn't create a descriptor
#define BIODYN_BLE_ERR_CANT_ADVERTISE 0x200		 // Couldn't start advertising
#define BIODYN_BLE_ERR_CANT_STOP_ADVERTISE 0x400 // Couldn't stop advertising

// Returns any accumulated errors in the BIODYN BLE driver
biodyn_ble_err_t biodyn_ble_get_err();

// A BIODYN BLE profile schematic
struct biodyn_ble_profile
{
	const char *name;					 // The name of this profile
	uint16_t n_services;				 // The number of services for this profile
	struct biodyn_ble_service *services; // The list of services for this profile

	// TODO: Add callback when profile is completely set up
};

// A BIODYN BLE service schematic
struct biodyn_ble_service
{
	const char *name;								   // The name of this service
	esp_gatt_srvc_id_t service_id;					   // The GATTS service ID
	uint16_t n_characteristics;						   // The number of characteristics this service has
	struct biodyn_ble_characteristic *characteristics; // The list of characteristics

	// TODO: Add callback when service is completely set up
};

// A BIODYN BLE characteristic schematic
struct biodyn_ble_characteristic
{
	const char *name;	// The name for the characteristic - REQUIRED
	esp_bt_uuid_t uuid; // The UUID for the characteristic - REQUIRED

	// The associated permissions. What is required for the properties to be accessed.
	// The actual GATTS permissions. We will fail accessing the attribute if proper perms not set.
	// Ex. normal read, requires encryption, requires authentication
	// REQUIRED
	esp_gatt_perm_t permissions;

	// Characteristic properties: what can be done with this characteristic.
	// This is what we tell the client, not what our internal GATTS permissions are.
	// Ex. this characteristic can be read, written to, notify the client, etc
	// REQUIRED
	esp_gatt_char_prop_t properties;

	uint16_t n_descriptors;							// The number of descriptors on this characteristic
	struct biodyn_ble_char_descriptor *descriptors; // Descriptors for this characteristic

	// The value this characteristic is initialized with
	// If a characteristic has an initial value, reading/writing will be automatically handled
	// by the GATTS.
	void *initial_value;
	uint16_t intial_value_size; // The size of this initial value in bytes

	// TODO: Add callback when characteristic is completely set up
};

// A BIODYN BLE characteristic descriptor schematic
struct biodyn_ble_char_descriptor
{
	esp_bt_uuid_t uuid;			 // The UUID for the descriptor
	esp_gatt_perm_t permissions; // The permissions given for this descriptor (ie. read, write)
	esp_attr_value_t value;		 // The value associated with this descriptor
};

// Helper functions for setting up BLE schematics
#define BIODYN_BLE_UUID_16(uuid_value) { .len = ESP_UUID_LEN_16, .uuid = { .uuid16 = uuid_value } }
#define BIODYN_BLE_SERVICE_ID_16(uuid_value) { .id = { BIODYN_BLE_UUID_16(uuid_value), .inst_id = 0}, .is_primary = true }

#endif // FITNET_BLE_H