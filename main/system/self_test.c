
#include "system/self_test.h"
#include "esp_log.h"
#include <string.h>
#include "biodyn_systems.h"
#include "system/led.h"

// TODO: self-testing

#define ST_TAG "SELF_TEST"


/* ----------------------------
		Data definitions
   ---------------------------- */

typedef enum self_test_state
{
	not_started = 0,
	running = 1,
	completed_with_err = 2,
	completed_ok = 3,
	cancelled = 4,
} self_test_state;

static struct
{
	self_test_state state;
	const biodyn_system *systems;	
	int n_systems;
} self_test_data = {not_started, biodyn_systems, LEN_OF_STATIC_ARRAY(biodyn_systems)};

/* ----------------------------
		Function declarations 
   ---------------------------- */

const char *get_state_string(self_test_state s);
void self_test_start();

/* ----------------------------
		Function definitions 
   ---------------------------- */


// Add relevant systems to test
esp_err_t biodyn_self_test_init()
{
	return ESP_OK;
}

// Bluetooth callback
void self_test_set_state(uint16_t size, void *src)
{
	self_test_state *in_state = (self_test_state *)src;
	switch (*in_state)
	{
		case running:
			self_test_start();
			return;
		case cancelled:
		 	// TODO: this
			ESP_LOGW(ST_TAG, "Not implemented: cancel self-test");
			return;
		default:
			ESP_LOGW(ST_TAG, "Tried to write a self-test value of \"%s\"", get_state_string(*in_state));
	}
}

// Tries to start self test if it can
void self_test_start()
{
	// If we're already running don't run again
	if (self_test_data.state == running)
	{
		ESP_LOGW(ST_TAG, "Tried to start already-running self-test");
		return;
	}

	if (self_test_data.state == completed_with_err)
		ESP_LOGW(ST_TAG, "Re-running self test which previously had error");
	else if (self_test_data.state == cancelled)
		ESP_LOGI(ST_TAG, "Re-running previously cancelled self-test");
	else if (self_test_data.state == completed_ok)
		ESP_LOGI(ST_TAG, "Re-runnning previously ok self-test");
	else if (self_test_data.state == not_started)
		ESP_LOGI(ST_TAG, "Running self test for the first time");

	// TODO: Finish impl
}

// Bluetooth callback
void self_test_get_state(uint16_t *len, void *dst)
{
	uint8_t state = self_test_data.state;
	*len = sizeof(state);
	memcpy(dst, &state, *len);
	ESP_LOGI(ST_TAG, "Read state as %s", get_state_string(self_test_data.state));
}

const char *get_state_string(self_test_state s)
{
	switch (s)
	{
	case not_started:
		return "Not started";
	case running:
		return "Running";
	case completed_with_err:
		return "Completed with errors";
	case completed_ok:
		return "Completed ok";
	case cancelled:
		return "Cancelled";
	}
	return "UNKNOWN";
}