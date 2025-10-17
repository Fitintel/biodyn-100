
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
	const char *err_msg;
} self_test_data = {not_started, biodyn_systems, LEN_OF_STATIC_ARRAY(biodyn_systems), ""};

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
	ESP_LOGI(ST_TAG, "Set self-test state called");
	self_test_state *in_state = (self_test_state *)src;
	switch (*in_state)
	{
	case running:
		self_test_start();
		return;
	case cancelled:
		// Right now self-testing is synchronous so we can't actually cancel it
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

	// Check if we've been here before
	if (self_test_data.state == completed_with_err)
		ESP_LOGW(ST_TAG, "Re-running self test which previously had error");
	else if (self_test_data.state == cancelled)
		ESP_LOGI(ST_TAG, "Re-running previously cancelled self-test");
	else if (self_test_data.state == completed_ok)
		ESP_LOGI(ST_TAG, "Re-runnning previously ok self-test");
	else if (self_test_data.state == not_started)
		ESP_LOGI(ST_TAG, "Running self test for the first time");

	self_test_data.state = running;
	self_test_data.err_msg = "";

	// Run all the tests
	for (int i = 0; i < self_test_data.n_systems; i++)
		if (self_test_data.systems[i].has_error())
		{
			const biodyn_system *sys = &self_test_data.systems[i];
			ESP_LOGE(ST_TAG, "Starting self-test for %s", sys->name);
			sys->self_test();
			// If we shat the bed
			if (sys->has_error())
			{
				const char *msg = sys->get_error();
				ESP_LOGE(ST_TAG, "System %s has error: %s", sys->name, msg);
				self_test_data.err_msg = msg;
				self_test_data.state = completed_with_err;
				return;
			}
		}

	// All good
	self_test_data.state = completed_ok;

	ESP_LOGI(ST_TAG, "All systems passed self-test");
}

// Bluetooth callback
void self_test_get_state(uint16_t *len, void *dst)
{
	uint32_t state = self_test_data.state;
	*len = sizeof(state);
	memcpy(dst, &state, *len);
	ESP_LOGI(ST_TAG, "Read state as %s", get_state_string(self_test_data.state));
}

// Bluetooth callback
void self_test_get_err_msg(uint16_t *len, void *dst)
{
	*len = strlen(self_test_data.err_msg) + 1;
	memcpy(dst, self_test_data.err_msg, *len);
	ESP_LOGI(ST_TAG, "Read error message as \"%s\"", self_test_data.err_msg);
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