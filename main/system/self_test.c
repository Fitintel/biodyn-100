
#include "system/self_test.h"
#include "esp_log.h"
#include <string.h>

// TODO: self-testing

#define ST_TAG "SELF_TEST"

typedef enum self_test_state
{
	not_started = 0,
	running = 1,
	completed_with_err = 2,
	completed_ok = 3,
} self_test_state;

static struct
{
	self_test_state state;
} self_test_data = {not_started};

esp_err_t biodyn_self_test_init()
{
	return ESP_OK;
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
	}
	return "UNKNOWN";
}

void self_test_set_state(uint16_t size, void *src)
{
	// TODO: this
}
void self_test_get_state(uint16_t *len, void *dst)
{
	uint8_t state = self_test_data.state;
	*len = sizeof(state);
	memcpy(dst, &state, *len);
	ESP_LOGI(ST_TAG, "Read state as %s", get_state_string(self_test_data.state));
}