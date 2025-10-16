
#include "led.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include <string.h>

#define LED_PIN GPIO_NUM_7

static struct
{
	bool led_error;
	const char *led_error_msg;
} led_data = {false, "Ok"};

void set_led_error(const char *msg)
{
	led_data.led_error = true;
	led_data.led_error_msg = msg;
}
void clear_led_error()
{
	led_data.led_error = false;
	led_data.led_error_msg = "Ok";
}
void set_and_log_led_error(const char *msg)
{
	set_led_error(msg);
	ESP_LOGE("LED", "Error: %s", led_data.led_error_msg);
}

esp_err_t biodyn_led_init()
{
	esp_err_t err = 0;
	if ((err = gpio_reset_pin(LED_PIN)))
	{
		set_and_log_led_error("Failed to reset pin");
		return err;
	}
	if ((err = gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT)))
	{
		set_and_log_led_error("Failed to set direction");
		return err;
	}

	ESP_LOGI("LED", "Initialized LED");
	clear_led_error();
	return 0;
}


// Bluetooth call
void led_set_state(uint16_t size, void *src)
{
	esp_err_t err = 0;
	if (size != 0)
	{
		uint8_t *s = (uint8_t *)src;
		if (s[0] & 1)
		{
			if ((err = gpio_set_level(LED_PIN, 1)))
			{
				ESP_LOGE("LED", "Failed to turn on, error code %d", err);
				set_led_error("Failed to turn on");
				return;
			}
			ESP_LOGI("LED", "Turned on");
		}
		else
		{
			if ((err = gpio_set_level(LED_PIN, 0)))
			{
				ESP_LOGE("LED", "Failed to turn off, error code %d", err);
				set_led_error("Failed to turn off");
				return;
			}
			ESP_LOGI("LED", "Turned off");
		}
	}
}

void led_get_state(uint16_t *len, void *dst)
{
	uint8_t state = gpio_get_level(LED_PIN);
	*len = sizeof(state);
	memcpy(dst, &state, *len);
	ESP_LOGI("LED", "Read state as %s", state ? "on" : "off");
}

bool led_has_error()
{
	return led_data.led_error;
}

const char *led_get_error()
{
	return led_data.led_error_msg;
}

esp_err_t led_self_test() {
	return 0;
}