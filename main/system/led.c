
#include "led.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include <string.h>


#define LED_PIN GPIO_NUM_7

esp_err_t biodyn_led_init()
{
	esp_err_t err = 0;
	if ((err = gpio_reset_pin(LED_PIN)))
	{
		ESP_LOGE("LED", "Failed to reset pin");
		return err;
	}
	if ((err = gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT)))
	{
		ESP_LOGE("LED", "Failed to set direction.");
		return err;
	}

	ESP_LOGI("LED", "Initialized LED");
	return 0;
}

void led_set_state(uint16_t size, void *src)
{
	if (size != 0) {
		uint8_t *s = (uint8_t *) src;
		if (s[0] & 1) {
			gpio_set_level(LED_PIN, 1);
			ESP_LOGI("LED", "Turned on");
		} else {
			gpio_set_level(LED_PIN, 0);
			ESP_LOGI("LED", "Turned off");
		}
	}
}

void led_get_state(uint16_t *len, void *dst)
{
	uint8_t state = gpio_get_level(LED_PIN);
	memcpy(dst, &state, *len);
	ESP_LOGI("LED", "Read state as %s", state ? "on" : "off");
}

