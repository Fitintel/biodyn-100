#ifndef BIODYN_LED_H
#define BIODYN_LED_H

#include "esp_system.h"
#include "constants.h"

esp_err_t biodyn_led_init();

// Bluetooth callback
void led_set_state(uint16_t size, void *src);
// Bluetooth callback
void led_get_state(uint16_t *len, void *dst);

bool led_has_error();
const char *led_get_error();

// System info
const static biodyn_system biodyn_led_system = {
	.name = "LED",
	.init = biodyn_led_init,
	.get_error = led_get_error,
	.has_error = led_has_error,
};

#endif // BIODYN_LED_H