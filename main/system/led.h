#ifndef BIODYN_LED_H
#define BIODYN_LED_H

#include "esp_system.h"

esp_err_t biodyn_led_init();

void led_set_state(uint16_t size, void *src);
void led_get_state(uint16_t *len, void *dst);

#endif // BIODYN_LED_H