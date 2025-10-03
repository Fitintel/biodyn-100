#ifndef BIODYN_SELF_TEST_H
#define BIODYN_SELF_TEST_H

#include "esp_system.h"

esp_err_t biodyn_self_test_init();

// Bluetooth callback
void self_test_set_state(uint16_t size, void *src);
// Bluetooth callback
void self_test_get_state(uint16_t *len, void *dst);

// Bluetooth callback
void self_test_get_err_msg(uint16_t *len, void *dst);

#endif // BIODYN_SELF_TEST_H