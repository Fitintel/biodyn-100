#ifndef BIODYN_TEMPERATURE_H
#define BIODYN_TEMPERATURE_H

#include "esp_system.h"


esp_err_t biodyn_temperature_init();

int biodyn_temperature_read_voltage_mv();

#endif // BIODYN_TEMPERATURE_H
