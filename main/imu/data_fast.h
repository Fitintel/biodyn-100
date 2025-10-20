#ifndef BIODYN_DATA_FAST_H
#define BIODYN_DATA_FAST_H

#include "constants.h"

void data_fast_read();
void ble_data_fast_packed_imu(uint16_t *size, void *out);

#endif	// BIODYN_DATA_FAST_H