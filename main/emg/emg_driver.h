#ifndef BIODYN_EMG_DRIVER_H
#define BIODYN_EMG_DRIVER_H

#include "esp_system.h"
#include "constants.h"

esp_err_t biodyn_emg_init();
esp_err_t biodyn_emg_self_test();
bool biodyn_emg_has_error();
const char *biodyn_emg_get_error();
void emg_read();

const static biodyn_system biodyn_emg_system = {
	.name = "EMG",
	.init = biodyn_emg_init,
	.has_error = biodyn_emg_has_error,
	.get_error = biodyn_emg_get_error,
	.self_test = biodyn_emg_self_test,
};

typedef uint32_t biodyn_emg_err_t;
#define BIODYN_EMG_OK 0x0
#define BIODYN_EMG_COULDNT_CREATE_TASK 0x1
#define BIODYN_EMG_TOO_MUCH_DATA 0x2
#define BIODYN_EMG_COULDNT_CREATE_MUTEX 0x4
#define BIODYN_EMG_RUNNING_TOO_SLOW 0x8
#define BIODYN_EMG_CANT_RESET_PIN 0x10
#define BIODYN_EMG_CANT_SET_INPUT 0x20


#endif // BIODYN_EMG_DRIVER_H