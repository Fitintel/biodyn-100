#include "emg/emg_driver.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "driver/gpio.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define TAG "EMG"

static struct
{
	biodyn_emg_err_t err;
	char err_msg[128];
	gpio_num_t emg_pin;
} emg_data = {
	.err = BIODYN_EMG_OK,
	.err_msg = "No error",
	.emg_pin = GPIO_NUM_4,
};

static esp_err_t collect_err(biodyn_emg_err_t err, const char *msg, esp_err_t code);

esp_err_t biodyn_emg_init()
{
	esp_err_t err = ESP_OK;

	if ((err = gpio_reset_pin(emg_data.emg_pin)) != ESP_OK)
		return collect_err(BIODYN_EMG_CANT_RESET_PIN, "Failed to reset EMG GPIO pin", err);

	if ((err = gpio_set_direction(emg_data.emg_pin, GPIO_MODE_INPUT)) != ESP_OK)
		return collect_err(BIODYN_EMG_CANT_SET_INPUT, "Failed to set EMG GPIO pin as input", err);

	// TODO: setup ADC for EMG reading

	return ESP_OK;
}

esp_err_t biodyn_emg_self_test()
{
	// TODO: self-test
	return ESP_OK;
}

bool biodyn_emg_has_error() { return emg_data.err != BIODYN_EMG_OK; }
const char *biodyn_emg_get_error() { return emg_data.err_msg; }

void emg_read()
{
	// TODO: read ADC
}

static esp_err_t collect_err(biodyn_emg_err_t err, const char *msg, esp_err_t code)
{
	emg_data.err |= err;
	snprintf(emg_data.err_msg, sizeof(emg_data.err_msg), "%s %x", msg, code);
	ESP_LOGE(TAG, "%s %x", msg, code);
	return code;
}