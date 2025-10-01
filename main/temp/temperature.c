#include "esp_system.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "driver/gpio.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "math.h"
#define READ_BUFFER_SIZE     1024
#define SAMPLE_FREQ_HZ       (10 * 100 * 1000)
#define TEMPERATURE_PIN      GPIO_NUM_2
#define TEMPERATURE_CHANNEL  ADC_CHANNEL_0
#define TEMPERATURE_UNIT     ADC_UNIT_1

#define VCC_CHANNEL  ADC_CHANNEL_3  // for ref voltage


#define TAG 				"TEMPERATURE"

static adc_continuous_handle_t adc_handle = NULL;
static adc_cali_handle_t adc_cali_handle = NULL;

int read_vcc_mv();


esp_err_t biodyn_temperature_init()
{
    esp_err_t err;

    err = gpio_reset_pin(TEMPERATURE_PIN);
    if (err != ESP_OK) return err;

    err = gpio_set_direction(TEMPERATURE_PIN, GPIO_MODE_INPUT);
    if (err != ESP_OK) return err;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = READ_BUFFER_SIZE,
        .conv_frame_size = READ_BUFFER_SIZE,
    };

    err = adc_continuous_new_handle(&adc_config, &adc_handle);
    if (err != ESP_OK) return err;

//    adc_digi_pattern_config_t pattern = {
//        .atten = ADC_ATTEN_DB_12,
//        .channel = TEMPERATURE_CHANNEL,
//        .unit = 0,
//        .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
//    };
//
//    adc_continuous_config_t dig_cfg = {
//        .sample_freq_hz = SAMPLE_FREQ_HZ,
//        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
//        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
//        .pattern_num = 1,
//        .adc_pattern = &pattern,
//    };


    adc_digi_pattern_config_t patterns[2] = {
        {
            .atten = ADC_ATTEN_DB_12,
            .channel = TEMPERATURE_CHANNEL,
            .unit = 0,
            .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
        },
        {
            .atten = ADC_ATTEN_DB_12,
            .channel = VCC_CHANNEL,
            .unit = 0,
            .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
        }
    };

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = SAMPLE_FREQ_HZ,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
        .pattern_num = 2,
        .adc_pattern = patterns,
    };



    err = adc_continuous_config(adc_handle, &dig_cfg);
    if (err != ESP_OK) return err;

    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = TEMPERATURE_UNIT,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = SOC_ADC_DIGI_MAX_BITWIDTH,
    };
    adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle);

    err = adc_continuous_start(adc_handle);
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "Initialized continuous ADC on GPIO2");
    return ESP_OK;
}

int biodyn_temperature_read_voltage_mv()
{
    uint8_t result[64];
    uint32_t bytes_read = 0;

    esp_err_t err = adc_continuous_read(adc_handle, result, sizeof(result), &bytes_read, 0);
    if (err != ESP_OK || bytes_read == 0) {
        return -1;
    }

    int raw = -1;
    for (int i = 0; i < bytes_read; i += sizeof(adc_digi_output_data_t)) {
        adc_digi_output_data_t *sample = (adc_digi_output_data_t *)&result[i];

        if (sample->type2.channel == TEMPERATURE_CHANNEL) {
            raw = sample->type2.data;
            break;
        }
    }

    if (raw < 0) return -1;

    if (adc_cali_handle) {
        int voltage = 0;
        if (adc_cali_raw_to_voltage(adc_cali_handle, raw, &voltage) == ESP_OK) {
            return voltage;
        }
    }

    ESP_LOGI(TAG, "Resorting to default");
    return (raw * 3300) / 4095;
}

float biodyn_get_temperature_celsius_debug()
{
    int voltage_mv = biodyn_temperature_read_voltage_mv();
    if (voltage_mv <= 0) return -273.15;

    float Vcc = read_vcc_mv();
    const float R_fixed = 10000.0;
    const float Beta = 3969.0;
    const float R0 = 10000.0;
    const float T0 = 298.15;

    float Vout = (float)voltage_mv;
    float R_thermistor = R_fixed * (Vcc / Vout - 1.0);
//    float R_thermistor = (Vcc) * (R_fixed / Vout) - R_fixed;
    if (R_thermistor <= 0) return -273.15;

    float lnR = logf(R_thermistor / R0);
    float inv_T = (1.0 / T0) + (lnR / Beta);
    float T = 1.0 / inv_T;
    float tempC = T - 273.15;

    ESP_LOGI(TAG, "Vin = %.2f mV, Vout = %.2f mV, R_thermistor = %.2f ohm, Temp = %.2f°C", Vcc, Vout, R_thermistor, tempC);

    return tempC;
}



float biodyn_get_temperature_celsius()
{
    int voltage_mv = biodyn_temperature_read_voltage_mv();
    if (voltage_mv <= 0) return -273.15;

    const float Vcc = read_vcc_mv();
    const float R_fixed = 10000.0;
    const float Beta = 3969.0;
    const float R0 = 10000.0;
    const float T0 = 298.15;


    float Vout = (float)voltage_mv;
    float R_thermistor = R_fixed * (Vcc / Vout - 1.0);

    if (R_thermistor <= 0) return -273.15;

    float lnR = logf(R_thermistor / R0);
    float inv_T = (1.0 / T0) + (lnR / Beta);
    float T = 1.0 / inv_T;

    return T - 273.15;
}



int read_vcc_mv()
{
    int raw = -1;
    uint8_t result[64];
    uint32_t bytes_read = 0;

    esp_err_t err = adc_continuous_read(adc_handle, result, sizeof(result), &bytes_read, 0);
    if (err != ESP_OK || bytes_read == 0) return -1;

    for (int i = 0; i < bytes_read; i += sizeof(adc_digi_output_data_t)) {
        adc_digi_output_data_t *sample = (adc_digi_output_data_t *)&result[i];
        if (sample->type2.channel == VCC_CHANNEL) {
            raw = sample->type2.data;
            break;
        }
    }

    if (raw < 0) return -1;

    int mv = 0;
    if (adc_cali_handle &&
        adc_cali_raw_to_voltage(adc_cali_handle, raw, &mv) == ESP_OK) {
        return mv * 2;
    }

//    return ((raw * 3300) / 4095) * 2;
    return -1;
}


