#include <stdio.h>
#include "sct013-30a.h"

#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

// ADC attenutation
#define ADC_ATTEN ADC_ATTEN_DB_11

//ADC Channels
#define ADC1_CHAN4          ADC1_CHANNEL_4 // Plugado
#define ADC1_CHAN7          ADC1_CHANNEL_7 // Plugado
#define ADC1_CHAN6          ADC1_CHANNEL_6 //Em uso

// #define ADC1_CHAN3          ADC1_CHANNEL_3 // Plugado
// #define ADC1_CHAN4          ADC1_CHANNEL_4 // Plugado
// #define ADC1_CHAN5          ADC1_CHANNEL_5 //Em uso


//ADC Calibration
#if CONFIG_IDF_TARGET_ESP32
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_VREF
#elif CONFIG_IDF_TARGET_ESP32S3
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP_FIT
#endif

static const char *TAG = "ADC SINGLE";
static bool cali_enable = false;
static esp_adc_cal_characteristics_t adc1_chars;

static int adc_raw[3], aux_adc, picoPico;
int vale[3] = {4096,4096,4096}; // O valor inicial de comparação 4096 é p/ achar achar o Menor valor
int crista[3] = {0,0,0}; // O valor inicial de comparação 0 é p/ achar achar o Maior valor
float pico, rms, potencia;
//float corrente[3];
static bool adc_calibration_init(void)
{
    esp_err_t ret;
    bool cali_enable = false;

    ret = esp_adc_cal_check_efuse(ADC_EXAMPLE_CALI_SCHEME);
    if (ret == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "Calibration scheme not supported, skip software calibration");
    } else if (ret == ESP_ERR_INVALID_VERSION) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else if (ret == ESP_OK) {
        cali_enable = true;
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH_BIT_12, 0, &adc1_chars);
    } else {
        ESP_LOGE(TAG, "Invalid arg");
    }

    return cali_enable;
}

void init_SCT013()
{
    bool cali_enable = adc_calibration_init();

    if (cali_enable) printf("\ninit_SCT013: CALIBRADO!\n");
    
    //ADC1 config
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_12));

    // ESP32S3
    // ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHAN3, ADC_ATTEN));
    // ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHAN4, ADC_ATTEN));
    // ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHAN5, ADC_ATTEN));

    // ESP32
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHAN4, ADC_ATTEN));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHAN6, ADC_ATTEN));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHAN7, ADC_ATTEN));
}

void get_SCT023_current(float *corrente)
{
    printf("============================================================i");
        
        aux_adc = 4;
        for (int j = 0 /*0*/; j < 3; j++)
        {
            if (j==1) aux_adc++;
            
            for (int i = 0; i < 600; i++)
            {
                adc_raw[j] = adc1_get_raw(aux_adc+j); // ESP32 (aux_adc+j) ADC1_CHAN4, ADC1_CHAN6 e ADC1_CHAN7 | ESP32-S3 (j+3) ADC1_CHAN3, ADC1_CHAN4 e ADC1_CHAN5

                if (adc_raw[j] > crista[j])
                {
                    crista[j] = adc_raw[j];
                }
                
                if (adc_raw[j] < vale[j])
                {
                    vale[j] = adc_raw[j];
                }
            }   
        }

        printf("\n");


        // Reinicia os valores de vale e crista
        for (int i = 0 /*0*/; i < 3; i++)
        {
            vale[i] = esp_adc_cal_raw_to_voltage(vale[i], &adc1_chars);
            crista[i] = esp_adc_cal_raw_to_voltage(crista[i], &adc1_chars);
            picoPico = crista[i] - vale[i];
            pico = picoPico / 2;
            if(picoPico <= 10) pico = 0;
            rms = pico / 1.41421356;
            corrente[i] = rms / 110;
            potencia = corrente[i] * 220;

            ESP_LOGI("[CALIBRATED]", "vale[%d]: %d, crista[%d]: %d, Pico a Pico: %d mV, Pico: %.2f mV, RMS: %.2f mV, Corrente %.2f mA, Potência: %.2f W \n--", i, vale[i], i, crista[i], picoPico, pico, rms, corrente[i] *1000, potencia);

            crista[i] = 0;
            vale[i] = 4096;
        }
}

