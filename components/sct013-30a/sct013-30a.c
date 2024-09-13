#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

const static char *TAG = "EXAMPLE";

/*---------------------------------------------------------------
        ADC General Macros
---------------------------------------------------------------*/
#define MCU_TARGET_ESP32 1
#define MCU_TARGET_ESP32S3 3

//ADC1 Channels
#if CONFIG_IDF_TARGET_ESP32
#define EXAMPLE_ADC1_CHAN0          ADC_CHANNEL_4
#define EXAMPLE_ADC1_CHAN1          ADC_CHANNEL_5
#define MCU_TARGET                ADC_MCU_TYPE_ESP32
#elif CONFIG_IDF_TARGET_ESP32S3
#define EXAMPLE_ADC1_CHAN0          ADC_CHANNEL_2 // IO4 // Sentido: esquerda -> direita
#define EXAMPLE_ADC1_CHAN1          ADC_CHANNEL_7 // IO3
#define EXAMPLE_ADC1_CHAN2          ADC_CHANNEL_8 // IO9
#define EXAMPLE_ADC1_CHAN3          ADC_CHANNEL_9 // IO10
#define EXAMPLE_ADC_ATTEN           ADC_ATTEN_DB_12
#define MCU_TARGET                  MCU_TARGET_ESP32S3
#else
#define EXAMPLE_ADC1_CHAN0          ADC_CHANNEL_0
#define EXAMPLE_ADC1_CHAN1          ADC_CHANNEL_1
#define EXAMPLE_ADC1_CHAN2          ADC_CHANNEL_2
#define EXAMPLE_ADC1_CHAN3          ADC_CHANNEL_3
#define EXAMPLE_ADC_ATTEN           ADC_ATTEN_DB_12
#define MCU_TARGET                MCU_TARGET_ESP32S3
#endif

#if (SOC_ADC_PERIPH_NUM >= 2) && !CONFIG_IDF_TARGET_ESP32C3
/**
 * On ESP32C3, ADC2 is no longer supported, due to its HW limitation.
 * Search for errata on espressif website for more details.
 */
#define EXAMPLE_USE_ADC2            1
#endif

#if EXAMPLE_USE_ADC2
//ADC2 Channels
#if CONFIG_IDF_TARGET_ESP32
#define EXAMPLE_ADC2_CHAN0          ADC_CHANNEL_0
#else
#define EXAMPLE_ADC2_CHAN0          ADC_CHANNEL_0
#endif
#endif  //#if EXAMPLE_USE_ADC2



static int adc_raw[10];
static int voltage[10];
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);
////////////
static bool do_calibration1_chan0 = 0;
static bool do_calibration1_chan1 = 0;
static bool do_calibration1_chan2 = 0;
static bool do_calibration1_chan3 = 0;

adc_oneshot_unit_handle_t adc1_handle;
adc_oneshot_unit_init_cfg_t init_config1 = {
    .unit_id = ADC_UNIT_1,
};

adc_cali_handle_t adc1_cali_chan0_handle = NULL;
adc_cali_handle_t adc1_cali_chan1_handle = NULL;
adc_cali_handle_t adc1_cali_chan2_handle = NULL;
adc_cali_handle_t adc1_cali_chan3_handle = NULL;


///////////

static int channels[4] = {EXAMPLE_ADC1_CHAN0, EXAMPLE_ADC1_CHAN1, EXAMPLE_ADC1_CHAN2, EXAMPLE_ADC1_CHAN3};
static int picoPico;
static int value[3];
int vale[4] = {4096,4096,4096}; // O valor inicial de comparação 4096 é p/ achar achar o Menor valor
int crista[4] = {0,0,0}; // O valor inicial de comparação 0 é p/ achar achar o Maior valor
float pico, rms, potencia;
float current;
float currentes[4] = {1,2,3,4};

/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void example_adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}




void init_SCT013()
{
    //-------------ADC1 Init---------------//
    
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = EXAMPLE_ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN0, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN1, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN2, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN3, &config));

    //-------------ADC1 Calibration Init---------------//
    do_calibration1_chan0 = example_adc_calibration_init(ADC_UNIT_1, EXAMPLE_ADC1_CHAN0, EXAMPLE_ADC_ATTEN, &adc1_cali_chan0_handle);
    do_calibration1_chan1 = example_adc_calibration_init(ADC_UNIT_1, EXAMPLE_ADC1_CHAN1, EXAMPLE_ADC_ATTEN, &adc1_cali_chan1_handle);
    do_calibration1_chan2 = example_adc_calibration_init(ADC_UNIT_1, EXAMPLE_ADC1_CHAN2, EXAMPLE_ADC_ATTEN, &adc1_cali_chan2_handle);
    do_calibration1_chan3 = example_adc_calibration_init(ADC_UNIT_1, EXAMPLE_ADC1_CHAN3, EXAMPLE_ADC_ATTEN, &adc1_cali_chan3_handle);
    
}

float *get_SCT023_current()
{
   
    printf("============================================================\n");
        
        for (int j = 0 ; j < 4; j++)
        {
            for (int i = 0; i < 360; i++)
            {
                adc_oneshot_read(adc1_handle, channels[j], &adc_raw[j]);
            
                switch (channels[j])
                {
                case EXAMPLE_ADC1_CHAN0:
                    adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw[j], &value[j]);
                    break;
                case EXAMPLE_ADC1_CHAN1:
                    adc_cali_raw_to_voltage(adc1_cali_chan1_handle, adc_raw[j], &value[j]);
                    break;
                case EXAMPLE_ADC1_CHAN2:
                    adc_cali_raw_to_voltage(adc1_cali_chan2_handle, adc_raw[j], &value[j]);
                    break;
                case EXAMPLE_ADC1_CHAN3:
                    adc_cali_raw_to_voltage(adc1_cali_chan3_handle, adc_raw[j], &value[j]);
                    break;
                
                default:
                    break;
                }
                
                
                if (value[j] > crista[j])
                {
                    crista[j] = value[j];
                }
                if (value[j] < vale[j])
                {
                    vale[j] = value[j];
                }
                
            }  
            ESP_LOGI("[CALIBRATED]", "vale[%d]: %d, crista[%d]: %d--", j, vale[j], j, crista[j]);
            
            picoPico = crista[j] - vale[j];
            pico = picoPico / 2;
            if(picoPico < 20) pico = 0;
            rms = pico / 1.41421356; // mA
            current = 2*(rms / 36); // corrente real medida (Ampere) | R = 36 ohm
            potencia = current * 220;

            ESP_LOGI("[CALIBRATED]", "vale[%d]: %d, crista[%d]: %d, Pico a Pico: %d mV, Pico: %.2f mV, RMS: %.2f mV, corrente (mA): %.3f, potencia: %f \n", j,
            vale[j], j, crista[j], picoPico, pico, rms, current, potencia);
        
        currentes[j] = current;
        crista[j] = 0, vale[j] = 4096;
        }
    return currentes;
}

