#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"

#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "mqtt_app.h"

esp_netif_t *netif_sta = NULL;
static const char *MESH_TAG = "mesh_main";

SemaphoreHandle_t semaphoreWifi;

#define WIFI_CONNECTED_BIT  BIT1
#define WIFI_FAIL_BIT       BIT0
extern EventGroupHandle_t wifi_event_group;

//static int s_retry_num = 0;
void got(){
    ESP_LOGE("GOT 0", "");
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    xSemaphoreGive(semaphoreWifi);
    xSemaphoreTake(semaphoreWifi, portMAX_DELAY);
    ESP_LOGE("GOT 1", "");
    } 

void ip_event_handler(void *arg, esp_event_base_t event_base,
                      int32_t event_id, void *event_data)
{
    ESP_LOGI("CONNECT IP STATUS", ": %d \n", event_id);
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    
    ESP_LOGI(MESH_TAG, "<IP_EVENT_STA_GOT_IP>IP:" IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI("CONNECT IP", "Got ip, then set mqtt! \n");
        xSemaphoreGive(semaphoreWifi);
}

void wifi_app_start()
{
    semaphoreWifi = xSemaphoreCreateBinary();

    ESP_LOGE("WIFI", "INITING ...");
    esp_err_t ret = nvs_flash_init();
    if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /*  tcpip initialization */
    ESP_ERROR_CHECK(esp_netif_init());

    /*  event initialization */
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    /*  create network interfaces for mesh (only station instance saved for further manipulation, soft AP instance ignored */
    ESP_ERROR_CHECK(esp_netif_create_default_wifi_mesh_netifs(&netif_sta, NULL));

/*  wifi initialization */
    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&config));
    /*  register IP events handler */
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL));
    //ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &ip_event_handler, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));

    ESP_ERROR_CHECK(esp_wifi_start());
    got();
    //vTaskDelay(12000 / portTICK_PERIOD_MS);
    xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
}