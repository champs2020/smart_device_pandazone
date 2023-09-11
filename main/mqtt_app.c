#include <string.h>
#include "esp_log.h"
#include "esp_system.h"
#include "esp_netif.h"
#include "esp_tls.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "mqtt_client.h" 

extern const uint8_t client_cert_pem_start[] asm("_binary_client_crt_start");
extern const uint8_t client_cert_pem_end[] asm("_binary_client_crt_end");
extern const uint8_t client_key_pem_start[] asm("_binary_client_key_start");
extern const uint8_t client_key_pem_end[] asm("_binary_client_key_end");
extern const uint8_t server_cert_pem_start[] asm("_binary_mosquitto_org_crt_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_mosquitto_org_crt_end");

static int8_t IS_MQTT_CONNECTED = 0;

static const char *TAG = "mesh_mqtt";

static esp_mqtt_client_handle_t s_client = NULL;

// Tópic para acessar todos os dispositivos da edificação
static const char building_topic_credential[35] = "ufcg/cg_sede/caa";
int len_building_topic_credential;
int is_ok_send_credencials = 1;
char strs[50] = "";

void send_credencial_area(char *topic, char *valor, int *len_data);


int is_mqtt_connected()
{
    return IS_MQTT_CONNECTED;
}


static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            if (esp_mqtt_client_subscribe(s_client, building_topic_credential, 0) == ESP_FAIL)
            {
                esp_mqtt_client_disconnect(s_client);
            }  
            
            if (esp_mqtt_client_subscribe(s_client, "+/caa/#", 1) > 0)
            {
                 ESP_LOGI(TAG, "READY TO COMAND A/C");
            }
            
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            ESP_LOGI(TAG, "TOPIC=%.*s, topic_len: %d", event->topic_len, event->topic, event->topic_len);
            ESP_LOGI(TAG, "DATA=%.*s", event->data_len, event->data);

            ESP_LOGW("MQTT","event->data_len: %d, Enviando o código de área (content): %.*s", strlen(event->data), event->data_len, event->data);
            int array_len[2] = {event->topic_len, event->data_len};
            send_credencial_area(event->topic, event->data, array_len);
            printf("\nGrad data\n");
            

            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    mqtt_event_handler_cb(event_data);
}

void mqtt_app_publish(char* topic, char *publish_string)
{
    if (s_client) {
        int msg_id = esp_mqtt_client_publish(s_client, topic, publish_string, 0, 1, 0);
        ESP_LOGI(TAG, "sent publish returned msg_id=%d", msg_id);
    }
}

// *Retorno: se maior que 0, sucesso
int mqtt_app_subscribe(char* topic, int qos)
{
    int msg_id = esp_mqtt_client_subscribe(s_client, topic, qos);
    ESP_LOGI(TAG, "subscribe returned msg_id=%d", msg_id);

    return msg_id;
}

int mqtt_app_start(void)
{
    int retorno;
    const esp_mqtt_client_config_t mqtt_cfg = {
        .uri = "mqtts://apx41ilnb808f-ats.iot.us-east-1.amazonaws.com:8883",
        .client_cert_pem = (const char *)client_cert_pem_start,
        .client_key_pem = (const char *)client_key_pem_start,
        .cert_pem = (const char *)server_cert_pem_start,
    };

    s_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID, mqtt_event_handler, s_client);
    retorno = esp_mqtt_client_start(s_client);

    return retorno;
}
