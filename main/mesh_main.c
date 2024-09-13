/* Mesh Internal Communication Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <inttypes.h>
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mesh.h"
#include "esp_mesh_internal.h"
#include "nvs_flash.h"

#include <driver/gpio.h>
#include "mqtt_app.h"

#include "esp32_rmt_midea_ir_tx.h"
#include "midea_ir.h"
#include "dht22.h"
#include "sct013-30a.h"
/*******************************************************
 *                Macros
 *******************************************************/
#define RX_SIZE          (1024)
#define TX_SIZE          (1024)

#define WIFI_SSID "brisa-3287883"
#define WIFI_PASS  "2hs5t788"

// #define WIFI_SSID "brisa-3287883"
// #define WIFI_PASS  "2hs5t788"

#if CONFIG_IDF_TARGET_ESP32
#define LED_BULTIN 2
#define GPIO_DHT22 33
#define GPIO_RCWL 10
#elif CONFIG_IDF_TARGET_ESP32S3
#define LED_BULTIN 2
#define GPIO_RCWL 10
#define GPIO_DHT22 7
#elif CONFIG_IDF_TARGET_ESP32C3
#define GPIO_DHT22 8
#define LED_BULTIN 7
#define GPIO_RCWL 6
#endif

/*******************************************************
 *                Constants
 *******************************************************/

const double C1 = -42.379;
const double C2 = 2.04901523;
const double C3 = 10.14333127;
const double C4 = -0.22475541;
const double C5 = -0.00683783;
const double C6 = -0.05481717;
const double C7 = 0.00122874;
const double C8 = 0.00085282;
const double C9 = -0.00000199;

/*******************************************************
 *                Variable Definitions
 *******************************************************/
static const char *MESH_TAG = "mesh_main";
static const uint8_t MESH_ID[6] = { 0x84, 0xfc, 0xe6, 0x00, 0xe6, 0x6c};
static char rx_buf[RX_SIZE];
static bool is_running = true;
static bool is_mesh_connected = false;
static mesh_addr_t mesh_parent_addr;
static int mesh_layer = -1;
static esp_netif_t *netif_sta = NULL;

// ip_event_handler
static esp_ip4_addr_t current_ip;
mesh_addr_t route_table[CONFIG_MESH_ROUTE_TABLE_SIZE];
int route_table_size = 0;

static char format_msg_mqtt[100] = ""; // Ex: ufcg/cg_sede/caa/{area}/sensor/temperatura
static char  building_area_credencial[35] = "";
static char any_string[50] = "";
static int size_data_mqtt = 0;
static char str_mac_base[13] = "";
unsigned char mac_base[6];


// xQueueHandle fila_comando_komeco;
static u_int8_t air_on_off = 0; // estado


/*******************************************************
 *                Function Declarations
 *******************************************************/

void init_sensors_tasks();
void SCT013_task(void *pvParameter);
void presence_rcwl();
float heat_index (float T, float RH );
void DHT_task(void *pvParameter);
void send_data_mqtt(char *dispTipo, char *dispNome, char *dispValor, char *area);
void mqtt_subscribe_via_root(char *topic);
void request_credencial_via_root(char *topic);
void request_credencial_direct_to_server(char* mqtt_topic);
void set_root_credencial(int lenght_topic, int lenght_data, char *topic, char *data);
void process_income_command(int lenght_topic, int lenght_data, char *topic, char *data);
void send_credencial_area(char *topic, char *data, int *len_array);
void esp_mesh_p2p_rx_main(void *arg);
esp_err_t esp_mesh_comm_p2p_start(void);
void route_table_available();
void init_blink_command();
void blink_command(int n_pisks);
void check_connection(void *pvParameter);
void mesh_event_handler(void *arg, esp_event_base_t event_base,
                        int32_t event_id, void *event_data);
void ip_event_handler(void *arg, esp_event_base_t event_base,
                      int32_t event_id, void *event_data);

/*******************************************************
 *                Function Definitions
 *******************************************************/

void init_sensors_tasks()
{
    ESP_LOGI("MESH/init_sensors_tasks", "init_sensors_tasks");
    // fila_comando_komeco = xQueueCreate(2, sizeof(int));

    // xTaskCreate(&SCT013_task, "SCT013", 6144, NULL, 5, NULL);
    xTaskCreate(DHT_task, "ReadTemperature&Umidity", 2048, NULL, 6, NULL);
    xTaskCreate(&presence_rcwl, "ReadPresence", 3072, NULL, 7, NULL);
    // xTaskCreate(&air_control, "ac_control", 4096, NULL, 2, NULL);
}

void presence_rcwl()
{
    int count = 0, presence_past = 0;
    esp_rom_gpio_pad_select_gpio(GPIO_RCWL);
    gpio_set_direction(GPIO_RCWL, GPIO_MODE_INPUT);

    gpio_pulldown_en(GPIO_RCWL);
    // Desabilitar o resistor do Pull-up por segurança
    gpio_pullup_dis(GPIO_RCWL);
    int presence_level;

     gpio_set_direction(8, GPIO_MODE_OUTPUT); // Pino de um ledo acredito. Averiguar quando for mexecer nessa função
        
    
    while (1)
    {   
        presence_level = gpio_get_level(GPIO_RCWL);
        
        if (count == 7 || presence_level != presence_past)
        {
            printf("PRESENCE_rcwl: %d\n", presence_level);
            strcpy(any_string, ""); //Zerar string
            sprintf(any_string, "%d", presence_level);
            send_data_mqtt("sensor", "presenca", any_string, building_area_credencial);
            
            count = 0;
        }

        presence_past = presence_level;
        vTaskDelay( 2 * 1000 / portTICK_PERIOD_MS );

    }
    
}

void SCT013_task(void *pvParameter)
{
    float *corrente[4];
    float corrente_past[4] = {0.0, 0.0, 0.0, 0.0};
    int count = 0;
    char salas[4][10] = {/*channel-0*/"caa/307",/*channel-1*/"caa/306", /*channel-2*/"caa/305", /*channel-3*/"caa/304"};
    init_SCT013();
    while (1)
    {
        vTaskDelay( 10 * 1000 / portTICK_PERIOD_MS ); // Frequência de envio
        float *corrente = get_SCT023_current(); 
        ESP_LOGI("CORRENTES", "0 %f | count: %d", corrente[0], count);
        ESP_LOGI("CORRENTES", "1 %f", corrente[1]);
        ESP_LOGI("CORRENTES", "2 %f", corrente[2]);
        ESP_LOGI("CORRENTES", "3 %f", corrente[3]);
        char corrente_string[7] = "";
        for (int i = 0; i < 4; i++)
        {
            if (count == 7)
           {
                sprintf(corrente_string, "%.4f", corrente[i]);
                send_data_mqtt("sensor", "corrente", corrente_string, salas[i]);
                vTaskDelay( 500 / portTICK_PERIOD_MS );
                
                corrente_past[i] = corrente[i]; // Ação para bloquear próximo if
                if (i == 3) count = 0;
           }
           
           if (corrente[i] < 0.90*corrente_past[i] || corrente[i] > 1.10*corrente_past[i])
           {
                sprintf(corrente_string, "%.4f", corrente[i]);
                send_data_mqtt("sensor", "corrente", corrente_string, salas[i]);
                vTaskDelay( 500 / portTICK_PERIOD_MS );
           }
           

            corrente_past[i] = corrente[i];

        }

        count++;
    }
}


float heat_index (float T, float RH )
{
    T = T * 9 / 5 + 32;
    return (float) ((C1 + C2*T + C3*RH + C4*T*RH + C5*T*T + C6*RH*RH + C7*T*T*RH + C8*T*RH*RH + C9*T*T*RH*RH) - 32) *5/9;
}

void DHT_task(void *pvParameter)
{   
    esp_rom_gpio_pad_select_gpio(GPIO_DHT22);    
    gpio_set_direction(GPIO_DHT22, GPIO_MODE_INPUT);

    // Configurar o resistor do Pull-up por segurança
    gpio_pulldown_en(GPIO_DHT22);
    // Desabilitar o resistor do Pull-up por segurança
    gpio_pullup_dis(GPIO_DHT22);

    setDHTgpio(GPIO_DHT22);
    printf( "Starting DHT Task %d\n\n", GPIO_DHT22);
    static float temperatura, umidade;
    static char tempString[5] = "", umiString[5] = "", heatIndexString[5] = "";
    static float heatIndex = 0;
    int isOk;

    while(1)
    {
        
        setDHTgpio(GPIO_DHT22); // Tem q alimentar os dois cabos usb/fontes
        isOk = readDHT(); // causando stackoverflow
        errorHandler(isOk); // Checa se não há problema pra a leitura
        

        temperatura =  20.0 + ((float) rand() / (float) (RAND_MAX/20));
        umidade =  30.0 + ((float) rand() / (float) (RAND_MAX/70));
        umidade = getHumidity();
        temperatura = getTemperature();
        heatIndex = heat_index(temperatura, umidade);

        ESP_LOGW("DHT_task", "temperatura: %f, umidade: %f", temperatura, umidade);

        sprintf(umiString, "%.2f", umidade);
        sprintf(tempString, "%.2f", temperatura);
        sprintf(heatIndexString, "%.2f", heatIndex);

        send_data_mqtt("sensor", "umidade", umiString, building_area_credencial);
        vTaskDelay( 25 / portTICK_PERIOD_MS );
        send_data_mqtt("sensor", "temperatura", tempString, building_area_credencial);
        vTaskDelay( 25 / portTICK_PERIOD_MS );
        send_data_mqtt("sensor", "indice_calor", heatIndexString, building_area_credencial);

        ESP_LOGI("DHT_TASK", "Heat Index : %s", heatIndexString);

        vTaskDelay( 20 * 1000 / portTICK_PERIOD_MS );

    }
}

// Paramenters: Tipo do dispositivo (ex: sensor ou atuador), área do edfício (building_area_credencial), nome do dispositvo, valor de envio
void send_data_mqtt(char *dispTipo, char *dispNome, char *dispValor, char *area)
{
    // if (strlen(building_area_credencial) == 0)
    // {
        // Formato [JSON] => {"\"{Nome aqui}\"": {valor numerico aqui}}
        strcpy(format_msg_mqtt, ""); //Zerar endereço
        sprintf(format_msg_mqtt, "%s/%s/%s|{\"%s\":%s}", area, dispTipo, dispNome, dispNome, dispValor);
        

        size_data_mqtt = strlen(format_msg_mqtt); 

        // Preparando dados p/ envio
        esp_err_t err;
        mesh_data_t data_mqtt;
        data_mqtt.data = format_msg_mqtt;
        data_mqtt.size = size_data_mqtt;
        data_mqtt.proto = MESH_PROTO_BIN;
        data_mqtt.tos = MESH_TOS_P2P;
        is_running = true;

        if (!esp_mesh_is_root()) {
            ESP_LOGI(MESH_TAG, "layer:%d, rtableSize:%d, %s", mesh_layer,
                     esp_mesh_get_routing_table_size(),
                     (is_mesh_connected && esp_mesh_is_root()) ? "ROOT" : is_mesh_connected ? "NODE" : "DISCONNECT");
        }
            
        //  Dispositivo PAI fará o envio para o Servidor
        err = esp_mesh_send(NULL, &data_mqtt, MESH_DATA_P2P, NULL, 0);
        if (err == ESP_OK) 
        {
            ESP_LOGI("MESH", "Envio /p o PAI com Sucesso");
            ESP_LOGI("MESH mqtt", "%s", format_msg_mqtt);

        } else ESP_LOGE("MESH Error",": Sent with ERRO code: %d", err);

} 

// Send any complete custom string 
void mqtt_subscribe_via_root(char *topic)
{
    strcpy(format_msg_mqtt, ""); //Zerar endereço
    sprintf(format_msg_mqtt, "%s|sub", topic);
   
    size_data_mqtt = strlen(format_msg_mqtt);

    printf("\n%s\n", topic);

    // Preparando dados p/ envio
    esp_err_t err;
    mesh_data_t data_mqtt;
    data_mqtt.data = format_msg_mqtt;
    data_mqtt.size = size_data_mqtt;
    data_mqtt.proto = MESH_PROTO_BIN;
    data_mqtt.tos = MESH_TOS_P2P;
    is_running = true;

    //  Dispositivo PAI fará o envio para o Servidor
    err = esp_mesh_send(NULL, &data_mqtt, MESH_DATA_P2P, NULL, 0);
    if (err == ESP_OK) 
    {
        ESP_LOGI("MESH", "Envio /p o PAI com Sucesso: mqtt_subscribe_via_root");
    } else ESP_LOGE("MESH Error",": Sent with ERRO code: %d", err);

    if (route_table_size < 10) {
        vTaskDelay(1 * 1000 / portTICK_PERIOD_MS);
        }
}

// request subscription to topic via root 
void request_credencial_via_root(char *topic)
{
    // Formato [JSON] => {"\"{Nome aqui}\"": {valor númerico aqui}}
    strcpy(format_msg_mqtt, ""); //Zerar endereço
    sprintf(format_msg_mqtt, "%s|", topic);
   
    size_data_mqtt = strlen(format_msg_mqtt);

    ESP_LOGI("mesh_main/request_credencial_via_root.","topic: %s", topic);
    // Preparando dados p/ envio
    esp_err_t err;
    mesh_data_t data_mqtt;
    data_mqtt.data = format_msg_mqtt;
    data_mqtt.size = strlen(format_msg_mqtt);
    data_mqtt.proto = MESH_PROTO_BIN;
    data_mqtt.tos = MESH_TOS_P2P;
    is_running = true;

    //  Dispositivo PAI fará o envio para o Servidor
    err = esp_mesh_send(NULL, &data_mqtt, MESH_DATA_P2P, NULL, 0);
    
    if (err) {
        ESP_LOGE(MESH_TAG,
                    "[L:%d]parent:"MACSTR", heap:%" PRId32 "[err:0x%x, proto:%d, tos:%d]",
                    mesh_layer, MAC2STR(mesh_parent_addr.addr), esp_get_minimum_free_heap_size(),
                    err, data_mqtt.proto, data_mqtt.tos);
    } else {
        ESP_LOGW(MESH_TAG,
                    "[L:%d][rtableSize:%d]parent:"MACSTR", heap:%" PRId32 "[err:0x%x, proto:%d, tos:%d]",
                    mesh_layer,
                    esp_mesh_get_routing_table_size(),
                    MAC2STR(mesh_parent_addr.addr), esp_get_minimum_free_heap_size(),
                    err, data_mqtt.proto, data_mqtt.tos);
    }

    if (err == ESP_OK) 
    {
        ESP_LOGI("mesh_main/request_credencial_via_root",
         "Envio /p o PAI com Sucesso: request_credencial_via_root");
    } else ESP_LOGE("mesh_main/request_credencial_via_root",": Sent with ERRO code: %d", err);

    if (route_table_size < 10) {
        vTaskDelay(1 * 1000 / portTICK_PERIOD_MS);
        }
}

// Send only credencial id to child device | Função exclusiva do root
// Esta função não conversa com a função esp_mesh_p2p_rx_main. Medida esta que evita msg ficar em loop
void send_credencial_area(char *topic, char *data, int *len_array)
{
    int lenght_topic = len_array[0];
    int lenght_data = len_array[1];
    u_int8_t is_not_root = 0;

    for (int i = 0; i < 12; i++)
    {
        printf("topic %c vs %c topic\n", topic[i], str_mac_base[i]);
        if (topic[i] != str_mac_base[i]) is_not_root = 1;
    }
    if (lenght_data == 0) ESP_LOGI("mesh_main/send_credencial_area", "lenght_data VAZIO");
    

    ESP_LOGI("mesh_main/send_credencial_area", "Entrada: %.*s, Mac Daqui: %s, is_not_root: %d", lenght_topic, topic, str_mac_base, is_not_root);
    if (is_not_root) // Envia para os dispositivos não root
    {
        printf("\n if valido: %d\n", is_not_root);
        
        printf("\n lenght_topic: %d, lenght_data: %d\n", lenght_topic, lenght_data);

        // Formato [JSON] => {"\"{Nome aqui}\"": {valor númerico aqui}}
        strcpy(format_msg_mqtt, ""); //Zerar endereço
        sprintf(format_msg_mqtt, "%.*s|%.*s", lenght_topic, topic, lenght_data, data);
        
        printf("\n Topic string: %.*s, total len_topic: %d\n", lenght_topic, topic, lenght_topic);
        printf("\n format_msg_mqtt string: %.*s\n", strlen(format_msg_mqtt), format_msg_mqtt);

        uint8_t mac_adrr[6] = {0x00,0x00,0x00,0x00,0x00,0x00};
        uint8_t hex_num = 0;
        char aux[5] = "";
        

        for (int h = 0; h < 12; h=h+2) // (mac=xxXXxxXXxxXX)
        {
            printf("0x%c%c\n", topic[h], topic[h+1]);
            sprintf(aux,"0x%c%c", topic[h] ,topic[h+1]);
            
            hex_num = (uint8_t)strtol(aux, NULL, 16); // Transfoma string em inteiro com 16 bit
            mac_adrr[((h)/2)] = (uint8_t)hex_num;

            strcpy(aux, "");
        }

        esp_err_t err_mac;
        mesh_data_t data_mac_device;
        data_mac_device.data = format_msg_mqtt;
        data_mac_device.size = strlen(format_msg_mqtt);
        data_mac_device.proto = MESH_PROTO_BIN;
        data_mac_device.tos = MESH_TOS_P2P;
        is_running = true;

        ESP_LOGI("mesh_main/send_credencial_area", ": Mac adress Destino " MACSTR "", MAC2STR(mac_adrr));

        err_mac = esp_mesh_send(&mac_adrr, &data_mac_device, MESH_DATA_P2P, NULL, 0);

        if (err_mac == ESP_OK) 
        {
            ESP_LOGI("mesh_main/send_credencial_area", ": sent to mac adress %s with SUCCESS", topic);
            
        } else ESP_LOGE("mesh_main/send_credencial_area",": sent with err_mac code: %d", err_mac);
        
    }  else // If ROOT
    {       
        if (lenght_topic == 15 && topic[12] == 47) // lenght = 15 => xxXXxxXXxxXX/id && data[13] == "/"
        {
            set_root_credencial(lenght_topic, lenght_data, topic, data);
        } else
        {
            process_income_command(lenght_topic, lenght_data, topic, data);
        }
        
    }

    if (route_table_size < 10)  vTaskDelay(1 * 1000 / portTICK_PERIOD_MS); 
}

// O tratamento e inclusão de credencial é feito aqui
void set_root_credencial(int lenght_topic, int lenght_data, char *topic, char *data)
{
    ESP_LOGI("mesh_main/set_root_credencial","Starting Device Credencial ...");

    // count_for_cred => count para obter apenas os caracteres de credencial no laço for
    for (int start = 0, count_for_cred = 0, i = 0; i < lenght_data; i++)
    {
        if (data[i] == 34) // Aqui indentifica " (aspas) pelo código da tabela ASCCII
        {
            start = i+1; // Posição seguinte a aspas começa o caractere útil da credencial
            count_for_cred++;
        }
        // Quando chega na 4º aspas encerra a leitura fechando a string com \0 e saindo do for com break. a ordem dos if importa.
        if(count_for_cred == 4) 
        {
            building_area_credencial[i] = "\0";
            break;
        } 
        // Na 3º aspas começa a leitura da credencial
        if(count_for_cred == 3) building_area_credencial[i-start] = data[i];
        
    }
    // strcpy(any_string, ""); //Zerar string
    // sprintf(any_string,"%s/%s/#",str_mac_base, building_area_credencial); // O device poderá receber qualquer tópico de seu interesse
    // if ( mqtt_app_subscribe(any_string, 1) > -1) ESP_LOGW("MQTT","credencial subscribed! topic: %s \n", any_string);
        
    ESP_LOGW("CREDENCIAL","Área assigned: %s, Initiating Tasks ...", building_area_credencial);
    init_sensors_tasks();

}

// Processa os comandos recebidos. Por enquanto recebe comando de ar condicionado.
void process_income_command(int lenght_topic, int lenght_data, char *topic, char *data)
{ 
    ESP_LOGI("mesh_main/process_income_command","KOMECO!");
        // count_for_cred => count para obter apenas os caracteres de credencial no laço for
        for (int count_for_cred = 0, i = 0; i < lenght_data; i++)
        {
            if (data[i] == 34) // Aqui indentifica " (aspas) pelo código da tabela ASCCII
            {
                count_for_cred++;
            }
            // Quando chega na 2º aspas encerra a leitura atribuindo a string da posição seguinte da aspas e sai do for com break.
            if(count_for_cred == 3) 
            {
                ESP_LOGI("KOMECOs","%c\n\n", data[i+1]);

                switch (data[i+1])
                {
                case 49: // 0 -> A/C off
                    komeco_ir_on();
                    ESP_LOGI("KOMECO","[1]\n\n");
                    break;
                case 48: // 0 -> A/C on
                    komeco_ir_off();
                    ESP_LOGI("KOMECO","[0]\n\n");
                    break;
                
                default:
                    break;
                }
                
                break;
            } 
            
        }
}

void esp_mesh_p2p_rx_main(void *arg)
{
    int recv_count = 0;
    esp_err_t err;
    mesh_addr_t from;
    int send_count = 0;
    mesh_data_t data;
    int flag = 0;
    data.data = rx_buf;
    data.size = RX_SIZE;
    is_running = true;

    mesh_addr_t route_table[CONFIG_MESH_ROUTE_TABLE_SIZE];
    int route_table_size = 0;

    char* mqtt_topic;
    char* mqtt_content;
    int setbreak = 0;

    ESP_LOGW("mesh_main/esp_mesh_p2p_rx_main","\n");

    while (is_running)
    {
        // Alocando memória estimada necessária
        mqtt_content = (char*)malloc(100 * sizeof(char));
        mqtt_topic = (char*)malloc(100 * sizeof(char));

        esp_mesh_get_routing_table((mesh_addr_t *) &route_table, CONFIG_MESH_ROUTE_TABLE_SIZE * 6, &route_table_size);

        data.size = RX_SIZE;
        err = esp_mesh_recv(&from, &data, portMAX_DELAY, &flag, NULL, 0);
        ESP_LOGE("mesh_main/esp_mesh_p2p_rx_main","CONTENT RX: %s, size: %d", data.data, data.size);
        if (err != ESP_OK || !data.size) {
            ESP_LOGE(MESH_TAG, "err:0x%x, size:%d", err, data.size);
            continue;
        }
        
        ESP_LOGW("mesh_main/esp_mesh_p2p_rx_main", "CAMPO RX: %s", data.data);

        // Extraindo tópico e mensagem do data
        int data_len = data.size;
        for (int i = 0; i < data.size ; i++) {
            if ((int)data.data[i] == 124) // "|" representa o número 124 na tabela ASCII
            { 
                setbreak = i;
                for (int j = setbreak+1; j < data_len; j++) mqtt_content[j-1-setbreak] = data.data[j];
                mqtt_topic[setbreak] = '\0';
                mqtt_content[data_len-1-setbreak] = '\0';
                break;
            }
            mqtt_topic[i] = data.data[i];
        }
        
        if (mqtt_topic == NULL) {
            ESP_LOGW("mesh_main/esp_mesh_p2p_rx_main","Memory not allocated.\n");
            exit(0);
        }

        int len_mqtt_topic = strlen(mqtt_topic);
        int len_mqtt_content = strlen(mqtt_content);

        ESP_LOGE(MESH_TAG, "Data RX COMING IN : %d, %d", len_mqtt_topic, len_mqtt_content);

        printf("\n len=%d, mqtt_topic=%s, mac_base=%s, mqtt_content=%s\n", len_mqtt_topic, mqtt_topic, str_mac_base, mqtt_content);


        // Toda mensagem que chega é avaliada pra chechar se é requisição de subscribe de algum dispositivo para credenciamento
        // Meg mqtt_conten = 0 é entendido como requisicção de credencial
        if(esp_mesh_is_root())
        {
            (len_mqtt_content == 0) ? request_credencial_direct_to_server(mqtt_topic) : mqtt_app_publish(mqtt_topic, mqtt_content);
        }
        else
        {
            // Obter a ID de credencial - Empacotr numa função
            if (len_mqtt_topic == 15 && mqtt_topic[12] == 47) // lenght = 15 => xxXXxxXXxxXX/id && data[13] == "/"
            {
                set_root_credencial(len_mqtt_topic, len_mqtt_content, mqtt_topic, mqtt_content);
            } 
            else
            {
                process_income_command(len_mqtt_topic, len_mqtt_content, mqtt_topic, mqtt_content);
                printf("\nINCOME DATA");
            }
        }
        

        free(mqtt_topic);
        free(mqtt_content);
        
        
        
        if (!(recv_count % 1)) {
            ESP_LOGW(MESH_TAG,
                     "[#RX:%d/%d][L:%d] parent:"MACSTR", receive from "MACSTR", size:%d, heap:%" PRIu32 ", flag:%d[err:0x%x, proto:%d, tos:%d], Data: %s",
                     recv_count, send_count, mesh_layer,
                     MAC2STR(mesh_parent_addr.addr), MAC2STR(from.addr),
                     data.size, esp_get_minimum_free_heap_size(), flag, err, data.proto,
                     data.tos, data.data);
        }
        
        //  if route_table_size is less than 10, add delay to avoid watchdog in this task. 
        if (route_table_size < 10) {
            vTaskDelay(1 * 1000 / portTICK_PERIOD_MS);
        }
    }
    vTaskDelete(NULL);
}

// Faz subscribe para solicitação de credencial do dispositivo não root
void request_credencial_direct_to_server(char* mqtt_topic)
{
    if ( mqtt_app_subscribe(mqtt_topic, 1) > -1)
    {
        ESP_LOGW("MQTT","Device subscribed! \n");
    }
    else
    {
        // Caso ocorra falha, uma nova tentativa é acionada
        ESP_LOGW("MQTT Sub falha","Nova tentativa! \n");
        vTaskDelay(4000 / portTICK_PERIOD_MS);
        if ( mqtt_app_subscribe(mqtt_topic, 1) > -1)
        {
            ESP_LOGW("MQTT","AREA subscribed! \n");
        }
    }

    strcpy(any_string, ""); //Zerar string
    sprintf(any_string,"%s/+",mqtt_topic);
    if ( mqtt_app_subscribe(any_string, 1) > -1)
    {
        ESP_LOGW("MQTT","Device subscribed | /+! \n");
    }

}

esp_err_t esp_mesh_comm_p2p_start(void)
{
    static bool is_comm_p2p_started = false;
    if (!is_comm_p2p_started) {
        is_comm_p2p_started = true;
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        ESP_LOGI("RX/TX","Initializing Comunucation Tasks ...\n");

        xTaskCreate(esp_mesh_p2p_rx_main, "MPRX", 4096, NULL, 5, NULL);
        xTaskCreate(route_table_available, "TABLE", 4096, NULL, 5, NULL);
        xTaskCreate(check_connection, "CheckConnection", 4096, NULL, 5, NULL);

    }
    return ESP_OK;
}

void route_table_available()
{
    while (1)
    {   
        esp_mesh_get_routing_table((mesh_addr_t *) &route_table,
                                   CONFIG_MESH_ROUTE_TABLE_SIZE * 6, &route_table_size);

        ESP_LOGE(MESH_TAG, "[L:%d]parent:"MACSTR",heap:%" PRId32 " ", mesh_layer, MAC2STR(mesh_parent_addr.addr), esp_get_minimum_free_heap_size());
        for (int i = 0; i < route_table_size; i++) 
        {
            ESP_LOGE(MESH_TAG, ""MACSTR"", MAC2STR(route_table[i].addr));
        }
        vTaskDelay(15 * 1000 / portTICK_PERIOD_MS);
    }
    
}

// Função que inicializa os pinos para a função blink_command()
void init_blink_command()
{
    esp_rom_gpio_pad_select_gpio(LED_BULTIN);
    // Configurar os pinos LED como output
    gpio_set_direction(LED_BULTIN, GPIO_MODE_OUTPUT);
}

// Função implementada na app_main (5 pisks), ip_event_handler (4 pisks) e na check_connection (2 pisk). Tbm foi usada em tasks relacionadas ao sensor de presença com pisks variados.
void blink_command(int n_pisks)
{
    for (int i = 0; i < n_pisks; i++)
    {
        gpio_set_level(LED_BULTIN, 1);
        vTaskDelay(200 / portTICK_PERIOD_MS);
        gpio_set_level(LED_BULTIN, 0);
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
    
    
}

// Task de verificação com relação a credencial atribuida ao dispositivo. Executada a cada 1 min.
void check_connection(void *pvParameter)
{
    while (1)
    {   
        // vTaskDelay(5 * 1000 / portTICK_PERIOD_MS);
        // blink_command(2);
        ESP_LOGI("mesh_main/check_connection", "Checando credencial do dispositivo ... , %s", building_area_credencial);
        if (strlen(building_area_credencial) == 0)
        {
            ESP_LOGI("CHECK", "Entrou no check_connection, %s", building_area_credencial);
            // A solicitação é encaminhada para o root e então enviado p/ o servidor mqtt para busca o id dste mac adress
            strcpy(any_string, ""); //Zerar string
            sprintf(any_string,"%s/id",str_mac_base);
            request_credencial_via_root(any_string);
            // Expadir licença de comando
            strcpy(any_string, ""); //Zerar string
            sprintf(any_string,"%s/id/+",str_mac_base);
            request_credencial_via_root(any_string);
        
        } else ESP_LOGI("CREDENCIAL", "credencial OK => %s", building_area_credencial);
    
        vTaskDelay(15 * 1000 / portTICK_PERIOD_MS);
    }
    
}

void mesh_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    mesh_addr_t id = {0,};
    static uint16_t last_layer = 0;

    switch (event_id) {
    case MESH_EVENT_STARTED: {
        esp_mesh_get_id(&id);
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_MESH_STARTED>ID:"MACSTR"", MAC2STR(id.addr));
        is_mesh_connected = false;
        mesh_layer = esp_mesh_get_layer();
    }
    break;
    case MESH_EVENT_STOPPED: {
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_STOPPED>");
        is_mesh_connected = false;
        mesh_layer = esp_mesh_get_layer();
    }
    break;
    case MESH_EVENT_CHILD_CONNECTED: {
        mesh_event_child_connected_t *child_connected = (mesh_event_child_connected_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_CHILD_CONNECTED>aid:%d, "MACSTR"",
                 child_connected->aid,
                 MAC2STR(child_connected->mac));
    }
    break;
    case MESH_EVENT_CHILD_DISCONNECTED: {
        mesh_event_child_disconnected_t *child_disconnected = (mesh_event_child_disconnected_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_CHILD_DISCONNECTED>aid:%d, "MACSTR"",
                 child_disconnected->aid,
                 MAC2STR(child_disconnected->mac));
    }
    break;
    case MESH_EVENT_ROUTING_TABLE_ADD: {
        mesh_event_routing_table_change_t *routing_table = (mesh_event_routing_table_change_t *)event_data;
        ESP_LOGW(MESH_TAG, "<MESH_EVENT_ROUTING_TABLE_ADD>add %d, new:%d, layer:%d",
                 routing_table->rt_size_change,
                 routing_table->rt_size_new, mesh_layer);
    }
    break;
    case MESH_EVENT_ROUTING_TABLE_REMOVE: {
        mesh_event_routing_table_change_t *routing_table = (mesh_event_routing_table_change_t *)event_data;
        ESP_LOGW(MESH_TAG, "<MESH_EVENT_ROUTING_TABLE_REMOVE>remove %d, new:%d, layer:%d",
                 routing_table->rt_size_change,
                 routing_table->rt_size_new, mesh_layer);
    }
    break;
    case MESH_EVENT_NO_PARENT_FOUND: {
        mesh_event_no_parent_found_t *no_parent = (mesh_event_no_parent_found_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_NO_PARENT_FOUND>scan times:%d",
                 no_parent->scan_times);
    }
    /* TODO handler for the failure */
    break;
    case MESH_EVENT_PARENT_CONNECTED: {
        mesh_event_connected_t *connected = (mesh_event_connected_t *)event_data;
        esp_mesh_get_id(&id);
        mesh_layer = connected->self_layer;
        memcpy(&mesh_parent_addr.addr, connected->connected.bssid, 6);
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_PARENT_CONNECTED>layer:%d-->%d, parent:"MACSTR"%s, ID:"MACSTR", duty:%d",
                 last_layer, mesh_layer, MAC2STR(mesh_parent_addr.addr),
                 esp_mesh_is_root() ? "<ROOT>" :
                 (mesh_layer == 2) ? "<layer2>" : "", MAC2STR(id.addr), connected->duty);
        last_layer = mesh_layer;
        is_mesh_connected = true;
        if (esp_mesh_is_root()) {
            esp_netif_dhcpc_stop(netif_sta);
            esp_netif_dhcpc_start(netif_sta);
        }
        esp_mesh_comm_p2p_start();
    }
    break;
    case MESH_EVENT_PARENT_DISCONNECTED: {
        mesh_event_disconnected_t *disconnected = (mesh_event_disconnected_t *)event_data;
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_PARENT_DISCONNECTED>reason:%d",
                 disconnected->reason);
        is_mesh_connected = false;
        mesh_layer = esp_mesh_get_layer();
    }
    break;
    case MESH_EVENT_LAYER_CHANGE: {
        mesh_event_layer_change_t *layer_change = (mesh_event_layer_change_t *)event_data;
        mesh_layer = layer_change->new_layer;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_LAYER_CHANGE>layer:%d-->%d%s",
                 last_layer, mesh_layer,
                 esp_mesh_is_root() ? "<ROOT>" :
                 (mesh_layer == 2) ? "<layer2>" : "");
        last_layer = mesh_layer;
    }
    break;
    case MESH_EVENT_ROOT_ADDRESS: {
        mesh_event_root_address_t *root_addr = (mesh_event_root_address_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_ROOT_ADDRESS>root address:"MACSTR"",
                 MAC2STR(root_addr->addr));
    }
    break;
    case MESH_EVENT_VOTE_STARTED: {
        mesh_event_vote_started_t *vote_started = (mesh_event_vote_started_t *)event_data;
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_VOTE_STARTED>attempts:%d, reason:%d, rc_addr:"MACSTR"",
                 vote_started->attempts,
                 vote_started->reason,
                 MAC2STR(vote_started->rc_addr.addr));
    }
    break;
    case MESH_EVENT_VOTE_STOPPED: {
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_VOTE_STOPPED>");
        break;
    }
    case MESH_EVENT_ROOT_SWITCH_REQ: {
        mesh_event_root_switch_req_t *switch_req = (mesh_event_root_switch_req_t *)event_data;
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_ROOT_SWITCH_REQ>reason:%d, rc_addr:"MACSTR"",
                 switch_req->reason,
                 MAC2STR( switch_req->rc_addr.addr));
    }
    break;
    case MESH_EVENT_ROOT_SWITCH_ACK: {
        /* new root */
        mesh_layer = esp_mesh_get_layer();
        esp_mesh_get_parent_bssid(&mesh_parent_addr);
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_ROOT_SWITCH_ACK>layer:%d, parent:"MACSTR"", mesh_layer, MAC2STR(mesh_parent_addr.addr));
    }
    break;
    case MESH_EVENT_TODS_STATE: {
        mesh_event_toDS_state_t *toDs_state = (mesh_event_toDS_state_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_TODS_REACHABLE>state:%d", *toDs_state);
    }
    break;
    case MESH_EVENT_ROOT_FIXED: {
        mesh_event_root_fixed_t *root_fixed = (mesh_event_root_fixed_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_ROOT_FIXED>%s",
                 root_fixed->is_fixed ? "fixed" : "not fixed");
    }
    break;
    case MESH_EVENT_ROOT_ASKED_YIELD: {
        mesh_event_root_conflict_t *root_conflict = (mesh_event_root_conflict_t *)event_data;
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_ROOT_ASKED_YIELD>"MACSTR", rssi:%d, capacity:%d",
                 MAC2STR(root_conflict->addr),
                 root_conflict->rssi,
                 root_conflict->capacity);
    }
    break;
    case MESH_EVENT_CHANNEL_SWITCH: {
        mesh_event_channel_switch_t *channel_switch = (mesh_event_channel_switch_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_CHANNEL_SWITCH>new channel:%d", channel_switch->channel);
    }
    break;
    case MESH_EVENT_SCAN_DONE: {
        mesh_event_scan_done_t *scan_done = (mesh_event_scan_done_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_SCAN_DONE>number:%d",
                 scan_done->number);
    }
    break;
    case MESH_EVENT_NETWORK_STATE: {
        mesh_event_network_state_t *network_state = (mesh_event_network_state_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_NETWORK_STATE>is_rootless:%d",
                 network_state->is_rootless);
    }
    break;
    case MESH_EVENT_STOP_RECONNECTION: {
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_STOP_RECONNECTION>");
    }
    break;
    case MESH_EVENT_FIND_NETWORK: {
        mesh_event_find_network_t *find_network = (mesh_event_find_network_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_FIND_NETWORK>new channel:%d, router BSSID:"MACSTR"",
                 find_network->channel, MAC2STR(find_network->router_bssid));
    }
    break;
    case MESH_EVENT_ROUTER_SWITCH: {
        mesh_event_router_switch_t *router_switch = (mesh_event_router_switch_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_ROUTER_SWITCH>new router:%s, channel:%d, "MACSTR"",
                 router_switch->ssid, router_switch->channel, MAC2STR(router_switch->bssid));
    }
    break;
    case MESH_EVENT_PS_PARENT_DUTY: {
        mesh_event_ps_duty_t *ps_duty = (mesh_event_ps_duty_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_PS_PARENT_DUTY>duty:%d", ps_duty->duty);
    }
    break;
    case MESH_EVENT_PS_CHILD_DUTY: {
        mesh_event_ps_duty_t *ps_duty = (mesh_event_ps_duty_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_PS_CHILD_DUTY>cidx:%d, "MACSTR", duty:%d", ps_duty->child_connected.aid-1,
                MAC2STR(ps_duty->child_connected.mac), ps_duty->duty);
    }
    break;
    default:
        ESP_LOGI(MESH_TAG, "unknown id:%" PRId32 "", event_id);
        break;
    }
}

void ip_event_handler(void *arg, esp_event_base_t event_base,
                      int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    current_ip = event->ip_info.ip;
    ESP_LOGI(MESH_TAG, "<IP_EVENT_STA_GOT_IP>IP:" IPSTR, IP2STR(&event->ip_info.ip));

    // Caso o dispositivo seja o root, conecta ao servidor broker aws
    if(esp_mesh_is_root()) 
    {   
        // Caso o esp seja o root, ativamos o mqtt
        if (mqtt_app_start() == ESP_OK) 
        {
            ESP_LOGI("MQTT","Connected to the Server. Initializing Tasks ...\n");

        } else ESP_LOGI("MQTT"," Error Connecting to Broker mqtt.\n");
        
    } 
    
}

void app_main(void)
{
    if (esp_efuse_mac_get_default(mac_base) == ESP_OK)
    {
        sprintf(str_mac_base,"%02x%02x%02x%02x%02x%02x", mac_base[0],mac_base[1],mac_base[2],mac_base[3],mac_base[4],mac_base[5]);
        printf("\n str_mac_base: %s\n\n",str_mac_base);
    } else printf("\n ERROR READING MAC BASE ADRESS\n");

    ESP_ERROR_CHECK(nvs_flash_init());
    /*  tcpip initialization */
    ESP_ERROR_CHECK(esp_netif_init());
    /*  event initialization */
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    /*  create network interfaces for mesh (only station instance saved for further manipulation, soft AP instance ignored */
    ESP_ERROR_CHECK(esp_netif_create_default_wifi_mesh_netifs(&netif_sta, NULL));
    /*  wifi initialization */
    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&config));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
    ESP_ERROR_CHECK(esp_wifi_start());
    /*  mesh initialization */
    ESP_ERROR_CHECK(esp_mesh_init());
    ESP_ERROR_CHECK(esp_event_handler_register(MESH_EVENT, ESP_EVENT_ANY_ID, &mesh_event_handler, NULL));
    /*  set mesh topology */
    ESP_ERROR_CHECK(esp_mesh_set_topology(CONFIG_MESH_TOPOLOGY));
    /*  set mesh max layer according to the topology */
    ESP_ERROR_CHECK(esp_mesh_set_max_layer(CONFIG_MESH_MAX_LAYER));
    ESP_ERROR_CHECK(esp_mesh_set_vote_percentage(1));
    ESP_ERROR_CHECK(esp_mesh_set_xon_qsize(128));
#ifdef CONFIG_MESH_ENABLE_PS
    /* Enable mesh PS function */
    ESP_ERROR_CHECK(esp_mesh_enable_ps());
    /* better to increase the associate expired time, if a small duty cycle is set. */
    ESP_ERROR_CHECK(esp_mesh_set_ap_assoc_expire(60));
    /* better to increase the announce interval to avoid too much management traffic, if a small duty cycle is set. */
    ESP_ERROR_CHECK(esp_mesh_set_announce_interval(600, 3300));
#else
    /* Disable mesh PS function */
    ESP_ERROR_CHECK(esp_mesh_disable_ps());
    ESP_ERROR_CHECK(esp_mesh_set_ap_assoc_expire(10));
#endif
    mesh_cfg_t cfg = MESH_INIT_CONFIG_DEFAULT();
    /* mesh ID */
    memcpy((uint8_t *) &cfg.mesh_id, MESH_ID, 6);
    /* router */
    cfg.channel = CONFIG_MESH_CHANNEL;
  cfg.router.ssid_len = strlen(WIFI_SSID); 
    memcpy((uint8_t *) &cfg.router.ssid, WIFI_SSID, cfg.router.ssid_len);
    memcpy((uint8_t *) &cfg.router.password, WIFI_PASS, strlen(WIFI_PASS));
    /* mesh softAP */
    ESP_ERROR_CHECK(esp_mesh_set_ap_authmode(CONFIG_MESH_AP_AUTHMODE));
    cfg.mesh_ap.max_connection = CONFIG_MESH_AP_CONNECTIONS;
    cfg.mesh_ap.nonmesh_max_connection = CONFIG_MESH_NON_MESH_AP_CONNECTIONS;
    memcpy((uint8_t *) &cfg.mesh_ap.password, CONFIG_MESH_AP_PASSWD,
           strlen(CONFIG_MESH_AP_PASSWD));
    ESP_ERROR_CHECK(esp_mesh_set_config(&cfg));
    /* mesh start */
    ESP_ERROR_CHECK(esp_mesh_start());
#ifdef CONFIG_MESH_ENABLE_PS
    /* set the device active duty cycle. (default:10, MESH_PS_DEVICE_DUTY_REQUEST) */
    ESP_ERROR_CHECK(esp_mesh_set_active_duty_cycle(CONFIG_MESH_PS_DEV_DUTY, CONFIG_MESH_PS_DEV_DUTY_TYPE));
    /* set the network active duty cycle. (default:10, -1, MESH_PS_NETWORK_DUTY_APPLIED_ENTIRE) */
    ESP_ERROR_CHECK(esp_mesh_set_network_duty_cycle(CONFIG_MESH_PS_NWK_DUTY, CONFIG_MESH_PS_NWK_DUTY_DURATION, CONFIG_MESH_PS_NWK_DUTY_RULE));
#endif
    ESP_LOGI(MESH_TAG, "mesh starts successfully, heap:%" PRId32 ", %s<%d>%s, ps:%d",  esp_get_minimum_free_heap_size(),
             esp_mesh_is_root_fixed() ? "root fixed" : "root not fixed",
             esp_mesh_get_topology(), esp_mesh_get_topology() ? "(chain)":"(tree)", esp_mesh_is_ps_enabled());
}
