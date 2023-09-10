#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <string.h>
#include <driver/gpio.h>
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mesh.h"
#include "esp_mesh_internal.h"
#include "nvs_flash.h"
#include "mqtt_app.h"
#include "dht22.h"
#include "sct013-30a.h"

// trecho de libs do IR
#include "driver/rmt.h"
#include "midea_ir.h"

static rmt_channel_t example_tx_channel = 0 /*CONFIG_EXAMPLE_RMT_TX_CHANNEL*/;

/*******************************************************
 *                Macros
 *******************************************************/

/*******************************************************
 *                Constants
 *******************************************************/

// #define WIFI_SSID "brisa-3287883"
// #define WIFI_PASS  "2hs5t788"
 
#define WIFI_SSID "tplinkcaa"
#define WIFI_PASS  "contadorcaa123"

unsigned char mac_base[6];
char str_mac_base[13] = "";

#define LED_BULTIN 2
  // --- thermal confort solution
#define GPIO_DHT22 33 // D33

#define RX_SIZE          (512)
#define TX_SIZE          (512)

const double C1 = -42.379;
const double C2 = 2.04901523;
const double C3 = 10.14333127;
const double C4 = -0.22475541;
const double C5 = -0.00683783;
const double C6 = -0.05481717;
const double C7 = 0.00122874;
const double C8 = 0.00085282;
const double C9 = -0.00000199;

//--- presence solution
#define PIN_INPUT_0 13 // Presença sensor 1
#define PIN_INPUT_1 12 // Presença sensor 2a
#define PIN_INPUT_2 14  // Presença sensor 2b
#define PIN_INPUT_3 33 // Presença sensor 3 - sem uso

#define PIN_OUTPUT 4 // Buzzer D4

#define TIME_DELAY_MILI 100
#define TIME_5MIN 1800     // Tempo cheio configurado em 3 minutos. Usado quando não confirmado presença
#define TIME_TRAVA 3600      // 6 MIN

/*******************************************************
 *                Variable Definitions
 *******************************************************/
// ---- mesh solution
static const char *MESH_TAG = "mesh_main";
static const uint8_t MESH_ID[6] = { 0x77, 0x77, 0x77, 0x77, 0x77, 0x77};
//static char tx_buf[TX_SIZE] = { 0, };
static char rx_buf[RX_SIZE] = { 0, };
static bool is_running = true;
static bool is_mesh_connected = false;
static mesh_addr_t mesh_parent_addr;
static int mesh_layer = -1;
static esp_netif_t *netif_sta = NULL;

static esp_ip4_addr_t current_ip;
mesh_addr_t route_table[CONFIG_MESH_ROUTE_TABLE_SIZE];
int route_table_size = 0;

// ---- presence solution
static int8_t trava = 0;                             // Trava desativada
static int16_t cont_time_presence = TIME_5MIN;   // Tempo de carência configurado
static int8_t last_state = 0;
static int8_t aux_last_state = 0;
static int8_t aux_2_states_before;
static int8_t current_state = 0; 

static int8_t input_sensor_1 = 0;
static int8_t input_sensor_2a = 0;
static int8_t input_sensor_2b = 0;
static int8_t input_sensor_3 = 0;

static int8_t ticks_area_1 = 0;
static int8_t ticks_area_2a = 0;
static int8_t ticks_area_2b = 0;
static int8_t ticks_area_3 = 0;

uint8_t last_detect = 0;

uint8_t estado = 0;

xSemaphoreHandle semaphorBinarioA;
xSemaphoreHandle semaphorBinarioB;


// 3 fila de interrupção para os 3 sensores de presença
xQueueHandle fila_ittr_presenca_pin_1;
xQueueHandle fila_ittr_presenca_pin_1;
xQueueHandle fila_ittr_presenca_pin_1;

/*******************************************************
 *                Function Declarations
 *******************************************************/

static char format_msg_mqtt[100] = ""; // Ex: ufcg/cg_sede/caa/{area}/sensor/temperatura
static char  building_id[35] = "ufcg/cg_sede/caa";
static char  building_area_credencial[35] = "";
static char any_string[30] = "";
static int size_data_mqtt = 0;
// 

/*******************************************************
 *                Function Definitions
 *******************************************************/


xQueueHandle fila_comando_komeco;
static u_int8_t air_on_off = 0; // estado

// Função que inicializa os pinos para a função blink_command()
void init_blink_command()
{
    gpio_pad_select_gpio(LED_BULTIN);
    // Configurar os pinos LED como output
    gpio_set_direction(LED_BULTIN, GPIO_MODE_OUTPUT);
}

// Função implementada na app_main (5 pisks), ip_event_handler (4 pisks) e na check_connection (2 pisk). Tbm foi usada em tasks relacionadas ao sensor de presença com pisks variados.
void blink_command(int pisks)
{
    for (int i = 0; i < pisks; i++)
    {
        gpio_set_level(LED_BULTIN, 1);
        vTaskDelay(200 / portTICK_PERIOD_MS);
        gpio_set_level(LED_BULTIN, 0);
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
    
    
}

// Paramenters: Tipo do dispositivo (ex: sensor ou atuador), área do edfício (building_area_credencial), nome do dispositvo, valor de envio
void send_data_mqtt(char *dispTipo, char *dispNome, char *dispValor, char *area)
{
    // if (strlen(building_area_credencial) == 0)
    // {
        // Formato [JSON] => {"\"{Nome aqui}\"": {valor numerico aqui}}
        strcpy(format_msg_mqtt, ""); //Zerar endereço
        sprintf(format_msg_mqtt, "%s/%s/%s/%s|{\"%s\":%s}",building_id, area, dispTipo, dispNome, dispNome, dispValor);
    
        size_data_mqtt = strlen(format_msg_mqtt);

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
            ESP_LOGI("MESH", "Envio /p o PAI com Sucesso");

        } else ESP_LOGE("MESH Error",": Sent with ERRO code: %d", err);

    // } else ESP_LOGI("MESH", "Não é possível enviar dados => SEM CREDENCIAL");

} 

// Send any complete custom string 
void send_data_mqtt_string(char *topic, char *valor)
{
    // Formato [JSON] => {"\"{Nome aqui}\"": {valor númerico aqui}}
    strcpy(format_msg_mqtt, ""); //Zerar endereço
    sprintf(format_msg_mqtt, "%s|{\"0\":%s}", topic, valor);
   
    size_data_mqtt = strlen(format_msg_mqtt);

    printf("\n%s: %s\n", topic, valor);

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
        ESP_LOGI("MESH", ": Envio /p o PAI com Sucesso");
    } else ESP_LOGE("MESH Error",": Sent with ERRO code: %d", err);
}

// Send only credencial area to child device
void send_credencial_area(char *topic, char *data, int *len_data)
{
    int lenght_data = len_data;
    // Formato [JSON] => {"\"{Nome aqui}\"": {valor númerico aqui}}
    strcpy(format_msg_mqtt, ""); //Zerar endereço
    sprintf(format_msg_mqtt, "%.*s|%.*s", 16, topic, lenght_data, data);
    
    printf("\n Topic string: %.*s, total len_data: %d\n", 16, topic, lenght_data);
    printf("\n format_msg_mqtt string: %.*s\n", lenght_data+17, format_msg_mqtt);

    uint8_t mac_adrr[6] = {0x00,0x00,0x00,0x00,0x00,0x00};
    uint8_t hex_num = 0;
    char aux[5] = "";
    

    for (int h = 4; h < 16; h=h+2) // Comeca do caractere 4 para ler endereço mac correto (mac/xxXXxxXXxxXX)
    {
        printf("0x%c%c\n", topic[h], topic[h+1]);

        sprintf(aux,"0x%c%c", topic[h] ,topic[h+1]);
        
        hex_num = (uint8_t)strtol(aux, NULL, 16);
        mac_adrr[((h-4)/2)] = (uint8_t)hex_num;

        strcpy(aux, "");
    }

    esp_err_t err_mac;
    mesh_data_t data_mac_device;
    data_mac_device.data = format_msg_mqtt;
    data_mac_device.size = lenght_data+17;
    data_mac_device.proto = MESH_PROTO_BIN;
    data_mac_device.tos = MESH_TOS_P2P;
    is_running = true;

    err_mac = esp_mesh_send(&mac_adrr, &data_mac_device, MESH_DATA_P2P, NULL, 0);

    if (err_mac == ESP_OK) 
    {
        ESP_LOGI("MAC", ": sent to mac adress %s with SUCCESS", topic);
        
    } else ESP_LOGE("MAC SENT ERROR",": sent with err_mac code: %d", err_mac);
    
    
        
}


float heat_index (float T, float RH )
{
    T = T * 9 / 5 + 32;
    return (float) ((C1 + C2*T + C3*RH + C4*T*RH + C5*T*T + C6*RH*RH + C7*T*T*RH + C8*T*RH*RH + C9*T*T*RH*RH) - 32) *5/9;
}

void DHT_task(void *pvParameter)
{   
    gpio_pad_select_gpio(GPIO_DHT22);    
    gpio_set_direction(GPIO_DHT22, GPIO_MODE_INPUT);

    // Configurar o resistor do Pull-up por segurança
    gpio_pulldown_en(GPIO_DHT22);
    gpio_pulldown_en(GPIO_DHT22);
    // Desabilitar o resistor do Pull-up por segurança
    gpio_pullup_dis(GPIO_DHT22);
    gpio_pullup_dis(GPIO_DHT22); 

    setDHTgpio(GPIO_DHT22);
    printf( "Starting DHT Task\n\n");
    int fire_alert_count = -1, umidity_alert_count = -1; // Na primeira comparação de ajuste vai pra zero
    static float temperatura, umidade;
    static char tempString[5] = "", umiString[5] = "", heatIndexString[5] = "";
    static float umidadeBefore = 0, temperaturaBefore = 0;
    static float heatIndex = 0;
    int isOk;
    while(1) {
        vTaskDelay( 20000 / portTICK_RATE_MS );
        setDHTgpio(GPIO_DHT22); // Tem q alimentar os dois cabos usb/fontes
        isOk = readDHT(); // causando stackoverflow
        errorHandler(isOk); // Checa se não há problema pra a leitura

        /*
        // PARTE 1: Tratamento segurança
        ** Nesse intervalo antes de atualizar a temperatura podemos considerar que os valores de temperatura e umidade são dados como valores anteriores. Serve para fazer comparações de mudanças drásticas de variáveis.
        */

        // temperatura =  20.0 + ((float) rand() / (float) (RAND_MAX/20));
        // umidade =  30.0 + ((float) rand() / (float) (RAND_MAX/70));
        umidade = getHumidity();
        temperatura = getTemperature();
        heatIndex = heat_index(temperatura, umidade);

        printf("\n\nUmidade[anterior]: %.2f\nUmidade[depois]: %.2f\nTemperatura[anterior]: %.2f\nTemperatura[depois]: %.2f\n\n", umidadeBefore, umidade, temperaturaBefore, temperatura);

        if ((temperatura - temperaturaBefore) >= 3) fire_alert_count++;
        if ((umidade - umidadeBefore) <= 5) fire_alert_count++;


        if (fire_alert_count > 0)
        {
            /* Enviar alerta de mudança drástica de temperatura */
        }

        if (umidity_alert_count > 0)
        {
            /* Enviar alerta de mudança drástica de umidade */
        }
        

        // PARTE 2: Obtenção de e envio de variáveis

        temperaturaBefore = temperatura; // Válido apenas na Parte 1
        umidadeBefore = umidade;         // Válido apenas na Parte 1

        sprintf(umiString, "%.2f", umidade);
        sprintf(tempString, "%.2f", temperatura);
        sprintf(heatIndexString, "%.2f", heatIndex);

        send_data_mqtt("sensor", "umidade", umiString, building_area_credencial);
        
        send_data_mqtt("sensor", "temperatura.", tempString, building_area_credencial);

        send_data_mqtt("sensor", "indice_calor", heatIndexString, building_area_credencial);

        ESP_LOGI("IC", "(float) : %s", heatIndexString);

    }
}

int timeCount = 0;
int cont = 0;

int aluno_ganhato = 0;
int aluno_saiu = 0;


void procress_presence(void * params)
{   
     while (1) {
        xSemaphoreTake(semaphorBinarioA, portMAX_DELAY);

        ticks_area_1 ? input_sensor_1 = 1 : (input_sensor_1 = 0);
        ticks_area_2a ? input_sensor_2a = 1 : (input_sensor_2a = 0);
        ticks_area_2b ? input_sensor_2b = 1 : (input_sensor_2b = 0);
        
        input_sensor_3 = 0; //ticks_area_3 ? input_sensor_3 = 1 : (input_sensor_3 = 0);
        
        aux_last_state = current_state; // Antes de atribuir o current_state ele fica como último estado.
        aux_2_states_before = last_state; // Na mesma lógica o último estado agora fica um anterior ao último por sem um novo ciclo.

        // Início leitura
        if (input_sensor_1 == 1) // Área 1 sendo ativada
        {
            if (input_sensor_2a == 1 || input_sensor_2b == 1) // Área 2 sendo ativada
            { 
                aluno_ganhato = 0;
                if (input_sensor_3 == 1) // [1] | Área 3 sendo ativada
                { 
                    if(aux_last_state == 1) last_state = aux_last_state;
                    current_state = 1;
                    // Trava geral ativada
                    cont_time_presence = TIME_TRAVA;
                    cont = 0;
                    trava = 1;
                }
                else // [2] | Área 3 sendo desativada
                {
                    if(aluno_saiu == 1) aluno_saiu ++;

                    if(aux_last_state == 2) last_state = aux_last_state;
                    current_state = 2;
                    // Trava geral desativada
                    cont_time_presence = TIME_5MIN;
                    cont = 0;
                    trava = 0;
                    ESP_LOGE("OFF","\nTrava desativada\n");
                }

            }
            else // Área 2 sendo desativada
            {
                if (input_sensor_3 == 1) // [3] |  Área 3 sendo ativada
                {
                    aluno_saiu = 0;
                    aluno_ganhato = 0;
                    if(aux_last_state != 3) last_state = aux_last_state;
                    current_state = 3;
                    // Trava geral ativada
                    cont_time_presence = TIME_TRAVA;
                    cont = 0;
                    trava = 1;
                    
                }
                else //  [4] | Área 2 e 3 sendo desativada 
                {
                    if(aux_last_state != 4) last_state = aux_last_state;
                    current_state = 4;
                    
                    
                    if(aluno_ganhato == 1) ESP_LOGE("IN/OUT","Aluno Ganhato\n");
                    
                    if(aluno_saiu == 2) 
                    {
                        ESP_LOGE("IN/OUT","Aluno Saiu\n");
                        // Saída de vdd
                        cont_time_presence = TIME_5MIN;
                        trava = 0;
                        cont = 0;
                    }
                }
            }
            
        }
        else // Área 1 sendo desativada
        {
            if (input_sensor_2a + input_sensor_2b) // [5] e [6] | Área 2 sendo ativada
            {   
                aluno_saiu = 0;
                aluno_saiu = 1;
                aluno_ganhato = 0;
                if(input_sensor_3) //[5] | Área 3 sendo ativada
                {
                    if(aux_last_state == 5) last_state = aux_last_state;
                    current_state = 5;
                    // Trava geral ativada
                    cont_time_presence = TIME_TRAVA;
                    cont = 0;
                    trava = 1;
                }
                else // [6] | Área 3 sendo desativada
                {
                    if(aux_last_state != 6) last_state = aux_last_state;
                    current_state = 6;
                    // Trava geral ativada
                    cont_time_presence = TIME_TRAVA;
                    cont = 0;
                    trava = 1;
                }
                
            } 
            else // Área 2 desativada
            {
                aluno_saiu = 0;
                aluno_ganhato = 1;
                if (input_sensor_3 == 1)  // [7] | Área 3 sendo ativada
                { 
                    if(aux_last_state != 7) last_state = aux_last_state;
                    current_state = 7;
                    // Trava geral ativada
                    cont_time_presence = TIME_TRAVA;
                    cont = 0;
                    trava = 1;
                }
                else // [8] | Área 3 sendo desativada
                {
                    aluno_ganhato = 1;
                    if(aux_last_state != 8) last_state = aux_last_state;
                    current_state = 8;
                    ESP_LOGE("State","Repouso\n");
                    // Presença ? Mantém trava geral ativada:  (não sei) Trava geral desativada
                } 
                
            } 
        }
        cont ++;
        // Final leitura

        // cont == cont_time_presence ? Desligar tudo : continua
        
        ESP_LOGI("Status sensores: ","\n Área 1: %d\n Área 2a: %d\n Área 2b: %d, \n", input_sensor_1, input_sensor_2a, input_sensor_2b);

        timeCount = cont*(TIME_DELAY_MILI+60)/1000;
        //ESP_LOGW("TEMPO","%d Seg , count target: %d\n", timeCount, cont_time_presence*(TIME_DELAY_MILI+60)/1000);
        
        ESP_LOGW("VARIÁVEIS","TRAVA: %d, current_state: %d, last_state: %d", trava, current_state, last_state);

        xSemaphoreGive(semaphorBinarioB);
        vTaskDelay(TIME_DELAY_MILI / portTICK_PERIOD_MS);

    }
}
// --
void regulator_master_hc()
{

    gpio_pad_select_gpio(PIN_INPUT_0);
    gpio_pad_select_gpio(PIN_INPUT_1);
    gpio_pad_select_gpio(PIN_INPUT_2);
    gpio_pad_select_gpio(PIN_INPUT_3);
    
    gpio_set_direction(PIN_INPUT_0, GPIO_MODE_INPUT);
    gpio_set_direction(PIN_INPUT_1, GPIO_MODE_INPUT);
    gpio_set_direction(PIN_INPUT_2, GPIO_MODE_INPUT);
    gpio_set_direction(PIN_INPUT_3, GPIO_MODE_INPUT);

    uint8_t ticks = 0;
    uint8_t last_tick_s1 = 0;
    uint8_t last_tick_s2a = 0;
    uint8_t last_tick_s2b = 0;

    int8_t correction_tick_s2a = 0;
    int8_t correction_tick_s2b = 0;

    while (1)
    {
        input_sensor_1 = gpio_get_level(PIN_INPUT_0);
        input_sensor_2a = gpio_get_level(PIN_INPUT_1);
        input_sensor_2b = gpio_get_level(PIN_INPUT_2);
        input_sensor_3 = 0;
        ESP_LOGI("A1","Última contagem: %d, Último detectado: s%d", last_tick_s1, last_detect);

        if(input_sensor_1)
        {
            ticks_area_1++;
            printf("A1-%d ", ticks_area_1);
        }
        else
        {
            if(ticks_area_1) 
            {
                last_tick_s1 = ticks_area_1;
                ticks_area_1 = 0;

                last_detect = 1;
            }
            
        }

        ESP_LOGE("A2a","Última contagem: %d, Último detectado: s%d", last_tick_s2a, last_detect);

        if(input_sensor_2a || correction_tick_s2a)
        {
            ticks_area_2a++;
            printf("A3-%d ", ticks_area_2a);
            if(correction_tick_s2a > 0) 
            {
                correction_tick_s2a --;
                last_tick_s2a = ticks_area_2a;
                last_detect = 21;

                if(correction_tick_s2a == 0) ticks_area_2a = 0;
            }
        }
        else
        {
            if(ticks_area_2a) 
            {
                correction_tick_s2a = 21;
                ticks_area_2a++;
            }
        }
        
        ESP_LOGW("A2b","Última contagem: %d, Último detectado: s%d", last_tick_s2b, last_detect);

        if(input_sensor_2b || correction_tick_s2b)
        {
            ticks_area_2b++;
            printf("A2-%d ", ticks_area_2b);
            if(correction_tick_s2b > 0) 
            {
                correction_tick_s2b --;
                last_tick_s2b = ticks_area_2b;
                last_detect = 22;

                if(correction_tick_s2b == 0) ticks_area_2b = 0;
            }
        }
        else
        {
            if(ticks_area_2b) 
            {
                correction_tick_s2b = 21;
                ticks_area_2b++;
            }
            
        }
        
        ticks++;

        vTaskDelay(100 / portTICK_PERIOD_MS);

        xSemaphoreGive(semaphorBinarioA);

        xSemaphoreTake(semaphorBinarioB, portMAX_DELAY);
    }

}

void saying_in_out()
{
    gpio_set_direction(PIN_OUTPUT, GPIO_MODE_OUTPUT); // Pino de um ledo acredito. Averiguar quando for mexecer nessa função
    gpio_set_level(PIN_OUTPUT, 1);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    gpio_set_level(PIN_OUTPUT, 0);
    char string[50] = "";

    while (1)
    {
        if (last_detect == 1) // Se o último detectado for sensor 1, pisca 1 vezes
        {
            blink_command(1); // 1 piscadas
        }
        else if (last_detect == 21) // Se o último detectado for sensor 2, pisca 2 vezes
        {
            blink_command(2); // 2 piscadas
        }
        else if (last_detect == 22)// Se o último detectado for sensor 3, pisca 3 vezes
        {
            blink_command(3); // 3 piscadas
        }
        vTaskDelay(3* 1000 / portTICK_PERIOD_MS);

        // Sendo time count
        sprintf(string, "%d", timeCount);
        send_data_mqtt("sensor","presenca/contagem_tempo", string, building_area_credencial);

        // Send trava status
        sprintf(string, "%d", trava);
        send_data_mqtt("sensor","presenca/trava", string, building_area_credencial);

        if (cont < cont_time_presence) // Desliga
        {
            // Sendo trava status
            send_data_mqtt("msg","ufcg_cg_sede/caa/2/sala2/sensor/presenca", "1", building_area_credencial);

            ESP_LOGW("VARIÁVEIS","\nTRAVA: %d\ncurrent_state: %d\nlast_state: %d\n\n", trava, current_state, last_state);

            // Send trava status
            sprintf(string, "%d", trava);
            send_data_mqtt("msg","presenca/estado_ultimo", string, building_area_credencial);
            
            // Send trava status
            sprintf(string, "%d", current_state);
            send_data_mqtt("msg","presenca/estado_atual", string, building_area_credencial);
        }
        else 
        {
            trava = 0;
            // Enviar status trava desativada
            send_data_mqtt("msg","presenca/trava", "0", building_area_credencial); //trava = 0;

            // Enviar status presença vazio
            send_data_mqtt("msg","presenca", "0", building_area_credencial); // presenca = 0

            ESP_LOGW("Status sala: ","VAZIA\n");
            printf("Desligando Ar-condicionado e Iluminacao\n");
        }
    }
    
}


void SCT013_task(void *pvParameter)
{
    float corrente_float[3];
    char salas[3][10] = {"sala303","sala305", "sala306"};
    init_SCT013();
    while (1)
    {
        vTaskDelay( 3000 / portTICK_RATE_MS ); // Frequência de envio
        get_SCT023_current(&corrente_float); 
        char corrente_string[7] = "";
        for (int i = 0; i < 3/*1*/; i++)
        {
            sprintf(corrente_string, "%.4f", corrente_float[i]/*[2]*/);
            send_data_mqtt("sensor", "corrente", corrente_string, salas[i]);
            vTaskDelay( 300 / portTICK_RATE_MS );

        }

    }
}

void air_control(void *pvParameter)
{
    int komeco_acao;
    // Início código reservado para IR send
    ESP_LOGI("KOMECO", "IInfraRed Control INIT fast config");

    rmt_config_t rmt_tx_config = RMT_DEFAULT_CONFIG_TX_MIDEA(18 /*CONFIG_EXAMPLE_RMT_TX_GPIO*/, example_tx_channel);
    rmt_config(&rmt_tx_config);
    rmt_driver_install(example_tx_channel, 0, 0);

    // Final código reservado

    while (1)
    {
        if (xQueueReceive(fila_comando_komeco, &komeco_acao, portMAX_DELAY))
        {
            blink_command(3); // 3 piscadas

            if (komeco_acao == 1)
            {
                
                ESP_LOGW("KOMECO", "Turning ON air condicioning");
                komeco_ir_on();

            } else 
            {
                ESP_LOGW("KOMECO", "Turning OFF air condicioning");
                komeco_ir_off();
            }
        }

    }
    
}

void init_panda_tasks()
{
    fila_comando_komeco = xQueueCreate(2, sizeof(int));

    xTaskCreate(&DHT_task, "ReadTemperature&Umidity", 2048, NULL, 2, NULL);
    // xTaskCreate(&SCT013_task, "ReadCurrent", 3072, NULL, 2, NULL);
    // xTaskCreate(&saying_in_out, "AvisoEntradaSaida", 2048, NULL, 1, NULL);
    // xTaskCreate(&procress_presence, "TrataPresenca", 2048, NULL, 1, NULL);
    // xTaskCreate(&regulator_master_hc, "ReguladorHC501", 2048, NULL, 1, NULL);
    //xTaskCreate(&air_control, "ac_control", 4096, NULL, 2, NULL);
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

    while (is_running) {
        // Alocando memória estimada necessária
        mqtt_content = (char*)malloc(100 * sizeof(char));
        mqtt_topic = (char*)malloc(100 * sizeof(char));

        esp_mesh_get_routing_table((mesh_addr_t *) &route_table,
                                   CONFIG_MESH_ROUTE_TABLE_SIZE * 6, &route_table_size);

        data.size = RX_SIZE;
        err = esp_mesh_recv(&from, &data, portMAX_DELAY, &flag, NULL, 0);
        ESP_LOGE("CONTENT RX"," %s, size: %d", data.data, data.size);
        if (err != ESP_OK || !data.size) {
            ESP_LOGE(MESH_TAG, "err:0x%x, size:%d", err, data.size);
            continue;
        }
        
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
            ESP_LOGW("MALLOC","Memory not allocated.\n");
            exit(0);
        }

        int len_mqtt_topic = strlen(mqtt_topic);
        int len_mqtt_content = strlen(mqtt_content);

        printf("\n len=%d, mqtt_topic=%s, mac_base=%s, mqtt_content=%s\n", len_mqtt_topic, mqtt_topic, str_mac_base, mqtt_content);

        // Toda mensagem que chega é avalida pra chechar se é requisição de subscribe de algum dispositivo para credenciamento
        if (len_mqtt_topic == 16 && mqtt_topic[0] == 109) // mqtt_topic == mac/XXxxXXxxXXxx => 16(4+12) caracteres
        {
            ESP_LOGW("CHECK","AREA DE CREDENCIAL!, content length: %d \n", len_mqtt_content);

            ////////////////////////////////////////////
            if (len_mqtt_content == 21) // se len_mqtt_topic == errado7+6 && mqtt_topic[0] == "c"
            {
                ESP_LOGW("KOMECO","COMANDO!");
                ESP_LOGW("KOMECO","mqtt_content: %s", mqtt_content);
                // count_for_cred => count para obter apenas os caracteres de credencial no laço for
                for (int count_for_cred = 0, i = 0; i < len_mqtt_content; i++)
                {
                    if (mqtt_content[i] == 34) // Aqui indentifica " (aspas) pelo código da tabela ASCCII
                    {
                        count_for_cred++;
                        if(count_for_cred == 4) printf("\ncatch\n");
                    }
                    // Quando chega na 4º aspas encerra a leitura atribuindo a string da posição seguinte da aspas e sai do for com break.
                    if(count_for_cred == 4) 
                    {
                        ESP_LOGW("KOMECO","mqtt_content: %c", mqtt_content[i-1]);
                        if (mqtt_content[i-1] == 49)
                        {
                            air_on_off = 1;
                            xQueueSendFromISR(fila_comando_komeco, &air_on_off , NULL);
                        } else 
                        {
                            air_on_off = 0;
                            xQueueSendFromISR(fila_comando_komeco, &air_on_off , NULL);
                        }
                        break;
                    } 
                    
                }
            }
            ///////////////////////////////////////////

            if (len_mqtt_content == 6) // JSON vazio ocupa 6 caracteres da formatação => {"0":}
            {
                vTaskDelay(5000 / portTICK_RATE_MS);
                if ( mqtt_app_subscribe(mqtt_topic, 1) > -1)
                {
                    ESP_LOGW("MQTT","AREA subscribed! \n");

                } else
                {
                    // Caso ocorra falha, uma nova tentativa é acionada
                    vTaskDelay(5000 / portTICK_RATE_MS);
                    if ( mqtt_app_subscribe(mqtt_topic, 1) > -1)
                    {
                        ESP_LOGW("MQTT","AREA subscribed! \n");
                    }
                }

            }else if (len_mqtt_content != 21) // Caso do dispositivo ser child. Mensagem de credencial de área vinda do root,  atribui e inicia as Tasks que precisam dessa credencial.
            {   
                // count_for_cred => count para obter apenas os caracteres de credencial no laço for
                for (int start = 0, count_for_cred = 0, i = 0; i < strlen(mqtt_content); i++)
                {
                    if (mqtt_content[i] == 34) // Aqui indentifica " (aspas) pelo código da tabela ASCCII
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
                    if(count_for_cred == 3) building_area_credencial[i-start] = mqtt_content[i];
                    
                }
                
                ESP_LOGW("CREDENCIAL","Área assigned: %s, Initiating Tasks ...", building_area_credencial);
                init_panda_tasks();
            }


        }else if (esp_mesh_is_root() && len_mqtt_content != 21) mqtt_app_publish(mqtt_topic, mqtt_content); // Se não for mensagem especial segue para publish


        free(mqtt_topic);
        free(mqtt_content);

        printf("CAMPO RX: %s \n", data.data); 
        
        if (!(recv_count % 1)) {
            ESP_LOGW(MESH_TAG,
                     "[#RX:%d/%d][L:%d] parent:"MACSTR", receive from "MACSTR", size:%d, heap:%d, flag:%d[err:0x%x, proto:%d, tos:%d]",
                     recv_count, send_count, mesh_layer,
                     MAC2STR(mesh_parent_addr.addr), MAC2STR(from.addr),
                     data.size, esp_get_minimum_free_heap_size(), flag, err, data.proto,
                     data.tos);
        }
    }
    vTaskDelete(NULL);
}

// Imprime todos os dispositvos conectados a ele.
void route_table_available()
{
    while (1)
    {   
        esp_mesh_get_routing_table((mesh_addr_t *) &route_table,
                                   CONFIG_MESH_ROUTE_TABLE_SIZE * 6, &route_table_size);

        ESP_LOGE(MESH_TAG, "[L:%d]parent:"MACSTR", heap:%d", mesh_layer, MAC2STR(mesh_parent_addr.addr), esp_get_minimum_free_heap_size());
        for (int i = 0; i < route_table_size; i++) 
        {
            ESP_LOGE(MESH_TAG, ""MACSTR"", MAC2STR(route_table[i].addr));
        }
        vTaskDelay(15000 / portTICK_RATE_MS);
    }
    
}

// Task de verificação com relação a credencial atribuida ao dispositivo. Executada a cada 1 min.
void check_connection(void *pvParameter)
{
    while (1)
    {   
        vTaskDelay(5000 / portTICK_RATE_MS);
        blink_command(2);
        ESP_LOGI("CREDENCIAL", "Checando credencial do dispositivo ... , %s", building_area_credencial);
        if (strlen(building_area_credencial) == 0 /*&& esp_mesh_is_root() == 0*/)
        {
            ESP_LOGI("CHECK", "Entrou no check_connection, %s", building_area_credencial);
            // A solicitação é enviada para o root processar com o servidor mqtt
            strcpy(any_string, ""); //Zerar string
            sprintf(any_string,"mac/%s",str_mac_base);
            send_data_mqtt_string(any_string,  "");
        
        } else ESP_LOGI("CREDENCIAL", "credencial OK, %s", building_area_credencial);
    
        vTaskDelay(115000 / portTICK_RATE_MS);
    }
    
}

esp_err_t esp_mesh_comm_p2p_start(void)
{
    static bool is_comm_p2p_started = false;

    if (!is_comm_p2p_started) {
        is_comm_p2p_started = true;
        vTaskDelay(2000 / portTICK_RATE_MS);
        ESP_LOGI("RX/TX","Initializing Tasks ...\n");
        xTaskCreate(esp_mesh_p2p_rx_main, "MPRX", 3072, NULL, 5, NULL);
        xTaskCreate(route_table_available, "TABLE", 2048, NULL, 5, NULL);
        xTaskCreate(check_connection, "CheckConnection", 2048, NULL, 5, NULL);

    }
    return ESP_OK;
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
        //mesh_disconnected_indicator();
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
        //mesh_connected_indicator(mesh_layer);
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
        ESP_LOGI(MESH_TAG, "unknown id:%d", event_id);
        break;
    }
}

void ip_event_handler(void *arg, esp_event_base_t event_base,
                      int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    current_ip = event->ip_info.ip;
    ESP_LOGI(MESH_TAG, "<IP_EVENT_STA_GOT_IP>IP:" IPSTR, IP2STR(&event->ip_info.ip));

    blink_command(4);

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
    init_blink_command();
    blink_command(5);
    
    if (esp_efuse_mac_get_default(mac_base) == ESP_OK)
    {
        sprintf(str_mac_base,"%02x%02x%02x%02x%02x%02x", mac_base[0],mac_base[1],mac_base[2],mac_base[3],mac_base[4],mac_base[5]);
        printf("\n str_mac_base: %s\n\n",str_mac_base);
    } else printf("\n ERROR READING MAC BASE ADRESS\n");

    semaphorBinarioA = xSemaphoreCreateBinary();
    semaphorBinarioB = xSemaphoreCreateBinary();


    // Configurar o resistor do Pull-up por segurança
    // gpio_pulldown_en(PIN_OUTPUT); // parece q esse PIN_OUTPUT é referente ao buzzer
    // gpio_pullup_dis(PIN_OUTPUT);
    // gpio_pulldown_en(PIN_INPUT_0);
    // gpio_pullup_dis(PIN_INPUT_0);
    // gpio_pulldown_en(PIN_INPUT_1);
    // gpio_pullup_dis(PIN_INPUT_1);
    // gpio_pulldown_en(PIN_INPUT_2);
    // gpio_pullup_dis(PIN_INPUT_2);

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
    ESP_LOGI(MESH_TAG, "mesh starts successfully, heap:%d, %s<%d>%s, ps:%d\n",  esp_get_minimum_free_heap_size(),
             esp_mesh_is_root_fixed() ? "root fixed" : "root not fixed",
             esp_mesh_get_topology(), esp_mesh_get_topology() ? "(chain)":"(tree)", esp_mesh_is_ps_enabled());

}
