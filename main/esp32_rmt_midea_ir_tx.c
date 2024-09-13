#include "esp32_rmt_midea_ir_tx.h"
#include "esp_log.h"
#include "esp_err.h"

#define PULSE_LENGTH 557; // Each pulse (one "time" unit or 1T) is 21 cycles of a 38KHz carrier burst (about 550µs long)

volatile uint8_t _rmt_midea_ir_tx_txPin = 18;
// volatile uint8_t _rmt_midea_ir_tx_channel;

rmt_channel_t _rmt_midea_ir_tx_channel = 0;
const uint16_t pulse_length = PULSE_LENGTH;

/*
    Convert a byte to its midea-protocol encoded counterpart as a RMT duration
    Idea borrowed from here: https://github.com/kimeckert/ESP32-RMT-Rx-raw
*/
void ConvertByteToMideaRmtValues(uint8_t byteValue, int16_t rmtValues[], int16_t rmtValuesIndex, int16_t *rmtCount)
{
    // A logic "1" is a 1T pulse followed by a 3T space and takes 2.25ms to transmit
    // A logic "0" is a 1T pulse followed by a 1T space and takes 1.125ms

    int16_t currentIndex = rmtValuesIndex;

    for (int8_t j = 7; j >= 0; j--)
    {
        currentIndex++; // avança para a posição de iniciar a inserção
        rmtValues[currentIndex] = 1*PULSE_LENGTH; // 1T pulse
        currentIndex++;

        int8_t currentBitValue = (byteValue >> j) & 1;
        if (currentBitValue == 1)
        {
            rmtValues[currentIndex] = -3*PULSE_LENGTH; // 3T space
        }
        else
        {
            rmtValues[currentIndex] = -1*PULSE_LENGTH; // 1T space
        }
    }
    *rmtCount = currentIndex;
}

/* Converts RMT durations to an array of rmt_item32_t */
void ConvertRmtValuesToRmtObjects(int16_t rmtValues[], int16_t rmtValuesCount, rmt_item32_t rmtObjects[], int16_t *rmtObjectsCount)
{
    int16_t rmtObjectsIndex = 0;
    bool isFirstPartOfRmtObjectOccupied = false;
    
    for (int16_t i = 0; i < rmtValuesCount; i++)
    {

        if (isFirstPartOfRmtObjectOccupied == false)
        {
            rmtObjects[rmtObjectsIndex].duration0 = abs(rmtValues[i]);
            rmtObjects[rmtObjectsIndex].level0 = (rmtValues[i] > 0) ? 0 : 1;
            isFirstPartOfRmtObjectOccupied = true;

            rmtObjects[rmtObjectsIndex].duration1 = 0; // Logo, duration1 e level1 não tem efeito
            rmtObjects[rmtObjectsIndex].level1 = 0;
        }
        else
        {
            rmtObjects[rmtObjectsIndex].duration1 = abs(rmtValues[i]);
            rmtObjects[rmtObjectsIndex].level1 = (rmtValues[i] > 0) ? 0 : 1;
            rmtObjectsIndex++;
            isFirstPartOfRmtObjectOccupied = false;
        }
    }
    *rmtObjectsCount = rmtObjectsIndex + 1;
}

// Add the bytes and their complements to the RmtArray 
void AddBytesToRmtArray(uint8_t data[], uint8_t lengthOfData, int16_t rmtValues[], int16_t rmtValuesIndex, int16_t *rmtCount)
{
    for (int i = 0; i < lengthOfData; i++) 
    {
        ESP_LOGI("\nCHECK1","Data: %x,%x", (int8_t)data[i], (int8_t)~data[i]);
    }
    printf("\n");
    int16_t rmtValueIndex = rmtValuesIndex;

    for (int8_t i = 0; i < lengthOfData; i++)
    {
        int8_t lean = (int8_t) data[i];
        ConvertByteToMideaRmtValues(lean, rmtValues, rmtValueIndex, &rmtValueIndex);
        int8_t flipped = (int8_t) ~data[i];
        ConvertByteToMideaRmtValues(flipped, rmtValues, rmtValueIndex, &rmtValueIndex);

        ESP_LOGI("CHECK2","Data: %x,%x", lean, flipped);
    }
    *rmtCount = rmtValueIndex;
}


static esp_err_t make_head(rmt_item32_t rmtObjects[], int16_t *rmtObjectsCount)
{
    uint16_t cursor = *rmtObjectsCount;
    rmtObjects[cursor].level0 = 0;
    rmtObjects[cursor].duration0 = (8*pulse_length);
    rmtObjects[cursor].level1 = 1;
    rmtObjects[cursor].duration1 = (8*pulse_length);
    *rmtObjectsCount = cursor+1;
    printf("\n[HEAD] rmtObjectsCount: %d\n, ",*rmtObjectsCount);
    return ESP_OK;
}

static esp_err_t make_bit0(rmt_item32_t rmtObjects[], int16_t *rmtObjectsCount)
{
    uint16_t cursor = *rmtObjectsCount;
    rmtObjects[cursor].level0 = 0;
    rmtObjects[cursor].duration0 = (pulse_length);
    rmtObjects[cursor].level1 = 1;
    rmtObjects[cursor].duration1 = (pulse_length);
    *rmtObjectsCount = cursor + 1;
    return ESP_OK;
}

static esp_err_t make_bit1(rmt_item32_t rmtObjects[], int16_t *rmtObjectsCount)
{
    uint16_t cursor = *rmtObjectsCount;
    rmtObjects[cursor].level0 = 0;
    rmtObjects[cursor].duration0 = (3*pulse_length);
    rmtObjects[cursor].level1 = 1;
    rmtObjects[cursor].duration1 = (pulse_length);
    *rmtObjectsCount = cursor + 1;
    return ESP_OK;
}

static esp_err_t make_end(rmt_item32_t rmtObjects[], int16_t *rmtObjectsCount)
{
    uint16_t cursor = *rmtObjectsCount;
    rmtObjects[cursor].level0 = 0;
    rmtObjects[cursor].duration0 = pulse_length;
    rmtObjects[cursor].level1 = 1;
    rmtObjects[cursor].duration1 = (8*pulse_length);
    *rmtObjectsCount = cursor + 1;
    // rmtObjects[cursor].val = 0;
    // *rmtObjectsCount = cursor + 1;
    printf("\n[END] rmtObjectsCount: %d\n, ",*rmtObjectsCount);
    return ESP_OK;
}

void encoder_protocol(uint8_t data[], uint8_t lengthOfData, rmt_item32_t rmtObjects[], int16_t *rmtObjectsCount)
{
    make_head(rmtObjects, rmtObjectsCount);

    for (int i = 0; i < lengthOfData; i++) ESP_LOGI("CHECK-encoder","Data: %.1x,%.1x", (int8_t)data[i], (int8_t)~data[i]);

    uint8_t byte_to_send;
    uint8_t mask;
    
    // Mask
    mask = 0x80;
    for (int8_t j = 0; j < lengthOfData; j++)
    {
        mask = 0x80;
        byte_to_send = data[j];
        // Emit the first Byte
        for (int i = 0;i < 8;i++) {
            (byte_to_send & mask)? make_bit1(rmtObjects, rmtObjectsCount) : make_bit0(rmtObjects, rmtObjectsCount);
            mask >>= 1; // => mask = mask >> 1;
        }

        // Neg bitwise this byte with his mask
        byte_to_send = ~byte_to_send;
        mask = 0x80;

        // And emit the same byte negate
        for (int i = 0;i < 8;i++) {
            (byte_to_send & mask) ? make_bit1(rmtObjects, rmtObjectsCount) : make_bit0(rmtObjects, rmtObjectsCount);
            mask >>= 1;
        }
    }
    
    make_end(rmtObjects, rmtObjectsCount);
    
}


/* Send the message using RMT */ // lengthOfData = n° de bytes recebendo
void rmt_midea_ir_tx_send_raw_message(uint8_t data[], uint8_t lengthOfData)
{
    //  [ffff ssss] [ttttcccc]
    // 0b0001_1111 = 0x1F-> FS - Auto/On
    // 0b0111_0000 = 0x70-> TC - 22°C/Cool
    // uint8_t datax[] = {0XB2, 0xF8, 0x1E};
    // uint8_t data[] = {};
    // data[0] = 0b10110010; //0xB2;
    // data[1] = 0b00011110; //0x1E;
    // data[2] = 0b11111000; //0xF8;

    for (int i = 0; i < lengthOfData; i++) ESP_LOGI("CHECK","Data: %.1x,%.1x", (int8_t)data[i], (int8_t)~data[i]);
    
    
    int16_t rmtValues[200] = { 0 };
    int16_t rmtValueIndex = 1; // &rmtValueIndex = endereço memória

    // #pragma region Add the whole message to the RmtArray for the first time

    // The AGC burst consists of a 4.5ms burst followed by 4.5ms space (8T pulse + 8T space)
    // frame start markers 4T down, 4T up
    rmtValues[0] = 8*PULSE_LENGTH;
    rmtValues[1] = -8*PULSE_LENGTH;

    // add the bytes and its complements to the RmtArray
    // uint8_t Komeco[] = {0xB2, 0xBF, 0x40};
    // uint8_t komeco[] = {0xB2, 0x1F, 0x58}; // Desligar COOL
    // uint8_t komeco[] = {0xB2, 0x7B, 0xE0}; // Desligar AUTO
    // uint8_t komeco[] = {0xB2, 0x7F, 0x58}; // Ligar AUTO
    // uint8_t komeco[] = {0xB2, 0xBF, 0x50}; // Ligar COOL
    // uint8_t midea[] = {0xB2, 0x1F, 0x58}; // Desligar COOL
    //{0xB2, 0x1F, 0x58};
    AddBytesToRmtArray(data/*komeco*/, lengthOfData, rmtValues, rmtValueIndex, &rmtValueIndex);
    
    // add the end of frame marker to the RmtArray (Jeito CERTO)
    rmtValueIndex++;
    rmtValues[rmtValueIndex] = PULSE_LENGTH;
    rmtValueIndex++;
    rmtValues[rmtValueIndex] = -10*PULSE_LENGTH;


    /*Jeito errado*/
    // rmtValueIndex++;
    // rmtValues[rmtValueIndex] = -PULSE_LENGTH;
    // rmtValueIndex++;
    // rmtValues[rmtValueIndex] = 10*PULSE_LENGTH;

    rmt_item32_t rmtObjects[200];
    int16_t rmtObjectsCount = 0;
    
    // encoder_protocol(data, lengthOfData, rmtObjects, &rmtObjectsCount);
    printf("rmtObjectsCount: %d\n", rmtObjectsCount);
    ConvertRmtValuesToRmtObjects(rmtValues, rmtValueIndex + 1, rmtObjects, &rmtObjectsCount);

    //
    printf("\n");
    ESP_LOGI("RMT OBJ","INIT");
    printf("level 1 \n");
    int i = 0; 
    for (i = 0; i < rmtObjectsCount; i++)
    {
        printf("%d, ",rmtObjects[i].duration1);
    }
    printf(" | count: %d\n ---------------------------------- \n", i);
    for (int i = 0; i < rmtObjectsCount; i++)
    {
        printf("%d, ",rmtObjects[i].level1);
    }

    printf("\n\n level 0 \n");

    for (i = 0; i < rmtObjectsCount; i++)
    {
        printf("%d, ",rmtObjects[i].duration0);
    }
    printf(" | count: %d\n ---------------------------------- \n", i);
    for (i = 0; i < rmtObjectsCount; i++)
    {
        printf("%d, ",rmtObjects[i].level0);
    }
    printf("\n");
    ESP_LOGI("RMT OBJ","END\n");
    //

    /*
    ESP_LOGI("RMT VALUES","INIT");
    for (i = 0; i < rmtValueIndex; i++)
    {
        printf("%d, ",rmtValues[i]);
    }
    ESP_LOGI("RMT VALUES","END -> Count: %d", i);
    */

    rmt_write_items(_rmt_midea_ir_tx_channel, rmtObjects, rmtObjectsCount, false);
    
}

/* Initialize RMT transmit channel */
void rmt_midea_ir_tx_channel_init(uint8_t channel, uint8_t txPin)
{
    _rmt_midea_ir_tx_channel = channel;
    _rmt_midea_ir_tx_txPin = txPin;
    rmt_config_t rmt_tx;

    rmt_tx.channel       = channel;
    rmt_tx.gpio_num      = txPin;
    rmt_tx.clk_div       = 80; // 1 MHz, 1 us - we set up sampling to every 1 microseconds
    rmt_tx.mem_block_num = 2;
    rmt_tx.rmt_mode      = RMT_MODE_TX;
    rmt_tx.tx_config.loop_en = false;
    rmt_tx.tx_config.carrier_duty_percent = 25;
    rmt_tx.tx_config.carrier_freq_hz = 38000;
    rmt_tx.tx_config.carrier_level   = RMT_CARRIER_LEVEL_LOW;
    rmt_tx.tx_config.carrier_en      = true;
    rmt_tx.tx_config.idle_level      = RMT_IDLE_LEVEL_HIGH;
    rmt_tx.tx_config.idle_output_en  = true;

    rmt_config(&rmt_tx);
    rmt_driver_install(rmt_tx.channel, 0, 0);

    // start the channel
    rmt_tx_start(_rmt_midea_ir_tx_channel, 1);
}


/* Uninstall the RMT driver */
void rmt_midea_ir_tx_channel_stop(uint8_t channel)
{
    rmt_driver_uninstall(channel);
}

/* add the end of frame marker to the RmtArray
rmtValueIndex++;
rmtValues[rmtValueIndex] = PULSE_LENGTH;
rmtValueIndex++;
rmtValues[rmtValueIndex] = 0;
rmtValueIndex++;
*/// #pragma endregion