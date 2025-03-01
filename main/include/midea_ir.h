#pragma once

#ifndef _midea_ir_h
    #define _midea_ir_h

    #ifdef __cplusplus
    extern "C" {
    #endif

    #include <stdint.h>
    #include <stdbool.h>

    typedef enum {
        MODE_COOL       = 0b0000,
        MODE_HEAT       = 0b1100,
        MODE_AUTO       = 0b1000,
        MODE_FAN        = 0b0100,
        MODE_VENTILATE  = 0b1111,
    } MideaMode;

    typedef struct {
        uint8_t temperature; // in Celsius
        uint8_t fan_level;   // 0..3
        MideaMode mode;
        bool enabled;        // on/off air conditioner
    } MideaIR;

    typedef struct
    {
        uint8_t magic;      // 0xB2 always
        uint8_t state   : 4;
        uint8_t fan     : 4;
        uint8_t command : 4;
        uint8_t temp    : 4;
    } DataPacketStruct;

    typedef struct {
        DataPacketStruct data;
        uint8_t DataAsByteArray[sizeof(DataPacketStruct)];
    } DataPacket;
    /**
     * Initialize Ir module and ir structure with default values
     */
    void midea_ir_init(MideaIR* ir, const uint8_t channel, const uint8_t txPin);

    /**
     * Send Ir signal to air conditioner
     */
    void midea_ir_send(MideaIR *ir);

    /**
     * Send Ir signal to move deflector
     */

    void pack_data(MideaIR *ir, DataPacketStruct *data);

    void midea_ir_move_deflector();

    void midea_ir_oscilate();

    void midea_ir_turbo_mode();

    void midea_ir_ionize_mode();

    void midea_ir_clean_device();

    void midea_ir_no_sound();

    void komeco_ir_off();

    void komeco_ir_on();

    /**
     * Stop Ir module
     */
    void midea_ir_stop(uint8_t channel);

    #ifdef __cplusplus
    } // extern C
    #endif
#endif