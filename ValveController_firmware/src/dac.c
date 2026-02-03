#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "dac.h"



static inline uint8_t clamp_pd(uint8_t pd)    { return pd & 0x03; }
static inline uint16_t clamp_code(uint16_t c) { return c & 0x0FFF; }

static void* s_dac_handler = NULL;

// Fast mode write (C2=0,C1=0,C0=don't care): 2 bytes after address.
// Byte0 = [PD1 PD0 D11 D10 D9 D8], Byte1 = [D7..D0]. [file:1]
int mcp4725_write_fast(uint16_t code12, uint8_t pd_bits, int timeout_ms)
{

    code12 = clamp_code(code12);
    pd_bits = clamp_pd(pd_bits);

    uint8_t buf[2];
    buf[0] = (uint8_t)((pd_bits << 4) | ((code12 >> 8) & 0x0F));
    buf[1] = (uint8_t)(code12 & 0xFF);

    return i2c_master_transmit(s_dac_handler, buf, sizeof(buf), timeout_ms);
}

// Write DAC register + EEPROM (C2=0,C1=1,C0=1): 3 bytes after address. [file:1]
// Byte0 = [0 1 1 X X PD1 PD0 X]
// Byte1 = [D11..D4]
// Byte2 = [D3..D0 XXXX] [file:1]
int mcp4725_write_eeprom(uint16_t code12, uint8_t pd_bits, int timeout_ms)
{

    code12 = clamp_code(code12);
    pd_bits = clamp_pd(pd_bits);

    uint8_t buf[3];
    buf[0] = (uint8_t)((0b011 << 5) | (pd_bits << 1));  // command + PD bits [file:1]
    buf[1] = (uint8_t)(code12 >> 4);
    buf[2] = (uint8_t)((code12 & 0x0F) << 4);

    // EEPROM write takes up to ~50 ms; ensure timeout_ms is big enough. [file:1]
    return i2c_master_transmit(s_dac_handler, buf, sizeof(buf), timeout_ms);
}



void current_loop_output_task() {

    int current_loop_value; // 0-1 
    uint8_t dac_value;

    while (1) {

        if (xQueueReceive(current_loop_queue, &current_loop_value, portMAX_DELAY) == pdTRUE) {


            dac_value=current_loop_value; // should be calculation

            mcp4725_write_fast(dac_value,0x00,100);
        }

        
    }


}
// mcp4725_write_fast(i2c_periphery.dac,(uint16_t)(current_loop_value),0x00,100);


void current_loop_init() {

    current_loop_queue = xQueueCreate(1, sizeof(float));

}