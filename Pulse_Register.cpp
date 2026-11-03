#include "Pulse_Register.h"

static int dshot_offset_pio0 = -1;
static int dshot_offset_pio1 = -1;

Mypulse::Mypulse(uint pin) : _pin(pin) {
    _isDshot = false;
    _sliceNum = pwm_gpio_to_slice_num(_pin);
    _chan = pwm_gpio_to_channel(_pin);
}

void Mypulse::begin(float frequency) {
    _isDshot = false;
    _freq = frequency;
    
    gpio_set_function(_pin, GPIO_FUNC_PWM);
    float system_clock = clock_get_hz(clk_sys);
    float divider = 1.0f;
    uint32_t top = (uint32_t)(system_clock / (divider * _freq)) - 1;

    if (top > 65535) {
        divider = 64.0f; 
        top = (uint32_t)(system_clock / (divider * _freq)) - 1;
    }

    _top = top;
    pwm_set_clkdiv(_sliceNum, divider);
    pwm_set_wrap(_sliceNum, _top);
    pwm_set_enabled(_sliceNum, true);
}

void Mypulse::writeMicroseconds(float us) {
    if (_isDshot) return; 
    float period_us = 1000000.0f / _freq;
    uint32_t level = (uint32_t)((us / period_us) * (_top + 1));
    if (level > _top) level = _top;
    pwm_set_chan_level(_sliceNum, _chan, level);
}

void Mypulse::writeDuty(float percent) {
    if (_isDshot) return;
    uint32_t level = (uint32_t)((percent / 100.0f) * (_top + 1));
    pwm_set_chan_level(_sliceNum, _chan, level);
}

// ==========================================
// DSHOT300 (Force LOW Initial State)
// ==========================================
void Mypulse::beginDshot300() {
    _isDshot = true;
    
    _pio = pio0;
    int sm = pio_claim_unused_sm(_pio, false);
    if (sm < 0) {
        _pio = pio1;
        sm = pio_claim_unused_sm(_pio, true);
    }
    _sm = sm;

    uint16_t instructions[] = {
        pio_encode_set(pio_pins, 0),                     
        pio_encode_pull(false, true),                      
        pio_encode_set(pio_x, 15),                          
        // loop:
        pio_encode_set(pio_pins, 1) | pio_encode_delay(15), 
        pio_encode_out(pio_y, 1),                           
        pio_encode_jmp_not_y(8),                            
        // bit_1:
        pio_encode_set(pio_pins, 1) | pio_encode_delay(17),
        pio_encode_jmp(9),                                  
        // bit_0:
        pio_encode_set(pio_pins, 0) | pio_encode_delay(17), 
        // end_bit:
        pio_encode_set(pio_pins, 0) | pio_encode_delay(12), 
        pio_encode_jmp_x_dec(3)                            
    };

    struct pio_program dshot_prog = {
        .instructions = instructions,
        .length = 11, // ความยาวเพิ่มเป็น 11
        .origin = -1, 
    };

    uint offset;
    if (_pio == pio0) {
        if (dshot_offset_pio0 < 0) dshot_offset_pio0 = pio_add_program(_pio, &dshot_prog);
        offset = dshot_offset_pio0;
    } else {
        if (dshot_offset_pio1 < 0) dshot_offset_pio1 = pio_add_program(_pio, &dshot_prog);
        offset = dshot_offset_pio1;
    }

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset, offset + 10); // อัปเดตจุด Wrap เป็นบรรทัด 10
    sm_config_set_set_pins(&c, _pin, 1);
    sm_config_set_out_shift(&c, false, false, 32); 

    // หารนาฬิกาให้เหลือ 15 MHz
    float div = (float)clock_get_hz(clk_sys) / 15000000.0f; 
    sm_config_set_clkdiv(&c, div);

    pio_gpio_init(_pio, _pin);
    pio_sm_set_consecutive_pindirs(_pio, _sm, _pin, 1, true);
    
    pio_sm_init(_pio, _sm, offset, &c);
    pio_sm_set_enabled(_pio, _sm, true);
}

void Mypulse::writeDshot(uint16_t value, bool telemetry) {
    if (!_isDshot) return;
    if (value > 2047) value = 2047; 
    
    uint16_t packet = (value << 1) | (telemetry ? 1 : 0);
    uint16_t csum = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
    packet = (packet << 4) | csum;
    
    // ตรวจสอบคิวข้อมูล ถ้าว่างถึงค่อยยัดลงไป ป้องกัน Pico ค้าง
    if (!pio_sm_is_tx_fifo_full(_pio, _sm)) {
        pio_sm_put(_pio, _sm, ((uint32_t)packet) << 16);
    }
}