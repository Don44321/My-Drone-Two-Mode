#ifndef PULSE_REGISTER_H
#define PULSE_REGISTER_H

#include <Arduino.h>
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "hardware/pio_instructions.h" // เพิ่มไลบรารีสำหรับสร้างชุดคำสั่ง PIO

class Mypulse {
public:
    // กำหนดขา GPIO ที่ต้องการ
    Mypulse(uint pin);

    // ==========================================
    // โหมดเดิม (Analog PWM / OneShot125)
    // ==========================================
    void begin(float frequency);
    void writeMicroseconds(float us);
    void writeDuty(float percent);

    // ==========================================
    // โหมดใหม่ (Digital DSHOT300)
    // ==========================================
    void beginDshot300();
    
    // ส่งข้อมูลดิจิทัล (value = 0 ถึง 2047, telemetry = ขอข้อมูลย้อนกลับหรือไม่)
    void writeDshot(uint16_t value, bool telemetry = false);

private:
    uint _pin;
    
    // ตัวแปรสำหรับ PWM เดิม
    uint _sliceNum;
    uint _chan;
    float _freq;
    uint32_t _top;

    // ตัวแปรสำหรับ DSHOT (PIO)
    bool _isDshot;
    PIO _pio;
    uint _sm;
};

#endif