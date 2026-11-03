#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h> // 🌟 เพิ่มเพื่อรองรับประเภทตัวแปร uint8_t, uint32_t

// ==========================================
// PIN UART (เชื่อมต่อกับ Raspberry Pi 5)
// ==========================================
#define UART_TX_PIN  0
#define UART_RX_PIN  1

// ==========================================
// PIN MOTOR (ESC โดรน) - อัปเดตตามจุดบัดกรีจริง
// ==========================================
#define PIN_ESC_RB   10 //esc2
#define PIN_ESC_LB   13 //esc4
#define PIN_ESC_RF   11 //esc1
#define PIN_ESC_LF   12 //esc3

// ==========================================
// PIN MOTOR DRIVER (TB6612 สำหรับโหมดรถ)
// ==========================================
// ตรงกับ schematic: GP14=CarA2, GP15=EN_A_Speed, GP16=CarA1, GP17=STBY, GP18=CarB1, GP19=CarB2, GP20=EN_B_Speed
#define TB6612_STANBY 17   // STBY
#define TB6612_ENA_A  15   // EN_A_Speed (PWM)
#define TB6612_ENA_B  20   // EN_B_Speed (PWM)
#define TB6612_IN1    16   // CarA1
#define TB6612_IN2    14   // CarA2
#define TB6612_IN3    18   // CarB1
#define TB6612_IN4    19   // CarB2

// ==========================================
// I2C Pins (MPU6050, BMP280 ใช้ Wire I2C0)
// ==========================================
#define I2C_SDA_PIN   8
#define I2C_SCL_PIN   9

// ==========================================
// LED STATUS (RGB 5mm Common Anode)
// ==========================================
#define LED_R_PIN 2
#define LED_G_PIN 3
#define LED_B_PIN 4

// ==========================================
// SENSORS & OTHERS
// ==========================================
#define BAT_devider   28 
#define BUZZER_PIN    5 // ขาสำหรับ Buzzer

// Switch for repuest PI 
#define ReqPI 26 

// ==========================================
// BATTERY (4S LiPo) & ARM INTERLOCK
// ==========================================
#define BATT_WARN_V        14.2f  // เตือนแบตต่ำ (LED + เสียง) ~3.55 V/เซลล์
#define BATT_MIN_ARM_V     14.3f  // ต่ำกว่านี้ไม่ยอม arm (ตอนมีแบตจริง; ไฟ USB < 5 V ข้ามการเช็ก)
#define ARM_MAX_TILT_DEG   15.0f  // เอียงเกินนี้ไม่ยอม arm
#define TILT_CUTOFF_DEG    21.0f  // ขณะ armed ถ้ามุม roll/pitch เกินนี้นาน 0.1 s จะตัดมอเตอร์
#define REQUIRE_GYRO_CAL   1      // 1 = ต้อง calibrate/โหลด gyro bias แล้วจึง arm โหมดโดรนได้

// magic สำหรับค่า trim มอเตอร์ (แยกจาก magic หลักเพื่อไม่ล้าง PID/offset/RGB)
#define MOTOR_TRIM_MAGIC   0x7A1B0C0Dul

// [เพิ่มใหม่] magic สำหรับค่าเพิ่มเติมของ Tinnakon
#define TIN_MAGIC  0x71A0C0DEul

// ==========================================
// EEPROM SETTINGS (โครงสร้างตัวแปรบันทึกค่า)
// ==========================================
struct System_Settings {
    float kp[7]; 
    float ki[7]; 
    float kd[7]; 
    float roll_offset;  
    float pitch_offset; 
    
    uint8_t rgb_car[3];
    uint8_t rgb_drone[3];
    uint8_t rgb_armed[3];
    uint8_t rgb_lowbatt[3];
    uint8_t rgb_failsafe[3];
    
    uint32_t magic_number; 

    int16_t  motor_trim_us[4];
    uint32_t trim_magic;

    // [เพิ่มใหม่] ค่าของ Tinnakon ที่ไม่มีบนหน้าเว็บ (เก็บใน EEPROM)
    float    tin_ka_alt;       // Ka_altitude
    float    tin_kp_yaw_ang;   // Kp_levelyaw (เก็บไว้ ยังไม่ใช้)
    float    tin_m_quad;       // น้ำหนักโดรน (kg)
    float    tin_tar;          // ค่ากรอง D (tar)
    uint32_t tin_magic;
};

extern System_Settings sys_settings;

#endif