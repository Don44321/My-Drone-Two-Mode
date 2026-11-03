#include "Comm.h"
#include "sensor.h" 
#include "PID.h" 
#include <HardwareSerial.h>
#include "hardware/watchdog.h" 
#include <EEPROM.h> // 🌟 เพิ่มเพื่อใช้บันทึกสี

extern float accX_G;
extern float accY_G;
extern float velX;
extern float velY;

uint8_t sys_mode = 0;       
uint8_t sys_armed = 0;      
int16_t rx_roll = 0;        
int16_t rx_pitch = 0;       
int16_t rx_yaw = 0;         
uint16_t rx_target_alt = 0; 
uint8_t rx_throttle_mode = 0; 
uint16_t rx_limit_val = 150;  
float batteryVoltage = 0.0f;  
bool is_landed_flag = true; 

bool req_send_pid = false;
volatile bool req_send_offset = false; 

// สถานะ calibrate / การปฏิเสธ arm (ส่งกลับไปให้เว็บรู้)
volatile bool req_send_calib = false;
volatile bool calib_ok = false;
volatile bool calib_requested = false;   // ตั้งโดย Core 1 (UART) / ประมวลผลโดย Core 0
bool req_send_arm_reject = false;
uint8_t arm_reject_reason = 0;
static uint8_t prev_req_armed = 0;
static uint8_t prev_req_mode = 0;

// [เพิ่มใหม่] ธงขอส่งค่า trim มอเตอร์กลับไปให้ Pi
static volatile bool req_send_trim = false;

uint8_t audio_tone_test = 0;
uint8_t audio_cfg_armed = 1, audio_cfg_lowbatt = 3, audio_cfg_trans = 2, audio_cfg_failsafe = 1;

unsigned long last_rx_time = 0; 
String inputString = "";
unsigned long prevDebugTime = 0;

void Comm_init() {
    Serial1.setRX(1);
    Serial1.setTX(0);
    Serial1.begin(115200);
    Serial.begin(115200); 
}

// คืน 0 = arm ได้, อื่น ๆ = เหตุผลที่ปฏิเสธ (Pi แปลเป็นข้อความให้ผู้ใช้)
static uint8_t check_Arm_Interlocks() {
    if (batteryVoltage > 5.0f && batteryVoltage < BATT_MIN_ARM_V) return 1;                 // แบตต่ำ (ไฟ USB < 5 V ข้าม)
    if (fabsf(kalAngleRoll) > ARM_MAX_TILT_DEG || fabsf(kalAnglePitch) > ARM_MAX_TILT_DEG) return 2; // เอียงเกิน
    if (mpu_fail_streak > 2) return 3;                                                      // อ่าน MPU ไม่ได้
#if REQUIRE_GYRO_CAL
    if (!gyro_calibrated) return 4;                                                         // ยังไม่ calibrate
#endif
    if (rx_throttle_mode == 0 && rx_target_alt > 0) return 5;                               // คันเร่งมือต้องเป็น 0
    if (!is_landed_flag) return 6;                                                          // ยังไม่ลงจอดสนิท
    return 0;
}

void read_UART_Commands() {
    while (Serial1.available() > 0) {
        uint8_t header = Serial1.peek(); 
        
        if (header == 0xDD) {
            delay(2);
            if (Serial1.available() >= 2) {
                uint8_t b2 = Serial1.read();
                uint8_t b3 = Serial1.read();
                if (b2 == 0xDD && b3 == 0xDD) {
                    Serial.println("♻️ REBOOT COMMAND RECEIVED! REBOOTING...");
                    watchdog_reboot(0, 0, 10); 
                    while (true) {} 
                }
            }
        }
        else if (header == 0xAA && Serial1.available() >= 15) {
            Serial1.read(); 
            uint8_t buffer[14];
            Serial1.readBytes(buffer, 14);
            uint8_t checksum = 0xAA;
            for (int i = 0; i < 13; i++) checksum += buffer[i];
            
            if (checksum == buffer[13]) {
                uint8_t  req_mode  = buffer[0];
                uint8_t  req_armed = buffer[1];
                int16_t  r_roll  = (int16_t)((buffer[3] << 8) | buffer[2]);
                int16_t  r_pitch = (int16_t)((buffer[5] << 8) | buffer[4]);
                int16_t  r_yaw   = (int16_t)((buffer[7] << 8) | buffer[6]);
                uint16_t r_alt   = (uint16_t)((buffer[9] << 8) | buffer[8]);
                uint8_t  r_tmode = buffer[10];
                uint16_t r_lim   = (uint16_t)((buffer[12] << 8) | buffer[11]);

                // จำกัดค่า ไม่เชื่อค่าจากภายนอก (โดรน: องศา / รถ: กำลังมอเตอร์)
                int16_t lim = (req_mode == 1) ? 35 : 300;
                sys_mode = (req_mode == 1) ? 1 : 0;
                rx_roll  = constrain(r_roll,  -lim, lim);
                rx_pitch = constrain(r_pitch, -lim, lim);
                rx_yaw   = constrain(r_yaw, -200, 200);
                rx_target_alt    = (r_alt > 100) ? 100 : r_alt;
                rx_throttle_mode = (r_tmode > 3) ? 0 : r_tmode;
                rx_limit_val     = (r_lim > 300) ? 300 : r_lim;

                // ตรวจเงื่อนไข arm เฉพาะตอน "ขอ arm ใหม่" ในโหมดโดรน
                if (req_mode != 1) {
                    sys_armed = (req_armed == 1) ? 1 : 0;                       // โหมดรถ: เหมือนเดิม
                } else if (req_armed == 0) {
                    sys_armed = 0;
                } else if (prev_req_armed == 0 || prev_req_mode != 1) {
                    uint8_t why = check_Arm_Interlocks();
                    if (why == 0) {
                        sys_armed = 1;
                    } else {
                        sys_armed = 0;
                        arm_reject_reason = why;
                        req_send_arm_reject = true;
                    }
                }
                // req_armed == 1 ต่อเนื่อง: คง sys_armed เดิมไว้ (ถ้า Pico ปลดเองแล้ว จะไม่ arm ซ้ำจนกว่าเว็บจะ disarm ก่อน)
                prev_req_armed = req_armed;
                prev_req_mode  = req_mode;
                last_rx_time = millis(); 
            }
        }
        else if (header == 0xBB && Serial1.available() >= 15) { 
            Serial1.read(); 
            uint8_t buffer[14];
            Serial1.readBytes(buffer, 14);
            uint8_t checksum = 0xBB;
            for (int i = 0; i < 13; i++) checksum += buffer[i];
            
            if (checksum == buffer[13]) {
                uint8_t target_id = buffer[0];
                float kp, ki, kd;
                memcpy(&kp, &buffer[1], 4);
                memcpy(&ki, &buffer[5], 4);
                memcpy(&kd, &buffer[9], 4);
                if (!(sys_mode == 1 && sys_armed == 1)) save_PID_to_EEPROM(target_id, kp, ki, kd);   // เขียน flash หยุดอีกคอร์: ห้ามตอนบิน
            }
        }
        else if (header == 0xBC && Serial1.available() >= 3) {
            Serial1.read();
            uint8_t buffer[2];
            Serial1.readBytes(buffer, 2);
            if (((0xBC + buffer[0]) & 0xFF) == buffer[1]) {
                req_send_pid = true; 
            }
        }
        else if (header == 0xBD && Serial1.available() >= 3) {
            Serial1.read();
            uint8_t buffer[2];
            Serial1.readBytes(buffer, 2);
            if (((0xBD + buffer[0]) & 0xFF) == buffer[1]) {
                watchdog_reboot(0, 0, 10); 
            }
        }
        else if (header == 0xBE && Serial1.available() >= 3) {
            Serial1.read();
            uint8_t buffer[2];
            Serial1.readBytes(buffer, 2);
            if (((0xBE + buffer[0]) & 0xFF) == buffer[1]) {
                audio_tone_test = buffer[0]; 
            }
        }
        else if (header == 0xBF && Serial1.available() >= 7) {
            Serial1.read();
            uint8_t buffer[6];
            Serial1.readBytes(buffer, 6);
            uint8_t checksum = 0xBF;
            for (int i = 0; i < 5; i++) checksum += buffer[i];
            if (checksum == buffer[5]) {
                audio_cfg_armed = buffer[0];
                audio_cfg_lowbatt = buffer[1];
                audio_cfg_trans = buffer[2];
                audio_cfg_failsafe = buffer[3];
            }
        }
        // 🌟 เพิ่มการดักจับกล่อง 0xFC (17 Bytes) สำหรับสี RGB
        else if (header == 0xFC && Serial1.available() >= 17) {
            Serial1.read();
            uint8_t buffer[16];
            Serial1.readBytes(buffer, 16);
            uint8_t checksum = 0xFC;
            for (int i = 0; i < 15; i++) checksum += buffer[i];
            
            if (checksum == buffer[15] && !(sys_mode == 1 && sys_armed == 1)) {
                memcpy(sys_settings.rgb_car, &buffer[0], 3);
                memcpy(sys_settings.rgb_drone, &buffer[3], 3);
                memcpy(sys_settings.rgb_armed, &buffer[6], 3);
                memcpy(sys_settings.rgb_lowbatt, &buffer[9], 3);
                memcpy(sys_settings.rgb_failsafe, &buffer[12], 3);
                
                EEPROM.put(0, sys_settings);
                EEPROM.commit();
                Serial.println("💾 Saved RGB LED Config to EEPROM");
            }
        }
        else if (header == 0xEE && Serial1.available() >= 10) {
            Serial1.read();
            uint8_t buffer[9];
            Serial1.readBytes(buffer, 9);
            uint8_t checksum = 0xEE;
            for (int i = 0; i < 8; i++) checksum += buffer[i];
            
            if (checksum == buffer[8] && !(sys_mode == 1 && sys_armed == 1)) {
                float r, p;
                memcpy(&r, &buffer[0], 4);
                memcpy(&p, &buffer[4], 4);
                save_Offset_to_EEPROM(r, p); 
                req_send_offset = true; 
            }
        }
        else if (header == 0xEF && Serial1.available() >= 3) {
            Serial1.read();
            uint8_t buffer[2];
            Serial1.readBytes(buffer, 2);
            if (((0xEF + buffer[0]) & 0xFF) == buffer[1]) {
                calib_requested = true;   // อย่าเรียก calibrate ที่นี่ (Core 1 ห้ามใช้ I2C) ให้ Core 0 ทำใน loop()
            }
        }
        else if (header == 0xED && Serial1.available() >= 3) {
            Serial1.read();
            uint8_t buffer[2];
            Serial1.readBytes(buffer, 2);
            if (((0xED + buffer[0]) & 0xFF) == buffer[1]) {
                req_send_offset = true; 
            }
        }
        // [เพิ่มใหม่] 0xF0: ตั้งค่า trim มอเตอร์ 4 ตัว (int16 x4 = LF, LB, RB, RF) + checksum รวม 10 ไบต์
        else if (header == 0xF0 && Serial1.available() >= 10) {
            Serial1.read();
            uint8_t buffer[9];
            Serial1.readBytes(buffer, 9);
            uint8_t checksum = 0xF0;
            for (int i = 0; i < 8; i++) checksum += buffer[i];

            if (checksum == buffer[8] && !(sys_mode == 1 && sys_armed == 1)) {
                for (int i = 0; i < 4; i++) {
                    int16_t v;
                    memcpy(&v, &buffer[i * 2], 2);
                    sys_settings.motor_trim_us[i] = constrain(v, -50, 50);
                }
                sys_settings.trim_magic = MOTOR_TRIM_MAGIC;
                EEPROM.put(0, sys_settings);
                EEPROM.commit();
                Serial.println("💾 Saved Motor Trim to EEPROM");
                req_send_trim = true;   // ส่งค่าที่บันทึกแล้วกลับไปให้หน้าเว็บยืนยัน
            }
        }
        // [เพิ่มใหม่] 0xF2: ขออ่านค่า trim มอเตอร์ (3 ไบต์)
        else if (header == 0xF2 && Serial1.available() >= 3) {
            Serial1.read();
            uint8_t buffer[2];
            Serial1.readBytes(buffer, 2);
            if (((0xF2 + buffer[0]) & 0xFF) == buffer[1]) {
                req_send_trim = true;
            }
        }
        else if (header != 0xAA && header != 0xBB && header != 0xBC && header != 0xBD && header != 0xBE && header != 0xBF && header != 0xFC && header != 0xEE && header != 0xEF && header != 0xED && header != 0xF0 && header != 0xF2) {
            Serial1.read(); 
        }
        else { break; }
    }
}

void send_PID_Telemetry() {
    uint8_t packet[74];
    packet[0] = 0xCC; 
    
    int offset = 1;
    for (int i = 1; i <= 6; i++) {
        memcpy(&packet[offset], &sys_settings.kp[i], 4); offset += 4;
        memcpy(&packet[offset], &sys_settings.ki[i], 4); offset += 4;
        memcpy(&packet[offset], &sys_settings.kd[i], 4); offset += 4;
    }
    
    uint8_t csum = 0;
    for(int i = 0; i < 73; i++) {
        csum += packet[i];
    }
    packet[73] = csum;
    
    Serial1.write(packet, 74);
}

void send_Offset_Telemetry() {
    uint8_t packet[10];
    packet[0] = 0xEE; 
    memcpy(&packet[1], &base_roll_offset, 4);
    memcpy(&packet[5], &base_pitch_offset, 4);
    
    uint8_t csum = 0;
    for(int i = 0; i < 9; i++) {
        csum += packet[i];
    }
    packet[9] = csum;
    
    Serial1.write(packet, 10);
}

// [เพิ่มใหม่] ส่งค่า trim มอเตอร์กลับไป Pi: [0xF3][LF lo,hi][LB lo,hi][RB lo,hi][RF lo,hi][checksum] = 10 ไบต์
void send_Trim_Telemetry() {
    uint8_t packet[10];
    packet[0] = 0xF3;
    for (int i = 0; i < 4; i++) {
        int16_t v = sys_settings.motor_trim_us[i];
        memcpy(&packet[1 + i * 2], &v, 2);
    }

    uint8_t csum = 0;
    for (int i = 0; i < 9; i++) csum += packet[i];
    packet[9] = csum;

    Serial1.write(packet, 10);
}

void send_Telemetry() {
    if (req_send_pid) {
        send_PID_Telemetry();
        req_send_pid = false;
        return; 
    }
    
    if (req_send_calib) {
        uint8_t p[3] = {0xEC, (uint8_t)(calib_ok ? 1 : 0), 0};
        p[2] = (uint8_t)(p[0] + p[1]);
        Serial1.write(p, 3);
        req_send_calib = false;
        return;
    }

    if (req_send_arm_reject) {
        uint8_t p[3] = {0xEB, arm_reject_reason, 0};
        p[2] = (uint8_t)(p[0] + p[1]);
        Serial1.write(p, 3);
        req_send_arm_reject = false;
        return;
    }

    if (req_send_offset) {
        send_Offset_Telemetry();
        req_send_offset = false;
        return;
    }

    // [เพิ่มใหม่] ส่งค่า trim เมื่อถูกขอหรือหลังบันทึก
    if (req_send_trim) {
        send_Trim_Telemetry();
        req_send_trim = false;
        return;
    }

    uint8_t vbat_int = (uint8_t)batteryVoltage; 
    uint8_t vbat_dec = (uint8_t)((batteryVoltage - vbat_int) * 100); 
    int16_t out_pitch = (int16_t)(kalAnglePitch * 100.0f);   // หน่วย 0.01 องศา (Pi หาร 100)
    int16_t out_roll  = (int16_t)(kalAngleRoll * 100.0f);
    int16_t out_alt = (int16_t)(fused_altitude * 100); 
    
    uint8_t header = is_landed_flag ? 0xAB : 0xAA;
    uint8_t packet[22]; 
    packet[0] = header; 
    packet[1] = vbat_int;
    packet[2] = vbat_dec;
    packet[3] = out_alt & 0xFF;
    packet[4] = (out_alt >> 8) & 0xFF;
    packet[5] = out_pitch & 0xFF;
    packet[6] = (out_pitch >> 8) & 0xFF;
    packet[7] = out_roll & 0xFF;
    packet[8] = (out_roll >> 8) & 0xFF;
    
    int16_t ax_send = (int16_t)(accX_G * 100);
    packet[9] = ax_send & 0xFF;
    packet[10] = (ax_send >> 8) & 0xFF;
    int16_t ay_send = (int16_t)(accY_G * 100);
    packet[11] = ay_send & 0xFF;
    packet[12] = (ay_send >> 8) & 0xFF;
    int16_t vx_send = (int16_t)(velX * 100);
    packet[13] = vx_send & 0xFF;
    packet[14] = (vx_send >> 8) & 0xFF;
    int16_t vy_send = (int16_t)(velY * 100);
    packet[15] = vy_send & 0xFF;
    packet[16] = (vy_send >> 8) & 0xFF;

    // สถานะ ToF ส่งไปแสดงบนหน้าเว็บ: [17] แฟล็ก bit0=tof_ok bit1=valid bit2=ใช้ ToF เป็นแหล่งความสูง, [18-19] ค่าดิบ mm, [20] สถานะการวัด
    packet[17] = (tof_ok ? 1 : 0) | (tof_is_valid ? 2 : 0) | (using_tof ? 4 : 0);
    packet[18] = tof_dbg_raw_mm & 0xFF;
    packet[19] = (tof_dbg_raw_mm >> 8) & 0xFF;
    packet[20] = tof_dbg_status;
    
    uint8_t checksum = 0;
    for (int i = 0; i < 21; i++) checksum += packet[i];
    packet[21] = checksum; 
    
    Serial1.write(packet, 22); 
}

void check_Failsafe() {
    if (millis() - last_rx_time > 500) { 
        if (sys_mode == 1 && sys_armed == 1 && rx_throttle_mode != 3) {
            rx_throttle_mode = 3;
            rx_pitch = 0;
            rx_roll = 0;
            rx_yaw = 0;
        } else if (sys_mode == 0) {
            sys_armed = 0; 
            rx_pitch = 0; rx_roll = 0; rx_yaw = 0;   // โหมดรถ: Pi หาย -> หยุดรถทันที (เดิมค้างคำสั่งเดิมไว้)
            is_landed_flag = true; 
        }
    }
}
void read_USB_Debug_Commands() { /* คงเดิม */ }
void print_Debug_Info() { /* คงเดิม */ }