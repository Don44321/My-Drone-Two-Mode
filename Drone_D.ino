#include <Arduino.h>
#include "hardware/watchdog.h" 
#include "Config.h"
#include "sensor.h"
#include "PID.h"
#include "Comm.h"       
#include "CarControl.h" 
#include "Tin_Control.h"   // [เพิ่มใหม่]

#ifndef TILT_CUTOFF_DEG
#define TILT_CUTOFF_DEG 70.0f
#endif

volatile bool core0_ready = false;

extern unsigned long last_rx_time;

float accX_G = 0.0;
float accY_G = 0.0;
float accZ_G = 0.0;     // [เพิ่มใหม่] mpu6050.cpp อ่านค่าไปใช้
float velX = 0.0;
float velY = 0.0;

unsigned long prevLoopTime = 0;
const unsigned long LOOP_TIME_US = 2500;
unsigned long prevTelemetryTime = 0;
unsigned long prevLedTime = 0;
bool led2State = false;

float auto_target_alt = 0.0f;

unsigned long land_detect_timer = 0;

unsigned long prevAudioTime = 0;
int audio_dshot_counter = 0; 
uint8_t active_tone = 0; 

uint8_t prev_sys_mode = 0;
int trans_step = 0;
unsigned long trans_timer = 0;

float peak_altitude = 0.0f;
uint8_t prev_rx_throttle_mode = 0;
unsigned long failsafe_start_timer = 0;
unsigned long tilt_exceed_t = 0;   

#define LOOP_TIMING_DEBUG 0

unsigned long btn_press_time = 0, btn_release_time = 0;
int click_count = 0;
bool btn_state_prev = HIGH, is_pressing = false;

void check_ReqPI_Button() {
    bool current_state = digitalRead(ReqPI);
    unsigned long now = millis();
    
    if (current_state == LOW && btn_state_prev == HIGH) { 
        btn_press_time = now; 
        is_pressing = true;
    } else if (current_state == HIGH && btn_state_prev == LOW) { 
        is_pressing = false;
        if (now - btn_press_time >= 2000) {
            uint8_t pkt[3] = {0xEA, 1, (uint8_t)((0xEA + 1) & 0xFF)}; 
            Serial1.write(pkt, 3); 
            click_count = 0;
        } else if (now - btn_press_time > 50) { 
            click_count++; 
            btn_release_time = now;
        }
    }
    
    if (!is_pressing && click_count > 0 && (now - btn_release_time > 400)) {
        if (click_count == 2) {
            uint8_t pkt[3] = {0xEA, 2, (uint8_t)((0xEA + 2) & 0xFF)}; 
            Serial1.write(pkt, 3); 
        }
        click_count = 0;
    }
    btn_state_prev = current_state;
}

void set_RGB_Color(uint8_t r, uint8_t g, uint8_t b) {
    analogWrite(LED_R_PIN, 255 - r);
    analogWrite(LED_G_PIN, 255 - g);
    analogWrite(LED_B_PIN, 255 - b);
}

void update_LEDs() {
    bool rpi_offline = (millis() - last_rx_time > 500);

    if (rpi_offline) {
        if (millis() % 500 < 250) set_RGB_Color(sys_settings.rgb_failsafe[0], sys_settings.rgb_failsafe[1], sys_settings.rgb_failsafe[2]);
        else set_RGB_Color(0, 0, 0);
    }
    else if (batteryVoltage > 5.0f && batteryVoltage < BATT_WARN_V) {
        if (millis() % 1000 < 500) set_RGB_Color(sys_settings.rgb_lowbatt[0], sys_settings.rgb_lowbatt[1], sys_settings.rgb_lowbatt[2]);
        else set_RGB_Color(0, 0, 0);
    }
    else if (sys_armed == 1) {
        if (millis() % 200 < 100) set_RGB_Color(sys_settings.rgb_armed[0], sys_settings.rgb_armed[1], sys_settings.rgb_armed[2]);
        else set_RGB_Color(0, 0, 0);
    }
    else if (sys_mode == 1) {
        set_RGB_Color(sys_settings.rgb_drone[0], sys_settings.rgb_drone[1], sys_settings.rgb_drone[2]); 
    }
    else {
        set_RGB_Color(sys_settings.rgb_car[0], sys_settings.rgb_car[1], sys_settings.rgb_car[2]); 
    }
}

void setup() {
    Car_init();

    pinMode(LED_R_PIN, OUTPUT);
    pinMode(LED_G_PIN, OUTPUT);
    pinMode(LED_B_PIN, OUTPUT);
    digitalWrite(LED_R_PIN, HIGH);
    digitalWrite(LED_G_PIN, HIGH);
    digitalWrite(LED_B_PIN, HIGH);

    pinMode(BUZZER_PIN, OUTPUT);

    analogReadResolution(12);
    pinMode(ReqPI, INPUT_PULLUP); 

    setup_MPU6050();
    InitSensor();     

    PID_init(); 
    load_Gyro_Bias_from_EEPROM();   
    load_ToF_Offset_from_EEPROM();  

    if (digitalRead(ReqPI) == LOW) ESC_calibration_mode();
    
    batteryVoltage = (analogRead(BAT_devider) * 3.3f / 4095.0f) * 5.999f; 
    watchdog_enable(500, 1);
    prevLoopTime = micros();
    core0_ready = true;
}

void loop() {
    watchdog_update();

    if (calib_requested) {
        calib_requested = false;
        calib_ok = calibrate_MPU6050_manual();
        req_send_calib = true;
        req_send_offset = true;
        prevLoopTime = micros();
    }
    check_ReqPI_Button(); 
    
    unsigned long currentMicros = micros();
    update_LEDs(); 

    if (currentMicros - prevLoopTime >= LOOP_TIME_US) {
        float dt = (currentMicros - prevLoopTime) / 1000000.0f;
        prevLoopTime = currentMicros;

        if (dt > 0.1f) dt = 0.0025f; 

        check_Failsafe();

        get_MPU_data(dt);
        ReadSensor(dt);

        float raw_batt = (analogRead(BAT_devider) * 3.3f / 4095.0f) * 5.999f; 
        batteryVoltage = (batteryVoltage * 0.99f) + (raw_batt * 0.01f);

        if (sys_mode != prev_sys_mode) {
            prev_sys_mode = sys_mode;
            trans_step = 1; 
            trans_timer = millis();
        }

        bool rpi_offline = (millis() - last_rx_time > 500);

        if (rpi_offline && sys_armed == 1) {
            if (failsafe_start_timer == 0) failsafe_start_timer = millis();
            if (millis() - failsafe_start_timer > 15000) { sys_armed = 0; }
        } else {
            failsafe_start_timer = 0;
        }

        if (rpi_offline && is_landed_flag) {
            if ((millis() % 6000) < 3000) { active_tone = audio_cfg_failsafe; } 
            else { active_tone = 0; }
            audio_dshot_counter = 0;
        }
        else if (audio_tone_test > 0) { 
            active_tone = audio_tone_test; 
            audio_dshot_counter++;
            if (audio_dshot_counter > 200) { 
                audio_tone_test = 0; audio_dshot_counter = 0; active_tone = 0;
            }
        }
        else if (trans_step > 0) {
            unsigned long elapsed = millis() - trans_timer;
            if (elapsed < 150) active_tone = (sys_mode == 1) ? 1 : 5; 
            else if (elapsed < 300) active_tone = 3;                       
            else if (elapsed < 450) active_tone = (sys_mode == 1) ? 5 : 1; 
            else { active_tone = 0; trans_step = 0; }
        }
        else if (sys_mode == 1 && sys_armed == 1 && is_landed_flag) { 
            if (millis() % 1000 < 300) active_tone = audio_cfg_armed;
            else active_tone = 0;
            audio_dshot_counter = 0;
        }
        else if (batteryVoltage < BATT_WARN_V && sys_armed == 1) { 
            if (millis() % 2000 < 500) active_tone = audio_cfg_lowbatt;
            else active_tone = 0;
            audio_dshot_counter = 0;
        }
        else { active_tone = 0; audio_dshot_counter = 0; } 
        
        if (!is_landed_flag && active_tone != audio_cfg_lowbatt) { active_tone = 0; }

        if (active_tone > 0) {
            int freq_map[] = {0, 600, 800, 1000, 1200, 1400}; 
            if (active_tone <= 5) tone(BUZZER_PIN, freq_map[active_tone]);
            else tone(BUZZER_PIN, 1000);
        } else {
            noTone(BUZZER_PIN);
        }

        if (sys_mode == 0) {
            apply_Motor_Mix(1000.0f); 
            apply_Car_Mix(rx_pitch, rx_roll); 
            is_landed_flag = true; 
            Tin_Reset();
        } 
        else {
            stop_Car(); 

            // [แก้ไข] AHRS + PID ของ Tinnakon (เครื่องหมายตามสูตรมุมเดิมของคุณ)
            Tin_AHRS_update(dt, -gyroRateRoll, -gyroRatePitch, gyroRateYaw,
                            -accY_G, -accX_G, accZ_G);
            Tin_Control(dt, sys_armed == 1);

            if (sys_armed == 1) { 
                if (fused_altitude > peak_altitude) peak_altitude = fused_altitude;

                bool cutoff = false;
                if (fabsf(kalAngleRoll) > TILT_CUTOFF_DEG || fabsf(kalAnglePitch) > TILT_CUTOFF_DEG) {
                    if (tilt_exceed_t == 0) tilt_exceed_t = millis();
                    if (millis() - tilt_exceed_t > 100) { sys_armed = 0; Tin_Reset(); cutoff = true; }
                } else {
                    tilt_exceed_t = 0;
                }

                // [แก้ไข] คันเร่งฐาน: โหมดคันเร่งมือใช้ slider, โหมดอัตโนมัติใช้ค่าลอยตัว
                float pwm_base = (rx_throttle_mode == 0)
                    ? (((float)rx_target_alt / 100.0f) * 1000.0f + 1000.0f)
                    : TIN_HOVER_BASE_US;
                float final_throttle = pwm_base + tin_alt_out;

                if (tin_touchdown) {
                    is_landed_flag = true;
                    final_throttle = 1000.0f;
                    rx_target_alt = 0;
                    sys_armed = 0;
                    Tin_Reset();
                } else {
                    is_landed_flag = false;
                }

                apply_Motor_Mix(cutoff ? 1000.0f : final_throttle); 
            } else {
                tilt_exceed_t = 0; peak_altitude = 0.0f; 
                rx_target_alt = 0; apply_Motor_Mix(1000.0f); 
                is_landed_flag = true;
            }
            prev_rx_throttle_mode = rx_throttle_mode;
        }
    }
}

void setup1() {
    Comm_init();
}

void loop1() {
    if (!core0_ready) { delay(1); return; }

    read_UART_Commands();

    if (millis() - prevTelemetryTime >= 50) { 
        prevTelemetryTime = millis(); 
        send_Telemetry(); 
    }

    delay(1);
}