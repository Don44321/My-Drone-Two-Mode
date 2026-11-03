#include "PID.h"             
#include "sensor.h"          
#include "Config.h"          
#include "Pulse_Register.h"  
#include <EEPROM.h> 

Mypulse motorLF(PIN_ESC_LF); 
Mypulse motorLB(PIN_ESC_LB); 
Mypulse motorRB(PIN_ESC_RB); 
Mypulse motorRF(PIN_ESC_RF); 

System_Settings sys_settings;

#define BYPASS_PID_FOR_TEST 0

const float MOTOR_IDLE_US[4] = {1000.0f, 1000.0f, 1000.0f, 1000.0f};
const float MOTOR_MAX_CMD_US = 2000.0f;
static inline float lin_motor(float cmd, float idle) {
    return idle + (cmd - 1000.0f) * (MOTOR_MAX_CMD_US - idle) / 1000.0f;
}

static inline float clamp_motor_us(float v) {
    if (v < 1000.0f) return 1000.0f;
    if (v > MOTOR_MAX_CMD_US) return MOTOR_MAX_CMD_US;
    return v;
}

float kp_angle_roll = 3.0, ki_angle_roll = 0.15, kd_angle_roll = 0.08; 
float kp_angle_pitch = 3.0, ki_angle_pitch = 0.15, kd_angle_pitch = 0.08; 

float kp_rate_roll = 1.7, ki_rate_roll = 0.45, kd_rate_roll = 0.035; 
float kp_rate_pitch = 1.7, ki_rate_pitch = 0.45, kd_rate_pitch = 0.035; 
float kp_rate_yaw = 2.3, ki_rate_yaw = 0.25, kd_rate_yaw = 0.0; 

float kp_alt = 10.0, ki_alt = 0.5, kd_alt = 2.5; 
float i_error_alt = 0, prev_error_alt = 0;
float pid_alt_out = 0;

float i_error_rate_roll = 0, i_error_rate_pitch = 0, i_error_rate_yaw = 0;
float prev_error_rate_roll = 0, prev_error_rate_pitch = 0, prev_error_rate_yaw = 0;

float i_error_angle_roll = 0, i_error_angle_pitch = 0;
float prev_error_angle_roll = 0, prev_error_angle_pitch = 0;

float pid_roll_out = 0, pid_pitch_out = 0, pid_yaw_out = 0;

const float MAX_PID_OUT = 400.0f; 
const float MAX_I_TERM = 150.0f;  
const float MAX_I_TERM_ANGLE = 40.0f;

void PID_init() {
    motorLF.begin(400.0); 
    motorLB.begin(400.0); 
    motorRB.begin(400.0); 
    motorRF.begin(400.0); 

    for (int i = 0; i < 150; i++) {
        motorLF.writeMicroseconds(1000);
        motorLB.writeMicroseconds(1000);
        motorRB.writeMicroseconds(1000);
        motorRF.writeMicroseconds(1000);
        delay(1);
    }

    EEPROM.begin(512); 
    EEPROM.get(0, sys_settings);

    if (sys_settings.magic_number != 0x8899AABB) {
        Serial.println(" Old or No Data. Loading Defaults to EEPROM...");
        sys_settings.kp[1] = 3.0f; sys_settings.ki[1] = 0.15f; sys_settings.kd[1] = 0.08f;
        sys_settings.kp[2] = 3.0f; sys_settings.ki[2] = 0.15f; sys_settings.kd[2] = 0.08f;
        sys_settings.kp[3] = 1.7f; sys_settings.ki[3] = 0.45f; sys_settings.kd[3] = 0.035f;
        sys_settings.kp[4] = 1.7f; sys_settings.ki[4] = 0.45f; sys_settings.kd[4] = 0.035f;
        sys_settings.kp[5] = 2.3f; sys_settings.ki[5] = 0.25f; sys_settings.kd[5] = 0.0f;
        sys_settings.kp[6] = 10.0f; sys_settings.ki[6] = 0.5f; sys_settings.kd[6] = 2.5f; 
        
        sys_settings.roll_offset = 0.0f;  
        sys_settings.pitch_offset = 0.0f; 

        sys_settings.rgb_car[0] = 0; sys_settings.rgb_car[1] = 255; sys_settings.rgb_car[2] = 0;
        sys_settings.rgb_drone[0] = 0; sys_settings.rgb_drone[1] = 0; sys_settings.rgb_drone[2] = 255;
        sys_settings.rgb_armed[0] = 255; sys_settings.rgb_armed[1] = 0; sys_settings.rgb_armed[2] = 0;
        sys_settings.rgb_lowbatt[0] = 255; sys_settings.rgb_lowbatt[1] = 255; sys_settings.rgb_lowbatt[2] = 0;
        sys_settings.rgb_failsafe[0] = 255; sys_settings.rgb_failsafe[1] = 0; sys_settings.rgb_failsafe[2] = 255;

        sys_settings.magic_number = 0x8899AABB; 
        EEPROM.put(0, sys_settings);
        EEPROM.commit();
    } else {
        Serial.println(" Data Loaded from EEPROM");
    }

    if (sys_settings.trim_magic != MOTOR_TRIM_MAGIC) {
        Serial.println(" Motor trim not set. Using 0.");
        for (int i = 0; i < 4; i++) sys_settings.motor_trim_us[i] = 0;
        sys_settings.trim_magic = MOTOR_TRIM_MAGIC;
        EEPROM.put(0, sys_settings);
        EEPROM.commit();
    }

    // ค่าเริ่มต้นของ Tinnakon (ตั้งครั้งแรกเท่านั้น)
    if (sys_settings.tin_magic != TIN_MAGIC) {
        sys_settings.tin_ka_alt     = 0.5f;
        sys_settings.tin_kp_yaw_ang = 4.5f;
        sys_settings.tin_m_quad     = 1.73f;
        sys_settings.tin_tar        = 0.015f;
        sys_settings.tin_magic      = TIN_MAGIC;
        EEPROM.put(0, sys_settings);
        EEPROM.commit();
    }

    kp_angle_roll = sys_settings.kp[1]; ki_angle_roll = sys_settings.ki[1]; kd_angle_roll = sys_settings.kd[1];
    kp_angle_pitch = sys_settings.kp[2]; ki_angle_pitch = sys_settings.ki[2]; kd_angle_pitch = sys_settings.kd[2];
    kp_rate_roll = sys_settings.kp[3]; ki_rate_roll = sys_settings.ki[3]; kd_rate_roll = sys_settings.kd[3];
    kp_rate_pitch = sys_settings.kp[4]; ki_rate_pitch = sys_settings.ki[4]; kd_rate_pitch = sys_settings.kd[4];
    kp_rate_yaw = sys_settings.kp[5]; ki_rate_yaw = sys_settings.ki[5]; kd_rate_yaw = sys_settings.kd[5];
    kp_alt = sys_settings.kp[6]; ki_alt = sys_settings.ki[6]; kd_alt = sys_settings.kd[6];

    base_roll_offset = sys_settings.roll_offset;
    base_pitch_offset = sys_settings.pitch_offset;
}

void save_PID_to_EEPROM(uint8_t target, float kp, float ki, float kd) {
    if (target >= 1 && target <= 6) { 
        sys_settings.kp[target] = kp;
        sys_settings.ki[target] = ki;
        sys_settings.kd[target] = kd;
        
        EEPROM.put(0, sys_settings);
        EEPROM.commit(); 
        
        if (target == 1) { kp_angle_roll = kp; ki_angle_roll = ki; kd_angle_roll = kd; }
        else if (target == 2) { kp_angle_pitch = kp; ki_angle_pitch = ki; kd_angle_pitch = kd; }
        else if (target == 3) { kp_rate_roll = kp; ki_rate_roll = ki; kd_rate_roll = kd; }
        else if (target == 4) { kp_rate_pitch = kp; ki_rate_pitch = ki; kd_rate_pitch = kd; }
        else if (target == 5) { kp_rate_yaw = kp; ki_rate_yaw = ki; kd_rate_yaw = kd; }
        else if (target == 6) { kp_alt = kp; ki_alt = ki; kd_alt = kd; }
        
        Serial.print("Saved & Applied PID Target ");
        Serial.println(target);
    }
}

void save_Offset_to_EEPROM(float roll, float pitch) {
    sys_settings.roll_offset = roll;
    sys_settings.pitch_offset = pitch;
    
    EEPROM.put(0, sys_settings);
    EEPROM.commit();
    
    base_roll_offset = roll;
    base_pitch_offset = pitch;
    Serial.println(" Saved Offsets to EEPROM");
}

// ==========================================
// Mixer (สูตรเดิมของคุณ)
// ==========================================
void apply_Motor_Mix(float base_throttle) {
    float cmd_LF, cmd_LB, cmd_RB, cmd_RF;

    if (base_throttle <= 1000.0f) {
        cmd_LF = base_throttle;
        cmd_LB = base_throttle;
        cmd_RB = base_throttle;
        cmd_RF = base_throttle;
    }
    else {
#if BYPASS_PID_FOR_TEST
        pid_roll_out = 0; pid_pitch_out = 0; pid_yaw_out = 0;
#endif
        cmd_LF = base_throttle - pid_roll_out - pid_pitch_out + pid_yaw_out; 
        cmd_LB = base_throttle - pid_roll_out + pid_pitch_out - pid_yaw_out; 
        cmd_RB = base_throttle + pid_roll_out + pid_pitch_out + pid_yaw_out; 
        cmd_RF = base_throttle + pid_roll_out - pid_pitch_out - pid_yaw_out; 
        
        if (cmd_LF < 1000.0f) cmd_LF = 1000.0f; if (cmd_LF > MOTOR_MAX_CMD_US) cmd_LF = MOTOR_MAX_CMD_US;
        if (cmd_LB < 1000.0f) cmd_LB = 1000.0f; if (cmd_LB > MOTOR_MAX_CMD_US) cmd_LB = MOTOR_MAX_CMD_US;
        if (cmd_RB < 1000.0f) cmd_RB = 1000.0f; if (cmd_RB > MOTOR_MAX_CMD_US) cmd_RB = MOTOR_MAX_CMD_US;
        if (cmd_RF < 1000.0f) cmd_RF = 1000.0f; if (cmd_RF > MOTOR_MAX_CMD_US) cmd_RF = MOTOR_MAX_CMD_US;

        cmd_LF = lin_motor(cmd_LF, MOTOR_IDLE_US[0]);
        cmd_LB = lin_motor(cmd_LB, MOTOR_IDLE_US[1]);
        cmd_RB = lin_motor(cmd_RB, MOTOR_IDLE_US[2]);
        cmd_RF = lin_motor(cmd_RF, MOTOR_IDLE_US[3]);

        cmd_LF = clamp_motor_us(cmd_LF + sys_settings.motor_trim_us[0]);
        cmd_LB = clamp_motor_us(cmd_LB + sys_settings.motor_trim_us[1]);
        cmd_RB = clamp_motor_us(cmd_RB + sys_settings.motor_trim_us[2]);
        cmd_RF = clamp_motor_us(cmd_RF + sys_settings.motor_trim_us[3]);
    }

    motorLF.writeMicroseconds((uint16_t)cmd_LF);
    motorLB.writeMicroseconds((uint16_t)cmd_LB);
    motorRB.writeMicroseconds((uint16_t)cmd_RB);
    motorRF.writeMicroseconds((uint16_t)cmd_RF);
}

void set_All_Motors_Raw(float us) {
    motorLF.writeMicroseconds(us); motorLB.writeMicroseconds(us);
    motorRB.writeMicroseconds(us); motorRF.writeMicroseconds(us);
}

void ESC_calibration_mode() {
    while (digitalRead(ReqPI) == LOW) delay(10);
    Serial.println("ESC CAL: sending MAX (2000us). Connect battery, wait for beeps, then press button");
    while (digitalRead(ReqPI) == HIGH) { set_All_Motors_Raw(2000.0f); delay(2); }
    while (digitalRead(ReqPI) == LOW) delay(10);
    Serial.println("ESC CAL: sending MIN (1000us). Wait for confirmation beeps, then unplug battery and reboot");
    while (true) { set_All_Motors_Raw(1000.0f); delay(2); }
}