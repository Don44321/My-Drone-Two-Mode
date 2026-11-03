#ifndef PID_H 
#define PID_H 

#include <Arduino.h> 
#include "Config.h" // 🌟 ดึงโครงสร้างตัวแปรมาจาก Config.h แทน

extern float kp_angle_roll, ki_angle_roll, kd_angle_roll;    
extern float kp_angle_pitch, ki_angle_pitch, kd_angle_pitch; 
extern float kp_rate_roll, ki_rate_roll, kd_rate_roll;       
extern float kp_rate_pitch, ki_rate_pitch, kd_rate_pitch;    
extern float kp_rate_yaw, ki_rate_yaw, kd_rate_yaw;          
extern float kp_alt, ki_alt, kd_alt;                         
extern float pid_alt_out; 

void PID_init();                             
void calculate_CascadePID(float dt, float setpoint_roll, float setpoint_pitch, float setpoint_yaw);         
void calculate_AltPID(float dt, float target_alt, float current_alt); 
void reset_AltPID(); 
void apply_Motor_Mix(float base_throttle);   
void save_PID_to_EEPROM(uint8_t target, float kp, float ki, float kd); 
void ESC_calibration_mode();                 // เข้าโดยกดปุ่ม ReqPI ค้างตอนเปิดเครื่อง (ถอดใบพัดก่อน!)
void set_All_Motors_Raw(float us);           // ส่งค่า µs ตรงไปทุก ESC (ไม่ผ่าน mix)
void save_Offset_to_EEPROM(float roll, float pitch); // 🌟 ประกาศฟังก์ชันบันทึก Offset ถาวร

#endif