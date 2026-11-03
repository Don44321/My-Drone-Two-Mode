#ifndef COMM_H
#define COMM_H

#include <Arduino.h>

extern uint8_t sys_mode;       
extern uint8_t sys_armed;      
extern int16_t rx_roll;        
extern int16_t rx_pitch;       
extern int16_t rx_yaw;         
extern uint16_t rx_target_alt; 
extern uint8_t rx_throttle_mode; 
extern uint16_t rx_limit_val;  
extern float batteryVoltage;
extern bool is_landed_flag; 

// 🌟 ตัวแปรใหม่สำหรับฟีเจอร์ใหม่
extern bool req_send_pid; // แฟล็กแจ้งว่า RPi5 ขออ่านค่า PID
extern uint8_t audio_tone_test; // เก็บหมายเลขโทนที่กดปุ่ม TEST
extern uint8_t audio_cfg_armed, audio_cfg_lowbatt, audio_cfg_trans, audio_cfg_failsafe; // เก็บการตั้งค่าโทน

// สถานะที่ใช้ข้ามคอร์ (volatile เพราะคอร์หนึ่งเขียน อีกคอร์อ่าน)
extern volatile bool calib_requested;   // Core 1 ตั้ง -> Core 0 ทำ calibrate
extern volatile bool calib_ok;
extern volatile bool req_send_calib;
extern volatile bool req_send_offset;

void Comm_init();
void read_UART_Commands();
void send_Telemetry();
void check_Failsafe();
void read_USB_Debug_Commands();
void print_Debug_Info();

// เพิ่มฟังก์ชันส่งค่า PID กลับ
void send_PID_Telemetry(); 

#endif