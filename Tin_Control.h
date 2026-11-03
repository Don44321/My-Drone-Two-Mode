#ifndef TIN_CONTROL_H
#define TIN_CONTROL_H

#include <Arduino.h>

// คันเร่งฐานตอน ALT HOLD / TAKE-OFF / LAND (µs) ~62%
#define TIN_HOVER_BASE_US 1620.0f

extern float tin_roll_deg;      // มุม roll จาก AHRS (องศา)
extern float tin_pitch_deg;     // มุม pitch จาก AHRS (องศา)
extern float tin_alt_out;       // เอาต์พุต ALT (µs)
extern float tin_accZ_ms2;      // accel แกน Z บนโลก (m/s^2)
extern float tin_cos_rp;        // cos(roll)*cos(pitch) สำหรับชดเชยการเอียง
extern bool  tin_touchdown;     // true เมื่อลงจอดแล้ว (โหมด LAND)

void Tin_AHRS_update(float dt, float rr_dps, float rp_dps, float ry_dps,
                     float ax, float ay, float az);
void Tin_Control(float dt, bool armed);
void Tin_Reset();

#endif