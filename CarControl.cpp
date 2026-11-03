#include "CarControl.h"
#include "Config.h"

// ขับ 1 ช่อง: speed > 0 เดินหน้า, < 0 ถอยหลัง, = 0 ปล่อยหยุด (IN ทั้งคู่ LOW + PWM 0)
// หมายเหตุ: ขา PWM ใช้ analogWrite อย่างเดียว ขา IN ใช้ digitalWrite อย่างเดียว (ห้ามสลับกันบนขาเดียวกัน)
static void drive_channel(int speed, int in_a, int in_b, int pwm_pin) {
    if (speed > 0) {
        digitalWrite(in_a, HIGH);
        digitalWrite(in_b, LOW);
        analogWrite(pwm_pin, speed);
    } else if (speed < 0) {
        digitalWrite(in_a, LOW);
        digitalWrite(in_b, HIGH);
        analogWrite(pwm_pin, -speed);
    } else {
        digitalWrite(in_a, LOW);
        digitalWrite(in_b, LOW);
        analogWrite(pwm_pin, 0);
    }
}

void Car_init() {
    pinMode(TB6612_STANBY, OUTPUT);
    pinMode(TB6612_ENA_A, OUTPUT);
    pinMode(TB6612_ENA_B, OUTPUT);
    pinMode(TB6612_IN1, OUTPUT);
    pinMode(TB6612_IN2, OUTPUT);
    pinMode(TB6612_IN3, OUTPUT);
    pinMode(TB6612_IN4, OUTPUT);

    stop_Car();   // เริ่มต้นในสถานะหยุดสนิท (standby) จนกว่าจะสั่งขับ
}

void apply_Car_Mix(int pitch, int roll) {
    int left_speed  = constrain(pitch + roll, -255, 255);
    int right_speed = constrain(pitch - roll, -255, 255);

    if (left_speed == 0 && right_speed == 0) {   // ไม่มีคำสั่ง: หยุดจริง (ปิด STBY ด้วย)
        stop_Car();
        return;
    }

    digitalWrite(TB6612_STANBY, HIGH);           // ปลุกไดรเวอร์เฉพาะตอนต้องขับ
    drive_channel(left_speed,  TB6612_IN1, TB6612_IN2, TB6612_ENA_A);   // ล้อซ้าย (Channel A)
    drive_channel(right_speed, TB6612_IN3, TB6612_IN4, TB6612_ENA_B);   // ล้อขวา (Channel B)
}

// หยุดทุกอย่าง: PWM = 0, IN ทั้งหมด LOW และ STBY = LOW (ไดรเวอร์ standby) -- แม้ขา IN/PWM จะต่อผิดก็ยังหยุดได้เพราะ STBY
void stop_Car() {
    analogWrite(TB6612_ENA_A, 0);
    analogWrite(TB6612_ENA_B, 0);
    digitalWrite(TB6612_IN1, LOW);
    digitalWrite(TB6612_IN2, LOW);
    digitalWrite(TB6612_IN3, LOW);
    digitalWrite(TB6612_IN4, LOW);
    digitalWrite(TB6612_STANBY, LOW);
}
