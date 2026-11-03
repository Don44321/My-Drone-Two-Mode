#ifndef SENSOR_H // ป้องกันการอิมพอร์ตไฟล์ซ้ำซ้อน
#define SENSOR_H // กำหนดชื่อโมดูล SENSOR_H

#include <Arduino.h> 
#include <Wire.h>    

// [เพิ่มใหม่] ตัวกรอง Complementary สำหรับมุม roll/pitch
// angle = alpha * (angle + gyro * dt) + (1 - alpha) * accel_angle
class ComplementaryFilter {
  public:
    ComplementaryFilter(float alpha) : _alpha(alpha), _angle(0.0f) {}

    float getAngle(float accelAngle, float gyroRate, float dt) {
        _angle = _alpha * (_angle + gyroRate * dt) + (1.0f - _alpha) * accelAngle;
        return _angle;
    }

    void setAngle(float angle) { _angle = angle; }
    void setAlpha(float alpha) { _alpha = alpha; }

  private:
    float _alpha;
    float _angle;
};

class SimpleKalmanFilter {
  public:
    SimpleKalmanFilter(float Q_a, float Q_b, float R_m) {
        Q_angle = Q_a; Q_bias = Q_b; R_measure = R_m; 
        angle = 0.0f; bias = 0.0f; rate = 0.0f;       
        P[0][0] = 0.0f; P[0][1] = 0.0f;               
        P[1][0] = 0.0f; P[1][1] = 0.0f;               
    }

    float getAngle(float newAngle, float newRate, float dt) {
        rate = newRate - bias;                             
        angle += dt * rate;                                

        P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle); 
        P[0][1] -= dt * P[1][1];                                      
        P[1][0] -= dt * P[1][1];                                      
        P[1][1] += Q_bias * dt;                                       

        float S = P[0][0] + R_measure;                     
        float K[2] = {P[0][0] / S, P[1][0] / S};           
        float y = newAngle - angle;                        

        angle += K[0] * y;                                 
        bias  += K[1] * y;                                 

        float P00_temp = P[0][0];                          
        float P01_temp = P[0][1];                          

        P[0][0] -= K[0] * P00_temp;                        
        P[0][1] -= K[0] * P01_temp;                        
        P[1][0] -= K[1] * P00_temp;                        
        P[1][1] -= K[1] * P01_temp;                        

        return angle;                                      
    };

    void setAngle(float angle) { this->angle = angle; }; 
    void setBias(float b) { this->bias = b; };           // รีเซ็ต bias หลัง calibrate
  private:
    float Q_angle, Q_bias, R_measure; 
    float angle, bias, rate;          
    float P[2][2];                    
};

// ==========================================
// ประกาศตัวแปรระดับ Global
// ==========================================
// MPU6050
extern float kalAngleRoll;  
extern float kalAnglePitch; 
extern float gyroRateRoll;  
extern float gyroRatePitch; 
extern float gyroRateYaw;   

// ประกาศตัวแปร Offset ให้ไฟล์ PID และ Comm ดึงไปใช้งานได้
extern float base_roll_offset;
extern float base_pitch_offset;

// gyro bias (ค่าดิบ LSB) และตัวนับ I2C error
extern float gyro_bias_x, gyro_bias_y, gyro_bias_z;
extern uint32_t mpu_error_count;
extern uint16_t mpu_fail_streak;   // อ่าน MPU พลาดติดกันกี่ครั้ง (0 = ปกติ)
extern bool gyro_calibrated;       // true เมื่อโหลดจาก EEPROM หรือ calibrate แล้ว

// ความสูง: BMP280 + VL53L0X (อยู่ใน Sensor.cpp)
extern float fused_altitude;        // ความสูงรวม (เมตร)
extern float tof_altitude;          // ความสูงเหนือพื้นจาก ToF
extern float bmp_altitude;          // ความสูงจาก BMP280 (อ้างอิง 1013.25 hPa)
extern float bmp_offset;            // ค่าชดเชย BMP280
extern bool  using_tof;             // true = ความสูงตอนนี้มาจาก ToF
extern bool  tof_ok;                // ToF เริ่มต้นสำเร็จ
extern bool  tof_is_valid;          // ToF มีค่าถูกต้องล่าสุด
extern uint16_t tof_dbg_raw_mm;     // ค่าดิบล่าสุด (mm) 0xFFFF = ยังไม่มี/อ่านไม่ได้
extern uint8_t  tof_dbg_status;     // สถานะการวัดล่าสุด 0 = ถูกต้อง, 4 = วัดไม่ได้, 255 = ยังไม่มี
extern float tof_mount_offset_m;    // ความสูงติดตั้งของเซ็นเซอร์เหนือพื้น (เมตร)
bool tof_calibrate_mount_offset();  // ตั้งความสูงติดตั้งอัตโนมัติ (เรียกจาก calibrate_MPU6050_manual)
void load_ToF_Offset_from_EEPROM(); // เรียกหลัง PID_init()

// ==========================================
// ประกาศฟังก์ชัน
// ==========================================
void setup_MPU6050();        
void get_MPU_data(float dt); 
bool calibrate_MPU6050_manual();  // สั่งจากหน้าเว็บ คืน true เมื่อสำเร็จ (ปฏิเสธถ้า armed / เครื่องไม่นิ่ง)
void load_Gyro_Bias_from_EEPROM(); // เรียกหลัง PID_init()

void InitSensor();            // BMP280 + VL53L0X (เรียกหลัง setup_MPU6050)
void ReadSensor(float dt);    // อ่านเซ็นเซอร์ + คำนวณ fused_altitude ทุกรอบ loop


#endif