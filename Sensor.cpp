// ==========================================================
//  Sensor.cpp  --  BMP280 + VL53L0X + ความสูงรวม (fused_altitude)
//  (เวอร์ชัน: อ่านค่า Single Shot เสถียรสุด + ใช้ค่าเลเซอร์ดิบ 100% ไม่หัก Offset)
// ==========================================================
#include "sensor.h"
#include "Config.h"
#include <Adafruit_VL53L0X.h>
#include <Adafruit_BMP280.h>
#include "hardware/watchdog.h"

// ==========================================
// ค่าที่ปรับได้
// ==========================================
#define SENSOR_DEBUG 0                

const float    TOF_MAX_ALT_M   = 1.0f;   
const float    TOF_RETURN_M    = 0.9f;   
const float    OFFSET_ALPHA    = 0.02f;  
const uint32_t TOF_FRESH_MS    = 150;    

// ==========================================
// ออบเจกต์และตัวแปร
// ==========================================
static Adafruit_VL53L0X lox = Adafruit_VL53L0X();
static Adafruit_BMP280  bmp;
static SimpleKalmanFilter kalmanToF(0.02f, 0.1f, 0.1f);
static SimpleKalmanFilter kalmanBmp(0.01f, 0.1f, 0.5f);

float fused_altitude = 0.0f;   
float tof_altitude   = 0.0f;   
float bmp_altitude   = 0.0f;   
float bmp_offset     = 0.0f;   
bool  using_tof      = false;

bool     tof_ok = false;               
bool     tof_is_valid = false;         
uint16_t tof_dbg_raw_mm = 0xFFFF;      
uint8_t  tof_dbg_status = 255;         
float    tof_mount_offset_m = 0.0f;    // บังคับเป็น 0 เพื่อใช้ค่าดิบ

static bool     bmp_ok = false;
static uint32_t tof_read_max_us = 0;   

// ==========================================
// เริ่มต้นเซ็นเซอร์
// ==========================================
void InitSensor() {
    tof_ok = lox.begin(0x29, false);
    if (tof_ok) {
        Serial.println(" VL53L0X Ready (Single Shot / RAW Mode)");
    } else {
        Serial.println(" Failed to boot VL53L0X (ToF disabled, using BMP280)");
    }

    bmp_ok = bmp.begin(0x76);
    if (bmp_ok) {
        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                        Adafruit_BMP280::SAMPLING_X2,
                        Adafruit_BMP280::SAMPLING_X16,
                        Adafruit_BMP280::FILTER_X16,
                        Adafruit_BMP280::STANDBY_MS_1);
        float sum = 0; int n = 0;
        for (int i = 0; i < 10; i++) {
            delay(25);
            float a = bmp.readAltitude(1013.25f);
            if (isfinite(a)) { sum += a; n++; }
        }
        if (n > 0) {
            bmp_offset   = sum / n;
            bmp_altitude = bmp_offset;
            kalmanBmp.setAngle(bmp_offset);
        }
        Serial.println(" BMP280 Ready");
    } else {
        Serial.println(" Failed to boot BMP280");
    }
}

// ==========================================
// ฟังก์ชันหลอกสำหรับ EEPROM (ปล่อยว่างไว้เพื่อไม่ให้ไฟล์อื่น Error)
// ==========================================
void load_ToF_Offset_from_EEPROM() {
    tof_mount_offset_m = 0.0f; 
    Serial.println("ToF EEPROM Disabled: Using RAW distance directly.");
}

bool tof_calibrate_mount_offset() {
    return true; 
}

// ==========================================
// อ่านเซ็นเซอร์ + เลือกแหล่งความสูง
// ==========================================
void ReadSensor(float dt) {
    (void)dt;
    uint32_t now = millis();

    // ---------- 1) BMP280 ----------
    static uint32_t last_bmp_ms = 0;
    if (bmp_ok && now - last_bmp_ms >= 20) {
        last_bmp_ms = now;
        float a = bmp.readAltitude(1013.25);
        if (isfinite(a)) bmp_altitude = a;
    }

    // ---------- 2) ToF: ใช้คำสั่ง Single Shot ----------
    static uint32_t last_poll_ms = 0, last_valid_ms = 0, backoff_until = 0;
    static uint16_t comm_fail = 0;
    static float    tof_hold_m = 0.0f;
    static bool     tof_has = false;
    static uint16_t n_valid = 0, n_reject = 0, n_comm = 0;

    if (tof_ok && now >= backoff_until && now - last_poll_ms >= 50) {
        last_poll_ms = now;
        
        VL53L0X_RangingMeasurementData_t measure;
        uint32_t t0 = micros();
        
        lox.rangingTest(&measure, false); 
        
        uint32_t dur = micros() - t0;
        if (dur > tof_read_max_us) tof_read_max_us = dur;

        uint16_t dist_mm = measure.RangeMilliMeter;
        uint8_t  status  = measure.RangeStatus;
        
        tof_dbg_raw_mm = dist_mm;
        tof_dbg_status = status;

        if (lox.Status != VL53L0X_ERROR_NONE) {
            n_comm++;
            if (++comm_fail >= 100) { comm_fail = 0; backoff_until = now + 1000; }
        } else {
            comm_fail = 0;
            bool in_range = (dist_mm >= 5 && dist_mm <= 1500 && dist_mm != 8190 && dist_mm != 8191);
            bool status_ok = (status == 0 || status == 1 || status == 2 || status == 4); 

            if (in_range && status_ok) {
                // 🌟 เชื่อค่าดิบจากเลเซอร์ 100% ไม่มีการหักลบ Offset ใดๆ ทั้งสิ้น[cite: 6]
                tof_hold_m = dist_mm / 1000.0f; 
                tof_has = true;
                last_valid_ms = now;
                n_valid++;
            } else {
                n_reject++;
            }
        }
    }

    tof_is_valid = tof_has && (now - last_valid_ms < TOF_FRESH_MS);
    if (tof_is_valid) tof_altitude = tof_hold_m;

    // ---------- 3) เลือกแหล่งความสูง ----------
    static bool in_bmp_zone = false;
    if (tof_is_valid) {
        if (!in_bmp_zone && tof_altitude > TOF_MAX_ALT_M) in_bmp_zone = true;
        else if (in_bmp_zone && tof_altitude < TOF_RETURN_M) in_bmp_zone = false;
    }

    if (tof_is_valid && !in_bmp_zone) {
        using_tof = true;
        fused_altitude = tof_altitude;
        if (bmp_ok) bmp_offset += OFFSET_ALPHA * ((bmp_altitude - tof_altitude) - bmp_offset);
    } else {
        using_tof = false;
        if (bmp_ok) {
            fused_altitude = bmp_altitude - bmp_offset;
            if (fused_altitude < 0.0f) fused_altitude = 0.0f;
        }
    }

#if SENSOR_DEBUG
    static uint32_t last_dbg_ms = 0;
    if (now - last_dbg_ms >= 500) {
        last_dbg_ms = now;
        Serial.print(using_tof ? "[ToF mode] " : "[BMP mode] ");
        Serial.print("Fused: "); Serial.print(fused_altitude, 3);
        Serial.print(" m | ToF(RAW): "); if (tof_is_valid) Serial.print(tof_altitude, 3); else Serial.print("n/a");
        Serial.print(" | BMP: "); Serial.print(bmp_altitude, 3);
        Serial.println();
    }
#endif
}