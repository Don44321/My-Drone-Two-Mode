#include "sensor.h"
#include "Config.h"
#include "PID.h"
#include <EEPROM.h>
#include "hardware/watchdog.h"

const uint8_t MPU_ADDR = 0x68;
const uint32_t I2C_CLOCK_HZ = 350000;

const uint8_t MPU_DLPF_CFG = 0x03;

const float KAL_Q_ANGLE   = 0.005f;
const float KAL_Q_BIAS    = 0.003f;
const float KAL_R_MEASURE = 0.3f;

const float CF_ALPHA = 0.98f;
const float ACC_TRUST_BAND_G = 0.15f;
const float ACC_WEIGHT_RISE = 0.05f;

#define MPU_DEBUG 0
#define MPU_REST_DEBUG 0
#define GYRO_AUTOZERO 1

const float GYRO_CAL_MAX_STD = 25.0f;

const int      GYRO_EE_ADDR  = 256;
const uint32_t GYRO_EE_MAGIC = 0x6B1A5E01;
struct GyroBiasStore { float x, y, z; uint32_t magic; };

ComplementaryFilter cfRoll(CF_ALPHA);
ComplementaryFilter cfPitch(CF_ALPHA);

float kalAngleRoll = 0, kalAnglePitch = 0;
float gyroRateRoll = 0, gyroRatePitch = 0, gyroRateYaw = 0;

float gyro_bias_x = 0, gyro_bias_y = 0, gyro_bias_z = 0;
uint32_t mpu_error_count = 0;
uint16_t mpu_fail_streak = 0;
bool gyro_calibrated = false;

extern float accX_G;
extern float accY_G;
extern float accZ_G;   // [เพิ่มใหม่] ใช้โดย Tin_Control
extern float velX;
extern float velY;
extern uint8_t sys_armed;

float base_roll_offset = 0.0;
float base_pitch_offset = 0.0;

static float acc_weight_roll  = 1.0f;
static float acc_weight_pitch = 1.0f;

static bool read_MPU_raw(int16_t &ax, int16_t &ay, int16_t &az,
                         int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) { mpu_error_count++; if (mpu_fail_streak < 65535) mpu_fail_streak++; return false; }
  if (Wire.requestFrom(MPU_ADDR, (size_t)14, true) != 14) { mpu_error_count++; if (mpu_fail_streak < 65535) mpu_fail_streak++; return false; }

  uint8_t b[14];
  for (int i = 0; i < 14; i++) b[i] = Wire.read();

  ax = (int16_t)((b[0]  << 8) | b[1]);
  ay = (int16_t)((b[2]  << 8) | b[3]);
  az = (int16_t)((b[4]  << 8) | b[5]);
  gx = (int16_t)((b[8]  << 8) | b[9]);
  gy = (int16_t)((b[10] << 8) | b[11]);
  gz = (int16_t)((b[12] << 8) | b[13]);
  mpu_fail_streak = 0;
  return true;
}

void setup_MPU6050() {
  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK_HZ);
  Wire.setTimeout(50);
  Wire.begin();

  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x00);          Wire.endTransmission(true);
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1A); Wire.write(MPU_DLPF_CFG);  Wire.endTransmission(true);
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x08);          Wire.endTransmission(true);
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1C); Wire.write(0x10);          Wire.endTransmission(true);

  cfRoll.setAngle(0.0f);
  cfPitch.setAngle(0.0f);
  Serial.println("MPU6050 Ready! (ไม่ calibrate ตอนบูต - ใช้ค่าจาก EEPROM / กดปุ่ม CALIBRATE บนเว็บ)");
}

void load_Gyro_Bias_from_EEPROM() {
  GyroBiasStore s;
  EEPROM.get(GYRO_EE_ADDR, s);
  if (s.magic == GYRO_EE_MAGIC) {
    gyro_bias_x = s.x; gyro_bias_y = s.y; gyro_bias_z = s.z;
    gyro_calibrated = true;
    Serial.println("Gyro bias loaded from EEPROM");
  } else {
    Serial.println("No saved gyro bias - กด CALIBRATE บนเว็บ");
  }
}

static void save_Gyro_Bias_to_EEPROM() {
  GyroBiasStore s = { gyro_bias_x, gyro_bias_y, gyro_bias_z, GYRO_EE_MAGIC };
  EEPROM.put(GYRO_EE_ADDR, s);
  EEPROM.commit();
}

bool calibrate_MPU6050_manual() {
  if (sys_armed == 1) {
    Serial.println("Calibrate rejected: disarm first");
    return false;
  }

  Serial.println("Manual Calibrating MPU6050... keep still");

  const int N = 500;
  double s_roll = 0, s_pitch = 0;
  double s_gx = 0, s_gy = 0, s_gz = 0;
  double q_gx = 0, q_gy = 0, q_gz = 0;
  int good = 0;

  for (int i = 0; i < N; i++) {
    watchdog_update();

    int16_t ax, ay, az, gx, gy, gz;
    if (read_MPU_raw(ax, ay, az, gx, gy, gz)) {
      s_roll  += atan2(-(float)ax, (float)az) * 180.0 / PI;
      s_pitch += atan2((float)ay, sqrt((float)ax * ax + (float)az * az)) * 180.0 / PI;
      s_gx += gx; s_gy += gy; s_gz += gz;
      q_gx += (double)gx * gx; q_gy += (double)gy * gy; q_gz += (double)gz * gz;
      good++;
    }
    delay(2);
  }

  if (good < (N * 9) / 10) {
    Serial.println("Calibrate FAILED: I2C read errors");
    return false;
  }

  double mx = s_gx / good, my = s_gy / good, mz = s_gz / good;
  double sx = sqrt(max(0.0, q_gx / good - mx * mx));
  double sy = sqrt(max(0.0, q_gy / good - my * my));
  double sz = sqrt(max(0.0, q_gz / good - mz * mz));

  if (sx > GYRO_CAL_MAX_STD || sy > GYRO_CAL_MAX_STD || sz > GYRO_CAL_MAX_STD) {
    Serial.println("Calibrate FAILED: drone moved during calibration");
    return false;
  }

  gyro_bias_x = (float)mx;
  gyro_bias_y = (float)my;
  gyro_bias_z = (float)mz;
  base_roll_offset  = (float)(s_roll  / good);
  base_pitch_offset = (float)(s_pitch / good);

  cfRoll.setAngle(0.0f);
  cfPitch.setAngle(0.0f);
  velX = 0; velY = 0;

  save_Offset_to_EEPROM(base_roll_offset, base_pitch_offset);
  save_Gyro_Bias_to_EEPROM();
  gyro_calibrated = true;

  tof_calibrate_mount_offset();

  Serial.println("MPU6050 Manual Calibration Done! (saved)");
  return true;
}

void get_MPU_data(float dt) {
  int16_t accX, accY, accZ, gyroX, gyroY, gyroZ;
  if (!read_MPU_raw(accX, accY, accZ, gyroX, gyroY, gyroZ)) return;

#if GYRO_AUTOZERO
  {
    static uint16_t still_count = 0;
    float rx = ((float)gyroX - gyro_bias_x) / 65.5f;
    float ry = ((float)gyroY - gyro_bias_y) / 65.5f;
    float rz = ((float)gyroZ - gyro_bias_z) / 65.5f;
    float a_mag = sqrtf((float)accX * accX + (float)accY * accY + (float)accZ * accZ) / 4096.0f;
    bool still = (sys_armed == 0) && fabsf(rx) < 4.0f && fabsf(ry) < 4.0f && fabsf(rz) < 4.0f && fabsf(a_mag - 1.0f) < 0.05f;
    if (still) { if (still_count < 65535) still_count++; } else still_count = 0;
    if (still_count > (uint16_t)(1.0f / dt)) {
      float alpha = dt / 3.0f;
      gyro_bias_x += alpha * ((float)gyroX - gyro_bias_x);
      gyro_bias_y += alpha * ((float)gyroY - gyro_bias_y);
      gyro_bias_z += alpha * ((float)gyroZ - gyro_bias_z);
    }
  }
#endif

  gyroRateRoll  = -((float)gyroY - gyro_bias_y) / 65.5f;
  gyroRatePitch = -((float)gyroX - gyro_bias_x) / 65.5f;
  gyroRateYaw   =  ((float)gyroZ - gyro_bias_z) / 65.5f;

  float accAngleRoll  = (atan2(-accX, accZ) * 180.0 / PI) - base_roll_offset;
  float accAnglePitch = (atan2(accY, sqrt((float)accX * accX + (float)accZ * accZ)) * 180.0 / PI) - base_pitch_offset;

  float a_mag_g = sqrtf((float)accX * accX + (float)accY * accY + (float)accZ * accZ) / 4096.0f;
  float acc_err = fabsf(a_mag_g - 1.0f);

  float target_weight = (acc_err < ACC_TRUST_BAND_G) ? 1.0f : 0.0f;
  acc_weight_roll  += (target_weight - acc_weight_roll)  * ((target_weight < acc_weight_roll) ? 1.0f : ACC_WEIGHT_RISE);
  acc_weight_pitch += (target_weight - acc_weight_pitch) * ((target_weight < acc_weight_pitch) ? 1.0f : ACC_WEIGHT_RISE);

  cfRoll.setAlpha(1.0f - (1.0f - CF_ALPHA) * acc_weight_roll);
  cfPitch.setAlpha(1.0f - (1.0f - CF_ALPHA) * acc_weight_pitch);

  kalAngleRoll  = cfRoll.getAngle(accAngleRoll,  -gyroRateRoll,  dt);
  kalAnglePitch = cfPitch.getAngle(accAnglePitch, -gyroRatePitch, dt);

  accX_G = (float)accX / 4096.0f;
  accY_G = (float)accY / 4096.0f;
  accZ_G = (float)accZ / 4096.0f;   // [เพิ่มใหม่]

  float g_comp_X = sin(kalAnglePitch * PI / 180.0);
  float g_comp_Y = -sin(kalAngleRoll * PI / 180.0);

  float linear_acc_X = accX_G - g_comp_X;
  float linear_acc_Y = accY_G - g_comp_Y;

  velX = (velX + (linear_acc_X * 9.81f * dt)) * 0.98f;
  velY = (velY + (linear_acc_Y * 9.81f * dt)) * 0.98f;
}