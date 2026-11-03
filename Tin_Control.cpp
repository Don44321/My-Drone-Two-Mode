#include "Tin_Control.h"
#include "Config.h"
#include "PID.h"
#include "sensor.h"
#include "Comm.h"

extern float pid_roll_out, pid_pitch_out, pid_yaw_out;   // PID.cpp
extern float pid_alt_out;                                 // PID.cpp

#define TIN_D2R        0.0174532925f
#define TIN_R2D        57.2957795f
#define TIN_H_CONTROL  0.9f
#define TIN_RAMP_STEP  0.023f
#define TIN_RAMP_T     0.1f
#define TIN_AHRS_KP    0.22f
#define TIN_AHRS_KI    0.072f

float tin_roll_deg = 0, tin_pitch_deg = 0;
float tin_alt_out = 0, tin_accZ_ms2 = 0, tin_cos_rp = 1;
bool  tin_touchdown = false;

// ---- AHRS (Mahony, IMU อย่างเดียว) ----
static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
static float exInt = 0, eyInt = 0, ezInt = 0;
static float d20 = 0, d21 = 0, d22 = 1;
static float rate_r = 0, rate_p = 0, rate_y = 0;

// ---- PID state ----
static float i_rr = 0, i_rp = 0, i_ry = 0;
static float dr_r = 0, dr_p = 0, dr_y = 0;
static float e_rr_old = 0, e_rp_old = 0, e_ry_old = 0;

// ---- ALT / ramp state ----
static float hz_I = 0, h_ref = 0, h_old = 0, ramp_t = 0, vz_target = 0, vz_f = 0;
static float alt_prev = 0, alt_elapsed = 0, land_t = 0;
static bool  alt_init = false;
static int   prev_mode = -1;

static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

void Tin_Reset() {
    i_rr = i_rp = i_ry = 0;
    dr_r = dr_p = dr_y = 0;
    e_rr_old = e_rp_old = e_ry_old = 0;
    hz_I = 0; h_ref = 0; h_old = 0; ramp_t = 0; vz_target = 0; land_t = 0;
    tin_alt_out = 0; tin_touchdown = false;
    pid_roll_out = pid_pitch_out = pid_yaw_out = 0;
    pid_alt_out = 0;
    prev_mode = -1;
    alt_init = false;
}

void Tin_AHRS_update(float dt, float rr_dps, float rp_dps, float ry_dps,
                     float ax, float ay, float az) {
    rate_r = rr_dps; rate_p = rp_dps; rate_y = ry_dps;

    float gx = rr_dps * TIN_D2R, gy = rp_dps * TIN_D2R, gz = ry_dps * TIN_D2R;

    float n = sqrtf(ax * ax + ay * ay + az * az);
    if (n > 0.2f) {
        ax /= n; ay /= n; az /= n;
        float ex = ay * d22 - az * d21;
        float ey = az * d20 - ax * d22;
        float ez = ax * d21 - ay * d20;
        exInt += ex * TIN_AHRS_KI * dt;
        eyInt += ey * TIN_AHRS_KI * dt;
        ezInt += ez * TIN_AHRS_KI * dt;
        gx += TIN_AHRS_KP * ex + exInt;
        gy += TIN_AHRS_KP * ey + eyInt;
        gz += TIN_AHRS_KP * ez + ezInt;
    }

    float h = 0.5f * dt;
    float nq0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * h;
    float nq1 = q1 + ( q0 * gx + q2 * gz - q3 * gy) * h;
    float nq2 = q2 + ( q0 * gy - q1 * gz + q3 * gx) * h;
    float nq3 = q3 + ( q0 * gz + q1 * gy - q2 * gx) * h;
    float qn = sqrtf(nq0 * nq0 + nq1 * nq1 + nq2 * nq2 + nq3 * nq3);
    q0 = nq0 / qn; q1 = nq1 / qn; q2 = nq2 / qn; q3 = nq3 / qn;

    d20 = 2 * (q1 * q3 - q0 * q2);
    d21 = 2 * (q2 * q3 + q0 * q1);
    d22 = 1 - 2 * (q1 * q1 + q2 * q2);

    tin_roll_deg  = atan2f(d21, d22) * TIN_R2D;
    tin_pitch_deg = -asinf(clampf(d20, -1.0f, 1.0f)) * TIN_R2D;
    tin_cos_rp    = (d22 < 0.82f) ? 0.82f : d22;
    tin_accZ_ms2  = (ax * d20 + ay * d21 + az * d22 - 1.0f) * 9.81f;
}

void Tin_Control(float dt, bool armed) {
    if (dt <= 0.0f) return;
    if (!armed) { Tin_Reset(); return; }

    const float kpLR = sys_settings.kp[1], kpLP = sys_settings.kp[2];
    const float kpRR = sys_settings.kp[3], kiRR = sys_settings.ki[3], kdRR = sys_settings.kd[3];
    const float kpRP = sys_settings.kp[4], kiRP = sys_settings.ki[4], kdRP = sys_settings.kd[4];
    const float kpRY = sys_settings.kp[5], kiRY = sys_settings.ki[5], kdRY = sys_settings.kd[5];
    const float kpA  = sys_settings.kp[6], kiA  = sys_settings.ki[6], kdA  = sys_settings.kd[6];
    const float tar  = sys_settings.tin_tar;

    // ROLL
    float err_r = ((float)rx_roll - tin_roll_deg) * kpLR - rate_r;
    i_rr += err_r * kiRR * dt;
    i_rr = clampf(i_rr, -50.0f, 50.0f);
    dr_r = (tar * dr_r / (tar + dt)) + ((err_r - e_rr_old) / (tar + dt));
    e_rr_old = err_r;
    pid_roll_out = clampf(kpRR * err_r + i_rr + kdRR * dr_r, -200.0f, 200.0f);

    // PITCH
    float err_p = ((float)rx_pitch - tin_pitch_deg) * kpLP - rate_p;
    i_rp += err_p * kiRP * dt;
    i_rp = clampf(i_rp, -50.0f, 50.0f);
    dr_p = (tar * dr_p / (tar + dt)) + ((err_p - e_rp_old) / (tar + dt));
    e_rp_old = err_p;
    pid_pitch_out = clampf(kpRP * err_p + i_rp + kdRP * dr_p, -200.0f, 200.0f);

    // YAW (rate อย่างเดียว)
    float err_y = (float)rx_yaw - rate_y;
    i_ry += err_y * kiRY * dt;
    i_ry = clampf(i_ry, -50.0f, 50.0f);
    dr_y = (tar * dr_y / (tar + dt)) + ((err_y - e_ry_old) / (tar + dt));
    e_ry_old = err_y;
    pid_yaw_out = clampf(kpRY * err_y + i_ry + kdRY * dr_y, -120.0f, 120.0f);

    // ความเร็วแนวดิ่ง (อัปเดตเมื่อความสูงเปลี่ยนจริง)
    float alt = fused_altitude;
    if (!alt_init) {
        alt_prev = alt; alt_elapsed = 0; vz_f = 0; alt_init = true;
    } else {
        alt_elapsed += dt;
        if (alt != alt_prev) {
            float vz_raw = (alt - alt_prev) / alt_elapsed;
            vz_f += 0.3f * (vz_raw - vz_f);
            alt_prev = alt;
            alt_elapsed = 0;
        }
    }

    // โหมดความสูง
    int mode = rx_throttle_mode;
    if (mode != prev_mode) {
        if (mode == 2) h_ref = 0.0f;
        if (mode == 3) h_ref = alt;
        ramp_t = 0; h_old = h_ref; land_t = 0; tin_touchdown = false;
        prev_mode = mode;
    }

    if (mode == 0) {
        hz_I = 0; tin_alt_out = 0; pid_alt_out = 0;
        h_ref = alt; vz_target = 0; land_t = 0;
        return;
    }
    else if (mode == 1) {
        h_ref = (float)rx_limit_val / 100.0f;
        vz_target = 0.0f;
    }
    else if (mode == 2) {
        ramp_t += dt;
        if (ramp_t >= TIN_RAMP_T) {
            ramp_t = 0;
            h_ref = min(h_ref + TIN_RAMP_STEP, TIN_H_CONTROL);
            vz_target = (h_ref - h_old) / TIN_RAMP_T;
            h_old = h_ref;
        }
    }
    else if (mode == 3) {
        ramp_t += dt;
        if (ramp_t >= TIN_RAMP_T) {
            ramp_t = 0;
            h_ref = max(h_ref - TIN_RAMP_STEP, 0.0f);
            vz_target = (h_ref - h_old) / TIN_RAMP_T;
            h_old = h_ref;
        }
    }

    // ALT state feedback
    float err_h = clampf(h_ref - alt, -2.0f, 2.0f);
    float err_v = vz_target - vz_f;
    float hII = err_h * 0.55f + err_v;
    hz_I += kiA * hII * dt;
    hz_I = clampf(hz_I, -120.0f, 120.0f);

    float u = err_h * kpA + hz_I + err_v * kdA - tin_accZ_ms2 * sys_settings.tin_ka_alt;
    u = u * sys_settings.tin_m_quad / tin_cos_rp;
    tin_alt_out = clampf(u, -200.0f, 200.0f);
    pid_alt_out = tin_alt_out;

    // ตรวจการแตะพื้นในโหมด LAND
    if (mode == 3 && h_ref <= 0.03f && alt <= 0.10f) {
        land_t += dt;
        if (land_t > 0.5f) tin_touchdown = true;
    } else {
        land_t = 0;
    }
}