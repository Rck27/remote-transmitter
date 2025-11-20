#include "filter.h"
#include <math.h>
#include <stdlib.h>
#include <string.h> // For memset

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define DEG2RAD(x) ((x) * (M_PI / 180.0f))
#define RAD2DEG(x) ((x) * (180.0f / M_PI))

#define GYRO_SCALE   ((1.0f / 65.5f) * DEG2RAD(1.0f)) // raw -> rad/s
#define ACCEL_SCALE  (1.0f / 8192.0f)                 // raw -> g


// ====== Internal Math & Helper functions ======
static inline float inv_sqrt(float x) {
    if (x <= 0) return 0;
    return 1.0f / sqrtf(x);
}

static inline float clamp(float v, float min, float max) {
    return fmaxf(min, fminf(max, v));
}

static inline float pt1_gain(float rate, float freq) {
  float rc = 1.f / (2.f * M_PI * freq);
  float dt = 1.f / rate;
  return dt / (dt + rc);
}

static inline float get_notch_q_approx(float freq, float cutoff) {
    if (freq <= cutoff) return 0.1f; // Avoid division by zero
    return ((float)(cutoff * freq) / ((float)(freq - cutoff) * (float)(freq + cutoff)));
}


// ====== Filter State Implementations ======

// --- PT1 ---
void pt1_init(FilterStatePt1* s, float rate, float freq) { s->k = pt1_gain(rate, freq); }
void pt1_reset(FilterStatePt1* s) { s->v = 0.f; }
float pt1_update(FilterStatePt1* s, float n) { s->v += s->k * (n - s->v); return s->v; }

// --- Biquad ---
void biquad_init(FilterStateBiquad* s, BiquadFilterType filterType, float rate, float freq, float q) {
    const float omega = (2.0f * M_PI * freq) / rate;
    const float sn = sinf(omega);
    const float cs = cosf(omega);
    const float alpha = sn / (2.0f * q);
    float b0 = 0.0f, b1 = 0.0f, b2 = 0.0f, a0 = 0.0f, a1 = 0.0f, a2 = 0.0f;
    switch (filterType) {
        case BIQUAD_FILTER_LPF:
            b0 = (1.f - cs) * 0.5f; b1 = 1.f - cs; b2 = (1.f - cs) * 0.5f;
            a0 = 1.f + alpha; a1 = -2.f * cs; a2 = 1.f - alpha;
            break;
        case BIQUAD_FILTER_NOTCH:
            b0 = 1.f; b1 = -2.f * cs; b2 = 1.f;
            a0 = 1.f + alpha; a1 = -2.f * cs; a2 = 1.f - alpha;
            break;
        case BIQUAD_FILTER_BPF:
            b0 = alpha; b1 = 0; b2 = -alpha;
            a0 = 1.f + alpha; a1 = -2.f * cs; a2 = 1.f - alpha;
            break;
    }
    s->b0 = b0 / a0; s->b1 = b1 / a0; s->b2 = b2 / a0;
    s->a1 = a1 / a0; s->a2 = a2 / a0;
}
void biquad_reset(FilterStateBiquad* s) { s->x1 = s->x2 = s->y1 = s->y2 = 0; }
float biquad_update(FilterStateBiquad* s, float n) { // DF2
    const float result = s->b0 * n + s->x1;
    s->x1 = s->b1 * n - s->a1 * result + s->x2;
    s->x2 = s->b2 * n - s->a2 * result;
    return result;
}
float biquad_update_df1(FilterStateBiquad* s, float n) {
    const float result = s->b0 * n + s->b1 * s->x1 + s->b2 * s->x2 - s->a1 * s->y1 - s->a2 * s->y2;
    s->x2 = s->x1; s->x1 = n; s->y2 = s->y1; s->y1 = result;
    return result;
}

// ... other filter type implementations ... (FIR2, Median, PT2, PT3, FO)
// For simplicity, I've included the most common ones. You can add the others following the same pattern.


// ====== Main Filter Functions ======
void filter_init_advanced(Filter* f) {
    if(!f) return;
    memset(f, 0, sizeof(Filter));
    f->_conf.type = FILTER_NONE;
}

void filter_reset(Filter* f) {
    if (!f) return;
    switch(f->_conf.type) {
        case FILTER_PT1: pt1_reset(&f->_state.pt1); break;
        case FILTER_BIQUAD: case FILTER_NOTCH: case FILTER_NOTCH_DF1: case FILTER_BPF:
            biquad_reset(&f->_state.bq);
            break;
        // Add other cases here
        default: break;
    }
}

static void filter_reconfigure(Filter* f, const FilterConfig* config, int rate, float q, float weight) {
    f->_rate = rate;
    f->_conf = *config;
    f->_output_weight = clamp(weight, 0.0f, 1.0f);
    f->_input_weight = 1.0f - f->_output_weight;

    switch(f->_conf.type) {
        case FILTER_PT1:
            pt1_init(&f->_state.pt1, rate, f->_conf.freq);
            break;
        case FILTER_BIQUAD:
            biquad_init(&f->_state.bq, BIQUAD_FILTER_LPF, rate, f->_conf.freq, q);
            break;
        case FILTER_NOTCH:
        case FILTER_NOTCH_DF1:
            biquad_init(&f->_state.bq, BIQUAD_FILTER_NOTCH, rate, f->_conf.freq, q);
            break;
        case FILTER_BPF:
            biquad_init(&f->_state.bq, BIQUAD_FILTER_BPF, rate, f->_conf.freq, q);
            break;
        // Add other cases here
        default: break;
    }
}

void filter_begin(Filter* f, const FilterConfig* config, int rate) {
    if (!f || !config) return;
    
    // Sanitize config
    FilterConfig sanitized_config;
    const int halfRate = rate * 0.49f;
    sanitized_config.type = config->type;
    sanitized_config.freq = clamp(config->freq, 0, halfRate);
    sanitized_config.cutoff = clamp(config->cutoff, 0, sanitized_config.freq * 0.98f);

    if (sanitized_config.freq == 0) sanitized_config.type = FILTER_NONE;

    float q = 0.7071f; // Default Q factor for LPF
    if(sanitized_config.type == FILTER_NOTCH || sanitized_config.type == FILTER_NOTCH_DF1 || sanitized_config.type == FILTER_BPF) {
        if (sanitized_config.cutoff == 0) sanitized_config.type = FILTER_NONE;
        else q = get_notch_q_approx(sanitized_config.freq, sanitized_config.cutoff);
    }
    
    filter_reconfigure(f, &sanitized_config, rate, q, 1.0f);
    filter_reset(f);
}

float filter_update(Filter* f, float v) {
    if (!f) return v;
    switch(f->_conf.type) {
        case FILTER_PT1:
            return pt1_update(&f->_state.pt1, v);
        case FILTER_BIQUAD:
        case FILTER_NOTCH:
        case FILTER_BPF:
            return biquad_update(&f->_state.bq, v);
        case FILTER_NOTCH_DF1:
            return f->_output_weight * biquad_update_df1(&f->_state.bq, v) + f->_input_weight * v;
        // Add other cases here
        default:
            return v;
    }
}

// ====== Your Original IMU Filter - Now Integrated ======

void filter_init(filter_t *f, float sample_hz) {
    if (!f) return;
    if (sample_hz <= 0) sample_hz = 500.0f;
    f->samplePeriod = 1.0f / sample_hz;

    f->twoKp = 2.0f;
    f->twoKi = 0.0f;
    f->q0 = 1; f->q1 = f->q2 = f->q3 = 0;
    f->integralFBx = f->integralFBy = f->integralFBz = 0;
    f->euler_deadzone = 0.5f;
    f->last_roll = f->last_pitch = f->last_yaw = 0;

    // Initialize all new filters to NONE
    for(int i = 0; i < MAX_FILTER_STAGES; i++) {
        for(int j = 0; j < 3; j++) {
            filter_init_advanced(&f->gyro_filters[i][j]);
            filter_init_advanced(&f->accel_filters[i][j]);
        }
    }
}

void filter_configure_gyro(filter_t *f, int stage, FilterType type, int freq, int cutoff) {
    if (!f || stage < 0 || stage >= MAX_FILTER_STAGES) return;
    FilterConfig conf = { .type = type, .freq = freq, .cutoff = cutoff };
    int rate = (f->samplePeriod > 0) ? (1.0f / f->samplePeriod) : 500;
    for(int i=0; i<3; i++) {
        filter_begin(&f->gyro_filters[stage][i], &conf, rate);
    }
}

void filter_configure_accel(filter_t *f, int stage, FilterType type, int freq, int cutoff) {
    if (!f || stage < 0 || stage >= MAX_FILTER_STAGES) return;
    FilterConfig conf = { .type = type, .freq = freq, .cutoff = cutoff };
    int rate = (f->samplePeriod > 0) ? (1.0f / f->samplePeriod) : 500;
    for(int i=0; i<3; i++) {
        filter_begin(&f->accel_filters[stage][i], &conf, rate);
    }
}

void filter_set_gains(filter_t *f, float Kp, float Ki) {
    if (!f) return;
    f->twoKp = 2.0f * Kp;
    f->twoKi = 2.0f * Ki;
}

void filter_set_deadzone(filter_t *f, float deg) {
    if (!f) return;
    if (deg < 0) deg = 0;
    f->euler_deadzone = deg;
}

void filter_update_raw(filter_t *f,
                       float ax_raw, float ay_raw, float az_raw,
                       float gx_raw, float gy_raw, float gz_raw)
{
    if (!f) return;

    // Apply all configured filters
    for(int i = 0; i < MAX_FILTER_STAGES; i++) {
        gx_raw = filter_update(&f->gyro_filters[i][0], gx_raw);
        gy_raw = filter_update(&f->gyro_filters[i][1], gy_raw);
        gz_raw = filter_update(&f->gyro_filters[i][2], gz_raw);
        ax_raw = filter_update(&f->accel_filters[i][0], ax_raw);
        ay_raw = filter_update(&f->accel_filters[i][1], ay_raw);
        az_raw = filter_update(&f->accel_filters[i][2], az_raw);
    }
    
    // --- Mahony algorithm continues with filtered data ---
    float gx = gx_raw * GYRO_SCALE;
    float gy = gy_raw * GYRO_SCALE;
    float gz = gz_raw * GYRO_SCALE;
    float ax = ax_raw * ACCEL_SCALE;
    float ay = ay_raw * ACCEL_SCALE;
    float az = az_raw * ACCEL_SCALE;

    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm <= 0.0f) return;
    ax /= norm; ay /= norm; az /= norm;

    float vx = 2.0f * (f->q1 * f->q3 - f->q0 * f->q2);
    float vy = 2.0f * (f->q0 * f->q1 + f->q2 * f->q3);
    float vz = f->q0 * f->q0 - f->q1 * f->q1 - f->q2 * f->q2 + f->q3 * f->q3;

    float ex = (ay * vz - az * vy);
    float ey = (az * vx - ax * vz);
    float ez = (ax * vy - ay * vx);

    if (f->twoKi > 0.0f) {
        f->integralFBx += f->twoKi * ex * f->samplePeriod;
        f->integralFBy += f->twoKi * ey * f->samplePeriod;
        f->integralFBz += f->twoKi * ez * f->samplePeriod;
        gx += f->integralFBx;
        gy += f->integralFBy;
        gz += f->integralFBz;
    } else {
        f->integralFBx = f->integralFBy = f->integralFBz = 0.0f;
    }

    gx += f->twoKp * ex;
    gy += f->twoKp * ey;
    gz += f->twoKp * ez;

    gx *= 0.5f * f->samplePeriod;
    gy *= 0.5f * f->samplePeriod;
    gz *= 0.5f * f->samplePeriod;

    float qa = f->q0, qb = f->q1, qc = f->q2;
    f->q0 += (-qb * gx - qc * gy - f->q3 * gz);
    f->q1 += (qa * gx + qc * gz - f->q3 * gy);
    f->q2 += (qa * gy - qb * gz + f->q3 * gx);
    f->q3 += (qa * gz + qb * gy - qc * gx);

    float invNorm = inv_sqrt(f->q0*f->q0 + f->q1*f->q1 + f->q2*f->q2 + f->q3*f->q3);
    f->q0 *= invNorm; f->q1 *= invNorm; f->q2 *= invNorm; f->q3 *= invNorm;
}

void filter_get_euler(filter_t *imu_right, filter_t *imu_left, float *roll, float *pitch, float *yaw, float *throttle){
    float roll_input  = 2.0f * (imu_right->q0 * imu_right->q1 + imu_right->q2 * imu_right->q3);
    float pitch_input = 2.0f * (imu_right->q1 * imu_right->q3 - imu_right->q0 * imu_right->q2);

    float yaw_input  = 2.0f * (imu_left->q0 * imu_left->q1 + imu_left->q2 * imu_left->q3);
    float throttle_input = 2.0f * (imu_left->q1 * imu_left->q3 - imu_left->q0 * imu_left->q2);
    
    *pitch = RAD2DEG(asinf(-pitch_input)); 
    *roll = RAD2DEG(asinf(roll_input)); 
    *yaw = RAD2DEG(asinf(yaw_input));   
    *throttle = RAD2DEG(asinf(-throttle_input)); 
}