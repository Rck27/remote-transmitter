#ifndef FILTER_H
#define FILTER_H

#include <stdint.h>

#define MAX_FILTER_STAGES 4 // Maximum number of filters you can chain for gyro or accel

// Enum for the different filter types available
typedef enum {
    FILTER_NONE = 0,
    FILTER_PT1,
    FILTER_BIQUAD,
    FILTER_NOTCH,
    FILTER_NOTCH_DF1,
    FILTER_BPF,
    FILTER_FIR2,
    FILTER_MEDIAN3,
    FILTER_PT2,
    FILTER_PT3,
    FILTER_FO, // First Order
} FilterType;

// Enum for biquad filter types
typedef enum {
    BIQUAD_FILTER_LPF,
    BIQUAD_FILTER_NOTCH,
    BIQUAD_FILTER_BPF,
} BiquadFilterType;


// Configuration for a filter
typedef struct {
    FilterType type;
    int16_t freq;
    int16_t cutoff;
} FilterConfig;

// Filter states
// Note: We use a union to save memory, as a filter can only be of one type at a time.
typedef struct { float v; float k; } FilterStatePt1;
typedef struct { float v[2]; } FilterStateFir2;
typedef struct { float x1, x2, y1, y2; float b0, b1, b2, a1, a2; } FilterStateBiquad;
typedef struct { float x1, y1; float b0, b1, a1; } FilterStateFirstOrder;
typedef struct { float v[3]; } FilterStateMedian;
typedef struct { float v[2]; float k; } FilterStatePt2;
typedef struct { float v[3]; float k; } FilterStatePt3;

typedef union {
    FilterStatePt1 pt1;
    FilterStateFir2 fir2;
    FilterStateBiquad bq;
    FilterStateFirstOrder fo;
    FilterStateMedian median;
    FilterStatePt2 pt2;
    FilterStatePt3 pt3;
} FilterState;

// Main filter object
typedef struct {
    FilterConfig _conf;
    FilterState _state;
    int _rate;
    float _output_weight;
    float _input_weight;
} Filter;

// Your original IMU filter struct, now with added filters for gyro and accel
typedef struct {
    float samplePeriod;
    float twoKp;
    float twoKi;
    float q0, q1, q2, q3;
    float integralFBx, integralFBy, integralFBz;
    float euler_deadzone;
    float last_roll, last_pitch, last_yaw;

    // NEW: Filters for accelerometer and gyroscope
    Filter gyro_filters[MAX_FILTER_STAGES][3]; // [stage][x,y,z]
    Filter accel_filters[MAX_FILTER_STAGES][3]; // [stage][x,y,z]

} filter_t;


// ====== New Advanced Filter Functions ======
void filter_init_advanced(Filter* filter);
void filter_begin(Filter* filter, const FilterConfig* config, int rate);
float filter_update(Filter* filter, float v);

// ====== Your Original IMU Filter Functions ======
void filter_init(filter_t *f, float sample_hz);
void filter_set_gains(filter_t *f, float Kp, float Ki);
void filter_set_deadzone(filter_t *f, float deg);
void filter_update_raw(filter_t *f, float ax_raw, float ay_raw, float az_raw, float gx_raw, float gy_raw, float gz_raw);
void filter_get_euler(filter_t *imu_right, filter_t *imu_left, float *roll, float *pitch, float *yaw, float *throttle);

// ====== New Configuration Helpers for Gyro/Accel ======
void filter_configure_gyro(filter_t *f, int stage, FilterType type, int freq, int cutoff);
void filter_configure_accel(filter_t *f, int stage, FilterType type, int freq, int cutoff);

#endif // FILTER_H