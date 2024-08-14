#pragma once

#include <stdbool.h>

#define AXES_COUNT 3

#define ACCEL_THRESHOLD 0.008f
#define GYRO_THRESHOLD 0.01f
#define STEADY_PERIOD 5
// #define FREEZE_PERIOD 2

typedef enum {
    GC_MOVING,
    GC_STEADY
} GyroCalibrationState;

typedef struct {
    float accel[AXES_COUNT];
    float accel_d[AXES_COUNT];
    float gyro[AXES_COUNT];

    GyroCalibrationState state;

    float old_time;
    float steady_time;
} GyroCalibration;

void gyro_calibration_init(GyroCalibration *gc);

void gyro_calibration_update(GyroCalibration *gc, float time);
