#include "gyro_calibration.h"

#include "utils.h"

#include <vesc_c_if.h>

#include <math.h>

void gyro_calibration_init(GyroCalibration *gc) {
    for (uint8_t i = 0; i < AXES_COUNT; ++i) {
        gc->accel[i] = 0;
        gc->gyro[i] = 0;
    }
    gc->state = GC_MOVING;
    // simplification hack: Old_time only used for derivation, to not have to
    // special-case a zero initialization, set a time 1ms behind to get a
    // reasonable first derivation.
    gc->old_time = VESC_IF->system_time() - 0.001f;
}

void gyro_calibration_update(GyroCalibration *gc, float time) {
    float raw_accel[AXES_COUNT];
    VESC_IF->imu_get_accel(raw_accel);

    float time_diff = time - gc->old_time;
    gc->old_time = time;

    bool steady = true;
    for (uint8_t i = 0; i < AXES_COUNT; ++i) {
        float old_accel = gc->accel[i];
        gc->accel[i] = gc->accel[i] * 0.999f + raw_accel[i] * 0.001f;
        gc->accel_d[i] = (gc->accel[i] - old_accel) / time_diff;
    }

    // maximum of absolute values of the three accelerometer axes
    float max_accel_d =
        fmaxf(fabsf(gc->accel_d[0]), fmaxf(fabsf(gc->accel_d[1]), fabsf(gc->accel_d[2])));
    if (max_accel_d > 0.01f) {
        steady = false;
    }

    if (steady && gc->state == GC_MOVING) {
        gc->state = GC_STEADY;
        gc->steady_time = time;
    } else if (!steady && gc->state == GC_STEADY) {
        gc->state = GC_MOVING;
    }

    if (gc->state == GC_STEADY && time - gc->steady_time > STEADY_PERIOD) {
        if (fabsf(gc->gyro[0]) > GYRO_THRESHOLD || fabsf(gc->gyro[1]) > GYRO_THRESHOLD ||
            fabsf(gc->gyro[2]) > GYRO_THRESHOLD) {
            log_msg("WRITE GYRO");
            VESC_IF->set_cfg_float(
                CFG_PARAM_IMU_gyro_offset_x,
                VESC_IF->get_cfg_float(CFG_PARAM_IMU_gyro_offset_x) + gc->gyro[0]
            );
            VESC_IF->set_cfg_float(
                CFG_PARAM_IMU_gyro_offset_y,
                VESC_IF->get_cfg_float(CFG_PARAM_IMU_gyro_offset_y) + gc->gyro[1]
            );
            VESC_IF->set_cfg_float(
                CFG_PARAM_IMU_gyro_offset_z,
                VESC_IF->get_cfg_float(CFG_PARAM_IMU_gyro_offset_z) + gc->gyro[2]
            );
            VESC_IF->store_cfg();
            gc->state = GC_MOVING;
        }
    }

    float raw_gyro[AXES_COUNT];
    VESC_IF->imu_get_gyro(raw_gyro);

    for (uint8_t i = 0; i < AXES_COUNT; ++i) {
        gc->gyro[i] = gc->gyro[i] * 0.998f + raw_gyro[i] * 0.002f;
    }
}
