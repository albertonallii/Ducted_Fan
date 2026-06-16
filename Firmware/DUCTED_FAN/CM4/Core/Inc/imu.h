
#ifndef INC_IMU_H_
#define INC_IMU_H_

#pragma once

typedef struct {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
} imu_angles_t;

/** Read current orientation and store as zero reference. */
void imu_set_reference(imu_angles_t *ref);

/** Read current orientation relative to stored reference. */
void imu_read_relative(const imu_angles_t *ref, imu_angles_t *out);

/** Read current orientation relative to ground reference. */
void imu_read_absolute(imu_angles_t *out);


typedef struct {
    float flap_roll;
    float flap_pitch;
} flap_axes_t;

/**
 * @brief Rotate IMU-frame errors into flap-frame errors.
 *
 * Applies a fixed 2D rotation by the mechanical offset angle
 * between the IMU measurement axes and the flap actuation axes.
 */
void axis_remap_imu_to_flaps(float imu_roll, float imu_pitch,flap_axes_t *out);


#endif /* INC_IMU_H_ */
