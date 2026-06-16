#include "imu.h"
#include "bno055.h"

static void imu_read_raw(imu_angles_t *out)
{
    struct bno055_euler_float_t euler;
    bno055_convert_float_euler_hpr_deg(&euler);

    out->roll_deg  = euler.r;      /* float preserves value with no truncation */
    out->pitch_deg = euler.p;
    out->yaw_deg   = euler.h;
}

void imu_read_absolute(imu_angles_t *out)
{
    imu_read_raw(out);
}


void imu_set_reference(imu_angles_t *ref)
{
    imu_read_raw(ref);
}

void imu_read_relative(const imu_angles_t *ref, imu_angles_t *out)
{
    imu_angles_t raw;
    imu_read_raw(&raw);

    out->roll_deg  = raw.roll_deg  - ref->roll_deg;
    out->pitch_deg = raw.pitch_deg - ref->pitch_deg;

    /* Yaw requires wraparound handling (BNO055 heading: [0, 360)) */
	float yaw_diff = raw.yaw_deg - ref->yaw_deg;

	/* Normalize to [-180, +180] */
	if (yaw_diff > 180.0f)
		yaw_diff -= 360.0f;
	else if (yaw_diff < -180.0f)
		yaw_diff += 360.0f;

	out->yaw_deg = yaw_diff;
}

/*
 * Mechanical offset: 15° between IMU axes and flap axes.
 * Precomputed — no runtime trigonometry
 *
 * cos(15°) = 0.96592582f
 * sin(15°) = 0.25881905f
 *
 * Rotation matrix  R(+15°):
 *   [ cos  sin ]     IMU → Flap
 *   [-sin  cos ]
 *
 * Sign convention: positive rotation = IMU x-axis toward flap-1 axis.
 * Verify on your airframe; negate the angle if servos respond backwards.
 */
#define OFFSET_COS  0.96592582f
#define OFFSET_SIN  0.25881905f

void axis_remap_imu_to_flaps(float imu_roll, float imu_pitch, flap_axes_t *out)
{
    out->flap_roll  = ( OFFSET_COS * imu_roll + OFFSET_SIN * imu_pitch);
    out->flap_pitch = -(-OFFSET_SIN * imu_roll + OFFSET_COS * imu_pitch);
}
