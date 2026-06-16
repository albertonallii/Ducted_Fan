#include "config.h"

/*-------------------------------------------------------------------------------------------------------*/
/*                             COMPILE-TIME FLIGHT CONFIGURATION 						                 */
/*-------------------------------------------------------------------------------------------------------*/
const flight_config_t FLIGHT_CFG = {

    /* References */
    .ref_roll_pitch   = 0.0f, // Degrees
    .ref_altitude     = 100U, // Millimeters
    .ref_yaw          = 0.0f, // Degrees
    .yaw_trim         = 0.0f, // Degrees

	/*                    Servo (flap/IMU)      Motor (propeller/ToF)    Yaw (propeller/IMU)
						  ──────────────────    ──────────────────────   ──────────────────────
		Median filter     No  (Gaussian noise)  Yes (impulse spikes)     Yes (impulse spikes)
		Deriv mode        On error              On measurement           On error
		LPF on D          α = 1.0 (off)         α = 0.3 (moderate)       α = 0.3 (moderate)
		Sensor filtering  No                    Median at PID input      No
	 */

    /* Servo Roll */
    .servo_roll = {
        .kp                  = -27.0f,
        .ki                  = -34.0f,
        .kd                  = 5.0f,
        .sample_time         = SERVO_LOOP_SAMPLE_TIME,
        .out_min             = -MAX_FLAP_ANGLE_DEG,
        .out_max             =  MAX_FLAP_ANGLE_DEG,
        .out_offset          = 0.0f,
        .lpf_alpha           = 0.57f,     /* Pass-through */
        .deriv_on_measurement = false,
    },

    /* Servo Pitch */
    .servo_pitch = {
        .kp                  = 11.17f,
        .ki                  = 35.0f,
        .kd                  = 0.0f,
        .sample_time         = SERVO_LOOP_SAMPLE_TIME,
        .out_min             = -MAX_FLAP_ANGLE_DEG,
        .out_max             =  MAX_FLAP_ANGLE_DEG,
        .out_offset          = 0.0f,
        .lpf_alpha           = 1.0f,     /* Pass-through */
        .deriv_on_measurement = false,
    },

    /* Motor (Altitude) */
    .motor = {
        .kp                  = 0.0f,
        .ki                  = 0.0f,
        .kd                  = 0.0f,
        .sample_time         = MOTOR_LOOP_SAMPLE_TIME,
        .out_min             = (float)LOWER_LIMIT_MOTOR,
        .out_max             = (float)UPPER_LIMIT_MOTOR,
        .out_offset          = 1300.0f,  // It counteracts gravity and maintain a stable hover. Found empirically.
        .lpf_alpha           = 0.3f,     /* Moderate filter */
        .deriv_on_measurement = true,
    },

    /* Yaw */
    .yaw = {
        .kp                  = 7.0f,
        .ki                  = 0.02f,
        .kd                  = 0.15f,
        .sample_time         = SERVO_LOOP_SAMPLE_TIME,
        .out_min             = -(((float)UPPER_LIMIT_MOTOR - (float)LOWER_LIMIT_MOTOR) / 2.0f),
        .out_max             =  (((float)UPPER_LIMIT_MOTOR - (float)LOWER_LIMIT_MOTOR) / 2.0f),
        .out_offset          = 0.0f,
        .lpf_alpha           = 0.3f,     /* Moderate filter */
        .deriv_on_measurement = false,
    },
};
