#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#include "pid.h"

/*-------------------------------------------------------------------------------------------------------*/
/*					  		 	HARDWARE & PLATFORM CONSTANTS		      						         */
/*-------------------------------------------------------------------------------------------------------*/

/**
 * @def STANDARD_MESSAGE_LENGTH
 *
 * @brief this macro is used to specify the length of the message for the output
 */
#define STANDARD_MESSAGE_LENGTH															200

/**
 * @def NUMBER_OF_TOGGLES
 *
 * @brief This macro is used to specify (or set) the number of seconds before the firmware startup.
 * 		  It defines the number of the toggles of the LED on NUCLEO-H745ZIQ. The number of seconds
 * 		  is obtained by dividing this value by two.
 */
#define NUMBER_OF_TOGGLES																10

/**
 * @def CCR_PER_DEGREE
 *
 * @brief This macro defines the ratio of CCR/degree of servos (based on measurement: 150 CCR / 15°).
 */
#define CCR_PER_DEGREE      10.0f

/**
 * @def MAX_FLAP_ANGLE_DEG
 *
 * @brief This macro defines the ROM of servo motors in degrees for both directions. So total ROM is 60°.
 */
#define MAX_FLAP_ANGLE_DEG  30.0f  /* Maximum safe physical angle for the flap */


/*-------------------------------------------------------------------------------------------------------*/
/*					  		 			MOTOR-RELATED MACROS 				      						 */
/*-------------------------------------------------------------------------------------------------------*/


/**
 * @def UPPER_LIMIT_MOTOR
 *
 * @brief This macro defines the upper limit for the compare register value used in PWM generation
 * 		  for the control of the motors.
 * 		  The value is calculated considering a prescaler of 99 and a period of 14999. Since
 * 		  standard RC maximum throttle is ~2000 μsμs (2.0 ms), it corresponds to 10.0% of duty cycle.
 *
 */
#define UPPER_LIMIT_MOTOR 1500

/**
 * @def LOWER_LIMIT_MOTOR
 *
 * @brief This macro defines the lower limit for the compare register value used in PWM generation
 * 		  for the control of the motors.
 * 		  The value is calculated considering a prescaler of 99 and a period of 14999. Since
 * 		  standard RC minimum throttle: ~1000 μsμs (1.0 ms), it corresponds to 5.0% of duty cycle.
 */
#define LOWER_LIMIT_MOTOR 700

/*-------------------------------------------------------------------------------------------------------*/
/*					  		 			SERVOS-RELATED MACROS 				      					     */
/*-------------------------------------------------------------------------------------------------------*/

/**
 * @def UPPER_LIMIT_SERVO
 *
 * @brief This macro defines the upper limit for the compare register value used in PWM generation
 * 		  for the control of the servos.
 * 		  This limit is applied in the anti-windup filter of the servo PID controller. The value is
 * 		  calculated considering a frequency of 50 Hz, a prescaler of 99 and a period of 14999.
 * 		  This value rotates the servo motor clockwise to its maximum range: 30°.
 *
 */
#define UPPER_LIMIT_SERVO													1545

/**
 * @def CENTER_SERVO
 *
 * @brief This macro defines the value to write into the compare register to set the PWM duty cycle
 * 		  corresponding to the servo position at 0°
 */
#define CENTER_SERVO												1125

/**
 * @def LOWER_LIMIT_SERVO
 *
 * @brief This macro defines the lower limit for the compare register value used in PWM generation
 * 		  for the control of the servos.
 * 		  This limit is applied in the anti-windup filter of the servo PID controller. The value is
 * 		  calculated considering a frequency of 50 Hz, a prescaler of 99 and a period of 14999.
 * 		  This value rotates the servo motor counter clockwise to its maximum range: -30°.
 */
#define LOWER_LIMIT_SERVO													705

/*  Timing (seconds) */
#define SERVO_LOOP_SAMPLE_TIME           0.01f       /* Equivalent to 100 Hz */
#define MOTOR_LOOP_SAMPLE_TIME           0.033f      /* Equivalent to ~30 Hz */

/*-------------------------------------------------------------------------------------------------------*/
/*                                    CONTROL & TUNABLE CONSTANTS                                        */
/*-------------------------------------------------------------------------------------------------------*/

/* Flight configuration struct */
typedef struct {
    float    ref_roll_pitch;
    uint16_t ref_altitude;
    float    ref_yaw;
    float    yaw_trim;

    pid_controller_t servo_roll;
    pid_controller_t servo_pitch;
    pid_controller_t motor;
    pid_controller_t yaw;
} flight_config_t;

/*  Single ROM instance (defined in config.c) */
extern const flight_config_t FLIGHT_CFG;

#endif /* INC_CONFIG_H_ */
