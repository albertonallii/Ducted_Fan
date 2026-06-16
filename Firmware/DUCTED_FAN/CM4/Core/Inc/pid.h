#ifndef INC_PID_H_
#define INC_PID_H_

#pragma once

#include <stdint.h>
#include <stdbool.h>

/* Median Filter*/
typedef struct {
	float buf[3];
    uint8_t idx;
    bool    initialized;
} median_filter_t;

float median_filter_compute(median_filter_t *filt, float new_val);

/* PID Controller */
typedef struct {
    /* Gains */
    float kp;
    float ki;
    float kd;
    float sample_time;

    /* Output limits & bias */
    float out_min;
    float out_max;
    float out_offset;           /* Added before clamping (e.g. servo center PWM) */

    /* Configuration */
    float lpf_alpha;            /* LPF on D: 1.0 = pass-through, 0.0 = full filter */
    bool  deriv_on_measurement; /* true = avoids derivative kick */

    /* State (zeroed by pid_init) */
    float int_term;
    float prev_error;
    float prev_meas;
    float prev_d_term;
    bool  initialized;
} pid_controller_t;

/**
 * @brief Initialise a PID controller. Must be called before pid_compute().
 * @note  Passing sample_time <= 0 is a no-op.
 */
void  pid_init(pid_controller_t *pid,
               float kp, float ki, float kd, float sample_time,
               float out_min, float out_max, float out_offset,
               float lpf_alpha, bool deriv_on_measurement);

/**
 * @brief Run one PID iteration.
 * @pre   pid must point to a valid, initialised instance (no NULL check).
 * @return Continuous control output (caller casts/rounds as needed).
 */
float pid_compute(pid_controller_t *pid, float setpoint, float measurement);
#endif /* INC_PID_H_ */
