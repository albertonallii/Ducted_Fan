#include "pid.h"
#include <string.h>

/* Median Filter */
float median_filter_compute(median_filter_t *filt, float new_val)
{
    if (!filt->initialized) {
        filt->buf[0] = filt->buf[1] = filt->buf[2] = new_val;
        filt->initialized = true;
    }

    filt->buf[filt->idx] = new_val;
    filt->idx = (filt->idx + 1) % 3;

    float a = filt->buf[0], b = filt->buf[1], c = filt->buf[2];

    return (a > b) ? ((b > c) ? b : ((a > c) ? c : a))
                   : ((a > c) ? a : ((b > c) ? c : b));
}


void pid_init(pid_controller_t *pid,
              float kp, float ki, float kd, float sample_time,
              float out_min, float out_max, float out_offset,
              float lpf_alpha, bool deriv_on_measurement)
{
    if (!pid || sample_time <= 0.0f)
        return;

    /* Zero all fields (state + padding) then set configuration */
    memset(pid, 0, sizeof(*pid));

    pid->kp          = kp;
    pid->ki          = ki;
    pid->kd          = kd;
    pid->sample_time = sample_time;

    pid->out_min     = out_min;
    pid->out_max     = out_max;
    pid->out_offset  = out_offset;

    pid->lpf_alpha            = lpf_alpha;
    pid->deriv_on_measurement = deriv_on_measurement;

    /* State fields are already 0 after memset; initialized = false */
}

float pid_compute(pid_controller_t *pid, float setpoint, float measurement)
{
    // First call: seed prev_meas to avoid a derivative spike
    if (!pid->initialized) {
        pid->prev_meas   = measurement;
        pid->prev_error = setpoint - measurement;
        pid->initialized = true;
    }

    // --- Error ---
    float error = setpoint - measurement;

    // --- Proportional ---
    float p_term = pid->kp * error;

    // --- Integral (forward Euler) ---
    float new_int = pid->int_term + (pid->ki * pid->sample_time * error);

    // --- Derivative --
    float d_raw;
    if (pid->deriv_on_measurement) {
        // On measurement: avoids derivative kick on setpoint change
        d_raw = -(pid->kd / pid->sample_time) * (measurement - pid->prev_meas);
    } else {
        // On error: backward difference
        d_raw = (pid->kd / pid->sample_time) * (error - pid->prev_error);
    }

    // Low-pass filter on derivative
    float d_term = (pid->lpf_alpha * d_raw)
                 + ((1.0f - pid->lpf_alpha) * pid->prev_d_term);

    // --- Total output with bias ---
    float output = pid->out_offset + p_term + new_int + d_term;

    // --- Anti-windup (clamping) & saturation
    if (output > pid->out_max) {
        output = pid->out_max;
        if (error < 0.0f)           // integrate only if it helps recover
            pid->int_term = new_int;
    } else if (output < pid->out_min) {
        output = pid->out_min;
        if (error > 0.0f)
            pid->int_term = new_int;
    } else {
        pid->int_term = new_int;    // normal accumulation
    }

    // --- Save state for next iteration ---
    pid->prev_error  = error;
    pid->prev_meas   = measurement;
    pid->prev_d_term = d_term;

    return output;
}
