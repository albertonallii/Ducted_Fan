#ifndef INC_TOF_H_
#define INC_TOF_H_

#pragma once

/**
 * @brief  Correct a ToF range for sensor tilt.
 *
 * Geometry:  h = d · cos(roll) · cos(pitch)
 * Valid for tilt < ~20°; exact for single-axis tilt.
 *
 * @param range_mm   Raw (or median-filtered) range in mm
 * @param roll_deg   Roll  relative to level (degrees)
 * @param pitch_deg  Pitch relative to level (degrees)
 * @return Estimated vertical distance in mm
 */
float tof_compensate_tilt(float range_mm, float roll_deg, float pitch_deg);

#endif /* INC_TOF_H_ */
