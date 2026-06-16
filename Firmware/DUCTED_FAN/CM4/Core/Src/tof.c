#include "tof.h"
#include <math.h>

#define DEG_TO_RAD  (3.14159265f / 180.0f)

float tof_compensate_tilt(float range_mm, float roll_deg, float pitch_deg)
{
    float cr = cosf(roll_deg  * DEG_TO_RAD);   /* cosf, not cos */
    float cp = cosf(pitch_deg * DEG_TO_RAD);
    return range_mm * cr * cp;
}
