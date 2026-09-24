#include "chassis_field_relative_math.hpp"

#include <cmath>

namespace control::chassis
{
void rotateFieldRelativeToChassisRelative(
    float fieldX,
    float fieldY,
    float yawRadians,
    float* chassisX,
    float* chassisY)
{
    // Standard 2D rotation by -yawRadians.
    const float cosYaw = cosf(yawRadians);
    const float sinYaw = sinf(yawRadians);

    *chassisX = fieldX * cosYaw + fieldY * sinYaw;
    *chassisY = -fieldX * sinYaw + fieldY * cosYaw;
}

}  // namespace control::chassis
