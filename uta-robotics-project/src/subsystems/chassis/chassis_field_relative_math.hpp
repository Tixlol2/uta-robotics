#ifndef CHASSIS_FIELD_RELATIVE_MATH_HPP_
#define CHASSIS_FIELD_RELATIVE_MATH_HPP_

namespace control::chassis
{
/**
 * Rotates a field-relative (fieldX, fieldY) translation command by
 * -yawRadians, re-expressing it in the chassis's current body frame -- the
 * frame ChassisSubsystem::setDesiredOutput expects its x/y arguments in.
 *
 * yawRadians is the chassis's current IMU yaw, in radians, using the same
 * sign convention as tap's AbstractIMU::getYaw() (typically positive =
 * counterclockwise, viewed from above, zeroed wherever the IMU was
 * calibrated).
 *
 * Shared by any chassis command that wants operator input to mean "this
 * direction across the field" rather than "this direction relative to
 * whichever way the chassis body currently happens to be facing" -- most
 * importantly while beyblading, but equally useful for normal driving.
 */
void rotateFieldRelativeToChassisRelative(
    float fieldX,
    float fieldY,
    float yawRadians,
    float* chassisX,
    float* chassisY);

}  // namespace control::chassis

#endif  // CHASSIS_FIELD_RELATIVE_MATH_HPP_
