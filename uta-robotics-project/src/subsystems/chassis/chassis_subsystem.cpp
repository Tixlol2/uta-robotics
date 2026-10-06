#include "chassis_subsystem.hpp"

#include <cmath>

namespace control::chassis
{
namespace
{
/// sqrt(2)/2, the sine/cosine of a 45 degree wheel mounting angle.
constexpr float SQRT2_OVER_2 = M_SQRT2 / 2.0f;

/// Converts RPM to radians/second.
inline float rpmToRadPerSec(float rpm)
{
    return rpm * (2.0f * static_cast<float>(M_PI)) / 60.0f;
}
}  // namespace

ChassisSubsystem::ChassisSubsystem(
    tap::Drivers* drivers,
    tap::motor::MotorId leftFrontId,
    tap::motor::MotorId rightFrontId,
    tap::motor::MotorId leftBackId,
    tap::motor::MotorId rightBackId,
    tap::can::CanBus canBus,
    float wheelRadius,
    float wheelbaseRadius,
    const tap::algorithms::SmoothPidConfig& velocityPidConfig,
    float maxTranslationalAccel,
    float maxRotationalAccel)

    : tap::control::Subsystem(drivers),
      drivers(drivers),
      motors{
          tap::motor::DjiMotor(drivers, leftFrontId, canBus, false, "LF Wheel"),
          tap::motor::DjiMotor(drivers, rightFrontId, canBus, false, "RF Wheel"),
          tap::motor::DjiMotor(drivers, leftBackId, canBus, false, "LB Wheel"),
          tap::motor::DjiMotor(drivers, rightBackId, canBus, false, "RB Wheel"),
      },
      velocityPid{
          tap::algorithms::SmoothPid(velocityPidConfig),
          tap::algorithms::SmoothPid(velocityPidConfig),
          tap::algorithms::SmoothPid(velocityPidConfig),
          tap::algorithms::SmoothPid(velocityPidConfig),
      },
      wheelVelToChassisVelMat(),
      desiredWheelRpm(),
      xRamp(0.0f),
      yRamp(0.0f),
      rRamp(0.0f),
      maxTranslationalAccel(maxTranslationalAccel),
      maxRotationalAccel(maxRotationalAccel)
{
    computeKinematics(wheelRadius, wheelbaseRadius);
    desiredWheelRpm = desiredWheelRpm.zeroMatrix();
}

void ChassisSubsystem::computeKinematics(float wheelRadius, float wheelbaseRadius)
{
    // Each wheel is mounted at a fixed +/-45 degree angle to the chassis's
    // forward axis, so its contribution to chassis-relative x and y
    // velocity is +/- sqrt(2)/2 of its own rolling speed. Rotational
    // contribution is the same sign for all four wheels (they all spin the
    // same direction to rotate the chassis in place), scaled by the
    // distance from the wheel to the chassis's center of rotation.
    wheelVelToChassisVelMat[X][LF] = SQRT2_OVER_2;
    wheelVelToChassisVelMat[X][RF] = -SQRT2_OVER_2;
    wheelVelToChassisVelMat[X][LB] = SQRT2_OVER_2;
    wheelVelToChassisVelMat[X][RB] = -SQRT2_OVER_2;

    wheelVelToChassisVelMat[Y][LF] = SQRT2_OVER_2;
    wheelVelToChassisVelMat[Y][RF] = SQRT2_OVER_2;
    wheelVelToChassisVelMat[Y][LB] = -SQRT2_OVER_2;
    wheelVelToChassisVelMat[Y][RB] = -SQRT2_OVER_2;

    wheelVelToChassisVelMat[R][LF] = 1.0f / wheelbaseRadius;
    wheelVelToChassisVelMat[R][RF] = 1.0f / wheelbaseRadius;
    wheelVelToChassisVelMat[R][LB] = 1.0f / wheelbaseRadius;
    wheelVelToChassisVelMat[R][RB] = 1.0f / wheelbaseRadius;

    wheelVelToChassisVelMat *= (wheelRadius / 4.0f);
}

void ChassisSubsystem::initialize()
{
    for (auto& motor : motors)
    {
        motor.initialize();
    }
}

void ChassisSubsystem::calculateDesiredWheelRpm(float x, float y, float r)
{
    // Inverse kinematics: each wheel's desired RPM is a linear combination
    // of the requested x, y, and r chassis velocities, with signs mirroring
    // the forward kinematics matrix built in computeKinematics().
    desiredWheelRpm[LF][0] = x + y - r;
    desiredWheelRpm[RF][0] = -x + y - r;
    desiredWheelRpm[LB][0] = x - y - r;
    desiredWheelRpm[RB][0] = -x - y - r;
}

void ChassisSubsystem::setDesiredOutput(float x, float y, float r)
{
    // Only the ramps' *targets* move immediately; the ramps themselves are
    // stepped forward in refresh(), at a bounded rate, regardless of how
    // abruptly the caller's x/y/r changes here. This is what keeps a sudden
    // stick snap or a beyblade toggle from demanding an instant speed/
    // direction change that could tip the chassis.
    xRamp.setTarget(x);
    yRamp.setTarget(y);
    rRamp.setTarget(r);
}

void ChassisSubsystem::setZeroRPM()
{
    // Ramp down to zero rather than snapping -- an instant stop from a high
    // speed or spin rate is exactly the kind of abrupt deceleration that can
    // tip the chassis, same as an abrupt start.
    setDesiredOutput(0.0f, 0.0f, 0.0f);
}

void ChassisSubsystem::refresh()
{
    const uint32_t now = tap::arch::clock::getTimeMilliseconds();
    const float dt = static_cast<float>(now - lastUpdateTimeMs) / 1000.0f;
    lastUpdateTimeMs = now;

    drivers->leds.set(
        drivers->leds.Green,
        !allMotorsOnline() &&
            !drivers->remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_VERTICAL) <
                0.1f);

    // Advance each ramp toward its target by at most (accel limit * dt),
    // then recompute the wheel RPMs from the ramped (not raw) x/y/r. This
    // runs every tick regardless of whether setDesiredOutput() was just
    // called, since a ramp in progress still needs to keep moving toward
    // its target.
    xRamp.update(maxTranslationalAccel * dt);
    yRamp.update(maxTranslationalAccel * dt);
    rRamp.update(maxRotationalAccel * dt);

    calculateDesiredWheelRpm(xRamp.getValue(), yRamp.getValue(), rRamp.getValue());

    for (int i = 0; i < getNumChassisMotors(); i++)
    {
        const float measuredRpm = static_cast<float>(motors[i].getShaftRPM());
        const float error = desiredWheelRpm[i][0] - measuredRpm;
        const float pidOutput = velocityPid[i].runControllerDerivateError(error, dt);
        motors[i].setDesiredOutput(pidOutput);
    }
}

void ChassisSubsystem::setRoughDrive(float x, float y, float r)
{
    desiredWheelRpm[LF][0] = x;
    desiredWheelRpm[RF][0] = y;
    desiredWheelRpm[LB][0] = r;
    desiredWheelRpm[RB][0] = r;
}

void ChassisSubsystem::refreshSafeDisconnect()
{
    for (auto& motor : motors)
    {
        motor.setDesiredOutput(0);
    }
}

bool ChassisSubsystem::allMotorsOnline() const
{
    for (const auto& motor : motors)
    {
        if (!motor.isMotorOnline())
        {
            return false;
        }
    }
    return true;
}

modm::Matrix<float, 3, 1> ChassisSubsystem::getActualVelocityChassisRelative() const
{
    modm::Matrix<float, 4, 1> wheelVel;
    for (int i = 0; i < getNumChassisMotors(); i++)
    {
        wheelVel[i][0] = rpmToRadPerSec(static_cast<float>(motors[i].getShaftRPM()));
    }

    return wheelVelToChassisVelMat * wheelVel;
}

void ChassisSubsystem::rotateFieldRelativeToChassisRelative(
    float fieldX,
    float fieldY,
    float yawRadians,
    float* chassisX,
    float* chassisY)
{
    // Standard 2D rotation by -yawRadians.
    const float cosYaw = cosf(yawRadians);
    const float sinYaw = sinf(yawRadians);

    *chassisY = fieldX * cosYaw + fieldY * sinYaw;
    *chassisX = -fieldX * sinYaw + fieldY * cosYaw;
}

}  // namespace control::chassis