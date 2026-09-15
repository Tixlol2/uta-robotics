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
    const tap::algorithms::SmoothPidConfig& velocityPidConfig)
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
      desiredWheelRpm()
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
    calculateDesiredWheelRpm(x, y, r);
}

void ChassisSubsystem::setZeroRPM() { desiredWheelRpm = desiredWheelRpm.zeroMatrix(); }

void ChassisSubsystem::refresh()
{
    const uint32_t now = tap::arch::clock::getTimeMilliseconds();
    const float dt = static_cast<float>(now - lastUpdateTimeMs) / 1000.0f;
    lastUpdateTimeMs = now;

    for (int i = 0; i < getNumChassisMotors(); i++)
    {
        const float measuredRpm = static_cast<float>(motors[i].getShaftRPM());
        const float error = desiredWheelRpm[i][0] - measuredRpm;
        const float pidOutput = velocityPid[i].runControllerDerivateError(error, dt);
        motors[i].setDesiredOutput(static_cast<int32_t>(pidOutput));
    }
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

}  // namespace control::chassis