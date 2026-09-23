#ifndef CHASSIS_SUBSYSTEM_HPP_
#define CHASSIS_SUBSYSTEM_HPP_

// --- Framework ---------------------------------------------------------
#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"

// --- Motors --------------------------------------------------------------
#include "tap/motor/dji_motor.hpp"
#include "tap/motor/motor_interface.hpp"

// --- Control / math --------------------------------------------------------
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/clock.hpp"
#include "modm/math/matrix.hpp"

#include <array>
#include <cstdint>

namespace control::chassis
{
/**
 * A 4-motor X-drive chassis subsystem, the standard holonomic drivetrain
 * layout seen on ARC-class robots: four omni wheels, each physically
 * mounted rotated ~45 degrees relative to the chassis's forward axis
 * (rather than mounted straight, as with mecanum wheels). This lets each
 * wheel contribute to both translation and rotation without angled
 * rollers on the wheel itself.
 *
 * Coordinate convention (right-handed, viewed from above):
 *   +x -> forward
 *   +y -> left
 *   +r -> counter-clockwise rotation
 *
 * All four DjiMotors, their velocity PID controllers, and the wheel
 * kinematics matrix are owned directly by this class -- there is no
 * separate interface/base-class split, so everything needed to run the
 * chassis lives in this one file.
 */
class ChassisSubsystem : public tap::control::Subsystem
{
public:
    /// Indexes into the motor array, PID array, and kinematics matrix columns.
    enum WheelIndex : uint8_t
    {
        LF = 0,  ///< Left front
        RF = 1,  ///< Right front
        LB = 2,  ///< Left back
        RB = 3,  ///< Right back
    };

    /// Indexes into 3-row chassis velocity vectors/matrices.
    enum ChassisVelIndex : uint8_t
    {
        X = 0,
        Y = 1,
        R = 2,
    };

    /**
     * @param drivers Pointer to the robot's Drivers object.
     * @param leftFrontId/... CAN motor IDs for each of the four drive motors.
     * @param canBus Which CAN bus all four chassis motors are wired to.
     * @param wheelRadius Radius of each wheel, in meters (used to convert
     *      between wheel angular velocity and chassis linear velocity).
     * @param wheelbaseRadius Distance from the chassis's center of rotation
     *      to each wheel's contact patch, in meters.
     * @param velocityPidConfig PID gains used to close the loop on each
     *      wheel's velocity (RPM) independently.
     */
    ChassisSubsystem(
        tap::Drivers* drivers,
        tap::motor::MotorId leftFrontId,
        tap::motor::MotorId rightFrontId,
        tap::motor::MotorId leftBackId,
        tap::motor::MotorId rightBackId,
        tap::can::CanBus canBus,
        float wheelRadius,
        float wheelbaseRadius,
        const tap::algorithms::SmoothPidConfig& velocityPidConfig);

    /// Initializes all four chassis motors over CAN. Must be called once
    /// before the chassis will respond to setDesiredOutput().
    void initialize() override;

    /// Called every scheduler tick: runs each wheel's velocity PID and
    /// pushes the resulting output to the corresponding DjiMotor.
    void refresh() override;

    /// Sends zero output to every motor immediately; used as a safety
    /// fallback if the subsystem becomes disconnected mid-tick.
    void refreshSafeDisconnect() override;

    /**
     * Sets the desired chassis velocity. Internally converts the
     * requested x/y/r velocity into a per-wheel RPM setpoint via the
     * X-drive inverse kinematics matrix.
     *
     * @param x Desired forward/backward wheel speed contribution (RPM-scale).
     * @param y Desired left/right wheel speed contribution (RPM-scale).
     * @param r Desired rotational wheel speed contribution (RPM-scale).
     */
    void setDesiredOutput(float x, float y, float r);

    /**
     * Sets values for the desired chassis velocity, but does not run the kinematics.
     * Instead, values are directyl set via the controller's channels multiplied
     * by the maximum translational and rotational speeds.
     */
    void setRoughDrive(float x, float y, float r);

    /// Zeros the desired wheel RPMs (does not disable the motors).
    void setZeroRPM();

    /// True only if all four chassis motors are currently reporting CAN feedback.
    bool allMotorsOnline() const;

    /// Number of physical drive motors this chassis has (always 4 for X-drive).
    int getNumChassisMotors() const { return static_cast<int>(motors.size()); }

    /**
     * @return The chassis's actual velocity in the chassis-relative frame,
     *      as <vx, vy, vr>, computed from each motor's measured shaft
     *      angular velocity (rad/s) via the forward kinematics matrix.
     */
    modm::Matrix<float, 3, 1> getActualVelocityChassisRelative() const;

    const char* getName() const { return "X-Drive Chassis"; }

private:
    /// Builds the 3x4 wheel-velocity-to-chassis-velocity matrix from the
    /// X-drive's fixed +/-45 degree wheel mounting angles, scaled by wheel
    /// and wheelbase radius.
    void computeKinematics(float wheelRadius, float wheelbaseRadius);

    /// Converts a desired chassis-relative x/y/r velocity into the four
    /// per-wheel RPM setpoints and stores them in desiredWheelRpm.
    void calculateDesiredWheelRpm(float x, float y, float r);

    tap::Drivers* drivers;

    std::array<tap::motor::DjiMotor, 4> motors;
    std::array<tap::algorithms::SmoothPid, 4> velocityPid;

    /// 3x4 matrix mapping wheel velocities -> chassis velocity (forward
    /// kinematics). Used directly for getActualVelocityChassisRelative(),
    /// and its structure mirrors the inverse kinematics used to compute
    /// per-wheel setpoints in calculateDesiredWheelRpm().
    modm::Matrix<float, 3, 4> wheelVelToChassisVelMat;

    /// Desired RPM for each wheel, indexed by WheelIndex.
    modm::Matrix<float, 4, 1> desiredWheelRpm;

    /// Timestamp (ms) of the previous refresh() call, used to compute dt
    /// for each wheel's velocity PID controller.
    uint32_t lastUpdateTimeMs = 0;
};

}  // namespace control::chassis

#endif  // CHASSIS_SUBSYSTEM_HPP_
