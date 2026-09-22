#ifndef CHASSIS_DRIVE_COMMAND_HPP_
#define CHASSIS_DRIVE_COMMAND_HPP_

#include "tap/communication/serial/remote.hpp"
#include "tap/control/command.hpp"

#include "chassis_subsystem.hpp"

namespace tap
{
class Drivers;
}

namespace control::chassis
{
/**
 * Default drive command for the ChassisSubsystem. Reads the operator's
 * remote control sticks every scheduler tick and forwards them to
 * ChassisSubsystem::setDesiredOutput as x (translation), y (strafe), and
 * r (rotation) wheel-speed commands.
 *
 * Stick mapping (standard ARUW convention):
 *  - Right stick vertical   -> x (forward/backward)
 *  - Right stick horizontal -> y (left/right strafe)
 *  - Left stick horizontal  -> r (rotation)
 *
 * This command never finishes on its own; schedule it as the chassis
 * subsystem's default command so the chassis is always under operator
 * control unless some other command (e.g. autoaim, autorotate) takes over
 * the subsystem.
 */
class ChassisDriveCommand : public tap::control::Command
{
public:
    ChassisDriveCommand(tap::Drivers* drivers, ChassisSubsystem* chassis);

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char* getName() const override { return "chassis drive"; }

private:
    /// Wheel RPM commanded when a translation stick is at full deflection.
    static constexpr float MAX_TRANSLATIONAL_SPEED_RPM = 8000.0f;

    /// Wheel RPM commanded when the rotation stick is at full deflection.
    static constexpr float MAX_ROTATIONAL_SPEED_RPM = 4000.0f;

    tap::Drivers* drivers;
    ChassisSubsystem* chassis;

    /// Reads a remote stick channel, returning 0.0f (rather than stale or
    /// garbage data) if the remote link is down.
    float getChannel(tap::communication::serial::Remote::Channel channel) const;
};

}  // namespace control::chassis

#endif  // CHASSIS_DRIVE_COMMAND_HPP_