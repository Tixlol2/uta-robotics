#ifndef CHASSIS_BEYBLADE_COMMAND_HPP_
#define CHASSIS_BEYBLADE_COMMAND_HPP_

#include "tap/communication/serial/remote.hpp"
#include "tap/control/command.hpp"

#include "chassis_field_relative_math.hpp"
#include "chassis_subsystem.hpp"

namespace tap
{
class Drivers;
}

namespace control::chassis
{
/**
 * "Beyblade" mode: spins the chassis continuously at a fixed rate so the
 * robot presents a moving target and spreads incoming damage across all
 * armor plates, while still letting the operator translate with the right
 * stick (same channel mapping as ChassisDriveCommand).
 *
 * This command is not meant to be a default command -- it is meant to be
 * scheduled only while a remote switch is held in a particular position,
 * via a ToggleCommandMapping (see main.cpp). When untoggled, the chassis
 * subsystem automatically falls back to its default command.
 *
 * Translation is field-relative: the right stick's x/y is interpreted as a
 * direction relative to the field (specifically, relative to whatever
 * heading the chassis had when the IMU's yaw reference was last zeroed),
 * not relative to the chassis's own (constantly spinning) body frame. The
 * stick command is rotated by the chassis's current IMU yaw every tick
 * before being handed to ChassisSubsystem::setDesiredOutput, so holding the
 * stick in one direction moves the robot in a straight line across the
 * field regardless of how the chassis is currently oriented mid-spin.
 */
class ChassisBeybladeCommand : public tap::control::Command
{
public:
    ChassisBeybladeCommand(tap::Drivers* drivers, ChassisSubsystem* chassis);

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char* getName() const override { return "chassis beyblade"; }

private:
    /// Constant wheel RPM commanded for chassis rotation while beyblading.
    /// Tune this for your robot's desired spin rate.
    static constexpr float BEYBLADE_ROTATION_RPM = 3000.0f;

    /// Wheel RPM commanded when a translation stick is at full deflection
    /// while beyblading. Kept lower than ChassisDriveCommand's translation
    /// authority so the chassis stays controllable while spinning.
    static constexpr float BEYBLADE_TRANSLATIONAL_SPEED_RPM = 2000.0f;

    tap::Drivers* drivers;
    ChassisSubsystem* chassis;

    /// Reads a remote stick channel, returning 0.0f if the remote is
    /// disconnected.
    float getChannel(tap::communication::serial::Remote::Channel channel) const;
};

}  // namespace control::chassis

#endif  // CHASSIS_BEYBLADE_COMMAND_HPP_
