#ifndef CHASSIS_BEYBLADE_COMMAND_HPP_
#define CHASSIS_BEYBLADE_COMMAND_HPP_

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
 * Note: translation here is chassis-relative, so as the chassis spins, the
 * "forward" direction of the sticks spins along with it. That's expected
 * for a basic beyblade. Making translation field-relative while spinning
 * would require compensating the x/y command with IMU yaw, which this
 * command does not do.
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
    
    static constexpr float BEYBLADE_ROTATION_RPM = 3000.0f;

    /// Wheel RPM commanded when a translation stick is at full deflection
    static constexpr float BEYBLADE_TRANSLATIONAL_SPEED_RPM = 2000.0f;

    tap::Drivers* drivers;
    ChassisSubsystem* chassis;

    /// Reads a remote stick channel, returning 0.0f if the remote is
    /// disconnected.
    float getChannel(tap::communication::serial::Remote::Channel channel) const;
};

}  // namespace control::chassis

#endif  // CHASSIS_BEYBLADE_COMMAND_HPP_