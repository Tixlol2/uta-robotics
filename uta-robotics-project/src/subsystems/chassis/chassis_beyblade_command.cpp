#include "chassis_beyblade_command.hpp"

#include "tap/drivers.hpp"

namespace control::chassis
{
ChassisBeybladeCommand::ChassisBeybladeCommand(tap::Drivers* drivers, ChassisSubsystem* chassis)
    : drivers(drivers),
      chassis(chassis)
{
    addSubsystemRequirement(chassis);
}

void ChassisBeybladeCommand::initialize() {}

float ChassisBeybladeCommand::getChannel(
    tap::communication::serial::Remote::Channel channel) const
{
    if (!drivers->remote.isConnected())
    {
        return 0.0f;
    }

    // Remote channels are normalized by tap to the range [-1.0, 1.0].
    return drivers->remote.getChannel(channel);
}

void ChassisBeybladeCommand::execute()
{
    const float x = getChannel(tap::communication::serial::Remote::Channel::LEFT_VERTICAL) *
                     BEYBLADE_TRANSLATIONAL_SPEED_RPM;
    const float y = getChannel(tap::communication::serial::Remote::Channel::LEFT_HORIZONTAL) *
                     BEYBLADE_TRANSLATIONAL_SPEED_RPM;

    // Rotation is constant regardless of stick input -- that's the "beyblade" part.
    chassis->setDesiredOutput(x, y, BEYBLADE_ROTATION_RPM);
}

void ChassisBeybladeCommand::end(bool) { chassis->setZeroRPM(); }

bool ChassisBeybladeCommand::isFinished() const { return false; }

}  // namespace control::chassis