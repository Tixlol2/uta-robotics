#include "chassis_drive_command.hpp"

#include "tap/drivers.hpp"

namespace control::chassis
{
ChassisDriveCommand::ChassisDriveCommand(tap::Drivers* drivers, ChassisSubsystem* chassis)
    : drivers(drivers),
      chassis(chassis)
{
    addSubsystemRequirement(chassis);
}

void ChassisDriveCommand::initialize() {}

float ChassisDriveCommand::getChannel(tap::communication::serial::Remote::Channel channel) const
{
    if (!drivers->remote.isConnected())
    {
        return 0.0f;
    }

    // Remote channels are normalized by tap to the range [-1.0, 1.0].
    return drivers->remote.getChannel(channel);
}

void ChassisDriveCommand::execute()
{
    const float x = getChannel(tap::communication::serial::Remote::Channel::LEFT_VERTICAL) *
                     MAX_TRANSLATIONAL_SPEED_RPM;
    const float y = getChannel(tap::communication::serial::Remote::Channel::LEFT_HORIZONTAL) *
                     MAX_TRANSLATIONAL_SPEED_RPM;
    const float r = getChannel(tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL) *
                     MAX_ROTATIONAL_SPEED_RPM;

    chassis->setDesiredOutput(x, y, r);
}

void ChassisDriveCommand::end(bool) { chassis->setZeroRPM(); }

bool ChassisDriveCommand::isFinished() const { return false; }

}  // namespace control::chassis