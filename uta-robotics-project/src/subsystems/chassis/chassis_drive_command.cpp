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
    // Raw stick input is interpreted as a *field*-relative direction: e.g.
    // "stick forward" should always drive the robot the same way across the
    // field, independent of the chassis's current heading.
    const float fieldX = getChannel(tap::communication::serial::Remote::Channel::LEFT_VERTICAL) *
                          MAX_TRANSLATIONAL_SPEED_RPM;
    const float fieldY =
        getChannel(tap::communication::serial::Remote::Channel::LEFT_HORIZONTAL) *
        MAX_TRANSLATIONAL_SPEED_RPM;
    const float r = getChannel(tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL) *
                     MAX_ROTATIONAL_SPEED_RPM;

    // Current chassis heading relative to the field, in radians.
    const float yaw = modm::toRadian(drivers->bmi088.getYaw());

    // Rotate the field-relative command into the chassis's own body frame
    // before handing it to the subsystem.
    float chassisX = 0.0f;
    float chassisY = 0.0f;
    rotateFieldRelativeToChassisRelative(fieldX, fieldY, yaw, &chassisX, &chassisY);

    chassis->setDesiredOutput(chassisX, chassisY, r);
}

void ChassisDriveCommand::end(bool) { chassis->setZeroRPM(); }

bool ChassisDriveCommand::isFinished() const { return false; }

}  // namespace control::chassis
