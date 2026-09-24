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
    // Raw stick input is interpreted as a *field*-relative direction: e.g.
    // "stick forward" should always drive the robot the same way across the
    // field, independent of which way the spinning chassis currently faces.
    const float fieldX = getChannel(tap::communication::serial::Remote::Channel::LEFT_VERTICAL) *
                          BEYBLADE_TRANSLATIONAL_SPEED_RPM;
    const float fieldY =
        getChannel(tap::communication::serial::Remote::Channel::LEFT_HORIZONTAL) *
        BEYBLADE_TRANSLATIONAL_SPEED_RPM;

    // Current chassis heading relative to the field, in radians.
    const float yaw = drivers->bmi088.getYaw();

    // Rotate the field-relative command into the chassis's spinning body
    // frame before handing it to the subsystem.
    float chassisX = 0.0f;
    float chassisY = 0.0f;
    rotateFieldRelativeToChassisRelative(fieldX, fieldY, yaw, &chassisX, &chassisY);

    // Rotation is constant regardless of stick input -- that's the "beyblade" part.
    chassis->setDesiredOutput(chassisX, chassisY, BEYBLADE_ROTATION_RPM);
}

void ChassisBeybladeCommand::end(bool) { chassis->setZeroRPM(); }

bool ChassisBeybladeCommand::isFinished() const { return false; }

}  // namespace control::chassis
