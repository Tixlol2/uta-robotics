/*
 * Copyright (c) 2020-2021 E404
 *
 * This file is part of Robot.
 *
 * Robot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Robot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Robot.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifdef PLATFORM_HOSTED
/* hosted environment (simulator) includes --------------------------------- */
#include <iostream>

#include "tap/communication/tcp-server/tcp_server.hpp"
#include "tap/motor/motorsim/dji_motor_sim_handler.hpp"
#endif

#include "tap/board/board.hpp"

#include "modm/architecture/interface/delay.hpp"

/* arch includes ------------------------------------------------------------*/
#include "tap/architecture/periodic_timer.hpp"
#include "tap/architecture/profiler.hpp"

/* communication includes ---------------------------------------------------*/
#include "tap/communication/gpio/analog.hpp"
#include "tap/communication/gpio/leds.hpp"



#include "drivers.hpp"
#include "drivers_singleton.hpp"

/* error handling includes --------------------------------------------------*/
#include "tap/errors/create_errors.hpp"

/* control includes ---------------------------------------------------------*/
#include "tap/architecture/clock.hpp"

#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/toggle_command_mapping.hpp"
#include "tap/control/toggle_command_mapping.hpp"
#include "tap/control/remote_map_state.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/communication/serial/terminal_serial.hpp"

#include "subsystems/chassis/chassis_subsystem.hpp"
#include "subsystems/chassis/chassis_drive_command.hpp"
#include "subsystems/chassis/chassis_beyblade_command.hpp"


/* define timers here -------------------------------------------------------*/
static constexpr float MAIN_LOOP_FREQUENCY = 500.0f;
tap::arch::PeriodicMilliTimer sendMotorTimeout(1000.0f / MAIN_LOOP_FREQUENCY);

// Place any sort of input/output initialization here. For example, place
// serial init stuff here.
static void initializeIo(src::Drivers *drivers);

// Anything that you would like to be called place here. It will be called
// very frequently. Use PeriodicMilliTimers if you don't want something to be
// called as frequently.
static void updateIo(src::Drivers *drivers);


class RemoteDebugHandler
    : public tap::communication::serial::TerminalSerialCallbackInterface
{
public:
    explicit RemoteDebugHandler(src::Drivers *drivers) : drivers(drivers) {}

    bool terminalSerialCallback(char *, modm::IOStream &output, bool) override
    {
        output << "connected=" << drivers->remote.isConnected()
               << " updates=" << drivers->remote.getUpdateCounter()
               << " right_switch="
               << static_cast<int>(drivers->remote.getSwitch(
                      tap::communication::serial::Remote::Switch::RIGHT_SWITCH))
               << modm::endl;
        return true;
    }

    void terminalSerialStreamCallback(modm::IOStream &output) override
    {
        terminalSerialCallback(nullptr, output, true);
    }

    

private:
    src::Drivers *drivers;
};

int main()
{
#ifdef PLATFORM_HOSTED
    std::cout << "Simulation starting..." << std::endl;
#endif

    /*
     * NOTE: We are using DoNotUse_getDrivers here because in the main
     *      robot loop we must access the singleton drivers to update
     *      IO states and run the scheduler.
     */
    src::Drivers *drivers = src::DoNotUse_getDrivers();
    
    

    Board::initialize();
    initializeIo(drivers);
    
    
    control::chassis::ChassisSubsystem chassis(
        drivers,
        tap::motor::MotorId::MOTOR1,   // LF
        tap::motor::MotorId::MOTOR2,   // RF
        tap::motor::MotorId::MOTOR4,   // LB
        tap::motor::MotorId::MOTOR3,   // RB
        tap::can::CanBus::CAN_BUS2,
        0.0762f,   // wheel radius (m)
        0.254f,    // wheelbase radius (m)
        {0.5f, 0.001f, 0.0f, 100.0f, 8000.0f, 1.0f, 0.0f, 1.0f, 0.0f, 75.0f, 0.0f});
        // Kp, Ki, Kd, maxICumulative, maxOutput, tQDerivativeKalman,
        // tRDerivativeKalman, tQProportionalKalman, tRProportionalKalman,
        // errDeadzone, errorDerivativeFloor

    
    chassis.registerAndInitialize();

    
    control::chassis::ChassisDriveCommand chassisDriveCommand(drivers, &chassis);

    
    chassis.setDefaultCommand(&chassisDriveCommand);
    
    // Beyblade command — scheduled only while toggled on
    control::chassis::ChassisBeybladeCommand chassisBeybladeCommand(drivers, &chassis);

    tap::control::RemoteMapState rightSwitchUp(
        tap::communication::serial::Remote::Switch::RIGHT_SWITCH,
        tap::communication::serial::Remote::SwitchState::UP);

    tap::control::HoldCommandMapping beybladeToggle(
        drivers,
        {&chassisBeybladeCommand},
        rightSwitchUp);

    drivers->commandMapper.addMap(&beybladeToggle);
    
    

#ifdef PLATFORM_HOSTED
    tap::motor::motorsim::DjiMotorSimHandler::getInstance()->resetMotorSims();
    // Blocking call, waits until Windows Simulator connects.
    tap::communication::TCPServer::MainServer()->getConnection();
#endif



    while (1)
    {
        #ifdef TARGET_STANDARD
        #endif

        

        // do this as fast as you can
        PROFILE(drivers->profiler, updateIo, (drivers));

        
        if (sendMotorTimeout.execute())
        {
            // PROFILE(drivers->profiler, drivers->mpu6500.periodicIMUUpdate, ()); // only for type
            // A
            PROFILE(drivers->profiler, drivers->bmi088.periodicIMUUpdate, ());  // only for type B
            PROFILE(drivers->profiler, drivers->commandScheduler.run, ());
            PROFILE(drivers->profiler, drivers->djiMotorTxHandler.encodeAndSendCanData, ());
            PROFILE(drivers->profiler, drivers->terminalSerial.update, ());

            
        }
        modm::delay_us(10);
    }
    return 0;
}

static void initializeIo(src::Drivers *drivers)
{
    drivers->analog.init();
    drivers->pwm.init();
    drivers->digital.init();
    drivers->leds.init();
    drivers->can.initialize();
    drivers->errorController.init();
    drivers->remote.initialize();
    // kp controls speed of trust in imu data, imu goes crazy -> lower, imu sluggish -> higher
    // ki compensates for drift over minutes, imu drifts -> start at 0.001, then increase as needed
    // drivers->mpu6500.init(MAIN_LOOP_FREQUENCY, 0.1, 0);
    drivers->bmi088.initialize(MAIN_LOOP_FREQUENCY, 0.1, 0);
    
    drivers->bmi088.requestRecalibration();
    drivers->refSerial.initialize();
    drivers->terminalSerial.initialize();
    drivers->schedulerTerminalHandler.init();
    drivers->djiMotorTerminalSerialHandler.init();
    static RemoteDebugHandler remoteDebugHandler(drivers);
    drivers->terminalSerial.addHeader("remote", &remoteDebugHandler);
    
}


static void updateIo(src::Drivers *drivers)
{
#ifdef PLATFORM_HOSTED
    tap::motor::motorsim::DjiMotorSimHandler::getInstance()->updateSims();
#endif

    drivers->canRxHandler.pollCanData();
    drivers->refSerial.updateSerial();
    drivers->remote.read();

    

    
}
