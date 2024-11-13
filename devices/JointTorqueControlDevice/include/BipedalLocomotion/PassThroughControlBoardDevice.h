/**
 * @file PassThroughControlBoardDevice.h
 * @authors Ines Sorrentino
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_FRAMEWORK_PASS_THROUGH_CONTROL_BOARD_DEVICE_H
#define BIPEDAL_LOCOMOTION_FRAMEWORK_PASS_THROUGH_CONTROL_BOARD_DEVICE_H

#include <BipedalLocomotion/PassThroughControlBoard.h>

#include <iostream>
#include <memory>
#include <mutex>
#include <thread>

#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Property.h>
#include <yarp/sig/Vector.h>

#include <JointTorqueControlCommands.h>


namespace BipedalLocomotion
{
class PassThroughControlBoardDevice;
}

class BipedalLocomotion::PassThroughControlBoardDevice
    : public BipedalLocomotion::PassThroughControlBoard,
      public yarp::os::PeriodicThread,
      public JointTorqueControlCommands
{
    private:
        yarp::os::Property PropertyConfigOptions;
    public:
        // CONSTRUCTOR/DESTRUCTOR
        PassThroughControlBoardDevice(double period,
                                    yarp::os::ShouldUseSystemClock useSystemClock
                                    = yarp::os::ShouldUseSystemClock::No);
        PassThroughControlBoardDevice();
        ~PassThroughControlBoardDevice();

        // DEVICE DRIVER
        virtual bool open(yarp::os::Searchable& config);
        virtual bool close();

        // IMULTIPLEWRAPPER
        virtual bool attachAll(const yarp::dev::PolyDriverList& p);
        virtual bool detachAll();

        // CONTROL THREAD
        virtual bool threadInit();
        virtual void run();
        virtual void threadRelease();
};

#endif /* BIPEDAL_LOCOMOTION_FRAMEWORK_PASS_THROUGH_CONTROL_BOARD_DEVICE_H */
