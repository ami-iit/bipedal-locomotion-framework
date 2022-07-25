/**
 * @file main.cpp
 * @authors Stefano Dafarra
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

// YARP
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>

#include <BipedalLocomotion/MasImuTest.h>

int main(int argc, char * argv[])
{
    // initialise yarp network
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"[main] Unable to find YARP network";
        return EXIT_FAILURE;
    }

    // prepare and configure the resource finder
    yarp::os::ResourceFinder& rf = yarp::os::ResourceFinder::getResourceFinderSingleton();

    rf.setDefaultConfigFile("MasImuTestConfig.ini");

    rf.configure(argc, argv);

    yInfo() << "[MasImuTest] Configuration file: " << rf.findFileByName("MasImuTestConfig.ini");

    BipedalLocomotion::MasImuTest test;

    return test.runModule(rf);
}
