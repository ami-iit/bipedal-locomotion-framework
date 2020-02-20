/**
 * @file MasImuTest.h
 * @authors Stefano Dafarra
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTROLLERS_MASIMUTEST_H
#define BIPEDAL_LOCOMOTION_CONTROLLERS_MASIMUTEST_H

// YARP
#include <yarp/os/RFModule.h>

//Thrifts
#include <thrifts/MasImuTestCommands.h>

#include <BipedalLocomotionControllers/ParametersHandler/YarpImplementation.h>

namespace BipedalLocomotionControllers
{
    class MasImuTest;
}


class BipedalLocomotionControllers::MasImuTest : public yarp::os::RFModule, public MasImuTestCommands {

    BipedalLocomotionControllers::ParametersHandler::YarpImplementation::unique_ptr m_parametersPtr;

    bool configureRobot();

public:

    /**
     * Get the period of the RFModule.
     * @return the period of the module.
     */
    double getPeriod() override;

    /**
     * Main function of the RFModule.
     * @return true in case of success and false otherwise.
     */
    bool updateModule() override;

    /**
     * Configure the RFModule.
     * @param rf is the reference to a resource finder object
     * @return true in case of success and false otherwise.
     */
    bool configure(yarp::os::ResourceFinder& rf) override;

    /**
     * Close the RFModule.
     * @return true in case of success and false otherwise.
     */
    bool close() override;

    /**
     * Call this method to start the test.
     * @return true/false in case of success/failure (for example if the preparation phase was not successfull);
     */
    bool startTest() override;

    /**
     * Stop the test
     */
    void stopTest() override;
};

#endif // BIPEDAL_LOCOMOTION_CONTROLLERS_MASIMUTEST_H
