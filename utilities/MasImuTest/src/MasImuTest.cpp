/**
 * @file MasImuTest.cpp
 * @authors Stefano Dafarra
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotionControllers/MasImuTest.h>

using namespace BipedalLocomotionControllers;



bool MasImuTest::configureRobot()
{

}

double MasImuTest::getPeriod()
{

}

bool MasImuTest::updateModule()
{

}

bool MasImuTest::configure(yarp::os::ResourceFinder &rf)
{
    m_parametersPtr = std::make_unique<BipedalLocomotionControllers::ParametersHandler::YarpImplementation>(rf);

}

bool MasImuTest::close()
{

}

bool MasImuTest::startTest()
{

}

void MasImuTest::stopTest()
{

}
