/**
 * @file CentroidalMPCTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

// Catch2
#include <catch2/catch.hpp>

#include <BipedalLocomotion/Contacts/ContactPhaseList.h>
#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/CentroidalDynamics.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/ForwardEuler.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/ReducedModelControllers/CentroidalMPC.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::ReducedModelControllers;
using namespace BipedalLocomotion::ContinuousDynamicalSystem;

#include <fstream>

TEST_CASE("CentroidalMPC")
{
    constexpr double dT = 0.1;

    std::shared_ptr<IParametersHandler> handler = std::make_shared<StdImplementation>();
    handler->setParameter("controller_sampling_time", dT);
    handler->setParameter("controller_horizon", 15);
    handler->setParameter("number_of_maximum_contacts", 2);
    handler->setParameter("number_of_slices", 1);
    handler->setParameter("static_friction_coefficient", 0.33);

    auto contact0Handler = std::make_shared<StdImplementation>();
    contact0Handler->setParameter("number_of_corners", 4);
    contact0Handler->setParameter("contact_name", "foot0");
    contact0Handler->setParameter("corner_0", std::vector<double>{0.1, 0.05, 0});
    contact0Handler->setParameter("corner_1", std::vector<double>{0.1, -0.05, 0});
    contact0Handler->setParameter("corner_2", std::vector<double>{-0.1, -0.05, 0});
    contact0Handler->setParameter("corner_3", std::vector<double>{-0.1, 0.05, 0});

    auto contact1Handler = std::make_shared<StdImplementation>();
    contact1Handler->setParameter("number_of_corners", 4);
    contact1Handler->setParameter("contact_name", "foot1");
    contact1Handler->setParameter("corner_0", std::vector<double>{0.1, 0.05, 0});
    contact1Handler->setParameter("corner_1", std::vector<double>{0.1, -0.05, 0});
    contact1Handler->setParameter("corner_2", std::vector<double>{-0.1, -0.05, 0});
    contact1Handler->setParameter("corner_3", std::vector<double>{-0.1, 0.05, 0});


    handler->setGroup("CONTACT_0", contact0Handler);
    handler->setGroup("CONTACT_1", contact1Handler);

    handler->setParameter("com_weight", std::vector<double>{1, 1, 100});
    handler->setParameter("contact_position_weight", 1.0);
    handler->setParameter("force_rate_of_change_weight", std::vector<double>{1, 1, 1});
    handler->setParameter("angular_momentum_weight", 1e5);

    CentroidalMPC mpc;

    REQUIRE(mpc.initialize(handler));


    BipedalLocomotion::Contacts::ContactPhaseList phaseList;
    BipedalLocomotion::Contacts::ContactListMap contactListMap;


    // left foot
    // first footstep
    Eigen::Vector3d leftPos;
    leftPos << 0, -0.8, 0;
    manif::SE3d leftTransform(leftPos, manif::SO3d::Identity());
    REQUIRE(contactListMap["foot0"].addContact(leftTransform, 0.0, 1.0));

    // second footstep
    // leftPos(0) = 0.25;
    // leftPos(2) = 0.2;
    leftTransform = manif::SE3d(leftPos, manif::SO3d::Identity());
    REQUIRE(contactListMap["foot0"].addContact(leftTransform, 2.0, 7.0));

    // right foot
    // first footstep
    Eigen::Vector3d rightPos;
    rightPos << 0, 0.8, 0;
    manif::SE3d rightTransform(rightPos, manif::SO3d::Identity());
    REQUIRE(contactListMap["foot1"].addContact(rightTransform, 0.0, 3.0));

    // second footstep
    // rightPos(0) = 0.25;
    // rightPos(2) = 0.2;
    rightTransform = manif::SE3d(rightPos, manif::SO3d::Identity());
    REQUIRE(contactListMap["foot1"].addContact(rightTransform, 4.0, 7.0));
    phaseList.setLists(contactListMap);

    Eigen::Vector3d com0;
    com0 << 0, 0, 0.53;

    Eigen::MatrixXd comTraj(3, 150);
    comTraj.colwise() = com0;


    // initialize the dynamical system
    auto system = std::make_shared<CentroidalDynamics>();
    system->setState({com0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()});

    ForwardEuler<CentroidalDynamics> integrator;
    integrator.setIntegrationStep(dT);
    REQUIRE(integrator.setDynamicalSystem(system));

    std::ofstream myFile;
    myFile.open("data.txt");

    // TODO reset contatcs
    for(int i = 0; i < 60; i++)
    {
        const auto& [com, dcom, angularMomentum] = system->getState();

        REQUIRE(mpc.setState(com, dcom, angularMomentum));
        REQUIRE(mpc.setReferenceTrajectory(comTraj));
        REQUIRE(mpc.setContactPhaseList(phaseList));
        REQUIRE(mpc.advance());

        for (const auto& [key, contact] : mpc.get().contacts)
        {
            for(const auto& corner : contact.corners)
            {
                myFile << corner.force.transpose() << "    ";
            }
        }
        myFile << com.transpose();

        myFile << std::endl;

        system->setControlInput({mpc.get().contacts});
        REQUIRE(integrator.integrate(0, dT));
    }

    myFile.close();
}
