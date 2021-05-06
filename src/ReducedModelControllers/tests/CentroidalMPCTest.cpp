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
#include <BipedalLocomotion/Planners/QuinticSpline.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::ReducedModelControllers;
using namespace BipedalLocomotion::ContinuousDynamicalSystem;

#include <fstream>

TEST_CASE("CentroidalMPC")
{
    constexpr double dT = 0.1;

    std::shared_ptr<IParametersHandler> handler = std::make_shared<StdImplementation>();
    handler->setParameter("controller_sampling_time", dT);
    handler->setParameter("controller_horizon", 20);
    handler->setParameter("number_of_maximum_contacts", 2);
    handler->setParameter("number_of_slices", 1);
    handler->setParameter("static_friction_coefficient", 0.33);

    auto contact0Handler = std::make_shared<StdImplementation>();
    contact0Handler->setParameter("number_of_corners", 4);
    contact0Handler->setParameter("contact_name", "left_foot");
    contact0Handler->setParameter("corner_0", std::vector<double>{0.1, 0.05, 0});
    contact0Handler->setParameter("corner_1", std::vector<double>{0.1, -0.05, 0});
    contact0Handler->setParameter("corner_2", std::vector<double>{-0.06, -0.05, 0});
    contact0Handler->setParameter("corner_3", std::vector<double>{-0.06, 0.05, 0});

    auto contact1Handler = std::make_shared<StdImplementation>();
    contact1Handler->setParameter("number_of_corners", 4);
    contact1Handler->setParameter("contact_name", "right_foot");
    contact1Handler->setParameter("corner_0", std::vector<double>{0.1, 0.05, 0});
    contact1Handler->setParameter("corner_1", std::vector<double>{0.1, -0.05, 0});
    contact1Handler->setParameter("corner_2", std::vector<double>{-0.06, -0.05, 0});
    contact1Handler->setParameter("corner_3", std::vector<double>{-0.06, 0.05, 0});

    handler->setGroup("CONTACT_0", contact0Handler);
    handler->setGroup("CONTACT_1", contact1Handler);

    handler->setParameter("com_weight", std::vector<double>{1, 1, 1000});
    handler->setParameter("contact_position_weight", 1.0);
    handler->setParameter("force_rate_of_change_weight", std::vector<double>{1, 1, 1});
    handler->setParameter("angular_momentum_weight", 1e5);

    CentroidalMPC mpc;

    REQUIRE(mpc.initialize(handler));


    BipedalLocomotion::Contacts::ContactPhaseList phaseList;
    BipedalLocomotion::Contacts::ContactListMap contactListMap;


    BipedalLocomotion::Planners::QuinticSpline comSpline;
    std::vector<Eigen::VectorXd> knots;
    std::vector<double> time;


    constexpr double scaling = 0.5;
    // t  0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17
    // L |+++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++++|
    // R |+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|+++|
    // left foot
    // first footstep
    Eigen::Vector3d leftPos;
    leftPos <<  -0.0852893,    0.0828008, -0.000110952;
    manif::SE3d leftTransform(leftPos, manif::SO3d::Identity());
    contactListMap["left_foot"].addContact(leftTransform, 0.0, 1.0 * scaling);
    contactListMap["left_foot"].addContact(leftTransform, 2.0* scaling, 5.0* scaling);
    contactListMap["left_foot"].addContact(leftTransform, 6.0* scaling, 9.0* scaling);
    contactListMap["left_foot"].addContact(leftTransform, 10.0* scaling, 13.0* scaling);
    contactListMap["left_foot"].addContact(leftTransform, 14.0* scaling, 17.0* scaling);

    // right foot
    // first footstep
    Eigen::Vector3d rightPos;
    rightPos << -0.0850079,    -0.073266, -0.000111252;
    manif::SE3d rightTransform(rightPos, manif::SO3d::Identity());
    contactListMap["right_foot"].addContact(rightTransform, 0.0, 3.0* scaling);
    contactListMap["right_foot"].addContact(rightTransform, 4.0* scaling, 7.0* scaling);
    contactListMap["right_foot"].addContact(rightTransform, 8.0* scaling, 11.0* scaling);
    contactListMap["right_foot"].addContact(rightTransform, 12.0* scaling, 15.0* scaling);
    contactListMap["right_foot"].addContact(rightTransform, 16.0* scaling, 17.0* scaling);

    phaseList.setLists(contactListMap);

    Eigen::Vector3d comtemp;
    comtemp << -0.0557448, 0.00497309,  0.528696;

    // knots.push_back(comtemp);
    // time.push_back(0);

    // comtemp(1) = rightPos(1);
    // knots.push_back(comtemp);
    // time.push_back(1.5);

    // comtemp(1) = leftPos(1);
    // knots.push_back(comtemp);
    // time.push_back(3.5);

    // comtemp(1) = rightPos(1);
    // knots.push_back(comtemp);
    // time.push_back(5.5);

    // comtemp(1) = leftPos(1);
    // knots.push_back(comtemp);
    // time.push_back(7.5);

    // comtemp(1) = rightPos(1);
    // knots.push_back(comtemp);
    // time.push_back(9.5);

    // comtemp(1) = leftPos(1);
    // knots.push_back(comtemp);
    // time.push_back(11.5);


    // comtemp(1) = rightPos(1);
    // knots.push_back(comtemp);
    // time.push_back(13.5);

    // comtemp(1) = leftPos(1);
    // knots.push_back(comtemp);
    // time.push_back(15.5);


    // comtemp(1) = 0;
    // knots.push_back(comtemp);
    // time.push_back(17);

    // comSpline.setKnots(knots, time);
    // comSpline.setInitialConditions(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    // comSpline.setFinalConditions(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    Eigen::MatrixXd comTraj(3, 1000);
    comTraj.colwise() = comtemp;

    // Eigen::Vector3d dummy;
    // for(int i = 0; i < 170; i++)
    // {
    //     comSpline.evaluatePoint(i * dT, comTraj.col(i), dummy, dummy);
    // }

    // std::cerr << comTraj.leftCols(10) << std::endl;

    // comTraj.rightCols(1000 - 170).colwise() = comTraj.col(170);

    // std::cerr << comTraj.leftCols(10) << std::endl;

    // initialize the dynamical system
    auto system = std::make_shared<CentroidalDynamics>();
    system->setState({comTraj.col(0), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()});

    ForwardEuler<CentroidalDynamics> integrator;
    integrator.setIntegrationStep(dT/10.0);
    REQUIRE(integrator.setDynamicalSystem(system));

    std::ofstream myFile;
    myFile.open("data.txt");

    // TODO reset contatcs
    int controllerIndex = 0;
    int index = 0;
    for(int i = 0; i < 1000; i++)
    {
        const auto& [com, dcom, angularMomentum] = system->getState();

        if(controllerIndex == 0)
        {
            REQUIRE(mpc.setState(com, dcom, angularMomentum));
            // REQUIRE(mpc.setReferenceTrajectory(comTraj.rightCols(comTraj.cols() - index)));
            REQUIRE(mpc.setReferenceTrajectory(comTraj));
            REQUIRE(mpc.setContactPhaseList(phaseList));
            REQUIRE(mpc.advance());
            index++;
        }

        for (const auto& [key, contact] : mpc.getOutput().contacts)
        {
            for(const auto& corner : contact.corners)
            {
                myFile << corner.force.transpose() << "    ";
            }
        }
        myFile << com.transpose() << " " << comTraj.col(index).transpose() << " " << angularMomentum.transpose();

        myFile << std::endl;

        system->setControlInput({mpc.getOutput().contacts});


        REQUIRE(integrator.integrate(0, 0.01));

        controllerIndex++;
        if(controllerIndex == 20)
            controllerIndex =0;
    }

    myFile.close();
}
