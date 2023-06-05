/**
 * @file CentroidalMPCTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <chrono>

// Catch2
#include <catch2/catch_test_macros.hpp>

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

void updateContactPhaseList(
    const std::map<std::string, BipedalLocomotion::Contacts::PlannedContact>& nextPlannedContacts,
    BipedalLocomotion::Contacts::ContactPhaseList& phaseList)
{
    auto newList = phaseList.lists();
    for (const auto& [key, contact] : nextPlannedContacts)
    {
        auto it = newList.at(key).getPresentContact(contact.activationTime);
        newList.at(key).editContact(it, contact);
    }

    phaseList.setLists(newList);
}

TEST_CASE("CentroidalMPC")
{
    using namespace std::chrono_literals;
    std::chrono::nanoseconds dT = 50ms;

    std::shared_ptr<IParametersHandler> handler = std::make_shared<StdImplementation>();
    handler->setParameter("sampling_time", dT);
    handler->setParameter("time_horizon", 500ms);
    handler->setParameter("number_of_maximum_contacts", 2);
    handler->setParameter("number_of_slices", 1);
    handler->setParameter("static_friction_coefficient", 0.33);
    handler->setParameter("solver_verbosity", 2);
    handler->setParameter("linear_solver", "mumps");

    auto contact0Handler = std::make_shared<StdImplementation>();
    contact0Handler->setParameter("number_of_corners", 4);
    contact0Handler->setParameter("contact_name", "left_foot");
    contact0Handler->setParameter("corner_0", std::vector<double>{0.1, 0.05, 0});
    contact0Handler->setParameter("corner_1", std::vector<double>{0.1, -0.05, 0});
    contact0Handler->setParameter("corner_2", std::vector<double>{-0.1, -0.05, 0});
    contact0Handler->setParameter("corner_3", std::vector<double>{-0.1, 0.05, 0});
    contact0Handler->setParameter("bounding_box_lower_limit", std::vector<double>{0, 0, 0});
    contact0Handler->setParameter("bounding_box_upper_limit", std::vector<double>{0, 0, 0});

    auto contact1Handler = std::make_shared<StdImplementation>();
    contact1Handler->setParameter("number_of_corners", 4);
    contact1Handler->setParameter("contact_name", "right_foot");
    contact1Handler->setParameter("corner_0", std::vector<double>{0.1, 0.05, 0});
    contact1Handler->setParameter("corner_1", std::vector<double>{0.1, -0.05, 0});
    contact1Handler->setParameter("corner_2", std::vector<double>{-0.1, -0.05, 0});
    contact1Handler->setParameter("corner_3", std::vector<double>{-0.1, 0.05, 0});
    contact1Handler->setParameter("bounding_box_lower_limit", std::vector<double>{0, 0, 0});
    contact1Handler->setParameter("bounding_box_upper_limit", std::vector<double>{0, 0, 0});


    handler->setGroup("CONTACT_0", contact0Handler);
    handler->setGroup("CONTACT_1", contact1Handler);

    handler->setParameter("com_weight", std::vector<double>{1, 1, 1000});
    handler->setParameter("contact_position_weight", 1e3);
    handler->setParameter("force_rate_of_change_weight", std::vector<double>{10, 10, 10});
    handler->setParameter("angular_momentum_weight", 1e5);
    handler->setParameter("contact_force_symmetry_weight", 10.0);

    CentroidalMPC mpc;

    REQUIRE(mpc.initialize(handler));


    BipedalLocomotion::Contacts::ContactPhaseList phaseList;
    BipedalLocomotion::Contacts::ContactListMap contactListMap;

    BipedalLocomotion::Planners::QuinticSpline comSpline;


    constexpr int scaling = 1;
    constexpr double scalingPos = 4.0;
    constexpr double scalingPosY = 12;

    // // t  0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17  18  19 20  21  22  23  24  25  26  27
    // // L |+++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|++++++++++|---|+++|
    // // R |+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++|

    Eigen::Vector3d leftPosition;
    leftPosition << 0, 0.08, 0;
    manif::SE3d leftTransform(leftPosition, manif::SO3d::Identity());
    contactListMap["left_foot"].addContact(leftTransform, 0s, scaling * 1s);

    leftPosition(0) += 0.05 * scalingPos;
    leftTransform.translation(leftPosition);
    contactListMap["left_foot"].addContact(leftTransform, 2s * scaling, 5s * scaling);

    leftPosition(0) += 0.1 * scalingPos;
    leftTransform.translation(leftPosition);
    contactListMap["left_foot"].addContact(leftTransform, 6s * scaling, 9s * scaling);

    leftPosition(0) += 0.05 * scalingPos;
    leftTransform.translation(leftPosition);
    contactListMap["left_foot"].addContact(leftTransform, 10s * scaling, 13s * scaling);

    leftPosition(0) += 0.15 * scalingPos;
    leftTransform.translation(leftPosition);
    contactListMap["left_foot"].addContact(leftTransform, 14s * scaling, 17s * scaling);

    leftPosition(1) -= 0.01 * scalingPosY;
    leftTransform.translation(leftPosition);
    contactListMap["left_foot"].addContact(leftTransform, 18s * scaling, 21s * scaling);

    leftPosition(1) -= 0.01 * scalingPosY;
    leftTransform.translation(leftPosition);
    contactListMap["left_foot"].addContact(leftTransform, 22s * scaling, 25s * scaling);

    leftPosition(1) -= 0.01 * scalingPosY;
    leftTransform.translation(leftPosition);
    contactListMap["left_foot"].addContact(leftTransform, 26s * scaling, 29s * scaling);

    // // t  0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17  18  19 20  21
    // 22  23  24  25  26  27
    // // L
    // |+++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|++++++++++|---|+++|
    // // R
    // |+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++|

    // right foot
    // first footstep
    Eigen::Vector3d rightPosition;
    rightPosition << 0, -0.08, 0;
    manif::SE3d rightTransform(rightPosition, manif::SO3d::Identity());

    contactListMap["right_foot"].addContact(rightTransform, 0s, 3s * scaling);

    rightPosition(0) += 0.1 * scalingPos;
    rightTransform.translation(rightPosition);
    contactListMap["right_foot"].addContact(rightTransform, 4s * scaling, 7s * scaling);

    rightPosition(0) += 0.1 * scalingPos;
    rightTransform.translation(rightPosition);
    contactListMap["right_foot"].addContact(rightTransform, 8s * scaling, 11s * scaling);

    rightPosition(0) += 0.1 * scalingPos;
    rightTransform.translation(rightPosition);
    contactListMap["right_foot"].addContact(rightTransform, 12s * scaling, 15s * scaling);

    rightPosition(0) += 0.05 * scalingPos;
    rightPosition(1) -= 0.01 * scalingPosY;
    rightTransform.translation(rightPosition);
    contactListMap["right_foot"].addContact(rightTransform, 16s * scaling, 19s * scaling);

    rightPosition(1) -= 0.01 * scalingPosY;
    rightTransform.translation(rightPosition);
    contactListMap["right_foot"].addContact(rightTransform, 20s * scaling, 23s * scaling);

    rightPosition(1) -= 0.01 * scalingPosY;
    rightTransform.translation(rightPosition);
    contactListMap["right_foot"].addContact(rightTransform, 24s * scaling, 27s * scaling);

    rightPosition(1) -= 0.01 * scalingPosY;
    rightTransform.translation(rightPosition);
    contactListMap["right_foot"].addContact(rightTransform, 28s * scaling, 29s * scaling);

    // contactListMap = BipedalLocomotion::Contacts::contactListMapFromJson("footsteps.json");
    phaseList.setLists(contactListMap);

    std::vector<Eigen::VectorXd> comKnots;
    std::vector<std::chrono::nanoseconds> timeKnots;

    timeKnots.push_back(phaseList.cbegin()->beginTime);
    Eigen::Vector3d com0;
    com0 << 0, 0, 0.53;
    comKnots.push_back(com0);
    for (auto it = phaseList.begin(); it != phaseList.end(); std::advance(it, 1))
    {
        if (it->activeContacts.size() == 2 && it != phaseList.begin()
            && it != phaseList.lastPhase())
        {
            timeKnots.emplace_back((it->endTime + it->beginTime) / 2);

            auto contactIt = it->activeContacts.cbegin();
            const Eigen::Vector3d p1 = contactIt->second->pose.translation();
            std::advance(contactIt, 1);
            const Eigen::Vector3d p2 = contactIt->second->pose.translation();

            Eigen::Vector3d desiredCoMPosition = (p1 + p2) / 2.0;
            desiredCoMPosition(2) += com0(2);

            comKnots.emplace_back(desiredCoMPosition);
        }

        else if (it->activeContacts.size() == 2 && it == phaseList.lastPhase())
        {
            timeKnots.push_back(it->endTime);
            auto contactIt = it->activeContacts.cbegin();
            const Eigen::Vector3d p1 = contactIt->second->pose.translation();
            std::advance(contactIt, 1);
            const Eigen::Vector3d p2 = contactIt->second->pose.translation();

            Eigen::Vector3d desiredCoMPosition = (p1 + p2) / 2.0;
            desiredCoMPosition(2) += com0(2);

            comKnots.emplace_back(desiredCoMPosition);
        }
    }

    comSpline.setInitialConditions(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    comSpline.setFinalConditions(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    comSpline.setKnots(comKnots, timeKnots);

    Eigen::Vector3d velocity, acceleration;
    Eigen::MatrixXd comTraj(3, 1500);


    int tempInt = 1000;
    for (int i = 0; i < tempInt / scaling; i++)
    {
        // TODO remove me
        comSpline.evaluatePoint(i * dT, comTraj.col(i), velocity, acceleration);
    }

    // i = 3
    // * *
    // 1 2 3 4 5
    comTraj.rightCols(comTraj.cols() - tempInt).colwise() = comTraj.col(tempInt - 1);

    // initialize the dynamical system
    auto system = std::make_shared<CentroidalDynamics>();
    system->setState({comTraj.col(0), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()});


    constexpr std::chrono::nanoseconds integratorStepTime = 10ms;
    ForwardEuler<CentroidalDynamics> integrator;
    integrator.setIntegrationStep(integratorStepTime);
    REQUIRE(integrator.setDynamicalSystem(system));

    std::ofstream myFile;
    myFile.open("data.txt");

    // TODO reset contatcs
    int controllerIndex = 0;
    int index = 0;

    std::chrono::nanoseconds elapsedTime = 0s;
    std::chrono::nanoseconds currentTime = 0s;
    auto phaseIt = phaseList.getPresentPhase(currentTime);

    const Eigen::MatrixXd angularMomentumTraj = 0 * comTraj;

    for (int i = 0; i < 20; i++)
    {
        const auto& [com, dcom, angularMomentum] = system->getState();

        if(controllerIndex == 0)
        {
            // update the phaseList this happens only when a new contact should be established
            auto newPhaseIt = phaseList.getPresentPhase(currentTime);
            if(newPhaseIt != phaseIt)
            {
                // check if new contact is established
                if (phaseIt->activeContacts.size() == 1 && newPhaseIt->activeContacts.size() == 2)
                {
                    updateContactPhaseList(mpc.getOutput().nextPlannedContact, phaseList);

                    // the iterators have been modified we have to compute the new one
                    phaseIt = phaseList.getPresentPhase(currentTime);
                }
                else
                {
                    // the iterators did not change no need to get the present phase again
                    phaseIt = newPhaseIt;
                }

            }

            REQUIRE(mpc.setState(com, dcom, angularMomentum));
            // REQUIRE(mpc.setReferenceTrajectory(comTraj.rightCols(comTraj.cols() - index)));
            REQUIRE(mpc.setReferenceTrajectory(comTraj, angularMomentumTraj));
            REQUIRE(mpc.setContactPhaseList(phaseList));
            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
            REQUIRE(mpc.advance());
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            elapsedTime = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
            index++;
            currentTime += dT;
        }

        if (i == 0)
        {
            for (const auto& [key, contact] : mpc.getOutput().contacts)
            {
                myFile << key << "_pos_x " << key << "_pos_y " << key << "_pos_z ";
                for (int j = 0; j < contact.corners.size(); j++)
                {
                    myFile << key << "_" << j << "_x"
                           << " " << key << "_" << j << "_y"
                           << " " << key << "_" << j << "_z ";
                }

                myFile << key << "_next_pos_x " << key << "_next_pos_y " << key << "_next_pos_z ";
            }
            myFile << "com_x com_y com_z des_com_x des_com_y des_com_z ang_x ang_y ang_z "
                      "elapsed_time"
                   << std::endl;
        }

        for (const auto& [key, contact] : mpc.getOutput().contacts)
        {
            myFile << contact.pose.translation().transpose() << " ";
            for (const auto& corner : contact.corners)
            {
                myFile << corner.force.transpose() << " ";
            }

            auto nextPlannedContact = mpc.getOutput().nextPlannedContact.find(key);
            if (nextPlannedContact == mpc.getOutput().nextPlannedContact.end())
            {
                myFile << 0.0 << " " << 0.0 << " " << 0.0 << " ";
            }
            else
            {
                myFile << nextPlannedContact->second.pose.translation().transpose() << " ";
            }
        }
        myFile << com.transpose() << " " << comTraj.col(index).transpose() << " "
               << angularMomentum.transpose() << " " << elapsedTime.count() << std::endl;


        Eigen::Vector3d externalInput = Eigen::Vector3d::Zero();
        if (i > 150 && i < 170)
        {
            externalInput(1) = 1;
        }

        system->setControlInput({mpc.getOutput().contacts, externalInput});

        REQUIRE(integrator.integrate(0s, integratorStepTime));

        controllerIndex++;
        if (controllerIndex == int(dT / integratorStepTime))
        {
            controllerIndex = 0;
        }
    }

    myFile.close();
}
