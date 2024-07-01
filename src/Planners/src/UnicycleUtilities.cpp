/**
 * @file UnicycleUtilities.cpp
 * @authors Lorenzo Moretti, Giulio Romualdi, Stefano Dafarra
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */
#include <BipedalLocomotion/Contacts/ContactList.h>
#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/Planners/UnicycleUtilities.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/SpatialAcc.h>
#include <iDynTree/VectorDynSize.h>
#include <iDynTree/VectorFixSize.h>


namespace BipedalLocomotion::Planners::UnicycleUtilities
{

bool getContactList(const std::chrono::nanoseconds& initTime,
                    const std::chrono::nanoseconds& dt,
                    const std::vector<bool>& inContact,
                    const std::deque<Step>& steps,
                    const int& contactFrameIndex,
                    const std::string& contactName,
                    BipedalLocomotion::Contacts::ContactList& contactList)
{

    constexpr auto logPrefix = "[UnicycleUtilities::getContactList]";

    if (contactList.size() > 1)
    {
        BipedalLocomotion::log()->error("{} The contact list has size greater than 1. Size should "
                                        "be 0 or 1.",
                                        logPrefix);
        return false;
    }

    size_t impactTimeIndex{0};
    auto stepIterator = steps.begin();

    while (stepIterator != steps.end())
    {
        auto step = *stepIterator;

        BipedalLocomotion::Contacts::PlannedContact contact{};

        contact.index = contactFrameIndex;
        contact.name = contactName;

        Eigen::Vector3d translation = Eigen::Vector3d::Zero();
        translation.head<2>() = iDynTree::toEigen(step.position);
        manif::SO3d rotation{0, 0, step.angle};
        contact.pose = manif::SE3d(translation, rotation);

        std::chrono::nanoseconds impactTime{static_cast<int64_t>(step.impactTime * 1e9)};

        contact.activationTime = impactTime;
        contact.deactivationTime = std::chrono::nanoseconds::max();

        impactTimeIndex = (impactTime <= initTime) ? 0
                                                   : static_cast<int>((impactTime - initTime) / dt);

        for (auto i = impactTimeIndex; i < inContact.size(); i++)
        {
            if (i > 0 && !inContact.at(i) && inContact.at(i - 1))
            {
                contact.deactivationTime = initTime + dt * i;
                break;
            }
        }

        if ((stepIterator == steps.begin()) && (contactList.size() == 1) && (impactTimeIndex == 0))
        {
            // editing the first step if the contact list is not empty
            // since the first contact, being the current active one,
            // is already in the contact list

            if (!contactList.editContact(contactList.begin(), contact))
            {
                BipedalLocomotion::log()->error("{} Error while editing the first contact of the "
                                                "contact list.",
                                                logPrefix);

                return false;
            }
        } else
        {
            if (!contactList.addContact(contact))
            {
                BipedalLocomotion::log()->error("{} Error while adding contact to the contact "
                                                "list.",
                                                logPrefix);

                return false;
            }
        }

        stepIterator++;
    };

    return true;
};

void mergeSteps(const std::deque<Step>& newSteps,
                std::deque<Step>& currentSteps,
                const std::chrono::nanoseconds& currentTime)
{
    // lambda function to check if a step (impact time) is later than the current time
    auto isLater = [currentTime](const Step& step) -> bool {
        return step.impactTime >= static_cast<int>(currentTime.count() * 1e-9);
    };

    // find the first new step with impact time greater than the current time
    auto itNew = std::find_if(newSteps.begin(), newSteps.end(), isLater);

    // find the first current step with impact time greater than the current time.
    // from this point on, the current steps are discarded, and the new steps are added.
    auto itCurrent = std::find_if(currentSteps.begin(), currentSteps.end(), isLater);
    Step firstDeletedStep;
    if (itCurrent != currentSteps.end())
    {
        firstDeletedStep = *itCurrent;
    }

    // erase all current steps which are after the current time
    currentSteps.erase(itCurrent, currentSteps.end());

    // if the are new valid steps, append them to the current steps
    if (itNew != newSteps.end())
    {
        // append new steps to the current steps
        currentSteps.insert(currentSteps.end(), itNew, newSteps.end());
    }

    // Check if merged steps are empty. In this case use firstDeletedStep.
    if (currentSteps.empty())
    {
        currentSteps.push_back(firstDeletedStep);
    }
}

namespace Conversions
{
void convertToBLF(const iDynTree::Transform& input, manif::SE3d& output)
{
    output.asSO3() = iDynTree::toEigen(input.getRotation().asQuaternion());
    output.translation(iDynTree::toEigen(input.getPosition()));
}

void convertToBLF(const iDynTree::Twist& input, manif::SE3d::Tangent& output)
{
    output = iDynTree::toEigen(input);
}

void convertToBLF(const iDynTree::SpatialAcc& input, manif::SE3d::Tangent& output)
{
    output = iDynTree::toEigen(input);
}

void convertToBLF(const iDynTree::VectorDynSize& input, Eigen::VectorXd& output)
{
    output = iDynTree::toEigen(input);
}

void convertToBLF(const iDynTree::Vector2& input, Eigen::Vector2d& output)
{
    output = iDynTree::toEigen(input);
}

void convertToBLF(const iDynTree::Vector3& input, Eigen::Vector3d& output)
{
    output = iDynTree::toEigen(input);
}
} // namespace Conversions
} // namespace BipedalLocomotion::Planners::UnicycleUtilities
