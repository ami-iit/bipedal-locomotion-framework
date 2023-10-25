/**
 * @file GlobalZMPEvaluator.cpp
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/Contacts/GlobalZMPEvaluator.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion::Contacts;

bool GlobalZMPEvaluator::initialize(
    std::weak_ptr<const ParametersHandler::IParametersHandler> handler)
{
    constexpr auto logPrefix = "[GlobalZMPEvaluator::initialize]";

    m_isInitialized = false;
    m_isOutputValid = false;

    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The handler is not valid.", logPrefix);
        return false;
    }

    if (!ptr->getParameter("minimum_normal_force", m_minimumNormalForce))
    {
        log()->error("{} Unable to retrieve the minimum normal force.", logPrefix);
        return false;
    }

    if (!ptr->getParameter("zmp_admissible_limits", m_zmpAdmissibleLimits))
    {
        log()->error("{} Unable to retrieve the zmp admissible limits.", logPrefix);
        return false;
    }

    if (!ptr->getParameter("constant_zmp_tolerance", m_constantZMPTolerance))
    {
        log()->error("{} Unable to retrieve the constant zmp tolerance.", logPrefix);
        return false;
    }

    if (!ptr->getParameter("constant_zmp_max_counter", m_constantZMPMaxCounter))
    {
        log()->error("{} Unable to retrieve the constant zmp max counter.", logPrefix);
        return false;
    }

    // set the ZMP to zero
    m_zmp.setZero();
    m_constantZMPCounter = 0;

    m_isInitialized = true;
    m_isOutputValid = false;

    return true;
}

bool GlobalZMPEvaluator::setInput(const std::initializer_list<Contacts::ContactWrench>& input)
{
    // save the contacts
    m_contacts = input;
    return true;
}

bool GlobalZMPEvaluator::setInput(const std::vector<Contacts::ContactWrench>& input)
{
    // save the contacts
    m_contacts = input;
    return true;
}

bool GlobalZMPEvaluator::isOutputValid() const
{
    return m_isOutputValid;
}

const Eigen::Vector3d& GlobalZMPEvaluator::getOutput() const
{
    return m_zmp;
}

bool GlobalZMPEvaluator::advance()
{
    constexpr auto logPrefix = "[GlobalZMPEvaluator::advance]";
    m_isOutputValid = false;

    if (!m_isInitialized)
    {
        log()->error("{} The object is not initialized. Please call "
                     "GlobalZMPEvaluator::initialize().",
                     logPrefix);
        return false;
    }

    Eigen::Vector3d globalZMP;
    Eigen::Vector3d localZMP;
    double totalForce = 0;
    int numberOfActiveSupports = 0;
    for (const auto& contact : m_contacts)
    {
        // check if the z component of the force is greater than a threshold
        if (contact.wrench.force()[2] < m_minimumNormalForce)
        {
            continue;
        }

        // the local ZMP is defined since fz is greater than zero
        localZMP = contact.getLocalZMP();

        // check if the zmp is outside the admissible limits
        if (std::abs(localZMP[0]) > m_zmpAdmissibleLimits[0]
            || std::abs(localZMP[1]) > m_zmpAdmissibleLimits[1])
        {
            continue;
        }

        globalZMP += contact.wrench.force()[2] * contact.pose.act(localZMP);
        totalForce += contact.wrench.force()[2];

        numberOfActiveSupports++;
    }

    if (numberOfActiveSupports == 0)
    {
        log()->error("{} Zero contacts are active the ZMP is not valid.", logPrefix);
        return false;
    }

    // compute the global ZMP.
    globalZMP /= totalForce;

    // at least two contacts are active and the counter is active
    if (numberOfActiveSupports > 1 && m_constantZMPMaxCounter > 0)
    {
        const double zmpDifference = (globalZMP - m_zmp).norm();

        // check if the ZMP is constant
        if (zmpDifference < m_constantZMPTolerance)
        {
            // ZMP considered constant so increase the counter
            m_constantZMPCounter++;
        } else
        {
            // ZMP is not constant so reset the counter
            m_constantZMPCounter = 0;
        }

        if (m_constantZMPCounter >= m_constantZMPMaxCounter / 2)
        {
            log()->warn("{} The ZMP was constant (in a {}m radius) for {} times.",
                        logPrefix,
                        m_constantZMPTolerance,
                        m_constantZMPCounter);
        }

        if (m_constantZMPCounter >= m_constantZMPMaxCounter)
        {
            log()->error("{} The ZMP was constant (in a {} m radius) for {} times.",
                         logPrefix,
                         m_constantZMPTolerance,
                         m_constantZMPCounter);

            return false;
        }
    } else
    {
        m_constantZMPCounter = 0;
    }

    // the zmp is valid
    m_zmp = std::move(globalZMP);
    m_isOutputValid = true;
    return m_isOutputValid;
}
