/**
 * @file GlobalCoPEvaluator.cpp
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/Contacts/GlobalCoPEvaluator.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion::Contacts;

bool GlobalCoPEvaluator::initialize(
    std::weak_ptr<const ParametersHandler::IParametersHandler> handler)
{
    constexpr auto logPrefix = "[GlobalCoPEvaluator::initialize]";

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

    if (!ptr->getParameter("cop_admissible_limits", m_CoPAdmissibleLimits))
    {
        log()->error("{} Unable to retrieve the CoP admissible limits.", logPrefix);
        return false;
    }

    if (!ptr->getParameter("constant_cop_tolerance", m_constantCoPTolerance))
    {
        log()->error("{} Unable to retrieve the constant CoP tolerance.", logPrefix);
        return false;
    }

    if (!ptr->getParameter("constant_cop_max_counter", m_constantCoPMaxCounter))
    {
        log()->error("{} Unable to retrieve the constant CoP max counter.", logPrefix);
        return false;
    }

    // set the CoP to zero
    m_cop.setZero();
    m_constantCoPCounter = 0;

    m_isInitialized = true;
    m_isOutputValid = false;

    return true;
}

bool GlobalCoPEvaluator::setInput(const std::initializer_list<Contacts::ContactWrench>& input)
{
    // save the contacts
    m_contacts = input;
    return true;
}

bool GlobalCoPEvaluator::setInput(const std::vector<Contacts::ContactWrench>& input)
{
    // save the contacts
    m_contacts = input;
    return true;
}

bool GlobalCoPEvaluator::isOutputValid() const
{
    return m_isOutputValid;
}

const Eigen::Vector3d& GlobalCoPEvaluator::getOutput() const
{
    return m_cop;
}

bool GlobalCoPEvaluator::advance()
{
    constexpr auto logPrefix = "[GlobalCoPEvaluator::advance]";
    m_isOutputValid = false;

    if (!m_isInitialized)
    {
        log()->error("{} The object is not initialized. Please call "
                     "GlobalCoPEvaluator::initialize().",
                     logPrefix);
        return false;
    }

    Eigen::Vector3d globalCoP = Eigen::Vector3d::Zero();
    Eigen::Vector3d localCoP;
    double totalForce = 0;
    int numberOfActiveSupports = 0;
    for (const auto& contact : m_contacts)
    {
        // check if the z component of the force is greater than a threshold
        if (contact.wrench.force()[2] < m_minimumNormalForce)
        {
            continue;
        }

        // the local CoP is defined since fz is greater than zero
        localCoP = contact.wrench.getLocalCoP();

        // check if the CoP is outside the admissible limits
        if (std::abs(localCoP[0]) > m_CoPAdmissibleLimits[0]
            || std::abs(localCoP[1]) > m_CoPAdmissibleLimits[1])
        {
            continue;
        }

        globalCoP += contact.wrench.force()[2] * contact.pose.act(localCoP);
        totalForce += contact.wrench.force()[2];

        numberOfActiveSupports++;
    }

    if (numberOfActiveSupports == 0)
    {
        log()->error("{} Zero contacts are active, the CoP is not valid.", logPrefix);
        return false;
    }

    // compute the global CoP.
    globalCoP /= totalForce;

    // at least two contacts are active and the counter is active
    if (numberOfActiveSupports > 1 && m_constantCoPMaxCounter > 0)
    {
        const double CoPDifference = (globalCoP - m_cop).norm();

        // check if the CoP is constant
        if (CoPDifference < m_constantCoPTolerance)
        {
            // CoP considered constant so increase the counter
            m_constantCoPCounter++;
        } else
        {
            // CoP is not constant so reset the counter
            m_constantCoPCounter = 0;
        }

        if (m_constantCoPCounter >= m_constantCoPMaxCounter / 2)
        {
            log()->warn("{} The CoP was constant (in a {}m radius) for {} times.",
                        logPrefix,
                        m_constantCoPTolerance,
                        m_constantCoPCounter);
        }

        if (m_constantCoPCounter >= m_constantCoPMaxCounter)
        {
            log()->error("{} The CoP was constant (in a {} m radius) for {} times.",
                         logPrefix,
                         m_constantCoPTolerance,
                         m_constantCoPCounter);

            return false;
        }
    } else
    {
        m_constantCoPCounter = 0;
    }

    // the CoP is valid
    m_cop = std::move(globalCoP);
    m_isOutputValid = true;
    return m_isOutputValid;
}
