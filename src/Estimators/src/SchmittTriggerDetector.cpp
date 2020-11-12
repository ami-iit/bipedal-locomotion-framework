/**
 * @file SchmittTriggerDetector.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/ContactDetectors/SchmittTriggerDetector.h>

using namespace BipedalLocomotion::Estimators;
using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::Planners;

class SchmittTriggerDetector::Impl
{
public:

    std::unordered_map<std::string, std::pair<SchmittTriggerParams, SchmittTriggerUnit> > manager; /**< Container for Params-SchmittTrigger pairs of contacts */
    std::unordered_map<std::string, std::pair<double, double> > forceMeasure; /**< Container for Timestamp-ForceMeasure pairs of contacts */

    /**
     * Utility function to check if contact exists
     */
    bool contactExists(const std::string& contactName);

    /**
     * Utility function to load vector parameters
     */
    template<typename Scalar>
    bool setupParamV(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
                     const std::string& param, std::vector<Scalar>& vec, const std::string& prefix)
    {
        auto handle = handler.lock();
        if (handle == nullptr)
        {
            return false;
        }

        if (!handle->getParameter(param, GenericContainer::make_vector(vec, GenericContainer::VectorResizeMode::Resizable)))
        {
            std::cerr << prefix << "The parameter handler could not find \"" << param << "\" in the configuration file. This is a required parameter." << std::endl;
            return false;
        }
        return true;
    }
};

SchmittTriggerDetector::SchmittTriggerDetector() : m_pimpl(std::make_unique<Impl>())
{
    m_contactStates.clear();
    m_pimpl->manager.clear();
    m_pimpl->forceMeasure.clear();
}

SchmittTriggerDetector::~SchmittTriggerDetector() = default;

bool SchmittTriggerDetector::customInitialization(std::weak_ptr<IParametersHandler> handler)
{
    std::string printPrefix{"[SchmittTriggerDetector::customInitialization] "};
    auto handle = handler.lock();
    if (handle == nullptr)
    {
        return false;
    }

    std::vector<std::string> contacts;
    if (! m_pimpl->setupParamV(handler, "contacts", contacts, printPrefix))
    {
        return false;
    }

    std::vector<double> onThreshold;
    if (! m_pimpl->setupParamV(handler, "contact_make_thresholds", onThreshold, printPrefix))
    {
        return false;
    }

    std::vector<double> offThreshold;
    if (! m_pimpl->setupParamV(handler, "contact_break_thresholds", offThreshold, printPrefix))
    {
        return false;
    }

    std::vector<double> switchOnAfter;
    if (! m_pimpl->setupParamV(handler, "contact_make_switch_times", switchOnAfter, printPrefix))
    {
        return false;
    }

    std::vector<double> switchOffAfter;
    if (! m_pimpl->setupParamV(handler, "contact_break_switch_times", switchOffAfter, printPrefix))
    {
        return false;
    }

    if ( (contacts.size() != onThreshold.size()) || (contacts.size() != offThreshold.size()) ||
        (contacts.size() != switchOnAfter.size()) || (contacts.size() != switchOffAfter.size()) )
    {
        std::cerr << printPrefix << "Configuration parameters size mismatch." << std::endl;
        return false;
    }

    for (std::size_t idx = 0; idx < contacts.size(); idx++)
    {
        const auto& name = contacts[idx];
        SchmittTriggerParams params;
        params.onThreshold = onThreshold[idx];
        params.offThreshold = offThreshold[idx];
        params.switchOnAfter = switchOnAfter[idx];
        params.switchOffAfter = switchOffAfter[idx];

        if (!addContact(contacts[idx], false, params))
        {
            std::cerr << printPrefix << "Could not add Schmitt Trigger unit for specified contact." << std::endl;
            return false;
        }
    }

    return true;
}

bool SchmittTriggerDetector::updateContactStates()
{
    for (auto& [contactName, schmittTrigger] : m_pimpl->manager)
    {
        auto& contact = m_contactStates.at(contactName);
        auto& detectorUnit =  m_pimpl->manager.at(contactName);
        auto prevState = contact.isActive;
        const auto& measure = m_pimpl->forceMeasure.at(contactName);
        detectorUnit.second.update(measure.first, measure.second);

        contact.isActive = detectorUnit.second.getState();
    }
    return true;
}

bool SchmittTriggerDetector::setTimedContactIntensity(const std::string& contactName,
                                                      const double& time,
                                                      const double& force)
{
    std::string_view printPrefix = "[SchmittTriggerDetector::setTimedContactIntensity] ";
    if (!m_pimpl->contactExists(contactName))
    {
        std::cerr << printPrefix << "Contact does not exist. Cannot set measurement." << std::endl;
        return false;
    }

    m_pimpl->forceMeasure.at(contactName).first = time;
    m_pimpl->forceMeasure.at(contactName).second = force;

    return true;
}

bool SchmittTriggerDetector::setTimedContactIntensities(const std::unordered_map<std::string, std::pair<double, double> >& timedForces)
{
    std::string_view printPrefix = "[SchmittTriggerDetector::setTimedContactIntensities] ";
    std::vector<std::string> skippedUpdates;
    for (auto& [contactName, measure] : timedForces)
    {
        if (!m_pimpl->contactExists(contactName))
        {
            skippedUpdates.emplace_back(contactName);
            continue;
        }

        m_pimpl->forceMeasure.at(contactName) = measure;
    }

    if (skippedUpdates.size() > 0)
    {
        std::cerr << printPrefix << "Skipped setting measurements for contact that does not exist." << std::endl;
        for (const auto& skipped : skippedUpdates)
        {
            std::cerr << " - " << skipped << std::endl;
        }
    }

    return true;
}

bool SchmittTriggerDetector::addContact(const std::string& contactName,
                                        const bool& initialState,
                                        const SchmittTriggerParams& params)
{
    std::string_view printPrefix = "[SchmittTriggerDetector::addContact] ";
    if (m_pimpl->contactExists(contactName))
    {
        std::cerr << printPrefix << "Contact already exists." << std::endl;
        return false;
    }

    Contact newContact;
    newContact.isActive = initialState;
    newContact.name = contactName;

    SchmittTriggerUnit schmittTrigger;
    schmittTrigger.setParams(params);
    schmittTrigger.setState(initialState);

    m_pimpl->manager[contactName] = std::make_pair(params, schmittTrigger);
    m_pimpl->forceMeasure[contactName] = std::make_pair(0.0, 0.0);
    m_contactStates[contactName] = newContact;
    return true;
}

bool SchmittTriggerDetector::removeContact(const std::string& contactName)
{
    std::string_view printPrefix = "[SchmittTriggerDetector::removeContact] ";
    if (!m_pimpl->contactExists(contactName))
    {
        std::cerr << printPrefix << "Contact does not exist." << std::endl;
        return false;
    }

    m_pimpl->manager.erase(contactName);
    m_pimpl->forceMeasure.erase(contactName);
    m_contactStates.erase(contactName);
    return true;
}

bool SchmittTriggerDetector::resetContact(const std::string& contactName,
                                          const bool& state,
                                          const SchmittTriggerParams& params)
{
    std::string_view printPrefix = "[SchmittTriggerDetector::resetContact] ";
    if (!m_pimpl->contactExists(contactName))
    {
        std::cerr << printPrefix << "Contact does not exist." << std::endl;
        return false;
    }

    m_pimpl->manager.at(contactName).first = params;
    m_pimpl->forceMeasure.at(contactName) = std::make_pair(0.0, 0.);
    m_contactStates.at(contactName).isActive = state;
    m_contactStates.at(contactName).activationTime = 0.0;
    m_contactStates.at(contactName).deactivationTime = 0.0;
    return true;
}


bool SchmittTriggerDetector::Impl::contactExists(const std::string& contactName)
{
    if (manager.find(contactName) == manager.end())
    {
        return false;
    }

    return true;
}

//////////////////////////////////
// Schmitt Trigger Unit methods///
//////////////////////////////////

void SchmittTriggerUnit::reset()
{
    timer = 0.;
    previousTime = 0.;

    state = false;
}

void SchmittTriggerUnit::setParams(const SchmittTriggerParams& paramsIn)
{
    params = paramsIn;
}

void SchmittTriggerUnit::setState(const bool& stateIn)
{
    state = stateIn;
}

bool SchmittTriggerUnit::getState()
{
    return state;
}

SchmittTriggerParams SchmittTriggerUnit::getParams()
{
    return params;
}

void SchmittTriggerUnit::update(const double& currentTime, const double& rawValue)
{
    if (previousTime == 0)
    {
        (currentTime > 0) ? previousTime = 0 : previousTime = currentTime;
    }

    if (!state)
    {
        // Check for transition from false to true - if valid over a timeframe, then switch
        if (rawValue >= params.onThreshold)
        {
            (timer >= params.switchOnAfter) ?  state = true : timer += (currentTime - previousTime);
            if (state)
            {
                timer = 0;
            }
        }
        else
        {
            timer = 0;
        }
    }
    else
    {
        // check for transition from true to false - if valid over a timeframe, then switch
        if (rawValue <= params.offThreshold)
        {
            (timer >= params.switchOffAfter) ? state = false : timer += (currentTime - previousTime);
            if (!state)
            {
                timer = 0;
            }
        }
        else
        {
            timer = 0;
        }
    }
    previousTime = currentTime;
}
