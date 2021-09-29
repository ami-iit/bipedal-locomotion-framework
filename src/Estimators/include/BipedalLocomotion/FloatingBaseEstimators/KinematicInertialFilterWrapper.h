/**
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_ESTIMATORS_KINEMATIC_INERTIAL_FILTER_WRAPPER_H
#define BIPEDAL_LOCOMOTION_ESTIMATORS_KINEMATIC_INERTIAL_FILTER_WRAPPER_H

#include <BipedalLocomotion/FloatingBaseEstimators/FloatingBaseEstimator.h>
#include <BipedalLocomotion/FloatingBaseEstimators/InvariantEKFBaseEstimator.h>

#include <BipedalLocomotion/ContactDetectors/ContactDetector.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/Advanceable.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <Eigen/Dense>
#include <memory>
#include <unordered_map>
#include <type_traits>

namespace BipedalLocomotion
{
namespace Estimators
{

using ProprioceptiveInput = FloatingBaseEstimators::Measurements;
struct KinematicInertialFilterOutput
{
    FloatingBaseEstimators::Output estimatorOut;
    BipedalLocomotion::Contacts::EstimatedContactList contactDetectorOut;
};

template<class BaseEstimatorType, class ContactDetectorType>
class KinematicInertialFilterWrapper : BipedalLocomotion::System::Advanceable<ProprioceptiveInput, KinematicInertialFilterOutput>
{
    static_assert(std::is_base_of_v<FloatingBaseEstimator, BaseEstimatorType>,
                  "The BaseEstimatorType class must be derived from BipedalLocomotion::Estimators::FloatingBaseEstimator class.");
    static_assert(std::is_base_of_v<BipedalLocomotion::Contacts::ContactDetector, ContactDetectorType>,
                  "The ContactDetectorType class must be derived from BipedalLocomotion::ContactDetectors::ContactDetector class.");

public:
    KinematicInertialFilterWrapper();

    bool initialize(std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler) override;
    bool initialize(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
                    std::shared_ptr<iDynTree::KinDynComputations> kindyn);

    bool setInput(const ProprioceptiveInput& input) override;
    bool advance() override;
    const KinematicInertialFilterOutput& getOutput() const override;
    bool isOutputValid() const override;

    BaseEstimatorType& estimator();
    ContactDetectorType& contactDetector();

private:
    BaseEstimatorType m_estimator;
    ContactDetectorType m_contactDetector;
    ProprioceptiveInput m_input;
    KinematicInertialFilterOutput m_output;
    bool m_initialized{false};
};

template <class BaseEstimatorType, class ContactDetectorType>
KinematicInertialFilterWrapper<BaseEstimatorType,
                               ContactDetectorType>::KinematicInertialFilterWrapper()
{
}

template <class BaseEstimatorType, class ContactDetectorType>
bool KinematicInertialFilterWrapper<BaseEstimatorType, ContactDetectorType>::initialize(
    std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
{
    const std::string printPrefix{"[KinematicInertialFilterWrapper::initialize]"};
    BipedalLocomotion::log()->error("{} Please use initialize(handler, kinDyn) method instead.",
                                    printPrefix);
    return false;
}

template <typename BaseEstimatorType, typename ContactDetectorType>
ContactDetectorType&
KinematicInertialFilterWrapper<BaseEstimatorType, ContactDetectorType>::contactDetector()
{
    return m_contactDetector;
}

template <typename BaseEstimatorType, typename ContactDetectorType>
bool KinematicInertialFilterWrapper<BaseEstimatorType, ContactDetectorType>::initialize(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
    std::shared_ptr<iDynTree::KinDynComputations> kindyn)
{
    const std::string printPrefix{"[KinematicInertialFilterWrapper::initialize]"};
    auto handle = handler.lock();
    if (handle == nullptr)
    {
        BipedalLocomotion::log()->error("{} The parameter handler has expired."
                                        "Please check its scope.",
                                        printPrefix);
        return false;
    }

    bool ok{true};
    ok = ok && m_contactDetector.initialize(handle->getGroup("CONTACT_DETECTOR"));
    if (!ok)
    {
        BipedalLocomotion::log()->error("{} Failed to initialize contact detector.",
                                        printPrefix);
        return false;
    }

    ok = ok && m_estimator.initialize(handle->getGroup("BASE_ESTIMATOR"), kindyn);
    if (!ok)
    {
        BipedalLocomotion::log()->error("{} Failed to initialize base estimator.",
                                        printPrefix);
        return false;
    }

    m_initialized = true;
    return true;
}

template <typename BaseEstimatorType, typename ContactDetectorType>
BaseEstimatorType&
KinematicInertialFilterWrapper<BaseEstimatorType, ContactDetectorType>::estimator()
{
    return m_estimator;
}

template <class BaseEstimatorType, class ContactDetectorType>
bool KinematicInertialFilterWrapper<BaseEstimatorType, ContactDetectorType>::setInput(
    const ProprioceptiveInput& input)
{
    std::string printPrefix{"[KinematicInertialFilterWrapper::setInput]"};
    if (!m_initialized)
    {
        BipedalLocomotion::log()->error("{} Please call initialize first", printPrefix);
        return false;
    }

    m_input = input;
    return true;
}

template <class BaseEstimatorType, class ContactDetectorType>
bool KinematicInertialFilterWrapper<BaseEstimatorType, ContactDetectorType>::advance()
{
    std::string printPrefix{"[KinematicInertialFilterWrapper::advance]"};
    if (!m_initialized)
    {
        BipedalLocomotion::log()->error("{} Please call initialize first", printPrefix);
        return false;
    }

    const int fzOffset{2};
    for (const auto& [contactFrame, contactWrench] : m_input.contactWrenches)
    {
        m_contactDetector.setTimedTriggerInput(contactFrame,
                                               m_input.ts,
                                               contactWrench(fzOffset));
    }

    bool ok{true};
    ok = ok && m_contactDetector.advance();
    m_output.contactDetectorOut = m_contactDetector.getOutput();
    const auto& contactMap = m_output.contactDetectorOut; // get an alias

    m_estimator.setIMUMeasurement(m_input.acc, m_input.gyro);
    m_estimator.setKinematics(m_input.encoders, m_input.encodersSpeed);

    if (std::is_same_v<BaseEstimatorType, BipedalLocomotion::Estimators::InvariantEKFBaseEstimator>)
    {
        if (contactMap.find(m_input.lfContactFrameName) != contactMap.end() &&
            contactMap.find(m_input.rfContactFrameName) != contactMap.end())
        {
            auto lfState = contactMap.at(m_input.lfContactFrameName).isActive;
            auto rfState = contactMap.at(m_input.rfContactFrameName).isActive;
            m_estimator.setContacts(lfState, rfState);
        }
    }
    else
    {
        for (const auto& [id, contact] : contactMap)
        {
            m_estimator.setContactStatus(contact.name,
                                         contact.isActive,
                                         contact.switchTime,
                                         m_input.ts);
        }
    }

    ok = ok && m_estimator.advance();
    m_output.estimatorOut = m_estimator.getOutput();

    return ok;
}

template <class BaseEstimatorType, class ContactDetectorType>
bool KinematicInertialFilterWrapper<BaseEstimatorType, ContactDetectorType>::isOutputValid() const
{
    return m_initialized;
}

template <class BaseEstimatorType, class ContactDetectorType>
const KinematicInertialFilterOutput&
KinematicInertialFilterWrapper<BaseEstimatorType, ContactDetectorType>::getOutput() const
{
    return m_output;
}


} // namespace Estimators
} // namespace KinDynVIO

#endif // BIPEDAL_LOCOMOTION_ESTIMATORS_KINEMATIC_INERTIAL_FILTER_WRAPPER_H
