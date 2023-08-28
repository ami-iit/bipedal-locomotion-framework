/**
 * @file CentroidalMPC.h
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_REDUCE_MODEL_CONTROLLERS_ADAPTIVE_CENTROIDAL_MPC_H
#define BIPEDAL_LOCOMOTION_REDUCE_MODEL_CONTROLLERS_ADAPTIVE_CENTROIDAL_MPC_H

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <BipedalLocomotion/Contacts/Contact.h>
#include <BipedalLocomotion/Contacts/ContactPhaseList.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/Source.h>
#include <BipedalLocomotion/ReducedModelControllers/BaseCentroidalMPC.h>

namespace BipedalLocomotion
{
namespace ReducedModelControllers
{

class StableCentroidalMPC : public  BaseCentroidalMPC
{
public:

    StableCentroidalMPC();

    ~StableCentroidalMPC();

    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler) override;

    bool setContactPhaseList(const Contacts::ContactPhaseList& contactPhaseList) override;

    bool setState(Eigen::Ref<const Eigen::Vector3d> com,
                  Eigen::Ref<const Eigen::Vector3d> dcom,
                  Eigen::Ref<const Eigen::Vector3d> angularMomentum) override;

    bool setState(Eigen::Ref<const Eigen::Vector3d> com,
                  Eigen::Ref<const Eigen::Vector3d> dcom,
                  Eigen::Ref<const Eigen::Vector3d> angularMomentum,
                  const Math::Wrenchd& externalWrench) override;

    bool setReferenceTrajectory(const std::vector<Eigen::Vector3d>& com,
                                const std::vector<Eigen::Vector3d>& angularMomentum) override;

    const CentroidalMPCOutput& getOutput() const override;

    bool isOutputValid() const override;

    bool advance() override;

private:

    struct Impl;

    std::unique_ptr<Impl> m_pimpl; /**< Pointer to private implementation */
};
} // namespace ReducedModelControllers
} // namespace BipedalLocomotion


#endif // BIPEDAL_LOCOMOTION_REDUCE_MODEL_CONTROLLERS_ADAPTIVE_CENTROIDAL_MPC_H
