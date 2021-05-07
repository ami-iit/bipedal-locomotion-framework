/**
 * @file ModelComputationsHelper.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/FloatingBaseEstimators/ModelComputationsHelper.h>

#include <string>
#include <iDynTree/ModelIO/ModelLoader.h>

using namespace BipedalLocomotion::Estimators;

KinDynComputationsDescriptor::KinDynComputationsDescriptor(std::shared_ptr<iDynTree::KinDynComputations> kindyn)
    : kindyn(kindyn)
{
}

KinDynComputationsDescriptor::KinDynComputationsDescriptor() = default;

bool BipedalLocomotion::Estimators::KinDynComputationsDescriptor::isValid() const
{
    return kindyn != nullptr;
}

int BipedalLocomotion::Estimators::KinDynComputationsDescriptor::getNrOfDegreesOfFreedom() const
{
    return kindyn->getNrOfDegreesOfFreedom();
}

bool BipedalLocomotion::Estimators::KinDynComputationsDescriptor::getJointPos(Eigen::Ref<Eigen::VectorXd> q) const
{
    return kindyn->getJointPos(q);
}

bool BipedalLocomotion::Estimators::KinDynComputationsDescriptor::setJointPos(const Eigen::Ref<const Eigen::VectorXd> s)
{
    return kindyn->setJointPos(s);
}

bool BipedalLocomotion::Estimators::KinDynComputationsDescriptor::getRobotState(Eigen::Ref<Eigen::Matrix<double, 4, 4>> world_T_base,
                                                                                Eigen::Ref<Eigen::VectorXd> s,
                                                                                Eigen::Ref<Eigen::VectorXd> base_velocity,
                                                                                Eigen::Ref<Eigen::VectorXd> s_dot,
                                                                                Eigen::Ref<Eigen::VectorXd> world_gravity) const
{
    return kindyn->getRobotState(world_T_base, s, base_velocity, s_dot, world_gravity);
}

bool BipedalLocomotion::Estimators::KinDynComputationsDescriptor::setRobotState(Eigen::Ref<const Eigen::Matrix<double, 4, 4>> world_T_base,
                                                                                Eigen::Ref<const Eigen::VectorXd> s,
                                                                                Eigen::Ref<const Eigen::VectorXd> base_velocity,
                                                                                Eigen::Ref<const Eigen::VectorXd> s_dot,
                                                                                Eigen::Ref<const Eigen::VectorXd> world_gravity)
{
    return kindyn->setRobotState(world_T_base, s, base_velocity, s_dot, world_gravity);
}

KinDynComputationsDescriptor BipedalLocomotion::Estimators::constructKinDynComputationsDescriptor(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
{
    constexpr std::string_view errorPrefix = "[constructKinDynComputationsDescriptor] ";

    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        std::cerr << errorPrefix << "IParametershandler is empty. Returning an invalid KinDynComputationsDescriptor" << std::endl;
        return KinDynComputationsDescriptor();
    }

    bool ok{true};
    std::vector<std::string> jointsList;
    bool loadFullModel{false};
    if (!ptr->getParameter("joints_list", jointsList))
    {
        loadFullModel = true;
    }

    std::string fileName;
    ok = ok && ptr->getParameter("model_file_name", fileName);

    if (!ok)
    {
        std::cerr << errorPrefix << "Unable to get all the parameters from configuration file."
                  << std::endl;
        return KinDynComputationsDescriptor();
    }

    iDynTree::ModelLoader mdlLdr;
    if (loadFullModel)
    {
        ok = ok && mdlLdr.loadModelFromFile(fileName);
    }
    else
    {
        ok = ok && mdlLdr.loadReducedModelFromFile(fileName, jointsList);
    }

    if (!ok)
    {
        std::cerr << errorPrefix << "Could not load the model using the specified parameters."
                  << std::endl;
        return KinDynComputationsDescriptor();
    }

    KinDynComputationsDescriptor kindynDesc(std::make_shared<iDynTree::KinDynComputations>());
    if (!kindynDesc.kindyn->loadRobotModel(mdlLdr.model()))
    {
        std::cerr << errorPrefix << "Could not load a valid KinDynComputations object."
                  << std::endl;
        return KinDynComputationsDescriptor();
    }

    return kindynDesc;
}

