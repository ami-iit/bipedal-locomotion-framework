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
    ok = ok && ptr->getParameter("joints_list", jointsList);

    std::string fileName;
    ok = ok && ptr->getParameter("model_file_name", fileName);

    if (!ok)
    {
        std::cerr << errorPrefix << "Unable to get all the parameters from configuration file."
                  << std::endl;
        return KinDynComputationsDescriptor();
    }

    iDynTree::ModelLoader mdlLdr;
    ok = ok && mdlLdr.loadReducedModelFromFile(fileName, jointsList);

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

