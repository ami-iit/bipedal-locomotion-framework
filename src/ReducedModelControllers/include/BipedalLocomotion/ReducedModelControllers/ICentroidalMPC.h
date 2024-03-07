/**
 * @file ICentroidalMPC.h
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_REDUCE_MODEL_CONTROLLERS_ICENTROIDAL_MPC_H
#define BIPEDAL_LOCOMOTION_REDUCE_MODEL_CONTROLLERS_ICENTROIDAL_MPC_H

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <casadi/casadi.hpp>

#include <BipedalLocomotion/Contacts/Contact.h>
#include <BipedalLocomotion/Contacts/ContactPhaseList.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/Source.h>

#define STR_(x) #x
#define STR(x) STR_(x)

inline bool casadiVersionIsAtLeast360()
{
    std::string str;
    std::stringstream ss(STR(casadi_VERSION));

    // Use while loop to check the getline() function condition.
    int index = 0;
    while (getline(ss, str, '.'))
    {
        if (index == 0 && stoi(str) < 3)
        {
            return false;
        }
        if (index == 1 && stoi(str) < 6)
        {
            return false;
        }
        index++;
    }

    return true;
}

inline double chronoToSeconds(const std::chrono::nanoseconds& d)
{
    return std::chrono::duration<double>(d).count();
}

inline std::vector<std::string> extractVariablesName(const std::vector<casadi::MX>& variables)
{
    std::vector<std::string> variablesName;
    variablesName.reserve(variables.size());
    for (const auto& variable : variables)
    {
        variablesName.push_back(variable.name());
    }

    return variablesName;
}

template <class T> inline auto extractFutureValuesFromState(T& variable)
{
    using Sl = casadi::Slice;
    return variable(Sl(), Sl(1, variable.columns()));
}

template <class T> inline auto extractFutureValuesFromState(const T& variable)
{
    using Sl = casadi::Slice;
    return variable(Sl(), Sl(1, variable.columns()));
}


namespace BipedalLocomotion
{
namespace ReducedModelControllers
{

struct CentroidalMPCOutput
{
    std::map<std::string, Contacts::DiscreteGeometryContact> contacts;
    Contacts::ContactPhaseList contactPhaseList; /**< Contact phase list generated by the
                                                    CentroidalMPC. */
    std::vector<Eigen::Vector3d> comTrajectory; /**< Desired CoM trajectory generated by the
                                                    CentroidalMPC. */
};

class ICentroidalMPC : public BipedalLocomotion::System::Source<CentroidalMPCOutput>
{
public:

    virtual bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler) = 0;

    virtual bool setContactPhaseList(const Contacts::ContactPhaseList& contactPhaseList) = 0;

    virtual bool setState(Eigen::Ref<const Eigen::Vector3d> com,
                          Eigen::Ref<const Eigen::Vector3d> dcom,
                          Eigen::Ref<const Eigen::Vector3d> angularMomentum)
        = 0;

    virtual bool setState(Eigen::Ref<const Eigen::Vector3d> com,
                          Eigen::Ref<const Eigen::Vector3d> dcom,
                          Eigen::Ref<const Eigen::Vector3d> angularMomentum,
                          const Math::Wrenchd& externalWrench)
        = 0;

    virtual bool setReferenceTrajectory(const std::vector<Eigen::Vector3d>& com,
                                        const std::vector<Eigen::Vector3d>& angularMomentum)
        = 0;

    virtual const CentroidalMPCOutput& getOutput() const = 0;

    virtual bool isOutputValid() const = 0;

    virtual bool advance() = 0;
};

} // namespace ReducedModelControllers
} // namespace BipedalLocomotion
#endif // BIPEDAL_LOCOMOTION_REDUCE_MODEL_CONTROLLERS_ICENTROIDAL_MPC_H

