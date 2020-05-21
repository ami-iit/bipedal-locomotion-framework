/**
 * @file LinearTimeInvariantSystem.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_LINEAR_TIME_INVARIANT_SYSTEM_H
#define BIPEDAL_LOCOMOTION_SYSTEM_LINEAR_TIME_INVARIANT_SYSTEM_H

#include <memory>
#include <vector>

#include <BipedalLocomotion/System/DynamicalSystem.h>

#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/VectorDynSize.h>

namespace BipedalLocomotion
{
namespace System
{
class LinearTimeInvariantSystem : public DynamicalSystem<std::tuple<iDynTree::VectorDynSize>,
                                                         std::tuple<iDynTree::VectorDynSize>,
                                                         std::tuple<iDynTree::VectorDynSize>>
{
    iDynTree::MatrixDynSize m_A;
    iDynTree::MatrixDynSize m_B;

    bool m_isInitialized{false};

public:
    bool setSystemMatrices(const iDynTree::MatrixDynSize& A, const iDynTree::MatrixDynSize& b);

    bool dynamics(const StateType& state,
                  const double& time,
                  StateDerivativeType& stateDerivative) final;
};

} // namespace System
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_LINEAR_TIME_INVARIANT_SYSTEM_H
