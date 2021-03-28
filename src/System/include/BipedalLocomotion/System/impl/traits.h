/**
 * @file traits.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_IMPL_TRAITS_H
#define BIPEDAL_LOCOMOTION_SYSTEM_IMPL_TRAITS_H

/**
 * The user must call this macro before defining a custom ContinuousDynamicalSystem::DynamicalSystem
 * @param DynamicalSystemType the type of the dynamical system
 * @param StateType the list of the types used to define the state. The list must be defined using
 * round parenthesis. E.g. `(Eigen::VectorXd, Eigen::VectorXd)`.
 * @param StateDerivativeType the list of the types used to define the state derivative. The list
 * must be defined using round parenthesis. E.g. `(Eigen::VectorXd, Eigen::VectorXd)`.
 * @param InputType the list of the types used to define inputs. The list must be defined using
 * round parenthesis. E.g. `(Eigen::VectorXd, Eigen::VectorXd)`.
 */
#define BLF_DEFINE_SYSTEM_BLOCK_INTERAL_STRUCTURE(BlockType, InputType, OutputTime) \
    namespace BipedalLocomotion                                                     \
    {                                                                               \
    namespace System                                                                \
    {                                                                               \
    namespace internal                                                              \
    {                                                                               \
    template <> struct traits<BlockType>                                            \
    {                                                                               \
        using Input = InputType;                                                    \
        using Output = OutputTime;                                                  \
    };                                                                              \
    }                                                                               \
    }                                                                               \
    }

namespace BipedalLocomotion
{
namespace System
{
namespace internal
{

template <typename T> struct traits;

/// @note the following is from the Eigen library
/// here we say once and for all that traits<const T> == traits<T>
///
/// When constness must affect traits, it has to be constness on
/// template parameters on which T itself depends.
/// For example, traits<Map<const T> > != traits<Map<T> >, but
///              traits<const Map<T> > == traits<Map<T> >
template <typename T> struct traits<const T> : traits<T>
{
};

} // namespace internal
} // namespace System
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_IMPL_TRAITS_H
