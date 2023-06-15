/**
 * @file CasadiConversions.h
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_CASADI_CONVERSIONS_H
#define BIPEDAL_LOCOMOTION_CASADI_CONVERSIONS_H

#include <Eigen/Dense>

#include <casadi/casadi.hpp>

namespace BipedalLocomotion
{
namespace Conversions
{

/**
 * @brief Conversion from a casadi::DM to an Eigen matrix,
 * @param input The casadi::DM matrix.
 * @return A map from the internal data of the casadi matrix.
 */
inline Eigen::Map<Eigen::MatrixXd> toEigen(casadi::DM& input)
{
    return Eigen::Map<Eigen::MatrixXd>(input.ptr(), input.rows(), input.columns());
}

/**
 * @brief Conversion from a casadi::DM to an Eigen matrix,
 * @param input The casadi::DM matrix.
 * @return A map from the internal data of the casadi matrix.
 */
inline Eigen::Map<const Eigen::MatrixXd> toEigen(const casadi::DM& input)
{
    return Eigen::Map<const Eigen::MatrixXd>(input.ptr(), input.rows(), input.columns());
}

} // namespace Conversions
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_CASADI_CONVERSIONS_H
