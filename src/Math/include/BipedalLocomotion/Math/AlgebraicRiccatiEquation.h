/**
 * @file AlgebraicRiccatiEquation.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_MATH_ALGEBRAIC_RICCATI_EQUATION_H
#define BIPEDAL_LOCOMOTION_MATH_ALGEBRAIC_RICCATI_EQUATION_H

#include <utility>

#include <Eigen/Dense>

namespace BipedalLocomotion
{
namespace Math
{
/*
 * Computes the unique stabilizing solution S to the continuous-time algebraic equation
 * @f[
 *   S A + A' S - S B R^{-1} B' S + Q = 0
 * @f]
 *
 * @param A n x n square matrix
 * @param B n x m matrix
 * @param Q n x n symmetric-square matrix
 * @param R m x m square positive definite matrix
 * @param tolerance max admissible error
 * @param maxIterations max number of iterations
 * @param verbosity level
 * @return a pair. If the Function is able to find a solution the first term is true and the second
 * one contains the solution. Otherwise the first element of the pair is false.
 * @note This function implements the algorithm presented in the paper: Solving the algebraic
 * Riccati equation with the matrix sign function
 * https://www.sciencedirect.com/science/article/pii/0024379587902229
 */
std::pair<bool, Eigen::MatrixXd>
ContinuousAlgebraicRiccatiEquation(Eigen::Ref<const Eigen::MatrixXd> A,
                                   Eigen::Ref<const Eigen::MatrixXd> B,
                                   Eigen::Ref<const Eigen::MatrixXd> Q,
                                   Eigen::Ref<const Eigen::MatrixXd> R,
                                   double tolerance = 1e-9,
                                   std::size_t maxIterations = 100,
                                   bool verbose = false);

} // namespace Math
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_MATH_ALGEBRAIC_RICCATI_EQUATION_H
