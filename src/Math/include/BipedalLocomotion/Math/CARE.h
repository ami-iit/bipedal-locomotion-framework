/**
 * @file CARE.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_MATH_CARE_H
#define BIPEDAL_LOCOMOTION_MATH_CARE_H

#include <Eigen/Dense>

#include <memory>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>

namespace BipedalLocomotion
{
namespace Math
{

/**
 * Computes the unique stabilizing solution S to the continuous-time algebraic equation
 * @f[
 *   S A + A' S - S B R^{-1} B' S + Q = 0
 * @f]
 * @note This class implements the algorithm presented in the paper: Solving the algebraic
 * Riccati equation with the matrix sign function
 * https://www.sciencedirect.com/science/article/pii/0024379587902229
 */
class CARE
{
    /**
     * Private implementation
     */
    struct Impl;
    std::unique_ptr<Impl> m_pimpl;

public:

    /**
     * Initialize the continuous algebraic riccati equation solver.
     * @param handler pointer to the parameter handler.
     * @note The following parameters are optional:
     * |  Parameter Name  |    Type   |                                  Description                                 | Mandatory |
     * |:----------------:|:---------:|:----------------------------------------------------------------------------:|:---------:|
     * |    `tolerance`   |  `double` |               Tolerance of the solution (default value `1e-9`)               |     No    |
     * |   `is_verbose`   | `boolean` | If `true` the algorithm will print some information (default value `false`)  |     No    |
     * | `max_iterations` |   `int`   |   Max number of the interation used by the algorithm (default value `100`)   |     No    |
     * @return true in case of success/false otherwise.
     */
    bool initialize(std::weak_ptr<ParametersHandler::IParametersHandler> handler);

    /**
     * Initialize the matrices.
     * @param A n x n square matrix
     * @param B n x m matrix
     * @param Q n x n symmetric-square matrix
     * @param R m x m square positive definite matrix
     * @return True in case of success and false otherwise.
     */
    bool setMatrices(Eigen::Ref<const Eigen::MatrixXd> A,
                     Eigen::Ref<const Eigen::MatrixXd> B,
                     Eigen::Ref<const Eigen::MatrixXd> Q,
                     Eigen::Ref<const Eigen::MatrixXd> R);

    /**
     * Run the algorithm to compute the unique stabilizing solution of the continuous algebriac
     * Riccati equation.
     * @return True in case of success and false otherwise.
     */
    bool solve();

    /**
     * Get the solution.
     * @return the solution
     */
    Eigen::Ref<const Eigen::MatrixXd> getSolution() const;

    /**
     * Constructor.
     */
    CARE();

    /**
     * Destructor.
     */
    ~CARE();
};

} // namespace Math
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_MATH_CARE_H
