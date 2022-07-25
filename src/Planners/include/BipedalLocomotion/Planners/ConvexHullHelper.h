/**
 * @file ConvexHullHelper.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_PLANNERS_CONVEX_HULL_HELPER_H
#define BIPEDAL_LOCOMOTION_PLANNERS_CONVEX_HULL_HELPER_H

#include <Eigen/Dense>

#include <vector>
#include <memory>

namespace BipedalLocomotion
{
namespace Planners
{
/**
 * ConvexHullHelper is an helper class that simplifies the building of a convex hull given a set of
 * points that belongs to \f$\mathbb{R} ^n\f$. The helper computes the vertex enumeration of the
 * convex hull. Then it converts the V-representation of the convex polyhedron in the
 * H-representation, i.e. it performs the convex-hull computation. The class can be used to retrieve
 * the matrices and vectors that describes the Half space representation of the form:
 * \f[
 * P = \left \{ x= \begin{bmatrix} x_1 & ... & x_d \end{bmatrix} ^ \top : b - A x \ge 0 \right \}.
 * \f]
 * Where \f$b\f$ is a m-vector and A is a m x n real matrix.
 * @warning The ConvexHullHelper is based on the qhull library https://github.com/qhull/qhull
 */
class ConvexHullHelper
{
    struct Impl;
    /** Private implementation */
    std::unique_ptr<Impl> m_pimpl;

public:
    /**
     * Constructor
     */
    ConvexHullHelper();

    /**
     * Destructor
     */
    ~ConvexHullHelper();

    /**
     * Given a set of points in \f$ \mathbb{R} ^ n\f$ the function build the convex hull.
     * @warning the points must belong to the same vectorial space.
     * @param points a matrix of the points that describes the convex hull. Each point is stored as
     * column of the matrix.
     * @return true/false in case of success/failure.
     */
    bool buildConvexHull(Eigen::Ref<const Eigen::MatrixXd> points);

    /**
     * Return the \f$A\f$ constraint matrix, such that \f$ Ax \le b\f$ iff the point \f$x\f$ is in
     * the convex hull.
     * @warning Please call buildConvexHull before asking for \f$A\f$. If the convex hull has not be
     * built yet a reference to a 0-size matrix is returned.
     * @return the constraint matrix.
     */
    Eigen::Ref<const Eigen::MatrixXd> getA() const;

    /**
     * Return the \f$b\f$ constraint vector, such that \f$ Ax \le b\f$ iff the point \f$x\f$ is in
     * the convex hull.
     * @warning Please call buildConvexHull before asking for \f$b\f$. If the convex hull has not be
     * built yet a reference to a 0-size vector is returned.
     * @return the constraint vector.
     */
    Eigen::Ref<const Eigen::VectorXd> getB() const;

    /**
     * Check if a point belong to the convex hull (The frontier of the set is also included).
     * @return true if the point belongs to the convex hull false otherwise.
     */
    bool doesPointBelongToConvexHull(Eigen::Ref<const Eigen::VectorXd> point) const;
};
} // namespace Planners
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_PLANNERS_CONVEX_HULL_HELPER_H
