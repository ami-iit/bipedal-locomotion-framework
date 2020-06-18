/**
 * @file ConvexHullHelper.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_PLANNERS_CONVEX_HULL_HELPER_H
#define BIPEDAL_LOCOMOTION_PLANNERS_CONVEX_HULL_HELPER_H

#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/VectorDynSize.h>

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
     * @param points a vector of the points that describes the convex hull.
     * @return true/false in case of success/failure.
     */
    bool buildConvexHull(const std::vector<iDynTree::VectorDynSize>& points);

    /**
     * Return the \f$A\f$ constraint matrix, such that \f$ Ax \le b\f$ iff the point \f$x\f$ is in
     * the convex hull.
     * @return the constraint matrix.
     */
    const iDynTree::MatrixDynSize& getA() const;

    /**
     * Return the \f$b\f$ constraint vector, such that \f$ Ax \le b\f$ iff the point \f$x\f$ is in
     * the convex hull.
     * @return the constraint vector.
     */
    const iDynTree::VectorDynSize& getB() const;

    /**
     * Check if a point belong to the convex hull.
     * @return true if the point belongs to the convex hull false otherwise.
     */
    bool doesPointBelongToCovexHull(const iDynTree::VectorDynSize& point) const;
};
} // namespace Planners
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_PLANNERS_CONVEX_HULL_HELPER_H
