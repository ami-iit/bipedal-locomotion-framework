/**
 * @file ConvexHullHelper.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

extern "C"
{
#include <libqhull_r/libqhull_r.h>
}

#include <BipedalLocomotion/Planners/ConvexHullHelper.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion::Planners;

// Implementation
struct ConvexHullHelper::Impl
{
    Eigen::VectorXd b;
    Eigen::MatrixXd A;
};

ConvexHullHelper::ConvexHullHelper()
{
    m_pimpl = std::make_unique<Impl>();

    m_pimpl->A.resize(0, 0);
    m_pimpl->b.resize(0);
}

ConvexHullHelper::~ConvexHullHelper() = default;

bool ConvexHullHelper::buildConvexHull(Eigen::Ref<const Eigen::MatrixXd> points)
{
    constexpr auto logPrefix = "[ConvexHullHelper::buildConvexHull]";

    const std::size_t numberOfPoints = points.cols();
    const std::size_t numberOfCoordinates = points.rows();

    // required since qh_new_qhull expects a non const pointer to a double
    Eigen::Matrix<coordT, Eigen::Dynamic, Eigen::Dynamic> pointsCoordinates = points;

    facetT* facet = nullptr;
    vertexT* vertex = nullptr;

    // Qhull's data structure.
    qhT qh_qh;
    qhT* qh = &qh_qh;

    // initialize qhull
    qh_zero(qh, nullptr);

    // set the input
    char flags[250];
    sprintf(flags, "qhull ");
    int exitCode = qh_new_qhull(qh,
                                numberOfCoordinates,
                                numberOfPoints,
                                pointsCoordinates.data(),
                                false,
                                flags,
                                nullptr,
                                nullptr);

    if (exitCode != 0)
    {
        log()->error("{} Unable to build the convex hull.", logPrefix);
        return false;
    }

    // Traverse the facets to count non-upper-Delaunay facets
    int numNonUpperDelaunay = 0;
    FORALLfacets
    {
        if (!facet->upperdelaunay)
        {
            numNonUpperDelaunay++;
        }
    }

    // resize the matrix A and the vector b
    m_pimpl->A.resize(numNonUpperDelaunay, numberOfCoordinates);
    m_pimpl->b.resize(numNonUpperDelaunay);

    int i = 0;
    FORALLfacets
    {
        // hyperplane contains d normal coefficients and an offset. The length of the normal is one.
        // The hyperplane defines a halfspace. If V is a normal, b is an offset, and x is a point
        // inside the convex hull, then V x + b < 0.

        // check if the facet is not an upper delaunay
        if (!facet->upperdelaunay)
        {
            // get the normal
            m_pimpl->A.row(i)
                = Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, 1>>(facet->normal,
                                                                             numberOfCoordinates,
                                                                             1);

            // now get the offset
            m_pimpl->b(i) = -facet->offset;

            i++;
        }
    }

    // check if the number of non upper delaunay facets is equal to the number of rows of A
    assert(i == numNonUpperDelaunay);

#ifdef qh_NOmem
    qh_freeqhull(qh, qh_ALL);
#else
    int curlong, totlong; /* memory remaining after qh_memfreeshort, used if !qh_NOmem  */
    qh_freeqhull(qh, !qh_ALL); /* free long memory  */
    qh_memfreeshort(qh, &curlong, &totlong); /* free short memory and memory allocator */
    if (curlong || totlong)
    {

        log()->warn("{} qhull internal warning. Did not free {} bytes of long memory ({} pieces).",
                    logPrefix,
                    totlong,
                    curlong);
    }
#endif

    return true;
}

Eigen::Ref<const Eigen::MatrixXd> ConvexHullHelper::getA() const
{
    return m_pimpl->A;
}

Eigen::Ref<const Eigen::VectorXd> ConvexHullHelper::getB() const
{
    return m_pimpl->b;
}

bool ConvexHullHelper::doesPointBelongToConvexHull(Eigen::Ref<const Eigen::VectorXd> point) const
{
    if (point.size() != m_pimpl->A.cols())
    {
        log()->error("[ConvexHullHelper::doesPointBelongToConvexHull] Unexpected size of the "
                     "point.");

        return false;
    }

    const Eigen::VectorXd tmp = m_pimpl->A * point;

    // check if all the elements of tmp are less or equal than the elements of b
    return (tmp.array() <= m_pimpl->b.array()).all();
}
