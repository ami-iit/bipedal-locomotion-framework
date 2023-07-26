/**
 * @file ConvexHullHelper.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <libqhull/libqhull.h>

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

ConvexHullHelper::~ConvexHullHelper()
{
}

bool ConvexHullHelper::buildConvexHull(Eigen::Ref<const Eigen::MatrixXd> points)
{

    constexpr auto logPrefix = "[ConvexHullHelper::buildConvexHull]";

    const std::size_t numberOfPoints = points.cols();
    const std::size_t numberOfCoordinates = points.rows();

    Eigen::Matrix<coordT, Eigen::Dynamic, Eigen::Dynamic> pointsCoordinates(numberOfCoordinates,
                                                                            numberOfPoints);

    // copy the points
    pointsCoordinates = points;
    facetT* facet = nullptr;
    vertexT* vertex = nullptr;
    vertexT** vertexp = nullptr;

    int exitCode = qh_new_qhull(numberOfCoordinates,
                                numberOfPoints,
                                pointsCoordinates.data(),
                                false,
                                nullptr,
                                nullptr,
                                nullptr);

    if (exitCode != 0)
    {
        log()->error("{} Unable to build the convex hull.", logPrefix);
        return false;
    }

    int numNonUpperDelaunay = 0;
    // Traverse the facets to count non-upper-Delaunay facets

    for (facet = qh facet_list; facet; facet = facet->next)
    {
        if (!facet->upperdelaunay)
        {
            numNonUpperDelaunay++;
        }
    }

    m_pimpl->A.resize(numNonUpperDelaunay, numberOfCoordinates);
    m_pimpl->b.resize(numNonUpperDelaunay);

    int i = 0;
    for (facet = qh facet_list; facet; facet = facet->next)
    {
        if (!facet->upperdelaunay)
        { // Skip upper envelope facets
            for (int j = 0; j < numberOfCoordinates; j++)
            {
                m_pimpl->A(i, j) = facet->normal[j];
            }

            m_pimpl->b(i) = 0;
            FOREACHvertex_(facet->vertices)
            {
                m_pimpl->b(i) += vertex->point[i] * facet->normal[i];
            }

            i++;
        }
    }

    qh_freeqhull(!qh_ALL);
    int curlong, totlong;

    qh_memfreeshort(&curlong, &totlong);
    if (curlong || totlong)
    {
        log()->warn("{} Qhull internal warning (qh_memfreeshort): {} bytes lost.",
                    logPrefix,
                    totlong);
    }

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

    // check if all the elements of tmo are less or equal than the elements of b
    return (tmp.array() <= m_pimpl->b.array()).all();
}
