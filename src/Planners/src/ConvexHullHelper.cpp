/**
 * @file ConvexHullHelper.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <libqhullcpp/PointCoordinates.h>
#include <libqhullcpp/Qhull.h>
#include <libqhullcpp/QhullFacetList.h>

#include <BipedalLocomotion/Planners/ConvexHullHelper.h>

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

    m_pimpl->A.resize(0,0);
    m_pimpl->b.resize(0);
}

ConvexHullHelper::~ConvexHullHelper()
{
}

bool ConvexHullHelper::buildConvexHull(Eigen::Ref<const Eigen::MatrixXd> points)
{
    const std::size_t numberOfPoints = points.cols();
    const std::size_t numberOfCoordinates = points.rows();

    // the qhull object can be called only once
    orgQhull::Qhull qhull;

    // it seems that the pointCoordinates element cannot be cleaned so a new point coordinates has to be instantiate
    orgQhull::PointCoordinates pointCoordinates;
    pointCoordinates.setDimension(numberOfCoordinates);

    std::vector<double> allCoordinates(numberOfPoints * numberOfCoordinates);
    Eigen::Map<Eigen::MatrixXd>(allCoordinates.data(), numberOfCoordinates, numberOfPoints) = points;

    // map = points;
    pointCoordinates.append(allCoordinates);

    // find the convex hull
    qhull.runQhull(pointCoordinates.comment().c_str(),
                   pointCoordinates.dimension(),
                   pointCoordinates.count(),
                   &*pointCoordinates.coordinates(),
                   "Qt");

    auto facetList = qhull.facetList();
    const std::size_t numberOfFacet = facetList.count();

    // resize matrix and vectors
    m_pimpl->A.resize(numberOfFacet, numberOfCoordinates);
    m_pimpl->b.resize(numberOfFacet);

    // fill the A matrix and the b vector
    std::size_t row = 0;
    for (const auto& facet : facetList)
    {
        // hyperplane contains d normal coefficients and an offset. The length of the normal is one.
        // The hyperplane defines a halfspace. If V is a normal, b is an offset, and x is a point
        // inside the convex hull, then V x + b < 0.
        const auto hyperplane = facet.hyperplane();
        if (hyperplane.isValid())
        {
            const auto coord = hyperplane.coordinates();
            for (std::size_t column = 0; column < numberOfCoordinates; column++)
            {
                m_pimpl->A(row, column) = coord[column];
            }

            m_pimpl->b[row] = -hyperplane.offset();
            row++;
        }
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
        std::cerr << "[ConvexHullHelper::doesPointBelongToConvexHull] Unexpected size of the point."
                  << std::endl;
        return false;
    }

    Eigen::VectorXd tmp = m_pimpl->A * point;

    for(std::size_t i = 0; i < m_pimpl->b.size(); i++)
        if(tmp[i] > m_pimpl->b[i])
            return false;

    return true;
}
