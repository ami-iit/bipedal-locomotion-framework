/**
 * @file ConvexHullHelper.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <libqhullcpp/PointCoordinates.h>
#include <libqhullcpp/Qhull.h>
#include <libqhullcpp/QhullFacetList.h>

#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/EigenHelpers.h>

#include <BipedalLocomotion/Planners/ConvexHullHelper.h>

using namespace BipedalLocomotion::Planners;

// Implementation
struct ConvexHullHelper::Impl
{
    iDynTree::VectorDynSize b;
    iDynTree::MatrixDynSize A;
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

bool ConvexHullHelper::buildConvexHull(const std::vector<iDynTree::VectorDynSize>& points)
{
    const std::size_t size = points.front().size();

    // check if the size of the vectors are all the same
    auto point = points.cbegin();
    // it is useless to check the size of the first vector
    std::advance(point, 1);
    for (; point < points.end(); std::advance(point, 1))
    {
        if (size != point->size())
        {
            std::cerr << "[ConvexHullHelper::buildConvexHull] All the vectors should belong to the "
                         "same vectorial space."
                      << std::endl;
            return false;
        }
    }

    // the qhull object can be called only once
    orgQhull::Qhull qhull;

    // it seems that the pointCoordinates element cannot be cleaned so a new point coordinates has to be instantiate
    orgQhull::PointCoordinates pointCooridinates;
    pointCooridinates.setDimension(size);

    std::vector<double> allPoints;
    for (const auto& point : points)
    {
        for (const auto& coordinate : point)
            allPoints.push_back(coordinate);
    }
    pointCooridinates.append(allPoints);

    // find the convex hull
    qhull.runQhull(pointCooridinates.comment().c_str(),
                   pointCooridinates.dimension(),
                   pointCooridinates.count(),
                   &*pointCooridinates.coordinates(),
                   "Qt");

    auto facetList = qhull.facetList();
    const std::size_t numberOfFacet = facetList.count();

    // resize matrix and vectors
    m_pimpl->A.resize(numberOfFacet, size);
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
            for (std::size_t column = 0; column < size; column++)
            {
                m_pimpl->A(row, column) = coord[column];
            }

            m_pimpl->b[row] = -hyperplane.offset();
            row++;
        }
    }

    return true;
}

const iDynTree::MatrixDynSize& ConvexHullHelper::getA() const
{
    return m_pimpl->A;
}

const iDynTree::VectorDynSize& ConvexHullHelper::getB() const
{
    return m_pimpl->b;
}

bool ConvexHullHelper::doesPointBelongToCovexHull(const iDynTree::VectorDynSize& point) const
{
    if (point.size() != m_pimpl->A.cols())
    {
        std::cerr << "[ConvexHullHelper::doesPointBelongToCovexHull] Unexpected size of the point."
                  << std::endl;
        return false;
    }

    Eigen::VectorXd tmp = iDynTree::toEigen(m_pimpl->A) * iDynTree::toEigen(point);

    for(std::size_t i = 0; i < m_pimpl->b.size(); i++)
        if(tmp[i] > m_pimpl->b[i])
            return false;

    return true;
}
