/**
 * @file LinearizedFrictionCone.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <cmath>

#include <BipedalLocomotion/Math/LinearizedFrictionCone.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion::Math;

bool LinearizedFrictionCone::initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler)
{
    constexpr auto errorPrefix = "[LinearizedFrictionCone::initialize]";

    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} Invalid parameter handler.", errorPrefix);
        return false;
    }

    bool ok = true;
    int numberOfSlices = -1;
    ok = ok && ptr->getParameter("number_of_slices", numberOfSlices);
    ok = ok && numberOfSlices > 0;

    double staticFrictionCoefficient = -1;
    ok = ok && ptr->getParameter("static_friction_coefficient", staticFrictionCoefficient);
    ok = ok && staticFrictionCoefficient > 0;

    if (!ok)
    {
        log()->error("{} Unable to retrieve all the parameters.", errorPrefix);
        return false;
    }

    // split the friction cone into slices
    const double segmentAngle = M_PI / (2 * numberOfSlices);
    const int numberOfEquations = 4 * numberOfSlices;

    m_A.resize(numberOfEquations, 3);
    m_b = Eigen::VectorXd::Zero(numberOfEquations);

    // evaluate friction cone constraint
    std::vector<double> angles;
    std::vector<double> pointsX;
    std::vector<double> pointsY;

    for (int i = 0; i < numberOfEquations; i++)
    {
        angles.push_back(i * segmentAngle);
        pointsX.push_back(std::cos(angles.back()));
        pointsY.push_back(std::sin(angles.back()));
    }

    double firstPointX, firstPointY, secondPointX, secondPointY;
    for (int i = 0; i < numberOfEquations; i++)
    {
        firstPointX = pointsX[i];
        firstPointY = pointsY[i];

        secondPointX = pointsX[(i + 1) % numberOfEquations];
        secondPointY = pointsY[(i + 1) % numberOfEquations];

        const double angularCoefficients
            = (secondPointY - firstPointY) / (secondPointX - firstPointX);
        const double offset = firstPointY - angularCoefficients * firstPointX;

        int inequalityFactor = 1;
        if (angles[i] > M_PI || angles[(i + 1) % numberOfEquations] > M_PI)
            inequalityFactor = -1;

        //  A(i,:) = inequalityFactor.* [-angularCoefficients, 1, (-offsets*staticFrictionCoefficient)];
        m_A.row(i) << -inequalityFactor * angularCoefficients, inequalityFactor,
            -inequalityFactor * offset * staticFrictionCoefficient;
    }

    m_isIntialized = true;

    return true;
}

Eigen::Ref<const Eigen::MatrixXd> LinearizedFrictionCone::getA() const
{
    constexpr auto error = "[LinearizedFrictionCone::getA] Please initialize the class before.";

    if (!m_isIntialized)
    {
        log()->warn(error);
        assert(m_isIntialized && error);
    }

    return m_A;
}

Eigen::Ref<const Eigen::VectorXd> LinearizedFrictionCone::getB() const
{
    constexpr auto error = "[LinearizedFrictionCone::getB] Please initialize the class before.";
    if (!m_isIntialized)
    {
        log()->warn(error);
        assert(m_isIntialized && error);
    }

    return m_b;
}
