/**
 * @file RecursiveLeastSquareTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <cmath>
#include <functional>
#include <random>

#include <Eigen/Dense>

// Catch2
#include <catch2/catch_test_macros.hpp>

#include <BipedalLocomotion/Estimators/RecursiveLeastSquare.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>

using namespace BipedalLocomotion::Estimators;
using namespace BipedalLocomotion::ParametersHandler;

/**
 * The model class implement a simple model used for testing the Recursive Least Square algorithm
 * |y1|   |x      x * x |   |p1|   |noise_1 |
 * |  | = |             | * |  | + |        |
 * |y2|   |sin(x) cos(x)|   |p2|   |noise_2 |
 */
class Model
{
    Eigen::Vector2d params;
    double x;

    /** pseudo random number generator */
    std::mt19937 gen;

    /** The noise is modelled as Gaussian vector with zero mean and diagonal covariance matrix */
    std::normal_distribution<> noise1{0, 0.5};
    std::normal_distribution<> noise2{0, 0.5};

public:
    Model(const Eigen::Ref<const Eigen::Vector2d>& params)
        : params(params)
        , gen{42}
    {
    }

    Eigen::Matrix2d regressor()
    {
        Eigen::Matrix2d regressor;
        regressor(0, 0) = x;
        regressor(0, 1) = x * x;

        regressor(1, 0) = std::sin(x);
        regressor(1, 1) = std::cos(x);

        return regressor;
    };

    void setX(const double& x)
    {
        this->x = x;
    };

    Eigen::Vector2d getOutput()
    {
        Eigen::Vector2d output;
        // compute the output assuming zero error
        output = regressor() * params;

        // add the noise to the output
        output(0) += noise1(gen);
        output(1) += noise2(gen);

        return output;
    };
};

TEST_CASE("Recursive Least Square")
{
    // instantiate model
    Eigen::Vector2d parameters;
    parameters(0) = 43.2;
    parameters(1) = 12.2;
    Model model(parameters);

    std::shared_ptr<IParametersHandler> parameterHandler = std::make_shared<StdImplementation>();
    parameterHandler->setParameter("lambda", 1.0);
    parameterHandler->setParameter("measurement_covariance", std::vector<double>{0.5, 0.5});
    parameterHandler->setParameter("state", std::vector<double>{0.0, 0.0});
    parameterHandler->setParameter("state_covariance", std::vector<double>{10.0, 10.0});

    // // Instantiate the estimator
    RecursiveLeastSquare estimator;
    REQUIRE(estimator.initialize(parameterHandler));

    auto regressor = std::bind(&Model::regressor, &model);
    estimator.setRegressorFunction(regressor);

    // estimate the parameters
    for (int i = 0; i < 10000; i++)
    {
        model.setX(std::cos(i / 10.0));
        estimator.setMeasurements(model.getOutput());

        REQUIRE(estimator.advance());
    }

    // the admissible error is 0.1%
    constexpr double admissibleError = 0.1 / 100.0;

    // compute the relative error of the parameters
    const auto& state = estimator.getOutput();
    double relativeError1 = std::abs((state.expectedValue(0) - parameters(0)) / parameters(0));
    double relativeError2 = std::abs((state.expectedValue(1) - parameters(1)) / parameters(1));

    REQUIRE(relativeError1 < admissibleError);
    REQUIRE(relativeError2 < admissibleError);
}
