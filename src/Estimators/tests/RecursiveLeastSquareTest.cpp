/**
 * @file RecursiveLeastSquareTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <cmath>
#include <functional>
#include <random>

// Catch2
#include <catch2/catch.hpp>

// YARP
#include <yarp/os/Bottle.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Searchable.h>
#include <yarp/os/Value.h>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/MatrixDynSize.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>

#include <BipedalLocomotion/Estimators/RecursiveLeastSquare.h>

#include <ConfigFolderPath.h>

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
    iDynTree::Vector2 params;
    double x;

    /** pseudo random number generator */
    std::mt19937 gen;

    /** The noise is modelled as Gaussian vector with zero mean and diagonal covariance matrix */
    std::normal_distribution<> noise1{0, 0.5};
    std::normal_distribution<> noise2{0, 0.5};

public:
    Model(const iDynTree::Vector2& params)
        : params(params)
        , gen{42}
    {
    }

    iDynTree::MatrixDynSize regressor()
    {
        iDynTree::MatrixDynSize regressor(2, 2);
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

    iDynTree::VectorDynSize getOutput()
    {
        iDynTree::VectorDynSize output(2);
        // compute the output assuming zero error
        iDynTree::toEigen(output) = iDynTree::toEigen(regressor()) * iDynTree::toEigen(params);

        // add the noise to the output
        output(0) += noise1(gen);
        output(1) += noise2(gen);

        return output;
    };
};

TEST_CASE("Recursive Least Square")
{
    // instantiate model
    iDynTree::Vector2 parameters;
    parameters(0) = 43.2;
    parameters(1) = 12.2;
    Model model(parameters);

    // Load the RLS parameters from configuration file
    yarp::os::ResourceFinder& rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
    rf.setDefaultConfigFile("config.ini");

    std::vector<std::string> arguments = {" ", "--from ", getConfigPath()};

    std::vector<char*> argv;
    for (const auto& arg : arguments)
        argv.push_back((char*)arg.data());
    argv.push_back(nullptr);

    rf.configure(argv.size() - 1, argv.data());

    auto parameterHandler = std::make_shared<YarpImplementation>();

    REQUIRE_FALSE(rf.isNull());
    parameterHandler->set(rf);

    // Instantiate the estimator
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
    double admissibleError = 0.1 / 100.0;

    // compute the relative error of the parameters
    double relativeError1 = std::abs((estimator.parametersExpectedValue()(0) - parameters(0)) / parameters(0));
    double relativeError2 = std::abs((estimator.parametersExpectedValue()(1) - parameters(1)) / parameters(1));

    REQUIRE(relativeError1 < admissibleError);
    REQUIRE(relativeError2 < admissibleError);
}
