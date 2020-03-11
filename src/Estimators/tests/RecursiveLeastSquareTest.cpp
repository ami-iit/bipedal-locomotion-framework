/**
 * @file RecursiveLeastSquareTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <cmath>
#include <random>
#include <functional>

// Catch2
#include <catch2/catch.hpp>

// YARP
#include <yarp/os/Property.h>
#include <yarp/os/Searchable.h>
#include <yarp/os/Value.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Bottle.h>

#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/EigenHelpers.h>

#include <BipedalLocomotionControllers/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotionControllers/ParametersHandler/YarpImplementation.h>

#include <BipedalLocomotionControllers/Estimators/RecursiveLeastSquare.h>

#include <ConfigFolderPath.h>

using namespace BipedalLocomotionControllers::Estimators;
using namespace BipedalLocomotionControllers::ParametersHandler;

class Model
{
    iDynTree::Vector2 params;
    double x;

    std::random_device rd{};
    std::mt19937 gen;

    std::normal_distribution<> noise1{0,0.5};
    std::normal_distribution<> noise2{0,0.5};

public:
    Model(const iDynTree::Vector2& params)
        : gen{rd()}
        , params(params)
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
        iDynTree::toEigen(output) = iDynTree::toEigen(regressor()) * iDynTree::toEigen(params);
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

    yarp::os::ResourceFinder& rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
    rf.setDefaultConfigFile("config.ini");

    std::vector<std::string> arguments = {" ", "--from ", getConfigPath()};

    std::vector<char*> argv;
    for (const auto& arg : arguments)
        argv.push_back((char*)arg.data());
    argv.push_back(nullptr);

    rf.configure(argv.size() - 1, argv.data());

    YarpImplementation::unique_ptr parameterHandler = YarpImplementation::make_unique();

    REQUIRE_FALSE(rf.isNull());
    parameterHandler->set(rf);
    REQUIRE_FALSE(parameterHandler->isEmpty());

    auto estimatorParameters = parameterHandler->getGroup("RLS");
    RecursiveLeastSquare estimator;
    REQUIRE(estimator.initialize(estimatorParameters));


    auto regressor = std::bind(&Model::regressor, &model);
    estimator.setRegressorFunction(regressor);

    for(int i = 0; i < 10000; i++)
    {
        model.setX(std::cos(i / 10.0));
        estimator.setMeasurements(model.getOutput());

        REQUIRE(estimator.advance());
    }

    // the admissible error is 0.1%
    double admissibleError = 0.1/100.0;
    double relativeError1 = abs((estimator.parametersExpectedValue()(0) - parameters(0)) / parameters(0));

    double relativeError2 = abs((estimator.parametersExpectedValue()(1) - parameters(1)) / parameters(1));

    REQUIRE(relativeError1 < admissibleError);
    REQUIRE(relativeError2 < admissibleError);
}
