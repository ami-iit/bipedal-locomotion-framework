/**
 * @file YarpUtilitiesTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

// std
#include <random>

// Catch2
#include <catch2/catch_test_macros.hpp>

// YARP
#include <yarp/os/Property.h>
#include <yarp/os/Value.h>

#include <BipedalLocomotion/YarpUtilities/Helper.h>

using namespace BipedalLocomotion::YarpUtilities;

template <typename T, typename U> void checkEqualVector(const T& vector1, const U& vector2)
{
    // check the size of the two vectors
    REQUIRE(vector1.size() == vector2.size());

    // iterate over all the elements
    for (unsigned int i = 0; i < vector1.size(); i++)
        REQUIRE(vector1[i] == vector2[i]);
}

TEST_CASE("Get int from searchable", "[int]")
{
    int element;
    yarp::os::Property property;
    std::string key = "answer_to_the_ultimate_question_of_life";
    int value = 42;

    property.put(key, value);

    // test
    REQUIRE(getElementFromSearchable(property, key, element));
    REQUIRE(element == value);
}

TEST_CASE("Get bool from searchable", "[bool]")
{
    bool element;
    yarp::os::Property property;
    std::string key = "toss_coin_result";
    bool value = false;

    property.put(key, value);

    // test
    REQUIRE(getElementFromSearchable(property, key, element));
    REQUIRE(element == value);
}

TEST_CASE("Get bool (int) from searchable", "[bool (int)]")
{
    bool element;
    yarp::os::Property property;
    std::string key = "toss_coin_result";
    int value = 1;

    property.put(key, value);

    // test
    REQUIRE(getElementFromSearchable(property, key, element));
    REQUIRE((value == 1) == element);
}


TEST_CASE("Get double from searchable", "[double]")
{
    double element;
    yarp::os::Property property;
    std::string key = "pi";
    double value = 3.14;

    property.put(key, value);

    // test
    REQUIRE(getElementFromSearchable(property, key, element));
    REQUIRE(element == value);
}

TEST_CASE("Get string from searchable", "[std::string]")
{
    std::string element;
    yarp::os::Property property;
    std::string key = "John";
    std::string value = "Smith";

    property.put(key, value);

    // test
    REQUIRE(getElementFromSearchable(property, key, element));
    REQUIRE(element == value);
}

TEST_CASE("Get int vector from searchable", "[std::vector<int>]")
{
    std::vector<int> element;

    yarp::os::Property property;
    std::string key = "Fibonacci number";
    std::vector<int> value{1, 1, 2, 3, 5, 8, 13, 21};
    yarp::os::Value yarpValue;
    auto list = yarpValue.asList();

    for (const auto& v : value)
        list->addInt32(v);

    property.put(key, yarpValue);

    // test
    REQUIRE(getVectorFromSearchable(property, key, element));
    REQUIRE(element == value);
}

TEST_CASE("Get string vector from searchable", "[std::vector<std::string>]")
{
    std::vector<std::string> element;

    yarp::os::Property property;
    std::string key = "Donald's nephews";
    std::vector<std::string> value{"Huey", "Dewey", "Louie"};
    yarp::os::Value yarpValue;
    auto list = yarpValue.asList();

    for (const auto& v : value)
        list->addString(v);

    property.put(key, yarpValue);

    // test
    REQUIRE(getVectorFromSearchable(property, key, element));
    REQUIRE(element == value);
}

TEST_CASE("Get bool vector from searchable", "[std::vector<bool>]")
{
    std::vector<bool> element;

    yarp::os::Property property;
    std::string key = "toss_coin_results";
    std::vector<bool> value;
    yarp::os::Value yarpValue;
    auto list = yarpValue.asList();

    // instantiate the random number generator and a Bernoulli distribution
    std::default_random_engine generator;
    generator.seed(42);
    std::bernoulli_distribution distribution(0.5);

    for (unsigned int i = 0; i < 10000; i++)
    {
        bool result = distribution(generator);
        value.push_back(result);

        // please check https://github.com/robotology/yarp/issues/2584#issuecomment-847778679
        if (result)
            list->add(yarp::os::Value::makeValue("true"));
        else
            list->add(yarp::os::Value::makeValue("false"));
    }

    property.put(key, yarpValue);

    // test
    REQUIRE(getVectorFromSearchable(property, key, element));
    REQUIRE(element == value);
}

TEST_CASE("Get iDynTree vector from searchable", "[iDynTree::VectorDynSize]")
{
    iDynTree::VectorDynSize element;

    yarp::os::Property property;
    std::string key = "Geometric series";
    iDynTree::VectorDynSize value;
    yarp::os::Value yarpValue;
    auto list = yarpValue.asList();

    value.resize(6);
    for (unsigned int i = 0; i < 6; i++)
    {
        double seriesElement = 0.5 * std::pow(0.5, i);
        list->addFloat64(seriesElement);
        value[i] = seriesElement;
    }
    property.put(key, yarpValue);

    // test
    REQUIRE(getVectorFromSearchable(property, key, element));
    checkEqualVector(value, element);
}

TEST_CASE("Get YARP vector from searchable", "[yarp::sig::Vector]")
{
    yarp::sig::Vector element;

    yarp::os::Property property;
    std::string key = "Geometric series";
    yarp::sig::Vector value;
    yarp::os::Value yarpValue;
    auto list = yarpValue.asList();

    value.resize(6);
    for (unsigned int i = 0; i < 6; i++)
    {
        double seriesElement = 0.5 * std::pow(0.5, i);
        list->addFloat64(seriesElement);
        value[i] = seriesElement;
    }
    property.put(key, yarpValue);

    // test
    REQUIRE(getVectorFromSearchable(property, key, element));
    checkEqualVector(value, element);
}

TEST_CASE("Merge Vector to yarp::sig::Vector", "[Merge to yarp::sig::Vector]")
{
    yarp::sig::Vector vector;

    // define geometric series
    std::vector<double> geometricSeries;
    for (unsigned int i = 0; i < 6; i++)
    {
        double seriesElement = 0.5 * std::pow(0.5, i);
        geometricSeries.push_back(seriesElement);
    }

    // define harmonic series
    iDynTree::VectorDynSize harmonicSeries(6);
    for (unsigned int i = 1; i < 7; i++)
    {
        double seriesElement = 1.0 / i;
        harmonicSeries[i - 1] = seriesElement;
    }

    mergeSigVector(vector, geometricSeries, harmonicSeries, 42, 3.14);

    checkEqualVector(vector.subVector(0, 5), geometricSeries);
    checkEqualVector(vector.subVector(6, 11), harmonicSeries);
    REQUIRE(vector(12) == 42);
    REQUIRE(vector(13) == 3.14);

}
