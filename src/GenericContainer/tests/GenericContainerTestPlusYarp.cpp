/**
 * @file GenericContainerTest.cpp
 * @authors Stefano Dafarra
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

// Catch2
#include <catch2/catch_test_macros.hpp>
#include <iDynTree/VectorDynSize.h>
#include <iDynTree/VectorFixSize.h>
#include <iDynTree/TestUtils.h>

#include <yarp/sig/Vector.h>

#include <memory>

#include <BipedalLocomotion/GenericContainer/Vector.h>

using namespace BipedalLocomotion;

TEST_CASE("GenericContainer::Vector + Yarp")
{
    SECTION("Constructible")
    {
        REQUIRE(GenericContainer::is_vector_constructible<yarp::sig::Vector>::value);
        REQUIRE(GenericContainer::is_vector_constructible<yarp::sig::VectorOf<int>>::value);
        REQUIRE(GenericContainer::is_vector_constructible<yarp::sig::VectorOf<char>>::value);
    }

    SECTION("Copy")
    {
        iDynTree::VectorDynSize vector(5);
        iDynTree::getRandomVector(vector);
        GenericContainer::Vector container(iDynTree::make_span(vector));

        yarp::sig::Vector copiedIn;
        copiedIn.resize(5);
        GenericContainer::Vector containerToBeCopied = GenericContainer::make_vector(copiedIn);

        containerToBeCopied = container;

        for (long i = 0; i < container.size(); ++i)
        {
            REQUIRE(vector[i] == copiedIn[i]);
        }
    }

    SECTION("Resize")
    {
        yarp::sig::Vector vector;

        GenericContainer::Vector container = make_vector(vector, GenericContainer::VectorResizeMode::Resizable);
        REQUIRE(container.resizeVector(5));
        REQUIRE(vector.size() == 5);

    }

    SECTION("Resize and copy")
    {
        iDynTree::VectorDynSize vector(5);
        iDynTree::getRandomVector(vector);
        GenericContainer::Vector container(iDynTree::make_span(vector));

        yarp::sig::Vector copiedIn;

        GenericContainer::Vector containerToBeCopied = make_vector(copiedIn, GenericContainer::VectorResizeMode::Resizable);

        containerToBeCopied = container;

        for (long i = 0; i < container.size(); ++i)
        {
            REQUIRE(vector[i] == copiedIn[i]);
        }
    }

}
