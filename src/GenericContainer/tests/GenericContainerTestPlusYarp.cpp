/**
 * @file GenericContainerTest.cpp
 * @authors Stefano Dafarra
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

// Catch2
#include <catch2/catch.hpp>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/TestUtils.h>

#include <yarp/sig/Vector.h>

#include <memory>

#include <BipedalLocomotionControllers/GenericContainer/Vector.h>

using namespace BipedalLocomotionControllers;

TEST_CASE("GenericContainer::Vector + Yarp")
{
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

    SECTION("Impossible to resize")
    {
        iDynTree::VectorDynSize vector(5);
        GenericContainer::Vector container = GenericContainer::make_vector(vector);
        REQUIRE_FALSE(container.resizeContainer(2));

        iDynTree::VectorFixSize<3> fixedVector;
        GenericContainer::Vector container2 = make_vector(fixedVector, GenericContainer::VectorResizeMode::Resizable);
        REQUIRE_FALSE(container.resizeContainer(2));
    }

    SECTION("Resize")
    {
        yarp::sig::Vector vector;

        GenericContainer::Vector container = make_vector(vector, GenericContainer::VectorResizeMode::Resizable);
        REQUIRE(container.resizeContainer(5));
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
