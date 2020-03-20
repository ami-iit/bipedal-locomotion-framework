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
#include <memory>
#include <vector>

#include <BipedalLocomotionControllers/GenericContainer.h>

using namespace BipedalLocomotionControllers;

TEST_CASE("Generic Container")
{
    SECTION("Copy")
    {
        iDynTree::VectorDynSize vector(5);
        iDynTree::getRandomVector(vector);
        GenericContainer container(iDynTree::make_span(vector));

        std::vector<double> copiedIn;
        copiedIn.resize(5);
        GenericContainer containerToBeCopied(iDynTree::make_span(copiedIn));

        containerToBeCopied = container;

        for (long i = 0; i < container.size(); ++i)
        {
            REQUIRE(vector[i] == copiedIn[i]);
        }
    }

    SECTION("Impossible to resize")
    {
        iDynTree::VectorDynSize vector(5);
        GenericContainer container = makeGenericContainer(vector);
        REQUIRE_FALSE(container.resizeContainer(2));

        iDynTree::VectorFixSize<3> fixedVector;
        GenericContainer container2 = makeGenericContainer(fixedVector, GenericContainerMode::Resizable);
        REQUIRE_FALSE(container.resizeContainer(2));
    }

    SECTION("Resize")
    {
        iDynTree::VectorDynSize vector;

        GenericContainer container = makeGenericContainer(vector, GenericContainerMode::Resizable);
        REQUIRE(container.resizeContainer(5));
        REQUIRE(vector.size() == 5);

    }

    SECTION("Resize and copy")
    {
        iDynTree::VectorDynSize vector(5);
        iDynTree::getRandomVector(vector);
        GenericContainer container(iDynTree::make_span(vector));

        std::vector<double> copiedIn;

        GenericContainer containerToBeCopied = makeGenericContainer(copiedIn, GenericContainerMode::Resizable);

        containerToBeCopied = container;

        for (long i = 0; i < container.size(); ++i)
        {
            REQUIRE(vector[i] == copiedIn[i]);
        }
    }

}
