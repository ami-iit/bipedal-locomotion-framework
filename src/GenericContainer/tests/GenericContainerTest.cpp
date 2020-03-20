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

#include <BipedalLocomotionControllers/GenericContainer/Vector.h>

using namespace BipedalLocomotionControllers;

TEST_CASE("GenericContainer::Vector")
{
    SECTION("Copy")
    {
        iDynTree::VectorDynSize vector(5);
        iDynTree::getRandomVector(vector);
        GenericContainer::Vector container(iDynTree::make_span(vector));

        std::vector<double> copiedIn;
        copiedIn.resize(5);
        GenericContainer::Vector containerToBeCopied(iDynTree::make_span(copiedIn));

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
        REQUIRE_FALSE(container.resizeVector(2));

        iDynTree::VectorFixSize<3> fixedVector;
        GenericContainer::Vector container2 = GenericContainer::make_vector(fixedVector, GenericContainer::VectorResizeMode::Resizable);
        REQUIRE_FALSE(container.resizeVector(2));
    }

    SECTION("Resize")
    {
        iDynTree::VectorDynSize vector;

        GenericContainer::Vector container = GenericContainer::make_vector(vector, GenericContainer::VectorResizeMode::Resizable);
        REQUIRE(container.resizeVector(5));
        REQUIRE(vector.size() == 5);

    }

    SECTION("Resize and copy")
    {
        iDynTree::VectorDynSize vector(5);
        iDynTree::getRandomVector(vector);
        GenericContainer::Vector container(iDynTree::make_span(vector));

        std::vector<double> copiedIn;

        GenericContainer::Vector containerToBeCopied = GenericContainer::make_vector(copiedIn,
                                                                                     GenericContainer::VectorResizeMode::Resizable);

        containerToBeCopied = container;

        for (long i = 0; i < container.size(); ++i)
        {
            REQUIRE(vector[i] == copiedIn[i]);
        }
    }

    SECTION("Create const")
    {
        iDynTree::VectorDynSize vector(5);
        iDynTree::getRandomVector(vector);

        const iDynTree::VectorDynSize constVector = vector;
        GenericContainer::Vector container = GenericContainer::make_vector(constVector,
                                                                           GenericContainer::VectorResizeMode::Resizable);
    }

    SECTION("Create from self")
    {
        iDynTree::VectorDynSize vector(5);
        iDynTree::getRandomVector(vector);
        GenericContainer::Vector container = GenericContainer::make_vector(vector,
                                                                           GenericContainer::VectorResizeMode::Resizable);

        GenericContainer::Vector inception = GenericContainer::make_vector(container, GenericContainer::VectorResizeMode::Resizable);
    }

    SECTION("Create from self custom resize")
    {
        iDynTree::VectorDynSize vector(5);
        iDynTree::getRandomVector(vector);
        GenericContainer::Vector container = GenericContainer::make_vector(vector,
                                                                           GenericContainer::VectorResizeMode::Resizable);

        using resize_function = GenericContainer::Vector<double>::resize_function_type;
        using index_type = GenericContainer::Vector<double>::index_type;

        GenericContainer::Vector<double>* inputPtr = &container;
        resize_function resizeLambda =
            [inputPtr](index_type newSize) -> iDynTree::Span<double>
        {
            inputPtr->resizeVector(newSize);
            std::cerr << "I am resizing!" << std::endl;
            return iDynTree::make_span(*inputPtr);
        };

        GenericContainer::Vector inception(iDynTree::make_span(container), resizeLambda);
        REQUIRE(inception.resizeVector(6));
        REQUIRE(vector.size() == 6);
    }


    SECTION("at_call")
    {
        int arr[4] = {1, 2, 3, 4};

        {
            GenericContainer::Vector s = GenericContainer::make_vector(arr);
            REQUIRE(s.at(0) == 1);
        }

        {
            int arr2d[2] = {1, 6};
            GenericContainer::Vector s = GenericContainer::make_vector(arr2d);
            REQUIRE(s.at(0) == 1);
            REQUIRE(s.at(1) == 6);
        }
    }

    SECTION("operator_function_call")
    {
        int arr[4] = {1, 2, 3, 4};

        {
            GenericContainer::Vector s = GenericContainer::make_vector(arr);
            REQUIRE(s(0) == 1);
        }

        {
            int arr2d[2] = {1, 6};
            GenericContainer::Vector s = GenericContainer::make_vector(arr2d);
            REQUIRE(s(0) == 1);
            REQUIRE(s(1) == 6);
        }
    }

    SECTION("iterator_default_init")
    {
        GenericContainer::Vector<int>::iterator it1;
        GenericContainer::Vector<int>::iterator it2;
        REQUIRE(it1 == it2);
    }

    SECTION("const_iterator_default_init")
    {
        GenericContainer::Vector<int>::const_iterator it1;
        GenericContainer::Vector<int>::const_iterator it2;
        REQUIRE(it1 == it2);
    }

    SECTION("iterator_conversions")
    {
        GenericContainer::Vector<int>::iterator badIt;
        GenericContainer::Vector<int>::const_iterator badConstIt;
        REQUIRE(badIt == badConstIt);

        int a[] = {1, 2, 3, 4};
        GenericContainer::Vector s = GenericContainer::make_vector(a);

        auto it = s.begin();
        auto cit = s.cbegin();

        REQUIRE(it == cit);
        REQUIRE(cit == it);

        GenericContainer::Vector<int>::const_iterator cit2 = it;
        REQUIRE(cit2 == cit);

        GenericContainer::Vector<int>::const_iterator cit3 = it + 4;
        REQUIRE(cit3 == s.cend());
    }

    SECTION("iterator_comparisons")
    {
        int a[] = {1, 2, 3, 4};
        {
            GenericContainer::Vector s = GenericContainer::make_vector(a);
            GenericContainer::Vector<int>::iterator it = s.begin();
            auto it2 = it + 1;
            GenericContainer::Vector<int>::const_iterator cit = s.cbegin();

            REQUIRE(it == cit);
            REQUIRE(cit == it);
            REQUIRE(it == it);
            REQUIRE(cit == cit);
            REQUIRE(cit == s.begin());
            REQUIRE(s.begin() == cit);
            REQUIRE(s.cbegin() == cit);
            REQUIRE(it == s.begin());
            REQUIRE(s.begin() == it);

            REQUIRE(it != it2);
            REQUIRE(it2 != it);
            REQUIRE(it != s.end());
            REQUIRE(it2 != s.end());
            REQUIRE(s.end() != it);
            REQUIRE(it2 != cit);
            REQUIRE(cit != it2);

            REQUIRE(it < it2);
            REQUIRE(it <= it2);
            REQUIRE(it2 <= s.end());
            REQUIRE(it < s.end());
            REQUIRE(it <= cit);
            REQUIRE(cit <= it);
            REQUIRE(cit < it2);
            REQUIRE(cit <= it2);
            REQUIRE(cit < s.end());
            REQUIRE(cit <= s.end());

            REQUIRE(it2 > it);
            REQUIRE(it2 >= it);
            REQUIRE(s.end() > it2);
            REQUIRE(s.end() >= it2);
            REQUIRE(it2 > cit);
            REQUIRE(it2 >= cit);
        }
    }

    SECTION("begin_end")
    {
        {
            int a[] = {1, 2, 3, 4};
            GenericContainer::Vector s = GenericContainer::make_vector(a);

            GenericContainer::Vector<int>::iterator it = s.begin();
            GenericContainer::Vector<int>::iterator it2 = std::begin(s);
            REQUIRE(it == it2);

            it = s.end();
            it2 = std::end(s);
            REQUIRE(it == it2);
        }

        {
            int a[] = {1, 2, 3, 4};
            GenericContainer::Vector s = GenericContainer::make_vector(a);

            auto it = s.begin();
            auto first = it;
            REQUIRE(it == first);
            REQUIRE(*it == 1);

            auto beyond = s.end();
            REQUIRE(it != beyond);

            REQUIRE(beyond - first == 4);
            REQUIRE(first - first == 0);
            REQUIRE(beyond - beyond == 0);

            ++it;
            REQUIRE(it - first == 1);
            REQUIRE(*it == 2);
            *it = 22;
            REQUIRE(*it == 22);
            REQUIRE(beyond - it == 3);

            it = first;
            REQUIRE(it == first);
            while (it != s.end()) {
                *it = 5;
                ++it;
            }

            REQUIRE(it == beyond);
            REQUIRE(it - beyond == 0);

            for (const auto& n : s) {
                REQUIRE(n == 5);
            }
        }
    }

    SECTION("cbegin_cend")
    {
        {
            int a[] = {1, 2, 3, 4};
            GenericContainer::Vector s = GenericContainer::make_vector(a);

            GenericContainer::Vector<int>::const_iterator cit = s.cbegin();
            GenericContainer::Vector<int>::const_iterator cit2 = std::cbegin(s);
            REQUIRE(cit == cit2);

            cit = s.cend();
            cit2 = std::cend(s);
            REQUIRE(cit == cit2);
        }

        {
            int a[] = {1, 2, 3, 4};
            GenericContainer::Vector s = GenericContainer::make_vector(a);

            auto it = s.cbegin();
            auto first = it;
            REQUIRE(it == first);
            REQUIRE(*it == 1);

            auto beyond = s.cend();
            REQUIRE(it != beyond);

            REQUIRE(beyond - first == 4);
            REQUIRE(first - first == 0);
            REQUIRE(beyond - beyond == 0);

            ++it;
            REQUIRE(it - first == 1);
            REQUIRE(*it == 2);
            REQUIRE(beyond - it == 3);

            int last = 0;
            it = first;
            REQUIRE(it == first);
            while (it != s.cend()) {
                REQUIRE(*it == last + 1);

                last = *it;
                ++it;
            }

            REQUIRE(it == beyond);
            REQUIRE(it - beyond == 0);
        }
    }

    SECTION("rbegin_rend")
    {
        {
            int a[] = {1, 2, 3, 4};
            GenericContainer::Vector s = GenericContainer::make_vector(a);

            auto it = s.rbegin();
            auto first = it;
            REQUIRE(bool(it == first));
            REQUIRE(bool(*it == 4));

            auto beyond = s.rend();
            REQUIRE(bool(it != beyond));

            REQUIRE(bool(beyond - first == 4));
            REQUIRE(bool(first - first == 0));
            REQUIRE(bool(beyond - beyond == 0));

            ++it;
            REQUIRE(bool(it - first == 1));
            REQUIRE(bool(*it == 3));
            *it = 22;
            REQUIRE(bool(*it == 22));
            REQUIRE(bool(beyond - it == 3));

            it = first;
            REQUIRE(bool(it == first));
            while (it != s.rend()) {
                *it = 5;
                ++it;
            }

            REQUIRE(bool(it == beyond));
            REQUIRE(bool(it - beyond == 0));

            for (const auto& n : s) {
                REQUIRE(bool(n == 5));
            }
        }
    }

    SECTION("crbegin_crend")
    {
        {
            int a[] = {1, 2, 3, 4};
            GenericContainer::Vector s = GenericContainer::make_vector(a);

            auto it = s.crbegin();
            auto first = it;
            REQUIRE(bool(it == first));
            REQUIRE(bool(*it == 4));

            auto beyond = s.crend();
            REQUIRE(bool(it != beyond));

            REQUIRE(bool(beyond - first == 4));
            REQUIRE(bool(first - first == 0));
            REQUIRE(bool(beyond - beyond == 0));

            ++it;
            REQUIRE(bool(it - first == 1));
            REQUIRE(bool(*it == 3));
            REQUIRE(bool(beyond - it == 3));

            it = first;
            REQUIRE(bool(it == first));
            int last = 5;
            while (it != s.crend()) {
                REQUIRE(bool(*it == last - 1));
                last = *it;

                ++it;
            }

            REQUIRE(bool(it == beyond));
            REQUIRE(bool(it - beyond == 0));
        }
    }

}
