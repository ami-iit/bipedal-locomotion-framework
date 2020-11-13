/**
 * @file matioCppConversionsTest.cpp
 * @authors Stefano Dafarra
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

// Catch2
#include <catch2/catch.hpp>
#include <BipedalLocomotion/Conversions/matioCppConversions.h>

using namespace BipedalLocomotion::Conversions;

template<typename Vector1, typename Vector2>
void checkSameVectors(const Vector1& a, const Vector2& b)
{
    REQUIRE(static_cast<size_t>(a.size()) == static_cast<size_t>(b.size()));

    for (size_t i = 0; i < static_cast<size_t>(a.size()); ++i)
    {
        REQUIRE(a[i] == b[i]);
    }
}

template<typename Matrix, typename type>
void checkSameMatrix(const Matrix& a, const matioCpp::MultiDimensionalArray<type>& b)
{
    REQUIRE(b.isValid());
    REQUIRE(b.dimensions().size() == 2);
    REQUIRE(static_cast<size_t>(a.rows()) == b.dimensions()(0));
    REQUIRE(static_cast<size_t>(a.cols()) == b.dimensions()(1));

    for (size_t i = 0; i < b.dimensions()(0); ++i)
    {
        for (size_t j = 0; j < b.dimensions()(1); ++j)
        {
            REQUIRE(a(i,j) == b({i,j}));
        }
    }
}

TEST_CASE("From vector to matio")
{
    std::vector<double> stdVec = {1, 2, 3, 4, 5};

    auto toMatio = tomatioCpp(stdVec, "test");

    checkSameVectors(stdVec, toMatio);

    Eigen::Vector3i eigenVec;
    eigenVec << 2, 4, 6;
    auto toMatioEigenVec = tomatioCpp(eigenVec, "testEigen");
    checkSameVectors(eigenVec, toMatioEigenVec);

    Eigen::Matrix3f eigenMatrix;
    eigenMatrix << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
    auto toMatioMatrix = tomatioCpp(eigenMatrix, "testMatrix");
    checkSameMatrix(eigenMatrix, toMatioMatrix);

    std::string string("something");
    auto toMatioString = tomatioCpp(string, "name");
    REQUIRE(string == toMatioString());

    std::vector<bool> vecOfBool = {true, false, true};
    auto toVecofBool = tomatioCpp(vecOfBool, "vecOfBool");
    checkSameVectors(vecOfBool, toVecofBool);
}
