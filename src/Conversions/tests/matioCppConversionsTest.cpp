/**
 * @file matioCppConversionsTest.cpp
 * @authors Stefano Dafarra
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

// Catch2
#include <catch2/catch_test_macros.hpp>
#include <BipedalLocomotion/Conversions/matioCppConversions.h>
#include <iDynTree/VectorDynSize.h>
#include <unordered_map>
#include <map>
#include <set>


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

TEST_CASE("To matio")
{
    std::vector<double> stdVec = {1.0, 2.0, 3.0, 4.0, 5.0};

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

    iDynTree::MatrixDynSize iDynTreeMatrix(2,2);
    iDynTree::toEigen(iDynTreeMatrix) << 1.0, 2.0, 3.0, 4.0;
    auto toMatioMatrixiDynTree = tomatioCpp(iDynTreeMatrix, "iDynTreeMatrix");
    checkSameMatrix(iDynTreeMatrix, toMatioMatrixiDynTree);

    iDynTree::MatrixFixSize<2,2> iDynTreeMatrixFixed;
    iDynTree::toEigen(iDynTreeMatrixFixed) = iDynTree::toEigen(iDynTreeMatrix);
    auto toMatioMatrixiDynTreeFixed = tomatioCpp(iDynTreeMatrixFixed, "iDynTreeMatrixFixed");
    checkSameMatrix(iDynTreeMatrixFixed, toMatioMatrixiDynTreeFixed);

    std::string string("something");
    auto toMatioString = tomatioCpp(string, "name");
    REQUIRE(string == toMatioString());

    std::vector<bool> vecOfBool = {true, false, true};
    auto toVecofBool = tomatioCpp(vecOfBool, "vecOfBool");
    checkSameVectors(vecOfBool, toVecofBool);

    auto matioDouble = tomatioCpp(5.0, "double");
    REQUIRE(matioDouble == 5.0);

    auto matioBool = tomatioCpp(true, "bool");
    REQUIRE(matioBool);

    auto matioInt = tomatioCpp(2, "bool");
    REQUIRE(matioInt == 2);

    auto matioChar = tomatioCpp('f', "char");
    REQUIRE(matioChar == 'f');

    std::vector<std::string> stringVector = {"Huey", "Dewey", "Louie"};
    auto matioCell = tomatioCpp(stringVector, "stringVector");
    for (size_t i = 0; i < stringVector.size(); ++i)
    {
        REQUIRE(matioCell[i].asString()() == stringVector[i]);
    }
}

TEST_CASE("to matio from iterators")
{
    std::unordered_map<std::string, int> unordered_map;
    unordered_map["one"] = 1;
    unordered_map["two"] = 2;
    unordered_map["three"] = 3;

    matioCpp::Struct toMatio = tomatioCppStruct(unordered_map.begin(), unordered_map.end(), "map");

    REQUIRE(toMatio["one"].asElement<int>() == 1);
    REQUIRE(toMatio["two"].asElement<int>() == 2);
    REQUIRE(toMatio["three"].asElement<int>() == 3);

    std::map<std::string, char> map;
    map["a"] = 'a';
    map["b"] = 'b';
    map["c"] = 'c';

    matioCpp::CellArray toMatioCell = toMatioCppCellArray(map.begin(), map.end(), "mapAsCell");
    REQUIRE(toMatioCell[0].asElement<char>() == 'a');
    REQUIRE(toMatioCell[0].name() == "a");
    REQUIRE(toMatioCell[1].asElement<char>() == 'b');
    REQUIRE(toMatioCell[1].name() == "b");
    REQUIRE(toMatioCell[2].asElement<char>() == 'c');
    REQUIRE(toMatioCell[2].name() == "c");

    std::set<std::string> set({"Huey", "Dewey", "Louie"});
    toMatioCell = toMatioCppCellArray(set.begin(), set.end(), "setAsCell");
    REQUIRE(toMatioCell[0].asString()() == "Dewey");
    REQUIRE(toMatioCell[1].asString()() == "Huey");
    REQUIRE(toMatioCell[2].asString()() == "Louie");
}

TEST_CASE("From matioCpp to Eigen")
{
    std::vector<double> stdVec = {1.0, 2.0, 3.0, 4.0, 5.0};

    matioCpp::Vector<double> vector("vector", 5);
    vector = stdVec;

    Eigen::VectorXd eigenVec = toEigen(vector);
    checkSameVectors(eigenVec, vector);

    Eigen::Matrix3f eigenMatrix;
    eigenMatrix << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;

    matioCpp::MultiDimensionalArray<float> matioCppMatrix("matrix", {3,3}, eigenMatrix.data());

    Eigen::MatrixXf toEigenMatrix = toEigen(matioCppMatrix);
    checkSameMatrix(toEigenMatrix, matioCppMatrix);

}
