/**
 * @file matioCppConversions.h
 * @authors Stefano Dafarra
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_MATIOCPPCONVERSIONS_H
#define BIPEDAL_LOCOMOTION_MATIOCPPCONVERSIONS_H

#include <Eigen/Core>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/MatrixFixSize.h>
#include <matioCpp/matioCpp.h>
#include <BipedalLocomotion/GenericContainer/Vector.h>

namespace BipedalLocomotion {
namespace Conversions {

/**
 * @brief Conversion from a matioCpp::MultiDimensionalArray to an Eigen matrix
 * @param input The matioCpp::MultiDimensionalArray
 * @return A map from the internal data of the MultiDimensionalArray
 */
template <typename type>
Eigen::Map<Eigen::Matrix<type, Eigen::Dynamic, Eigen::Dynamic>> toEigen(matioCpp::MultiDimensionalArray<type>& input)
{
    assert(input.isValid());
    assert(input.dimensions().size() == 2);
    return Eigen::Map<Eigen::Matrix<type, Eigen::Dynamic, Eigen::Dynamic>>(input.data(), input.dimensions()(0), input.dimensions()(1));
}

/**
 * @brief Conversion from a const matioCpp::MultiDimensionalArray to an Eigen matrix
 * @param input The matioCpp::MultiDimensionalArray
 * @return A const map from the internal data of the MultiDimensionalArray
 */
template <typename type>
const Eigen::Map<Eigen::Matrix<type, Eigen::Dynamic, Eigen::Dynamic>> toEigen(const matioCpp::MultiDimensionalArray<type>& input)
{
    assert(input.isValid());
    assert(input.dimensions().size() == 2);
    return Eigen::Map<const Eigen::Matrix<type, Eigen::Dynamic, Eigen::Dynamic>>(input.data(), input.dimensions()(0), input.dimensions()(1));
}

/**
 * @brief Conversion from a matioCpp::Vector to an Eigen vector
 * @param input The matioCpp::Vector
 * @return A map from the internal data of the Vector
 */
template <typename type>
Eigen::Map<Eigen::Matrix<type, Eigen::Dynamic, 1>> toEigen(matioCpp::Vector<type>& input)
{
    assert(input.isValid());
    return Eigen::Map<Eigen::Matrix<type, Eigen::Dynamic, 1>>(input.data(), input.size());
}

/**
 * @brief Conversion from a const matioCpp::Vector to an Eigen vector
 * @param input The matioCpp::Vector
 * @return A const map from the internal data of the Vector
 */
template <typename type>
const Eigen::Map<Eigen::Matrix<type, Eigen::Dynamic, 1>> toEigen(const matioCpp::Vector<type>& input)
{
    assert(input.isValid());
    return Eigen::Map<const Eigen::Matrix<type, Eigen::Dynamic, 1>>(input.data(), input.size());
}

/**
 * @brief Conversion from an Eigen matrix to a matioCpp::MultiDimensionalArray
 * @param input The input matrix.
 * @param name The name of the resulting matioCpp variable.
 * @return A matioCpp::MultiDimensionalArray containing a copy of the input data
 */
template <typename EigenDerived, typename = std::enable_if_t<Eigen::MatrixBase<EigenDerived>::RowsAtCompileTime != 1 &&
                                                             Eigen::MatrixBase<EigenDerived>::ColsAtCompileTime != 1>>
matioCpp::MultiDimensionalArray<typename EigenDerived::Scalar> tomatioCpp(const Eigen::MatrixBase<EigenDerived>& input, const std::string& name)
{
    matioCpp::MultiDimensionalArray<typename EigenDerived::Scalar> matio(name, {static_cast<size_t>(input.rows()), static_cast<size_t>(input.cols())});
    toEigen(matio) = input;
    return matio;
}

/**
 * @brief Conversion from an iDynTree::MatrixDynSize to a matioCpp::MultiDimensionalArray
 * @param input The input matrix.
 * @param name The name of the resulting matioCpp variable.
 * @return A matioCpp::MultiDimensionalArray containing a copy of the input data
 */
inline matioCpp::MultiDimensionalArray<double> tomatioCpp(const iDynTree::MatrixDynSize& input, const std::string& name)
{
    Eigen::MatrixXd eigenCopy = iDynTree::toEigen(input); //Need to make it column major first
    return tomatioCpp(eigenCopy, name);
}

/**
 * @brief Conversion from an iDynTree::MatrixFixSize to a matioCpp::MultiDimensionalArray
 * @param input The input matrix.
 * @param name The name of the resulting matioCpp variable.
 * @return A matioCpp::MultiDimensionalArray containing a copy of the input data
 */
template<unsigned int nRows, unsigned int nCols>
matioCpp::MultiDimensionalArray<double> tomatioCpp(const iDynTree::MatrixFixSize<nRows, nCols>& input, const std::string& name)
{
    Eigen::Matrix<double, nRows, nCols> eigenCopy = iDynTree::toEigen(input); //Need to make it column major first
    return tomatioCpp(eigenCopy, name);
}

/**
 * @brief Conversion from a generic Vector to a matioCpp::Vector
 * @param input The input vector.
 * @param name The name of the resulting matioCpp variable.
 * @return A matioCpp::Vector containing a copy of the input data
 */
template <class Vector, typename = typename std::enable_if_t<BipedalLocomotion::GenericContainer::is_vector_constructible<Vector>::value &&
                                                             !std::is_same_v<Vector, std::string>>>
matioCpp::Vector<typename std::remove_cv_t<typename BipedalLocomotion::container_data<Vector>::type>> tomatioCpp(const Vector& input, const std::string& name)
{
    using type = typename BipedalLocomotion::container_data<Vector>::type;
    typename BipedalLocomotion::GenericContainer::Vector<const type>::Ref ref(input);
    return matioCpp::Vector<typename std::remove_cv_t<type>>(name, ref); //data is copied
}

/**
 * @brief Conversion from a std::string to a matioCpp::String
 * @param input The input string.
 * @param name The name of the resulting matioCpp variable.
 * @return A matioCpp::String containing a copy of the input data
 */
inline matioCpp::String tomatioCpp(const std::string& input, const std::string& name)
{
    return matioCpp::String(name, input);
}

/**
 * @brief Conversion from a boolean vector to a matioCpp::Vector<matioCpp::Logical>
 * @param input The input vector.
 * @param name The name of the resulting matioCpp variable.
 * @return A matioCpp::Vector<matioCpp::Logical> containing a copy of the input data
 */
inline matioCpp::Vector<matioCpp::Logical> tomatioCpp(const std::vector<bool>& input, const std::string& name)
{
    return matioCpp::Vector<matioCpp::Logical>(name, input);
}

/**
 * @brief Conversion from a fundamental type to the corresponding matioCpp::Element
 * @param input The input element.
 * @param name The name of the resulting matioCpp variable.
 * @return A matioCpp::Element containing a copy of the input data
 */
template<typename type, typename = std::enable_if_t<std::is_fundamental_v<type> && !std::is_same_v<type, bool>>>
matioCpp::Element<type> tomatioCpp(type input, const std::string& name)
{
    return matioCpp::Element<type>(name, input);
}

/**
 * @brief Conversion from a boolean to a matioCpp::Element<matioCpp::Logical>
 * @param input The input element.
 * @param name The name of the resulting matioCpp variable.
 * @return A matioCpp::Element<matioCpp::Logical> whose value is equal to the input.
 */
inline matioCpp::Element<matioCpp::Logical> tomatioCpp(bool input, const std::string& name)
{
    return matioCpp::Element<matioCpp::Logical>(name, input);
}

} //namespace Conversions
} //namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_MATIOCPPCONVERSIONS_H
