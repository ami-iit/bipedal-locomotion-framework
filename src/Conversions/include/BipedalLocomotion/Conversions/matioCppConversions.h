/**
 * @file matioCppConversions.h
 * @authors Stefano Dafarra
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_MATIOCPPCONVERSIONS_H
#define BIPEDAL_LOCOMOTION_MATIOCPPCONVERSIONS_H

#include <Eigen/Core>
#include <iDynTree/MatrixDynSize.h>
#include <iDynTree/EigenHelpers.h>
#include <iDynTree/MatrixFixSize.h>
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
matioCpp::Element<type> tomatioCpp(const type& input, const std::string& name)
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

/**
 * @brief Conversion from a vector of strings to a matioCpp::CellArray containing the input strings
 * @param input The input vector of strings.
 * @param name The name of the resulting matioCpp variable.
 * @return A matioCpp::CellArray of dimensions nx1 (with n the number of strings)
 */
inline matioCpp::CellArray tomatioCpp(const std::vector<std::string>& input, const std::string& name)
{
    matioCpp::CellArray stringsArray(name, {input.size(), 1});
    for (size_t i = 0; i < input.size(); ++i)
    {
        stringsArray.setElement(i, tomatioCpp(input[i], input[i]));
    }

    return stringsArray;
}

/**
 * @brief Create a matioCpp::Struct starting from the begin and end iterators of a map-like container
 * The dereferenced value of the iterator has to be a pair (like with std::maps and std::unordered_map)
 * with the key being a string. For each key, there is the corresponding field in the Struct.
 * @param begin The iterator to the first element
 * @param end The iterator to the element after the last.
 * @param name The name of the struct.
 * @return The corresponding matioCpp::Struct
 */
template<class iterator,
          typename = typename std::enable_if_t<is_pair_iterator_string<iterator>::value>>
matioCpp::Struct tomatioCppStruct(iterator begin, iterator end, const std::string& name)
{
    matioCpp::Struct matioStruct(name);
    for (iterator it = begin; it != end; it++)
    {
        bool ok = matioStruct.setField(tomatioCpp(it->second, it->first));
        unused(ok);
        assert(ok);
    }

    return matioStruct;
}

/**
 * @brief Create a matioCpp::CellArray starting from the begin and end iterators of a container.
 * If dereferenced value of the iterator is a pair (like with std::maps and std::unordered_map),
 * only the "second" element is considered. The name of the imported variable in the CellArray
 * is "imported_element_x", where "x" is the corresponding raw index. If the iterator is a pair,
 * and "first" is a string, this will be used as a name.
 * @param begin The iterator to the first element
 * @param end The iterator to the element after the last.
 * @param name The name of the CellArray.
 * @return The corresponding matioCpp::CellArray.
 */
template<class iterator>
matioCpp::CellArray toMatioCppCellArray(const iterator& begin, const iterator& end, const std::string& name)
{
    matioCpp::CellArray matioCellArray(name, {static_cast<size_t>(std::distance(begin, end)), 1});

    size_t index = 0;
    iterator it = begin;
    while (it != end)
    {
        bool ok = false;
        if constexpr (is_pair<decltype(*std::declval<iterator>())>::value)
        {
            std::string name = "imported_element_" + std::to_string(index);

            if constexpr (std::is_convertible<decltype(std::declval<iterator>()->first), std::string>::value)
            {
                name = it->first;
            }

            ok = matioCellArray.setElement(index, tomatioCpp(it->second, name));
        }
        else
        {
            ok = matioCellArray.setElement(index, tomatioCpp(*it, "imported_element_" + std::to_string(index)));
        }
        unused(ok);
        assert(ok);
        index++;
        it++;
    }

    return matioCellArray;
}


} //namespace Conversions
} //namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_MATIOCPPCONVERSIONS_H
