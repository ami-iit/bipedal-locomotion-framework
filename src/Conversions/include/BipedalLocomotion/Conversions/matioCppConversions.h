/**
 * @file matioCppConversions.h
 * @authors Stefano Dafarra
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_MATIOCPPCONVERSIONS_H
#define BIPEDAL_LOCOMOTION_MATIOCPPCONVERSIONS_H

#include <Eigen/Core>
#include <matioCpp/matioCpp.h>
#include <BipedalLocomotion/GenericContainer/Vector.h>

namespace BipedalLocomotion {
namespace Conversions {

template <typename type>
Eigen::Map<Eigen::Matrix<type, Eigen::Dynamic, Eigen::Dynamic>> toEigen(matioCpp::MultiDimensionalArray<type>& input)
{
    assert(input.isValid());
    assert(input.dimensions().size() == 2);
    return Eigen::Map<Eigen::Matrix<type, Eigen::Dynamic, Eigen::Dynamic>>(input.data(), input.dimensions()(0), input.dimensions()(1));
}

template <typename type>
const Eigen::Map<Eigen::Matrix<type, Eigen::Dynamic, Eigen::Dynamic>> toEigen(const matioCpp::MultiDimensionalArray<type>& input)
{
    assert(input.isValid());
    assert(input.dimensions().size() == 2);
    return Eigen::Map<const Eigen::Matrix<type, Eigen::Dynamic, Eigen::Dynamic>>(input.data(), input.dimensions()(0), input.dimensions()(1));
}

template <typename type>
Eigen::Map<Eigen::Matrix<type, Eigen::Dynamic, 1>> toEigen(matioCpp::Vector<type>& input)
{
    assert(input.isValid());
    return Eigen::Map<Eigen::Matrix<type, Eigen::Dynamic, 1>>(input.data(), input.size());
}

template <typename type>
const Eigen::Map<Eigen::Matrix<type, Eigen::Dynamic, 1>> toEigen(const matioCpp::Vector<type>& input)
{
    assert(input.isValid());
    return Eigen::Map<const Eigen::Matrix<type, Eigen::Dynamic, 1>>(input.data(), input.size());
}

template <typename EigenDerived, typename = std::enable_if_t<Eigen::MatrixBase<EigenDerived>::RowsAtCompileTime != 1 &&
                                                             Eigen::MatrixBase<EigenDerived>::ColsAtCompileTime != 1>>
matioCpp::MultiDimensionalArray<typename EigenDerived::Scalar> tomatioCpp(const Eigen::MatrixBase<EigenDerived>& input, const std::string& name)
{
    matioCpp::MultiDimensionalArray<typename EigenDerived::Scalar> matio(name, {static_cast<size_t>(input.rows()), static_cast<size_t>(input.cols())});
    toEigen(matio) = input;
    return matio;
}

template <class Vector, typename = typename std::enable_if_t<BipedalLocomotion::GenericContainer::is_vector_constructible<Vector>::value &&
                                                             !std::is_same_v<Vector, std::string>>>
matioCpp::Vector<typename std::remove_cv_t<typename BipedalLocomotion::container_data<Vector>::type>> tomatioCpp(const Vector& input, const std::string& name)
{
    using type = typename BipedalLocomotion::container_data<Vector>::type;
    typename BipedalLocomotion::GenericContainer::Vector<const type>::Ref ref(input);
    return matioCpp::Vector<typename std::remove_cv_t<type>>(name, ref); //data is copied
}

inline matioCpp::String tomatioCpp(const std::string& input, const std::string& name)
{
    return matioCpp::String(name, input);
}

inline matioCpp::Vector<matioCpp::Logical> tomatioCpp(const std::vector<bool>& input, const std::string& name)
{
    return matioCpp::Vector<matioCpp::Logical>(name, input);
}

template<typename type, typename = std::enable_if_t<std::is_fundamental_v<type> && !std::is_same_v<type, bool>>>
matioCpp::Element<type> tomatioCpp(type input, const std::string& name)
{
    return matioCpp::Element<type>(name, input);
}

inline matioCpp::Element<matioCpp::Logical> tomatioCpp(bool input, const std::string& name)
{
    return matioCpp::Element<matioCpp::Logical>(name, input);
}

} //namespace Conversions
} //namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_MATIOCPPCONVERSIONS_H
