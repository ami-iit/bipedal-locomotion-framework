/**
 * @file CppAD.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_AUTODIFF_CPPAD_H
#define BIPEDAL_LOCOMOTION_AUTODIFF_CPPAD_H

#include <Eigen/Dense>
#include <cppad/cppad.hpp>
#include <cppad/example/cppad_eigen.hpp>

namespace Eigen
{
namespace internal
{

/**
 * cast_impl
 * @note Please find more information here:
 * https://stackoverflow.com/questions/10088002/how-to-implement-static-cast-in-c/10088161
 */
template <typename Scalar> struct cast_impl<CppAD::AD<Scalar>, Scalar>
{
    // Define eigen function
    EIGEN_DEVICE_FUNC

    static inline Scalar run(const CppAD::AD<Scalar>& x)
    {
        return CppAD::Value(x);
    }
};
} // namespace internal
} // namespace Eigen

namespace BipedalLocomotion
{
namespace AutoDiff
{
namespace CppAD
{

// Generate the VectorX and MatrixX typedef for CppAD
#define EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, Size, SizeSuffix)             \
    typedef Eigen::Matrix<Type, Size, Size> Matrix##SizeSuffix##TypeSuffix; \
    typedef Eigen::Matrix<Type, Size, 1> Vector##SizeSuffix##TypeSuffix;    \
    typedef Eigen::Matrix<Type, 1, Size> RowVector##SizeSuffix##TypeSuffix;

#define EIGEN_MAKE_TYPEDEFS_ALL_SIZES(Type, TypeSuffix) \
    EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, Eigen::Dynamic, X)

EIGEN_MAKE_TYPEDEFS_ALL_SIZES(::CppAD::AD<double>, AD)

#undef EIGEN_MAKE_TYPEDEFS_ALL_SIZES
#undef EIGEN_MAKE_TYPEDEFS
#undef EIGEN_MAKE_FIXED_TYPEDEFS

} // namespace CppAD
} // namespace AutoDiff
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_AUTODIFF_CPPAD_H
