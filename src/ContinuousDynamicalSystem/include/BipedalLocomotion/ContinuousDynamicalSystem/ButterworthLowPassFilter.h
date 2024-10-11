/**
 * @file ButterworthLowPassFilter.h
 * @authors Giulio Romualdi
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_BUTTERWORTH_LOW_PASS_FILTER_H
#define BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_BUTTERWORTH_LOW_PASS_FILTER_H

#include <memory>

#include <BipedalLocomotion/ContinuousDynamicalSystem/ForwardEuler.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/LinearTimeInvariantSystem.h>
#include <BipedalLocomotion/System/Advanceable.h>

#include <Eigen/Dense>

namespace BipedalLocomotion
{
namespace ContinuousDynamicalSystem
{

/**
 * ButterworthLowPass implements a low pass filter of order N.
 *
 * The system is described by the following transfer function
 * \f[
 * H(s) = \frac{1}{\sqrt{1 + \left(\frac{s}{\omega_c}\right)^{2N}}}
 * \f]
 * where \f$\omega_c\f$ is the cutoff frequency and \f$N\f$ is the order of the filter and \f$s\f$
 * is the Laplace variable.
 *
 * What follows is a brief description of the filter and how it is implemented.
 *
 * @section ButterworthLowPassFilter_continuous Compute the transfer function of the continuous system
 * What follows is taken from from Passive and Active Network Analysis and Synthesis, Aram Budak,
 * Houghton Mifflin, 1974 and from
 * https://dsp.stackexchange.com/questions/79498/butterworth-filter-poles
 * The poles of the Butterworth filter  are evenly spaced on a circle of radius \f$\omega_c\f$ in
 * the s-plane. The poles are given by
 * \f[
 * p_k = \omega_c e^{j \frac{\pi}{2} \left(1 + \frac{2k - 1}{2N}\right)}
 * \f]
 * where \f$k = 0, 1, \ldots, N-1\f$ and \f$j\f$ is the imaginary unit.
 *  By construction, the Butterworth filter does not have zeros.
 * The gain of the filter is  given by
 * \f[
 * K = \prod_{k=0}^{N-1} s_k = \omega_c^N
 * \f]
 * @section ButterworthLowPassFilter_discrete Compute the transfer function of the discrete system
 * As mentioned before, the transfer function of the discrete system is obtained by the bilinear
 * transform
 * \f[
 * s = \frac{2}{\delta t} \frac{1 - z^{-1}}{1 + z^{-1}}
 * \f]
 * The poles of the discrete system are obtained by substituting the poles of the continuous system
 * in the bilinear transformation as explained in
 * https://it.mathworks.com/help/signal/ref/bilinear.html
 * The poles of the discrete system are given by
 * \f[
 * p^d_k = \frac{1 + p_k \delta t/2}{1 - p_k \delta t/2}
 * \f]
 * where \f$p_k\f$ are the poles of the continuous system, \f$\delta t\f$ is the sampling time and
 * \f$k = 0, 1, \ldots, N-1\f$.
 * All the zeros of the continuous system are mapped to -1.
 * Finally, the gain of the discrete system is given by
 * \f[
 * K^d = \text{real}  \frac{K}{ \prod (\frac{2}{\delta T} - p_k) }
 * \f]
 * @subsection ButterworthLowPassFilter_prewrapping Pre-wrapping
 * The ButterworthLowPass supports the pre-wrapping of the filter. The pre-wrapping is a technique
 * used to mitigate the distortion that can occur during the bilinear transformation. It
 * consists in shifting the poles of the continuous system in the s-plane.
 * To easily implement the pre-wrapping, we slightly modify the bilinear transformation as
 * \f[
 * s = \frac{\omega_c}{\tan\left(\frac{\omega_c \delta t}{2}\right)} \frac{1 - z^{-1}}{1 + z^{-1}}
 * \f]
 * where \f$\omega_c\f$ is the cutoff frequency and \f$\delta t\f$ is the sampling time.
 * In the class the pre-wrapping is enabled by default and can be disabled by setting the parameter
 * `enable_prewrapping` to false.
 * The interested reader can find more information about the pre-wrapping at
 * [this link](https://en.wikipedia.org/wiki/Bilinear_transform#Frequency_warping).
 * @section ButterworthLowPassFilter_coefficients Compute the coefficients of the discrete system
 * Once we have the poles and the gain of the discrete system we can compute the coefficients of the
 * filter by applying the [Vieta's formulas](https://en.wikipedia.org/wiki/Vieta%27s_formulas).
 * The transfer function of the discrete system is given by
 * \f[
 * H(z) = \frac{a_n + a_{n-1} z^{-1} + \ldots + a_1 z^{-n+1} + a_0 z^{-n}}{1 + b_{n-1} z^{-1} + \ldots + b_1 z^{-n+1} + b_0 z^{-n}}
 * \f]
 * Once the numerator and the denominator are computed we can easily antitransform the transfer function
 * to obtain the coefficients of the filter as
 * \f[
 * y[k] = \frac{1}{b_0} \left( a_0 x[k] + a_1 x[k-1] + \ldots + a_n x[k-n] - b_1 y[k-1] - \ldots - b_n y[k-n] \right)
 * \f]
 * where \f$x[k]\f$ is the input of the filter and \f$y[k]\f$ is the output of the filter.
 */
class ButterworthLowPassFilter
    : public BipedalLocomotion::System::Advanceable<Eigen::VectorXd, Eigen::VectorXd>
{
public:
    /**
     * Constructor
     */
    ButterworthLowPassFilter();

    /**
     * Destructor
     */
    ~ButterworthLowPassFilter() override;

    // clang-format off
    /**
     * Initialize the Dynamical system.
     * @param handler pointer to the parameter handler.
     * @note the following parameters are required
     * |   Parameter Name   |   Type   |                        Description                           | Mandatory |
     * |:------------------:|:--------:|:------------------------------------------------------------:|:---------:|
     * |  `sampling_time`   | `double` |        Sampling time used to discrete the linear system      |    Yes    |
     * |  `cutoff_frequency`| `double` |          Cutoff frequency of the low pass filter.            |    Yes    |
     * |      `order`       |  `int`   |               Order of the low pass filter.                  |    Yes    |
     * |`enable_prewrapping`|  `bool`  | Enable the pre-wrapping of the filter. (Default value True). |    No     |
     * @return true in case of success/false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler) override;
    // clang-format on

    /**
     * Set the state of the smoother.
     * @param initialState initial state of the smoother
     * @return true in case of success, false otherwise.
     */
    bool reset(Eigen::Ref<const Eigen::VectorXd> initialState);

    /**
     * Compute the output of the filter.
     * @return true in case of success, false otherwise.
     * @note The function update also the internal state of the filter.
     */
    bool advance() override;

    /**
     * Get the output of the filter.
     * @return a vector containing the output of the smoother
     */
    const Eigen::VectorXd& getOutput() const override;

    /**
     * Set the input of the filter
     * @param input the vector representing the input of the filter
     * @return True in case of success and false otherwise
     */
    bool setInput(const Eigen::VectorXd& input) override;

    /**
     * Determines the validity of the object retrieved with getOutput()
     * @return True if the object is valid, false otherwise.
     */
    bool isOutputValid() const override;

private:
    struct Impl; /**< Pimpl idiom */
    std::unique_ptr<Impl> m_pimpl; /**< Pointer to the implementation */
};

} // namespace ContinuousDynamicalSystem
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_BUTTERWORTH_LOW_PASS_FILTER_H
