/**
 * @file ButterworthLowPassFilter.cpp
 * @authors Giulio Romualdi
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/ContinuousDynamicalSystem/ButterworthLowPassFilter.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <complex>
#include <deque>

using namespace BipedalLocomotion::ContinuousDynamicalSystem;

struct ButterworthLowPassFilter::Impl
{
    double frequency{0.0};
    double cutOffPulsation{0.0};
    int order{0};

    std::vector<std::complex<double>> continuousPoles;
    double continuousGain{0.0};

    std::vector<std::complex<double>> discreteZeros;
    std::vector<std::complex<double>> discretePoles;
    double discreteGain{0.0};

    std::vector<double> numerator; /**< Numerator of the discrete transfer function. The first
                                      element is the highest order coefficient. */
    std::vector<double> denominator; /**< Denominator of the discrete transfer function. The first
                                        element is the highest order coefficient. */

    std::deque<Eigen::VectorXd> inputBuffer;
    std::deque<Eigen::VectorXd> outputBuffer;

    Eigen::VectorXd output;
    double bilinearTransformationFactor{0.0};

    bool isOutputValid{false};

    void computePole()
    {
        this->continuousPoles.clear();
        for (int i = 0; i < this->order; i++)
        {
            using namespace std::complex_literals;
            // equation taken from
            std::complex<double> pole
                = this->cutOffPulsation
                  * std::exp(1i * M_PI_2 * (1.0 + (1.0 + 2.0 * i) / double(this->order)));
            this->continuousPoles.push_back(pole);
        }

        // the gain is computed as the product of the poles
        std::complex<double> tmp = 1.0;
        for (const auto& pole : this->continuousPoles)
        {
            tmp *= pole;
        }
        this->continuousGain = this->order % 2 == 0 ? tmp.real() : -tmp.real();
    }

    void bilinearTransformation()
    {
        const double k = bilinearTransformationFactor;
        // bilinear transformation
        for (const auto& pole : this->continuousPoles)
        {
            std::complex<double> z = (1.0 + pole / k) / (1.0 - pole / k);
            this->discretePoles.push_back(z);
        }

        this->discreteZeros = std::vector<std::complex<double>>(this->discretePoles.size(), -1.0);

        std::complex<double> tmp = 1.0;
        for (const auto& pole : this->continuousPoles)
        {
            tmp *= (k - pole);
        }

        this->discreteGain = (this->continuousGain / tmp).real();
    }

    std::vector<double> computeVietaFormula(const std::vector<std::complex<double>>& roots)
    {
        int n = roots.size();
        std::vector<std::complex<double>> coeffs(n + 1, 0.0);
        coeffs[0] = 1; // Leading coefficient is always 1
        for (int i = 0; i < n; ++i)
        {
            for (int j = n; j > 0; --j)
            {
                coeffs[j] = coeffs[j] - roots[i] * coeffs[j - 1];
            }
        }

        std::vector<double> coefficients(n + 1);
        for (int i = 0; i < n + 1; i++)
        {
            coefficients[i] = coeffs[i].real();
        }

        return coefficients;
    }
};

ButterworthLowPassFilter::ButterworthLowPassFilter()
{
    m_pimpl = std::make_unique<Impl>();
}

ButterworthLowPassFilter::~ButterworthLowPassFilter() = default;

bool ButterworthLowPassFilter::initialize(
    std::weak_ptr<const ParametersHandler::IParametersHandler> handler)
{
    constexpr auto logPrefix = "[ButterworthLowPassFilter::initialize]";
    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} Invalid parameter handler.", logPrefix);
        return false;
    }

    std::chrono::nanoseconds samplingTime;
    if (!ptr->getParameter("sampling_time", samplingTime))
    {
        log()->error("{} Unable to get the parameter named 'sampling_time'.", logPrefix);
        return false;
    }
    const double dt = std::chrono::duration<double>(samplingTime).count();
    m_pimpl->frequency = 1.0 / dt;

    double cutOffFrequency;
    if (!ptr->getParameter("cutoff_frequency", cutOffFrequency))
    {
        log()->error("{} Unable to get the parameter named 'cutoff_frequency'.", logPrefix);
        return false;
    }
    m_pimpl->cutOffPulsation = 2 * M_PI * cutOffFrequency;

    if (!ptr->getParameter("order", m_pimpl->order) || m_pimpl->order < 1)
    {
        log()->error("{} Unable to get the parameter named 'order'. The order must be greater than "
                     "or equal to 1.",
                     logPrefix);
        return false;
    }

    bool enablePrewrapping{true};
    if (!ptr->getParameter("enable_prewrapping", enablePrewrapping))
    {
        log()->info("{} Unable to get the parameter named 'enable_prewrapping'. Using the default "
                    "value {}.",
                    logPrefix,
                    enablePrewrapping);
    }

    m_pimpl->bilinearTransformationFactor
        = enablePrewrapping ? m_pimpl->cutOffPulsation / tan((m_pimpl->cutOffPulsation * dt / 2.0))
                            : 2.0 * m_pimpl->frequency;

    // compute the poles and the gain of the continuous system
    m_pimpl->computePole();

    // apply the bilinear transformation
    m_pimpl->bilinearTransformation();

    // compute the coefficients of the discrete system
    m_pimpl->numerator = m_pimpl->computeVietaFormula(m_pimpl->discreteZeros);
    for (auto& element : m_pimpl->numerator)
    {
        element *= m_pimpl->discreteGain;
    }
    m_pimpl->denominator = m_pimpl->computeVietaFormula(m_pimpl->discretePoles);

    return true;
}

bool ButterworthLowPassFilter::reset(Eigen::Ref<const Eigen::VectorXd> initialState)
{
    m_pimpl->inputBuffer.clear();
    m_pimpl->outputBuffer.clear();

    // initialize the input buffer
    for (int i = 0; i < m_pimpl->order + 1; i++)
    {
        m_pimpl->inputBuffer.push_back(initialState);
    }
    for (int i = 0; i < m_pimpl->order; i++)
    {
        m_pimpl->outputBuffer.push_back(initialState);
    }

    m_pimpl->isOutputValid = false;

    return true;
}

bool ButterworthLowPassFilter::setInput(const Eigen::VectorXd& input)
{
    constexpr auto logPrefix = "[ButterworthLowPassFilter::setInput]";
    if (m_pimpl->inputBuffer.empty() || m_pimpl->outputBuffer.empty())
    {
        log()->error("{} The filter has not been reset.", logPrefix);
        return false;
    }

    if (input.size() != m_pimpl->inputBuffer.front().size())
    {
        log()->error("{} The input has a wrong size.", logPrefix);
        return false;
    }

    m_pimpl->inputBuffer.push_front(input);
    m_pimpl->inputBuffer.pop_back();

    m_pimpl->isOutputValid = false;
    return true;
}

bool ButterworthLowPassFilter::advance()
{
    constexpr auto logPrefix = "[ButterworthLowPassFilter::advance]";
    m_pimpl->isOutputValid = false;

    if (m_pimpl->inputBuffer.size() != m_pimpl->order + 1)
    {
        log()->error("{} The input buffer has a wrong size.", logPrefix);
        return false;
    }

    if (m_pimpl->outputBuffer.size() != m_pimpl->order)
    {
        log()->error("{} The output buffer has a wrong size.", logPrefix);
        return false;
    }

    m_pimpl->output = Eigen::VectorXd::Zero(m_pimpl->inputBuffer.front().size());
    for (int i = 0; i < m_pimpl->order + 1; i++)
    {
        m_pimpl->output += m_pimpl->numerator[i] * m_pimpl->inputBuffer[i];
    }

    for (int i = 0; i < m_pimpl->outputBuffer.size(); i++)
    {
        m_pimpl->output -= m_pimpl->denominator[i + 1] * m_pimpl->outputBuffer[i];
    }

    m_pimpl->output /= m_pimpl->denominator[0];

    m_pimpl->outputBuffer.push_front(m_pimpl->output);
    m_pimpl->outputBuffer.pop_back();

    m_pimpl->isOutputValid = true;

    return true;
}

const Eigen::VectorXd& ButterworthLowPassFilter::getOutput() const
{
    return m_pimpl->output;
}

bool ButterworthLowPassFilter::isOutputValid() const
{
    return m_pimpl->isOutputValid;
}
