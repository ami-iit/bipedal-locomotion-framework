/**
 * @file PINNFrictionEstimator.cpp
 * @authors Ines Sorrentino
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <memory>
#include <deque>
#include <string>
#include <cmath>

#include <Eigen/Dense>

// onnxruntime
#include <onnxruntime_cxx_api.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/VariablesHandler.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <BipedalLocomotion/PINNFrictionEstimator.h>


struct PINNFrictionEstimator::Impl
{
    Ort::Env env;
    std::unique_ptr<Ort::Session> session;
    Ort::MemoryInfo memoryInfo;

    std::deque<float> jointVelocityBuffer;
    std::deque<float> motorVelocityBuffer;
    double inputMotorTemperature;
    bool alsoMotorTemperature = false;

    size_t historyLength;

    struct DataStructured
    {
        std::vector<float> rawData;

        Ort::Value tensor{nullptr};
        std::array<int64_t, 2> shape;
    };

    DataStructured structuredInput;
    DataStructured structuredOutput;

    Impl()
        : memoryInfo(::Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU))
    {
    }
};

PINNFrictionEstimator::PINNFrictionEstimator()
{
    m_pimpl = std::make_unique<PINNFrictionEstimator::Impl>();
}

PINNFrictionEstimator::~PINNFrictionEstimator() = default;

bool PINNFrictionEstimator::initialize(const std::string& networkModelPath,
                                       const std::size_t intraOpNumThreads,
                                       const std::size_t interOpNumThreads)
{
    std::basic_string<ORTCHAR_T> networkModelPathAsOrtString(networkModelPath.begin(),
                                                             networkModelPath.end());

    Ort::SessionOptions sessionOptions;

	// Set the number of intra-op threads
	if (intraOpNumThreads > 0)
    {
        sessionOptions.SetIntraOpNumThreads(intraOpNumThreads);
    }
	// Set the number of inter-op threads
	if (interOpNumThreads > 0)
    {
        sessionOptions.SetInterOpNumThreads(interOpNumThreads);
    }
	m_pimpl->session = std::make_unique<Ort::Session>(m_pimpl->env,
                                                      networkModelPathAsOrtString.c_str(),
                                                      sessionOptions);

    if (m_pimpl->session == nullptr)
    {
        BipedalLocomotion::log()->error("Unable to load the model from the file: {}", networkModelPath);
        return false;
    }

    // Get model input size
    std::vector<int64_t> inputShape = m_pimpl->session->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();

    // We need to understand if the model takes as input also the motor temperature
    // To do this, since the model eventually takes only one sample of motor temperature
    // we can check if the size of the input is divisible by 2:
    // if it is divisible by 2, the model does not takes as input the motor temperature
    // instead, if it is not divisible by 2, the model takes as input the motor temperature
    // and we need to remove the last element of the input to compute the historyLength correctly
    const std::size_t inputCount = inputShape[1];

    int numberOfInputs = 2;

    if (inputCount % numberOfInputs == 0)
    {
        // No need to remove any element, so historyLength is just inputCount / 2
        m_pimpl->historyLength = inputCount / numberOfInputs;
    }else
    {
        // If not divisible by 2, remove one element (assumed to be motor temperature) and calculate historyLength
        m_pimpl->alsoMotorTemperature = true;
        m_pimpl->historyLength = (inputCount - 1) / numberOfInputs;
    }

    // format the input
    m_pimpl->structuredInput.rawData.resize(inputCount);
    m_pimpl->structuredInput.shape[0] = 1; // batch
    m_pimpl->structuredInput.shape[1] = inputCount;

    m_pimpl->jointVelocityBuffer.resize(m_pimpl->historyLength);
    m_pimpl->motorVelocityBuffer.resize(m_pimpl->historyLength);

    // create tensor required by onnx
    m_pimpl->structuredInput.tensor
        = Ort::Value::CreateTensor<float>(m_pimpl->memoryInfo,
                                          m_pimpl->structuredInput.rawData.data(),
                                          m_pimpl->structuredInput.rawData.size(),
                                          m_pimpl->structuredInput.shape.data(),
                                          m_pimpl->structuredInput.shape.size());

    // format the output
    const std::size_t outputSize = 1;

    // resize the output
    m_pimpl->structuredOutput.rawData.resize(outputSize);
    m_pimpl->structuredOutput.shape[0] = 1; // batch
    m_pimpl->structuredOutput.shape[1] = outputSize;

    // create tensor required by onnx
    m_pimpl->structuredOutput.tensor
        = Ort::Value::CreateTensor<float>(m_pimpl->memoryInfo,
                                          m_pimpl->structuredOutput.rawData.data(),
                                          m_pimpl->structuredOutput.rawData.size(),
                                          m_pimpl->structuredOutput.shape.data(),
                                          m_pimpl->structuredOutput.shape.size());

    return true;
}

void PINNFrictionEstimator::resetEstimator()
{
    m_pimpl->motorVelocityBuffer.clear();
    m_pimpl->jointVelocityBuffer.clear();
    m_pimpl->inputMotorTemperature = 0.0;
}

bool PINNFrictionEstimator::estimate(double inputMotorVelocity,
                                     double inputJointVelocity,
                                     double inputMotorTemperature,
                                     double& output)
{
    if (m_pimpl->motorVelocityBuffer.size() == m_pimpl->historyLength)
    {
        // The buffer is full, remove the oldest element
        m_pimpl->motorVelocityBuffer.pop_front();
        m_pimpl->jointVelocityBuffer.pop_front();
    }

    // Push element into the queue
    m_pimpl->motorVelocityBuffer.push_back(inputMotorVelocity);
    m_pimpl->jointVelocityBuffer.push_back(inputJointVelocity);

    // Check if the buffer is full
    if (m_pimpl->motorVelocityBuffer.size() < m_pimpl->historyLength)
    {
        // The buffer is not full yet
        return false;
    }

    // Fill the input
    // Copy the joint positions and then the motor positions in the
    // structured input without emptying the buffer
    // Use iterators to copy the data to the vector
    std::size_t index = 0;
    std::copy(m_pimpl->motorVelocityBuffer.cbegin(),
              m_pimpl->motorVelocityBuffer.cend(),
              m_pimpl->structuredInput.rawData.begin() + index);
    index += m_pimpl->historyLength;
    std::copy(m_pimpl->jointVelocityBuffer.cbegin(),
              m_pimpl->jointVelocityBuffer.cend(),
              m_pimpl->structuredInput.rawData.begin() + index);
    index += m_pimpl->historyLength;
    if (m_pimpl->alsoMotorTemperature)
    {
        m_pimpl->structuredInput.rawData[index] = static_cast<float>(m_pimpl->inputMotorTemperature);
        index += 1;

    }

    // perform the inference
    const char* inputNames[] = {"input"};
    const char* outputNames[] = {"output"};

    try
    {
        m_pimpl->session->Run(Ort::RunOptions(),
                            inputNames,
                            &(m_pimpl->structuredInput.tensor),
                            1,
                            outputNames,
                            &(m_pimpl->structuredOutput.tensor),
                            1);

    } catch (const Ort::Exception& e) {
        BipedalLocomotion::log()->error("Error during the inference: {}", e.what());
        return false;
    }

    // copy the output
    output = static_cast<double>(m_pimpl->structuredOutput.rawData[0]);

    return true;
}

