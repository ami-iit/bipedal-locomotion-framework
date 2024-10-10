/**
 * @file VelMANN.cpp
 * @authors Evelyn D'Elia
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <memory>
#include <string>

#include <Eigen/Dense>

// onnxruntime
#include <onnxruntime_cxx_api.h>

#include <BipedalLocomotion/ML/VelMANN.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/VariablesHandler.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion::ML;
using namespace BipedalLocomotion;

VelMANNInput VelMANNInput::generateDummyVelMANNInput(Eigen::Ref<const Eigen::VectorXd> jointPositions,
                                            std::size_t projectedBaseHorizon)
{
    VelMANNInput input;
    input.jointPositions = jointPositions;
    input.jointVelocities = Eigen::VectorXd::Zero(jointPositions.size());
    input.baseLinearVelocityTrajectory = Eigen::Matrix3Xd::Zero(3, projectedBaseHorizon);
    input.baseAngularVelocityTrajectory = Eigen::Matrix3Xd::Zero(3, projectedBaseHorizon);
    input.basePosition = Eigen::Vector3d(0.0, 0.0, 0.7748); //TODO should these have correct ht?
    input.baseAngle = Eigen::Vector3d::Zero();

    return input;
}

VelMANNOutput VelMANNOutput::generateDummyVelMANNOutput(Eigen::Ref<const Eigen::VectorXd> jointPositions,
                                               std::size_t futureProjectedBaseHorizon)
{
    VelMANNOutput output;
    output.jointPositions = jointPositions;
    output.jointVelocities = Eigen::VectorXd::Zero(jointPositions.size());
    output.futureBaseLinearVelocityTrajectory = Eigen::Matrix3Xd::Zero(3, futureProjectedBaseHorizon);
    output.futureBaseAngularVelocityTrajectory = Eigen::Matrix3Xd::Zero(3, futureProjectedBaseHorizon);
    output.basePosition = Eigen::Vector3d(0.0, 0.0, 0.7748); //TODO should these have correct ht?
    output.baseAngle = Eigen::Vector3d::Zero();

    return output;
}

struct VelMANN::Impl
{
    enum class FSM
    {
        NotInitialized,
        Initialized,
        Running,
    };

    struct DataStructured
    {
        BipedalLocomotion::System::VariablesHandler handler;
        Eigen::VectorXf rawData;

        Ort::Value tensor{nullptr};
        std::array<int64_t, 2> shape;
    };

    DataStructured structuredInput;
    DataStructured structuredOutput;

    VelMANNOutput output;

    Ort::MemoryInfo memoryInfo;

    Ort::Env env;
    std::unique_ptr<Ort::Session> session;

    Impl();
    bool populateInput(const VelMANNInput& input);

    FSM state{FSM::NotInitialized};
};

VelMANN::Impl::Impl()
    : memoryInfo(::Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU))
{
}

bool VelMANN::Impl::populateInput(const VelMANNInput& input)
{
    constexpr auto logPrefix = "[VelMANN::Impl::populateInput]";

    auto populateVectorData
        = [&input, this, logPrefix](const std::string& variableName,
                                    Eigen::Ref<const Eigen::VectorXd> data) -> bool {
        const auto& variable = this->structuredInput.handler.getVariable(variableName);
        if (data.size() != variable.size)
        {
            log()->error("{} Invalid size of the variable named '{}'. Expected size: {}, "
                         "vector size: {}.",
                         logPrefix,
                         variableName,
                         variable.size,
                         data.size());
            return false;
        }

        this->structuredInput.rawData.segment(variable.offset, variable.size) = data.cast<float>();
        return true;
    };

    auto populateProjectedData
        = [&input, this, logPrefix](const std::string& variableName,
                                    Eigen::Ref<const Eigen::Matrix3Xd> data) -> bool {
        const auto& variable = this->structuredInput.handler.getVariable(variableName);
        if (data.size() != variable.size)
        {
            log()->error("{} Invalid size of the variable named '{}'. Expected size: {}, "
                         "Passed matrix size: {} = {} x {}.",
                         logPrefix,
                         variableName,
                         variable.size,
                         data.size(),
                         data.cols(),
                         data.rows());
            return false;
        }

        // Eigen considers the matrix stored in column wise form, so:
        // [x1, x2, ..., xn;
        //  y1, y2, ..., yn]
        // becomes [x1, y1, x2, y2, ...., xn, yn]
        this->structuredInput.rawData.segment(variable.offset, variable.size)
            = Eigen::Map<const Eigen::VectorXd>(data.data(), data.cols() * data.rows())
                  .cast<float>();

        return true;
    };

    // populate input data for the network
    bool ok = populateVectorData("joint_velocities", input.jointVelocities);
    ok = ok && populateVectorData("joint_positions", input.jointPositions);
    ok = ok && populateProjectedData("base_linear_velocities", input.baseLinearVelocityTrajectory);
    ok = ok && populateProjectedData("base_angular_velocities", input.baseAngularVelocityTrajectory);
    ok = ok && populateProjectedData("base_position", input.basePosition);
    ok = ok && populateProjectedData("base_angle", input.baseAngle);

    return ok;
}

VelMANN::VelMANN()
{
    m_pimpl = std::make_unique<VelMANN::Impl>();
}

VelMANN::~VelMANN() = default;

bool VelMANN::initialize(
    std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
{
    constexpr auto logPrefix = "[VelMANN::initialize]";
    auto ptr = handler.lock();

    if (ptr == nullptr)
    {
        log()->error("{} Invalid parameter handler.", logPrefix);
        return false;
    }

    auto loadParam = [ptr, logPrefix](const std::string& paramName, auto& param) -> bool {
        if (!ptr->getParameter(paramName, param))
        {
            log()->error("{} Unable to get the parameter named '{}'.", logPrefix, paramName);
            return false;
        }
        return true;
    };

    int numberOfJoints, projectedBaseDatapoints;
    std::string networkModelPath;
    bool ok = loadParam("number_of_joints", numberOfJoints);
    ok = ok && loadParam("projected_base_datapoints", projectedBaseDatapoints);
    ok = ok && loadParam("onnx_model_path", networkModelPath);

    if (projectedBaseDatapoints % 2 != 0)
    {
        log()->error("{} Project base horizon must be an even number. This is required by mann.",
                     logPrefix);
        return false;
    }

    if (!ok)
    {
        return false;
    }

    // Ort::Session's constructor is OS-dependent, wants wchar_t* on Windows and char* on other OSs
    // Note: this only works with single-byte characters, such as ASCII or ISO-8859-1,
    // see
    // https://stackoverflow.com/questions/2573834/c-convert-string-or-char-to-wstring-or-wchar-t
    std::basic_string<ORTCHAR_T> networkModelPathAsOrtString(networkModelPath.begin(),
                                                             networkModelPath.end());

    // As explained in https://onnxruntime.ai/docs/performance/tune-performance/threading.html,
    // onnxruntime uses intra-op parallelism to speed up the inference. The number of threads can be
    // set by the user. The default value is the number of available cores on the machine.
    int numberOfIntraThreads{0};
    if (!ptr->getParameter("number_of_threads", numberOfIntraThreads))
    {
        log()->info("{} The number of threads has not been set. The default value will be used. "
                    "The model will use the number of available cores. Please check "
                    "https://onnxruntime.ai/docs/performance/tune-performance/threading.html for "
                    "more information.",
                    logPrefix);
    }

    if (numberOfIntraThreads > 0)
    {
        Ort::SessionOptions sessionOptions;
        sessionOptions.SetIntraOpNumThreads(numberOfIntraThreads);
        m_pimpl->session = std::make_unique<Ort::Session>(m_pimpl->env,
                                                          networkModelPathAsOrtString.c_str(),
                                                          sessionOptions);
    } else
    {
        m_pimpl->session = std::make_unique<Ort::Session>(m_pimpl->env,
                                                          networkModelPathAsOrtString.c_str(),
                                                          Ort::SessionOptions{nullptr});
    }

    if (m_pimpl->session == nullptr)
    {
        log()->error("{} Unable to instantiate the model in: '{}'.", logPrefix, networkModelPath);
        return false;
    }

    // the input of the network is composed by
    const std::size_t inputSize = 3 * projectedBaseDatapoints // linear velocity of the base in xyz
                                                              // coordinates in the horizon
                                  + 3 * projectedBaseDatapoints // angular velocity of the base in xyz
                                                              // coordinates in the horizon
                                  + numberOfJoints // joint positions
                                  + numberOfJoints // joint velocities
                                  + 6; //base position and euler angles

    // resize the input
    m_pimpl->structuredInput.rawData.resize(inputSize);
    m_pimpl->structuredInput.shape[0] = 1; // batch
    m_pimpl->structuredInput.shape[1] = inputSize;

    // create tensor required by onnx
    m_pimpl->structuredInput.tensor
        = Ort::Value::CreateTensor<float>(m_pimpl->memoryInfo,
                                          m_pimpl->structuredInput.rawData.data(),
                                          m_pimpl->structuredInput.rawData.size(),
                                          m_pimpl->structuredInput.shape.data(),
                                          m_pimpl->structuredInput.shape.size());

    // populate variable handler related to the input
    // the serialization matters
    m_pimpl->structuredInput.handler.addVariable("base_linear_velocities", 3 * projectedBaseDatapoints);
    m_pimpl->structuredInput.handler.addVariable("base_angular_velocities", 3 * projectedBaseDatapoints);
    m_pimpl->structuredInput.handler.addVariable("joint_positions", numberOfJoints);
    m_pimpl->structuredInput.handler.addVariable("joint_velocities", numberOfJoints);
    m_pimpl->structuredInput.handler.addVariable("base_position", 3);
    m_pimpl->structuredInput.handler.addVariable("base_angle", 3);

    // populate the output
    const std::size_t outputSize = 3 * (1 + projectedBaseDatapoints / 2) // linear velocity of the base in xyz
                                                              // coordinates in the future horizon incl. current
                                   + 3 * (1 + projectedBaseDatapoints / 2) // linear velocity of the base in xyz
                                                              // coordinates in the future horizon incl. current
                                   + numberOfJoints // joint positions
                                   + numberOfJoints // joint velocities
                                   + 6; //base position and euler angles

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
    // populate variable handler related to the output
    // the serialization matters
    m_pimpl->structuredOutput.handler.addVariable("future_base_linear_velocities",
                                                  3 * (1 + projectedBaseDatapoints / 2));
    m_pimpl->structuredOutput.handler.addVariable("future_base_angular_velocities",
                                                  3 * (1 + projectedBaseDatapoints / 2));
    m_pimpl->structuredOutput.handler.addVariable("joint_positions", numberOfJoints);
    m_pimpl->structuredOutput.handler.addVariable("joint_velocities", numberOfJoints);
    m_pimpl->structuredOutput.handler.addVariable("base_position", 3);
    m_pimpl->structuredOutput.handler.addVariable("base_angle", 3);

    // resize the output
    m_pimpl->output.futureBaseLinearVelocityTrajectory.resize(3, (1 + projectedBaseDatapoints / 2));
    m_pimpl->output.futureBaseAngularVelocityTrajectory.resize(3, (1 + projectedBaseDatapoints / 2));
    m_pimpl->output.jointPositions.resize(numberOfJoints);
    m_pimpl->output.jointVelocities.resize(numberOfJoints);
    m_pimpl->output.basePosition.resize(3);
    m_pimpl->output.baseAngle.resize(3);

    m_pimpl->state = Impl::FSM::Initialized;

    return true;
}

bool VelMANN::setInput(const VelMANNInput& input)
{
    if (m_pimpl->state != Impl::FSM::Initialized && m_pimpl->state != Impl::FSM::Running)
    {
        log()->error("[VelMANN::setInput] The network is not initialized, please call initialize()");
        return false;
    }
    return m_pimpl->populateInput(input);
}

bool VelMANN::advance()
{
    auto unpackMatrix = [this](const std::string& variableName,
                               Eigen::Ref<Eigen::MatrixXd> matrix) {
        const auto& variable = m_pimpl->structuredOutput.handler.getVariable(variableName);
        assert(variable.isValid());

        // the matrix has been already allocated
        const std::size_t rows = matrix.rows();
        const std::size_t cols = matrix.cols();

        matrix = Eigen::Map<const Eigen::MatrixXf>(m_pimpl->structuredOutput.rawData
                                                       .segment(variable.offset, variable.size)
                                                       .data(),
                                                   rows,
                                                   cols)
                     .cast<double>();

        return;
    };

    const char* inputNames[] = {"input"};
    const char* outputNames[] = {"output"};

    m_pimpl->session->Run(Ort::RunOptions(),
                          inputNames,
                          &(m_pimpl->structuredInput.tensor),
                          1,
                          outputNames,
                          &(m_pimpl->structuredOutput.tensor),
                          1);

    unpackMatrix("future_base_linear_velocities", m_pimpl->output.futureBaseLinearVelocityTrajectory);
    unpackMatrix("future_base_angular_velocities", m_pimpl->output.futureBaseAngularVelocityTrajectory);
    unpackMatrix("joint_positions", m_pimpl->output.jointPositions);
    unpackMatrix("joint_velocities", m_pimpl->output.jointVelocities);
    unpackMatrix("base_position", m_pimpl->output.basePosition);
    unpackMatrix("base_angle", m_pimpl->output.baseAngle);

    m_pimpl->state = Impl::FSM::Running;

    return true;
}

bool VelMANN::isOutputValid() const
{
    return m_pimpl->state == Impl::FSM::Running;
}

const VelMANNOutput& VelMANN::getOutput() const
{
    return m_pimpl->output;
}
