/**
 * @file MANN.cpp
 * @authors Paolo Maria Viceconte, Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <memory>
#include <string>

#include <Eigen/Dense>

// onnxruntime
#include <onnxruntime_cxx_api.h>

#include <BipedalLocomotion/ML/MANN.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/VariablesHandler.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion::ML;
using namespace BipedalLocomotion;

MANNInput MANNInput::generateDummyMANNInput(Eigen::Ref<const Eigen::VectorXd> jointPositions,
                                            std::size_t projectedBaseHorizon)
{
    MANNInput input;
    input.jointPositions = jointPositions;
    input.jointVelocities = Eigen::VectorXd::Zero(jointPositions.size());
    input.basePositionTrajectory = Eigen::Matrix2Xd::Zero(2, projectedBaseHorizon);
    input.baseVelocitiesTrajectory = Eigen::Matrix2Xd::Zero(2, projectedBaseHorizon);
    input.facingDirectionTrajectory = Eigen::Matrix2Xd::Zero(2, projectedBaseHorizon);
    input.facingDirectionTrajectory.row(0).setOnes();

    return input;
}

MANNOutput MANNOutput::generateDummyMANNOutput(Eigen::Ref<const Eigen::VectorXd> jointPositions,
                                               std::size_t futureProjectedBaseHorizon)
{
    MANNOutput output;
    output.jointPositions = jointPositions;
    output.jointVelocities = Eigen::VectorXd::Zero(jointPositions.size());
    output.futureBasePositionTrajectory = Eigen::Matrix2Xd::Zero(2, futureProjectedBaseHorizon);
    output.futureBaseVelocitiesTrajectory = Eigen::Matrix2Xd::Zero(2, futureProjectedBaseHorizon);
    output.futureFacingDirectionTrajectory = Eigen::Matrix2Xd::Zero(2, futureProjectedBaseHorizon);
    output.futureFacingDirectionTrajectory.row(0).setOnes();

    return output;
}

struct MANN::Impl
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

    MANNOutput output;

    Ort::MemoryInfo memoryInfo;

    Ort::Env env;
    std::unique_ptr<Ort::Session> session;

    Impl();
    bool populateInput(const MANNInput& input);

    FSM state{FSM::NotInitialized};
};

MANN::Impl::Impl()
    : memoryInfo(::Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU))
{
}

bool MANN::Impl::populateInput(const MANNInput& input)
{
    constexpr auto logPrefix = "[MANN::Impl::populateInput]";

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
                                    Eigen::Ref<const Eigen::Matrix2Xd> data) -> bool {
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

        // Eigen considers the matrix stored in column wise form
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
    ok = ok && populateProjectedData("base_positions", input.basePositionTrajectory);
    ok = ok && populateProjectedData("base_velocities", input.baseVelocitiesTrajectory);
    ok = ok && populateProjectedData("facing_directions", input.facingDirectionTrajectory);

    return ok;
}

MANN::MANN()
{
    m_pimpl = std::make_unique<MANN::Impl>();
}

MANN::~MANN() = default;

bool MANN::initialize(
    std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
{
    constexpr auto logPrefix = "[MANN::initialize]";
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
    const std::size_t inputSize = 2 * projectedBaseDatapoints // position of the base on x and y
                                                              // coordinate in the horizon
                                  + 2 * projectedBaseDatapoints // facing direction on x and y
                                                                // coordinate in the horizon (equal
                                                                // to projectedBaseHorizon)
                                  + 2 * projectedBaseDatapoints // velocity of the base on x and y
                                                                // coordinate in the horizon
                                  + numberOfJoints // joints positions
                                  + numberOfJoints; // joints velocities

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
    m_pimpl->structuredInput.handler.addVariable("base_positions", 2 * projectedBaseDatapoints);
    m_pimpl->structuredInput.handler.addVariable("facing_directions", 2 * projectedBaseDatapoints);
    m_pimpl->structuredInput.handler.addVariable("base_velocities", 2 * projectedBaseDatapoints);
    m_pimpl->structuredInput.handler.addVariable("joint_positions", numberOfJoints);
    m_pimpl->structuredInput.handler.addVariable("joint_velocities", numberOfJoints);

    // populate the output
    const std::size_t outputSize = 2 * projectedBaseDatapoints / 2 // position of the base on x and y
                                                                // coordinate in the future horizon
                                   + 2 * projectedBaseDatapoints / 2 // facing direction on x and y
                                                                  // coordinate in the future
                                                                  // horizon (equal to
                                                                  // projectedBaseHorizon)
                                   + 2 * projectedBaseDatapoints / 2 // velocity of the base on x and y
                                                                  // coordinate in the future
                                                                  // horizon
                                   + numberOfJoints // joints positions
                                   + numberOfJoints // joints velocities
                                   + 3; // x y and omega component of the velocity

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
    m_pimpl->structuredOutput.handler.addVariable("future_base_positions",
                                                  2 * projectedBaseDatapoints / 2);
    m_pimpl->structuredOutput.handler.addVariable("future_facing_directions",
                                                  2 * projectedBaseDatapoints / 2);
    m_pimpl->structuredOutput.handler.addVariable("future_base_velocities",
                                                  2 * projectedBaseDatapoints / 2);
    m_pimpl->structuredOutput.handler.addVariable("joint_positions", numberOfJoints);
    m_pimpl->structuredOutput.handler.addVariable("joint_velocities", numberOfJoints);
    m_pimpl->structuredOutput.handler.addVariable("base_velocity", 3);

    // resize the output
    m_pimpl->output.futureBasePositionTrajectory.resize(2, projectedBaseDatapoints / 2);
    m_pimpl->output.futureFacingDirectionTrajectory.resize(2, projectedBaseDatapoints / 2);
    m_pimpl->output.futureBaseVelocitiesTrajectory.resize(2, projectedBaseDatapoints / 2);
    m_pimpl->output.jointPositions.resize(numberOfJoints);
    m_pimpl->output.jointVelocities.resize(numberOfJoints);
    m_pimpl->output.projectedBaseVelocity = manif::SE2Tangentd::Zero();

    m_pimpl->state = Impl::FSM::Initialized;

    return true;
}

bool MANN::setInput(const MANNInput& input)
{
    if (m_pimpl->state != Impl::FSM::Initialized && m_pimpl->state != Impl::FSM::Running)
    {
        log()->error("[MANN::setInput] The network is not initialized, please call initialize()");
        return false;
    }
    return m_pimpl->populateInput(input);
}

bool MANN::advance()
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

    unpackMatrix("future_base_positions", m_pimpl->output.futureBasePositionTrajectory);
    unpackMatrix("future_facing_directions", m_pimpl->output.futureFacingDirectionTrajectory);
    unpackMatrix("future_base_velocities", m_pimpl->output.futureBaseVelocitiesTrajectory);
    unpackMatrix("joint_positions", m_pimpl->output.jointPositions);
    unpackMatrix("joint_velocities", m_pimpl->output.jointVelocities);

    Eigen::Vector3d tempVector;
    unpackMatrix("base_velocity", tempVector);
    m_pimpl->output.projectedBaseVelocity
        = manif::SE2Tangentd(tempVector(0), tempVector(1), tempVector(2));

    m_pimpl->state = Impl::FSM::Running;

    return true;
}

bool MANN::isOutputValid() const
{
    return m_pimpl->state == Impl::FSM::Running;
}

const MANNOutput& MANN::getOutput() const
{
    return m_pimpl->output;
}
