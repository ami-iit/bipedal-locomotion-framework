/**
 * @file VelMANNAutoregressiveInputBuilder.cpp
 * @authors Evelyn D'Elia
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/ML/VelMANNAutoregressiveInputBuilder.h>
#include <BipedalLocomotion/Math/QuadraticBezierCurve.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion::ML;

struct VelMANNAutoregressiveInputBuilder::Impl
{
    VelMANNDirectionalInput input;
    VelMANNAutoregressiveInput output;

    struct BezierCurve
    {
        Math::QuadraticBezierCurve curve;
        Eigen::Vector2d initialPoint;
        Eigen::Vector2d controlPoint;
        Eigen::Vector2d finalPoint;
    };
    BezierCurve curve;

    Eigen::Vector2d baseDirection;

    double baseVelocityNorm;
    double baseAngVelocityNorm;
    double ellipsoidForwardAxis;
    double ellipsoidSideAxis;
    double ellipsoidBackwardAxis;
    double ellipsoidScalingFactor;
    double maxBaseDirectionAngleForward;
    double maxBaseDirectionAngleBackward;
    double maxBaseDirectionAngleSideOppositeSign;
    double maxBaseDirectionAngleSideSameSign;
    double forwardDirectionThreshold{0.2};
    double sideDirectionThreshold{0.2};
    int numberOfKnots;

    enum class FSM
    {
        Idle,
        Initialized,
        OutputValid,
        OutputInvalid,
    };

    FSM fsm{FSM::Idle};

    double
    evaluateMaxBaseDirectionAngle(Eigen::Ref<const Eigen::Vector2d> normalizedMotionDirection,
                                    Eigen::Ref<const Eigen::Vector2d> rawBaseDirection) const;
};

double VelMANNAutoregressiveInputBuilder::Impl::evaluateMaxBaseDirectionAngle(
    Eigen::Ref<const Eigen::Vector2d> normalizedMotionDirection,
    Eigen::Ref<const Eigen::Vector2d> rawBaseDirection) const
{
    if (normalizedMotionDirection[0] < this->forwardDirectionThreshold)
    {
        return maxBaseDirectionAngleBackward;
    }

    if (std::abs(normalizedMotionDirection[1]) > this->sideDirectionThreshold)
    {
        // same sign
        if (normalizedMotionDirection[1] * rawBaseDirection[1] >= 0)
        {
            return maxBaseDirectionAngleSideSameSign;
        }
        return maxBaseDirectionAngleSideOppositeSign;
    }

    return maxBaseDirectionAngleForward;
}

VelMANNAutoregressiveInputBuilder::VelMANNAutoregressiveInputBuilder()
{
    m_pimpl = std::make_unique<Impl>();
}

VelMANNAutoregressiveInputBuilder::~VelMANNAutoregressiveInputBuilder() = default;

bool VelMANNAutoregressiveInputBuilder::setInput(const Input& input)
{
    constexpr auto logPrefix = "[VelMANNAutoregressiveInputBuilder::setInput]";

    if (m_pimpl->fsm == Impl::FSM::Idle)
    {
        log()->error("{} Please initialize the class before calling setInput.", logPrefix);
        return false;
    }

    // Eigen normalized handles the case in which the input is almost zero
    m_pimpl->input.motionDirection = input.motionDirection.normalized();

    // if all the components of the motion direction are almost zero then the base direction
    // should point forward
    constexpr double tolerance = 1e-4;
    if ((m_pimpl->input.motionDirection.array().abs() <= tolerance).all())
    {
        m_pimpl->input.baseDirection << 1, 0;
        return true;
    }

    // if the base direction is almost zero in all the components then it is forced to point
    // forward
    if ((input.baseDirection.array().abs() <= tolerance).all())
    {
        m_pimpl->input.baseDirection << 1, 0;
        return true;
    }

    // we check that the base direction is between a given angle
    const double maxBaseDirectionAngle
        = m_pimpl->evaluateMaxBaseDirectionAngle(m_pimpl->input.motionDirection,
                                                   input.baseDirection);

    m_pimpl->input.baseDirection = input.baseDirection.normalized();
    const double mimAdmissibleX = std::cos(maxBaseDirectionAngle);
    if (m_pimpl->input.baseDirection[0] >= mimAdmissibleX)
    {
        return true;
    }
    m_pimpl->input.baseDirection[0] = mimAdmissibleX;
    m_pimpl->input.baseDirection[1] = m_pimpl->input.baseDirection[1] < 0
                                            ? -std::sin(maxBaseDirectionAngle)
                                            : std::sin(maxBaseDirectionAngle);

    return true;
}

const VelMANNAutoregressiveInput& VelMANNAutoregressiveInputBuilder::getOutput() const
{
    return m_pimpl->output;
}

bool VelMANNAutoregressiveInputBuilder::initialize(
    std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler)
{
    constexpr auto logPrefix = "[VelMANNAutoregressiveInputBuilder::initialize]";
    auto ptr = paramHandler.lock();

    if (ptr == nullptr)
    {
        log()->error("{} Invalid parameter handler.", logPrefix);
        return false;
    }

    auto getParameter = [logPrefix, ptr](const std::string& paramName, auto& param) -> bool {
        if (!ptr->getParameter(paramName, param))
        {
            log()->error("{} Unable to find the parameter {}.", logPrefix, paramName);
            return false;
        }
        if (param <= 0)
        {
            log()->error("{} The parameter {} cannot be negative.", logPrefix, paramName);
            return false;
        }
        return true;
    };

    auto getOptionalParameter
        = [logPrefix, ptr](const std::string& paramName, auto& param) -> void {
        typename std::remove_reference<decltype(param)>::type temp;

        if (!ptr->getParameter(paramName, temp))
        {
            log()->info("{} Unable to find the parameter {}. The default one will be used {}.",
                        logPrefix,
                        paramName,
                        param);
            return;
        }
        if (temp <= 0)
        {
            log()->error("{} The parameter {} cannot be negative. The default one will be used {}.",
                         logPrefix,
                         paramName,
                         param);
            return;
        }
        param = temp;
        return;
    };

    // get all the required parameters
    bool ok = getParameter("base_vel_norm", m_pimpl->baseVelocityNorm);
    ok = ok && getParameter("base_ang_vel_norm", m_pimpl->baseAngVelocityNorm);
    ok = ok && getParameter("ellipsoid_forward_axis", m_pimpl->ellipsoidForwardAxis);
    ok = ok && getParameter("ellipsoid_side_axis", m_pimpl->ellipsoidSideAxis);
    ok = ok && getParameter("ellipsoid_backward_axis", m_pimpl->ellipsoidBackwardAxis);
    ok = ok && getParameter("ellipsoid_scaling_factor", m_pimpl->ellipsoidScalingFactor);
    ok = ok
         && getParameter("max_base_direction_angle_forward",
                         m_pimpl->maxBaseDirectionAngleForward);
    ok = ok
         && getParameter("max_base_direction_angle_backward",
                         m_pimpl->maxBaseDirectionAngleBackward);
    ok = ok
         && getParameter("max_base_direction_angle_side_opposite_sign",
                         m_pimpl->maxBaseDirectionAngleSideOppositeSign);
    ok = ok
         && getParameter("max_base_direction_angle_side_same_sign",
                         m_pimpl->maxBaseDirectionAngleSideSameSign);
    ok = ok && getParameter("number_of_knots", m_pimpl->numberOfKnots);
    ok = ok && m_pimpl->curve.curve.initialize(ptr);

    // if everything went fine we can resize the matrices
    if (!ok)
    {
        log()->error("{} Unable complete the initialization of the VelMANNAutoregressiveInputBuilder.",
                     logPrefix);
        return false;
    }

    // resize the matrices
    m_pimpl->output.desiredFutureBaseTrajectory.resize(2, m_pimpl->numberOfKnots);
    m_pimpl->output.desiredFutureBaseVelocities.resize(2, m_pimpl->numberOfKnots);
    m_pimpl->output.desiredFutureBaseDirections.resize(2, m_pimpl->numberOfKnots);
    m_pimpl->output.desiredFutureBaseAngVelocities.resize(m_pimpl->numberOfKnots);

    // set the initial points of the Bezier curve. It is constant
    m_pimpl->curve.initialPoint.setZero();
    m_pimpl->baseDirection << 1, 0;

    getOptionalParameter("forward_direction_threshold", m_pimpl->forwardDirectionThreshold);
    getOptionalParameter("side_direction_threshold", m_pimpl->sideDirectionThreshold);

    m_pimpl->fsm = Impl::FSM::Initialized;

    return ok;
}

bool VelMANNAutoregressiveInputBuilder::advance()
{
    constexpr auto logPrefix = "[VelMANNAutoregressiveInputBuilder::advance]";
    if (m_pimpl->fsm == Impl::FSM::Idle)
    {
        log()->error("{} Please initialize the class before calling advance.", logPrefix);
        return false;
    }

    // first of all we invalidate the output
    m_pimpl->fsm = Impl::FSM::OutputInvalid;

    // evaluate the control point and the x coordinate of the final point of the Bezier curve
    m_pimpl->curve.controlPoint[1] = 0;

    // if the robot is walking forward (motion direction positive)
    if (m_pimpl->input.motionDirection[0] >= 0)
    {
        m_pimpl->curve.controlPoint[0]
            = m_pimpl->ellipsoidScalingFactor
              * std::min(m_pimpl->ellipsoidForwardAxis, m_pimpl->input.motionDirection[0]);
        m_pimpl->curve.finalPoint[0] = m_pimpl->input.motionDirection[0]
                                       * m_pimpl->ellipsoidScalingFactor
                                       * m_pimpl->ellipsoidForwardAxis;
    }
    // if the robot is walking backward (forward direction negative)
    else
    {
        m_pimpl->curve.controlPoint[0]
            = m_pimpl->ellipsoidScalingFactor
              * std::max(-m_pimpl->ellipsoidBackwardAxis, m_pimpl->input.motionDirection[0]);
        m_pimpl->curve.finalPoint[0] = m_pimpl->input.motionDirection[0]
                                       * m_pimpl->ellipsoidScalingFactor
                                       * m_pimpl->ellipsoidBackwardAxis;
    }

    m_pimpl->curve.finalPoint[1] = m_pimpl->input.motionDirection[1]
                                   * m_pimpl->ellipsoidScalingFactor * m_pimpl->ellipsoidSideAxis;

    // compute the future trajectory of the robot base
    m_pimpl->output.desiredFutureBaseTrajectory
        = m_pimpl->curve.curve.evaluateCurve(m_pimpl->curve.initialPoint,
                                             m_pimpl->curve.controlPoint,
                                             m_pimpl->curve.finalPoint);

    // evaluate the future base velocity as numerical differentiation of the base position
    //                 ┌                         ┐
    // base_position = │ x0, x1, x2, x3, ..., xn │
    //                 │ y0, y1, y2, y3, ..., yn │
    //                 └                         ┘
    // the base velocity is computed using the following steps
    //
    // 1. numerically differentiate the base position
    //                 ┌                                                         ┐
    // base_velocity = │ x1 - x0, x2 - x1, x3 - x2, x4 - x3, ..., x{n-1} - xn, 0 │
    //                 │ y1 - y0, y2 - y1, y3 - y2, y4 - y3, ..., y{n-1} - yn, 0 |
    //                 └                                                         ┘
    //
    // 2. normalize the base velocity computed in step 1 and multiply it by
    //    the base velocity norm.
    //
    // 3. copy the second to last element of the base_velocity computed in step 2 to
    //    the last element

    // [BASE VELOCITY EVALUATION] Step 1
    m_pimpl->output.desiredFutureBaseVelocities.leftCols(m_pimpl->numberOfKnots - 1)
        = m_pimpl->output.desiredFutureBaseTrajectory.rightCols(m_pimpl->numberOfKnots - 1)
          - m_pimpl->output.desiredFutureBaseTrajectory.leftCols(m_pimpl->numberOfKnots - 1);

    // [BASE VELOCITY EVALUATION] Step 2
    for (int i = 0; i < m_pimpl->numberOfKnots - 1; i++)
    {
        m_pimpl->output.desiredFutureBaseVelocities.col(i).normalize();
        m_pimpl->output.desiredFutureBaseVelocities.col(i) *= m_pimpl->baseVelocityNorm;
    }

    // [BASE VELOCITY EVALUATION] Step 3
    m_pimpl->output.desiredFutureBaseVelocities.col(m_pimpl->numberOfKnots - 1)
        = m_pimpl->output.desiredFutureBaseVelocities.col(m_pimpl->numberOfKnots - 2);

    const double h = 1.0 / static_cast<double>(m_pimpl->numberOfKnots - 1);
    for (int i = 0; i < m_pimpl->numberOfKnots; i++)
    {
        const double t = i * h;
        m_pimpl->output.desiredFutureBaseDirections.col(i)
            = t * m_pimpl->baseDirection + (1 - t) * m_pimpl->input.baseDirection;
    }

    // evaluate the future base angular velocity as numerical differentiation of the base direction
    //                  ┌                              ┐
    // base_direction = │ r0,  r1,  r2,  r3,  ..., rn  │
    //                  │ rz0, rz1, rz2, rz3, ..., rzn │
    //                  └                              ┘
    // the base angular velocity is computed using the following steps
    //
    // 1. calculate the change in yaw angle between each successive base direction
    //         ┌                                                                                        ┐
    // theta = │ atan2(r0 \cdot r1, r0 \times r1), ..., atan2(r{n-1} \cdot r{n}, r{n-1} \times r{n}), 0 │
    //         └                                                                                        ┘
    //
    // 2. normalize the base yaw angle computed in step 1 and multiply it by
    //    the base angular velocity norm.
    //
    // 3. copy the second to last element of the base_angular_velocity computed in step 2 to
    //    the last element

    Eigen::Matrix3Xd desiredFutureBaseDirections3d = (Eigen::Matrix3Xd(m_pimpl->output.desiredFutureBaseDirections.rows() + 1, m_pimpl->output.desiredFutureBaseDirections.cols()) << m_pimpl->output.desiredFutureBaseDirections, Eigen::RowVectorXd::Zero(m_pimpl->output.desiredFutureBaseVelocities.cols())).finished();

    for (int i = 0; i < m_pimpl->numberOfKnots - 1; i++)
    {
        // [BASE ANGULAR VELOCITY EVALUATION] Step 1
        m_pimpl->output.desiredFutureBaseAngVelocities(i)
            = std::atan2((desiredFutureBaseDirections3d.col(i).cross(desiredFutureBaseDirections3d.col(i + 1)))(2),
            desiredFutureBaseDirections3d.col(i).dot(desiredFutureBaseDirections3d.col(i + 1)));
        // [BASE ANGULAR VELOCITY EVALUATION] Step 2
        m_pimpl->output.desiredFutureBaseAngVelocities.col(i).normalize();
        m_pimpl->output.desiredFutureBaseAngVelocities.col(i) *= m_pimpl->baseAngVelocityNorm;
    }

    // [BASE ANGULAR VELOCITY EVALUATION] Step 3
    m_pimpl->output.desiredFutureBaseAngVelocities.col(m_pimpl->numberOfKnots - 1)
        = m_pimpl->output.desiredFutureBaseAngVelocities.col(m_pimpl->numberOfKnots - 2);

    m_pimpl->fsm = Impl::FSM::OutputValid;

    return true;
}

bool VelMANNAutoregressiveInputBuilder::isOutputValid() const
{
    return m_pimpl->fsm == Impl::FSM::OutputValid;
}
