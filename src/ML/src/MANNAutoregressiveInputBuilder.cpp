/**
 * @file MANNAutoregressiveInputBuilder.cpp
 * @authors Paolo Maria Viceconte, Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/ML/MANNAutoregressiveInputBuilder.h>
#include <BipedalLocomotion/Math/QuadraticBezierCurve.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion::ML;

struct MANNAutoregressiveInputBuilder::Impl
{
    MANNDirectionalInput input;
    MANNAutoregressiveInput output;

    struct BezierCurve
    {
        Math::QuadraticBezierCurve curve;
        Eigen::Vector2d initialPoint;
        Eigen::Vector2d controlPoint;
        Eigen::Vector2d finalPoint;
    };
    BezierCurve curve;

    Eigen::Vector2d frontalDirection;

    double baseVelocityNorm;
    double ellipsoidForwardAxis;
    double ellipsoidSideAxis;
    double ellipsoidBackwardAxis;
    double ellipsoidScalingFactor;
    double maxFacingDirectionAngleForward;
    double maxFacingDirectionAngleBackward;
    double maxFacingDirectionAngleSideOppositeSign;
    double maxFacingDirectionAngleSideSameSign;
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
    evaluateMaxFacingDirectionAngle(Eigen::Ref<const Eigen::Vector2d> normalizedMotionDirection,
                                    Eigen::Ref<const Eigen::Vector2d> rawFacingDirection) const;
};

double MANNAutoregressiveInputBuilder::Impl::evaluateMaxFacingDirectionAngle(
    Eigen::Ref<const Eigen::Vector2d> normalizedMotionDirection,
    Eigen::Ref<const Eigen::Vector2d> rawFacingDirection) const
{
    if (normalizedMotionDirection[0] < this->forwardDirectionThreshold)
    {
        return maxFacingDirectionAngleBackward;
    }

    if (std::abs(normalizedMotionDirection[1]) > this->sideDirectionThreshold)
    {
        // same sign
        if (normalizedMotionDirection[1] * rawFacingDirection[1] >= 0)
        {
            return maxFacingDirectionAngleSideSameSign;
        }
        return maxFacingDirectionAngleSideOppositeSign;
    }

    return maxFacingDirectionAngleForward;
}

MANNAutoregressiveInputBuilder::MANNAutoregressiveInputBuilder()
{
    m_pimpl = std::make_unique<Impl>();
}

MANNAutoregressiveInputBuilder::~MANNAutoregressiveInputBuilder() = default;

bool MANNAutoregressiveInputBuilder::setInput(const Input& input)
{
    constexpr auto logPrefix = "[MANNAutoregressiveInputBuilder::setInput]";

    if (m_pimpl->fsm == Impl::FSM::Idle)
    {
        log()->error("{} Please initialize the class before calling setInput.", logPrefix);
        return false;
    }

    // Eigen normalized handle the case in which the input is almost zero
    m_pimpl->input.motionDirection = input.motionDirection.normalized();

    // if the motion direction is almost zero  in all the components then the facing direction
    // should point forward
    constexpr double tolerance = 1e-4;
    if ((m_pimpl->input.motionDirection.array().abs() <= tolerance).all())
    {
        m_pimpl->input.facingDirection << 1, 0;
        return true;
    }

    // if the facing direction is almost zero in all the components then it is forced to point
    // forward
    if ((input.facingDirection.array().abs() <= tolerance).all())
    {
        m_pimpl->input.facingDirection << 1, 0;
        return true;
    }

    // we check that the facing direction is between a given angle
    const double maxFacingDirectionAngle
        = m_pimpl->evaluateMaxFacingDirectionAngle(m_pimpl->input.motionDirection,
                                                   input.facingDirection);

    m_pimpl->input.facingDirection = input.facingDirection.normalized();
    const double mimAdmissibleX = std::cos(maxFacingDirectionAngle);
    if (m_pimpl->input.facingDirection[0] >= mimAdmissibleX)
    {
        return true;
    }
    m_pimpl->input.facingDirection[0] = mimAdmissibleX;
    m_pimpl->input.facingDirection[1] = m_pimpl->input.facingDirection[1] < 0
                                            ? -std::sin(maxFacingDirectionAngle)
                                            : std::sin(maxFacingDirectionAngle);

    return true;
}

const MANNAutoregressiveInput& MANNAutoregressiveInputBuilder::getOutput() const
{
    return m_pimpl->output;
}

bool MANNAutoregressiveInputBuilder::initialize(
    std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler)
{
    constexpr auto logPrefix = "[MANNAutoregressiveInputBuilder::initialize]";
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
    ok = ok && getParameter("ellipsoid_forward_axis", m_pimpl->ellipsoidForwardAxis);
    ok = ok && getParameter("ellipsoid_side_axis", m_pimpl->ellipsoidSideAxis);
    ok = ok && getParameter("ellipsoid_backward_axis", m_pimpl->ellipsoidBackwardAxis);
    ok = ok && getParameter("ellipsoid_scaling_factor", m_pimpl->ellipsoidScalingFactor);
    ok = ok
         && getParameter("max_facing_direction_angle_forward",
                         m_pimpl->maxFacingDirectionAngleForward);
    ok = ok
         && getParameter("max_facing_direction_angle_backward",
                         m_pimpl->maxFacingDirectionAngleBackward);
    ok = ok
         && getParameter("max_facing_direction_angle_side_opposite_sign",
                         m_pimpl->maxFacingDirectionAngleSideOppositeSign);
    ok = ok
         && getParameter("max_facing_direction_angle_side_same_sign",
                         m_pimpl->maxFacingDirectionAngleSideSameSign);
    ok = ok && getParameter("number_of_knots", m_pimpl->numberOfKnots);
    ok = ok && m_pimpl->curve.curve.initialize(ptr);

    // if everything went fine we can resize the matrices
    if (!ok)
    {
        log()->error("{} Unable complete the initialization of the MANNAutoregressiveInputBuilder.",
                     logPrefix);
        return false;
    }

    // resize the matrices
    m_pimpl->output.desiredFutureBaseTrajectory.resize(2, m_pimpl->numberOfKnots);
    m_pimpl->output.desiredFutureBaseVelocities.resize(2, m_pimpl->numberOfKnots);
    m_pimpl->output.desiredFutureFacingDirections.resize(2, m_pimpl->numberOfKnots);

    // set the initial points of the Bezier curve. It is constant
    m_pimpl->curve.initialPoint.setZero();
    m_pimpl->frontalDirection << 1, 0;

    getOptionalParameter("forward_direction_threshold", m_pimpl->forwardDirectionThreshold);
    getOptionalParameter("side_direction_threshold", m_pimpl->sideDirectionThreshold);

    m_pimpl->fsm = Impl::FSM::Initialized;

    return ok;
}

bool MANNAutoregressiveInputBuilder::advance()
{
    constexpr auto logPrefix = "[MANNAutoregressiveInputBuilder::advance]";
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
    // the base velocity is computed with the following step
    //
    // 1. numerically differentiate the base position
    //                 ┌                                                         ┐
    // base_velocity = │ x1 - x0, x2 - x1, x3 - x2, x4 - x3, ..., x{n-1} - xn, 0 │
    //                 │ y1 - y0, y2 - y1, y3 - y2, y4 - y3, ..., y{n-1} - yn, 0 |
    //                 └                                                         ┘
    //
    // 2. normalize the base velocity computed at the step 1 and multiply it for
    //    the base velocity norm.
    //
    // 3. copy the last but one element of base_velocity computed at the step 2 to
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
        m_pimpl->output.desiredFutureFacingDirections.col(i)
            = t * m_pimpl->frontalDirection + (1 - t) * m_pimpl->input.facingDirection;
    }

    m_pimpl->fsm = Impl::FSM::OutputValid;

    return true;
}

bool MANNAutoregressiveInputBuilder::isOutputValid() const
{
    return m_pimpl->fsm == Impl::FSM::OutputValid;
}
