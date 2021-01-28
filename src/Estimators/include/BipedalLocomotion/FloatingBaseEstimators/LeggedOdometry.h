/**
 * @file LeggedOdometry.h
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_LEGGED_ODOMETRY_H
#define BIPEDAL_LOCOMOTION_LEGGED_ODOMETRY_H

#include <BipedalLocomotion/FloatingBaseEstimators/FloatingBaseEstimator.h>

namespace BipedalLocomotion
{
namespace Estimators
{
class LeggedOdometry : public FloatingBaseEstimator
{
public:
    /**
     * Constructor
     */
    LeggedOdometry();

    /**
     * Destructor (necessary for PIMPL idiom)
     */
    ~LeggedOdometry();

    /**
     * To prevent function hiding due to overloading of virtual methods
     */
    using FloatingBaseEstimator::resetEstimator;

    /**
     * Reset the internal state of the estimator
     * @param[in] newIMUOrientation  IMU orientation of the estimator
     * @param[in] newIMUPosition  IMU position of the estimator
     * @return True in case of success, false otherwise.
     *
     * @note reset and advance estimator to get updated estimator output
     */
    virtual bool resetEstimator(const Eigen::Quaterniond& newIMUOrientation, 
                                const Eigen::Vector3d& newIMUPosition) override;

    bool resetEstimator();

    bool resetEstimator(const std::string refFramForWorld,
                        const Eigen::Quaterniond& worldOrientationInRefFrame,
                        const Eigen::Vector3d& worldPositionInRefFrame);

    int getFixedFrameIdx();

protected:
    /**
    * These custom parameter specifications should be specified by the derived class.
    * @param[in] handler configure the custom parameters for the estimator
    * @return bool
    */
    virtual bool customInitialization(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler) override;

    /**
    * Update the base state estimate using kinematics and contact measurements
    * @param[in] meas measurements to update states
    * @param[in] dt sampling period in seconds
    * @return True in case of success, false otherwise.
    */
    virtual bool updateKinematics(FloatingBaseEstimators::Measurements& meas,
                                  const double& dt) override;


private:
    /**
    * Private implementation of the class
    */
    class Impl;
    std::unique_ptr<Impl> m_pimpl; /**< Pointer to implementation */
};

} // namespace Estimators
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_INVEKF_BASE_ESTIMATOR_H
