/**
 * @file Visualizer.h
 * @authors Stefano Dafarra Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTROLLERS_SIMULATOR_VISUALIZER_H
#define BIPEDAL_LOCOMOTION_CONTROLLERS_SIMULATOR_VISUALIZER_H

#include <memory>

#include <iDynTree/Core/Direction.h>
#include <iDynTree/Core/Position.h>
#include <iDynTree/Model/Model.h>

namespace BipedalLocomotionControllers
{
namespace Simulator
{
/**
 * Handler for the iDynTree visualizer
 */
class Visualizer
{
    struct Impl;
    std::unique_ptr<Impl> m_pimpl; /**< Private implementation */

public:
    /**
     * Constructor
     */
    Visualizer();

    /**
     * Destructor
     */
    ~Visualizer();

    /**
     * Add a new model to the visualizer
     * @param model reference to the model
     * @param modelName name of the model
     * @return true/false in case of success/failure
     */
    bool addModel(const iDynTree::Model& model, const std::string& modelName);

    /**
     * Visualize the state
     * @param world_T_Base homogeneous transformation of the base w.r.t. the inertial frame
     * @param jointPosition position of the joints
     * @param contactWrenches vector containing the contact wrenches. A contact wrench is defined by
     * an homogeneous transformation and a wrench
     * @return true/false in case of success/failure
     */
    bool visualizeState(const iDynTree::Transform& world_T_Base,
                        const iDynTree::VectorDynSize& jointsPosition,
                        const std::vector<std::pair<iDynTree::Transform, iDynTree::Wrench>>& contactWrenches);

    /**
     * Set the position of the camera
     * @param cameraPosition position of the camera
     * @return true/false in case of success/failure
     */
    bool setCameraPosition(const iDynTree::Position& cameraPosition);

    /**
     * Set the target of the camera
     * @param cameraTarget position of the target
     * @return true/false in case of success/failure
     */
    bool setCameraTarget(const iDynTree::Position& cameraTarget);

    /**
     * Set the light direction
     * @param lightDirection direction of the light
     * @return true/false in case of success/failure
     */
    bool setLightDirection(const iDynTree::Direction& lightDirection);

    /**
     * Salve a frame as png image
     * @return true/false in case of success/failure
     */
    bool saveFrame();
};
} // namespace Simulator
} // namespace BipedalLocomotionControllers

#endif // BIPEDAL_LOCOMOTION_CONTROLLERS_SIMULATOR_VISUALIZER_H
