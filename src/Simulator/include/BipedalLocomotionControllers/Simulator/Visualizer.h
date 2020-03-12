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
class Visualizer
{
    struct Impl;
    std::unique_ptr<Impl> m_pimpl;

public:
    Visualizer();

    ~Visualizer();

    bool addModel(const iDynTree::Model& model, const std::string& modelName);

    bool visualizeState(const iDynTree::Transform& world_T_Base,
                        const iDynTree::VectorDynSize& jointsPosition,
                        const std::vector<std::pair<iDynTree::Transform, iDynTree::Wrench>>& contactWrenches);


    bool setCameraPosition(const iDynTree::Position& cameraPosition);

    bool setCameraTarget(const iDynTree::Position& cameraTarget);

    bool setLightDirection(const iDynTree::Direction& lightDirection);

    bool saveFrame();
};
} // namespace Simulator
} // namespace BipedalLocomotionControllers

#endif // BIPEDAL_LOCOMOTION_CONTROLLERS_SIMULATOR_VISUALIZER_H
