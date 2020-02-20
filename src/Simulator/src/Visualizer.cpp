/**
 * @file Visualizer.h
 * @authors Stefano Dafarra Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

// std
#include <cmath>
#include <iostream>

#include <iDynTree/Visualizer.h>

#include <BipedalLocomotionControllers/Simulator/Visualizer.h>

using namespace BipedalLocomotionControllers::Simulator;

struct Visualizer::Impl
{
    iDynTree::Visualizer viz;
    iDynTree::Position defaultCameraPosition, defaultCameraTarget;
};

Visualizer::Visualizer()
    : m_pimpl(std::make_unique<Impl>())
{
    iDynTree::VisualizerOptions options;
    options.winWidth = 800;
    options.winHeight = 600;
    m_pimpl->viz.init(options);
    setCameraPosition(iDynTree::Position(2.0, 0.0, 0.5));
    setCameraTarget(iDynTree::Position(0.4, 0.0, 0.5));
    double sqrt2 = std::sqrt(2.0);
    setLightDirection(iDynTree::Direction(-0.5 / sqrt2, 0, -0.5 / sqrt2));
    m_pimpl->viz.vectors().setVectorsAspect(0.01, 0.0, 0.01);
}

bool Visualizer::addModel(const iDynTree::Model& model, const std::string& modelName)
{
    return m_pimpl->viz.addModel(model, modelName);
}

Visualizer::~Visualizer()
{
    m_pimpl->viz.close();
}

bool Visualizer::visualizeState(const iDynTree::Transform& world_H_base,
                                const iDynTree::VectorDynSize& jointsPosition)
{
    if (m_pimpl->viz.getNrOfVisualizedModels() == 0)
    {
        std::cerr << "[Visualizer::visualizeState] First you have to load a model." << std::endl;
        return false;
    }

    if (!m_pimpl->viz.modelViz(0).setPositions(world_H_base, jointsPosition))
    {
        std::cerr << "[Visualizer::visualizeState] Unable to set the position." << std::endl;
        return false;
    }

    m_pimpl->viz.draw();

    return true;
}

bool Visualizer::setCameraPosition(const iDynTree::Position& cameraPosition)
{
    m_pimpl->viz.camera().setPosition(cameraPosition);
    m_pimpl->defaultCameraPosition = cameraPosition;
    return true;
}

bool Visualizer::setCameraTarget(const iDynTree::Position& cameraTarget)
{
    m_pimpl->viz.camera().setTarget(cameraTarget);
    m_pimpl->defaultCameraTarget = cameraTarget;
    return true;
}

bool Visualizer::setLightDirection(const iDynTree::Direction& lightDirection)
{
    m_pimpl->viz.enviroment().lightViz("sun").setDirection(lightDirection);

    return true;
}
