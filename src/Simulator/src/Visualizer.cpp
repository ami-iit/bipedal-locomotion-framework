/**
 * @file Visualizer.h
 * @authors Stefano Dafarra Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

// std
#include <cmath>
#include <iostream>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Visualizer.h>

#include <BipedalLocomotionControllers/Simulator/Visualizer.h>

using namespace BipedalLocomotionControllers::Simulator;

struct Visualizer::Impl
{
    iDynTree::Visualizer viz;
    iDynTree::Position defaultCameraPosition, defaultCameraTarget;
    double scaling = 0.4;
    bool followModel = true;

    size_t indexFrame{0};
};

Visualizer::Visualizer()
    : m_pimpl(std::make_unique<Impl>())
{
    iDynTree::VisualizerOptions options;
    options.winWidth = 1366;
    options.winHeight = 768;
    m_pimpl->viz.init(options);
    m_pimpl->defaultCameraPosition = iDynTree::Position(1.6, 0.8, 0.8);
    m_pimpl->defaultCameraTarget = iDynTree::Position(0.4, 0.0, 0.5);
    m_pimpl->viz.camera().setPosition(m_pimpl->defaultCameraPosition);
    m_pimpl->viz.camera().setTarget(m_pimpl->defaultCameraTarget);
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
                                const iDynTree::VectorDynSize& jointsPosition,
                                const std::vector<std::pair<iDynTree::Transform, iDynTree::Wrench>>& contactWrenches)
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

    iDynTree::IVectorsVisualization& forcesViz = m_pimpl->viz.vectors();
    std::size_t vectorIndex = 0;
    for (const auto& contatWrench : contactWrenches)
    {
        iDynTree::Vector3 scaledForces;
        iDynTree::toEigen(scaledForces)
            = m_pimpl->scaling * iDynTree::toEigen(contatWrench.second.getLinearVec3());

        if (vectorIndex < forcesViz.getNrOfVectors())
            forcesViz.updateVector(vectorIndex, contatWrench.first.getPosition(), scaledForces);
        else
            forcesViz.addVector(contatWrench.first.getPosition(), scaledForces);
        vectorIndex++;
    }

    m_pimpl->viz.draw();

    return true;
}

bool Visualizer::setCameraPosition(const iDynTree::Position& cameraPosition)
{
    m_pimpl->viz.camera().setPosition(cameraPosition);
    return true;
}

bool Visualizer::setCameraTarget(const iDynTree::Position& cameraTarget)
{
    m_pimpl->viz.camera().setTarget(cameraTarget);
    if (m_pimpl->followModel)
    {
        iDynTree::Position newCameraPosition = m_pimpl->defaultCameraPosition;
        newCameraPosition(0)
            = cameraTarget(0) + m_pimpl->defaultCameraPosition(0) - m_pimpl->defaultCameraTarget(0);
        newCameraPosition(1)
            = cameraTarget(1) + m_pimpl->defaultCameraPosition(1) - m_pimpl->defaultCameraTarget(1);

        m_pimpl->viz.camera().setPosition(newCameraPosition);
    }
    return true;
}

bool Visualizer::setLightDirection(const iDynTree::Direction& lightDirection)
{
    m_pimpl->viz.enviroment().lightViz("sun").setDirection(lightDirection);

    return true;
}

bool Visualizer::saveFrame()
{
    if (!(m_pimpl->viz.getNrOfVisualizedModels()))
    {
        std::cerr << "[Visualizer::saveFrame] First you have to load a model." << std::endl;
        return false;
    }

    bool ok = m_pimpl->viz.drawToFile("./img/simulation_img_" + std::to_string(m_pimpl->indexFrame)
                                      + ".png");

    m_pimpl->indexFrame++;
    return ok;
}
