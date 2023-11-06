/**
 * @file RobotDynamicsEstimationTest.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators_all.hpp>

#include <iCubModels/iCubModels.h>
#include <yarp/os/ResourceFinder.h>

#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/ModelTestUtils.h>
#include <iDynTree/ModelIO/ModelLoader.h>

#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/System/VariablesHandler.h>

#include <BipedalLocomotion/RobotDynamicsEstimator/AccelerometerMeasurementDynamics.h>

using namespace BipedalLocomotion::Estimators::RobotDynamicsEstimator;
using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;