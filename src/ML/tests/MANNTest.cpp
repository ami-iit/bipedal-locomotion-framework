/**
 * @file MANNTest.cpp
 * @authors Paolo Maria Viceconte, Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <memory>

// Catch2
#include <catch2/catch_test_macros.hpp>

#include <BipedalLocomotion/ML/MANN.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>

#include <MANNModelFolderPath.h>

using namespace BipedalLocomotion::ML;
using namespace BipedalLocomotion::ParametersHandler;

TEST_CASE("MANN")
{
    MANN mann;

    auto handler = std::make_shared<StdImplementation>();
    handler->setParameter("number_of_joints", 26);
    handler->setParameter("projected_base_horizon", 12);
    handler->setParameter("onnx_model_path", getMANNModelPath());

    REQUIRE(mann.initialize(handler));

    // set the input. The values are taken from a python test
    MANNInput input;
    input.basePositionTrajectory.resize(2, 12);
    input.facingDirectionTrajectory.resize(2, 12);
    input.baseVelocitiesTrajectory.resize(2, 12);
    input.jointPositions.resize(26);
    input.jointVelocities.resize(26);

    std::array<double, 12 * 2> basePositionTemp{0.020465626137500123, -0.028526278638462008,
                                                0.0204657739768999, -0.02852627656493037,
                                                0.020465822166082566, -0.028526210469724575,
                                                0.020465802925311055, -0.028526161526930927,
                                                0.020465772830464538, -0.02852614179540976,
                                                0.0, 0.0,
                                                -0.009585547395551581, -0.02175600333393604,
                                                -0.017817201285188853, -0.0332971138031881,
                                                -0.02033611982326177, -0.030715595478538575,
                                                -0.016440085949668892, -0.01989833515449035,
                                                -0.008254458531998493, -0.008547476348220699,
                                                0.0, 0.0};

    input.basePositionTrajectory = Eigen::Map<Eigen::MatrixXd>(basePositionTemp.data(), 2, 12);

    std::array<double, 12 * 2> facingDirectionTemp{0.9998046173508225, 0.019766818762118485,
                                                   0.9998046180476308, 0.019766783517618902,
                                                   0.9998046178527036, 0.01976679337697099,
                                                   0.9998046175664108, 0.01976680785767637,
                                                   0.9998046173199947, 0.019766820321406378,
                                                   1.0, 0.0,
                                                   1.000195227589745, -0.0019179446719683577,
                                                   1.0010723856647088, -0.002920366320472865,
                                                   1.0019448535892073, -0.006219539528750589,
                                                   1.00220806134242, -0.005959674409629769,
                                                   1.0015735374358077, -0.0019812837541796284,
                                                   1.0, 0.0};
    input.facingDirectionTrajectory
        = Eigen::Map<Eigen::MatrixXd>(facingDirectionTemp.data(), 2, 12);

    std::array<double, 12 * 2> baseVelocitiesTemp{0.02856285193696938, 0.019224769398957234,
                                                  0.028562823489490678, 0.019225725173215375,
                                                  0.02856269486540722, 0.01922613391891199,
                                                  0.028562619171457148, 0.01922618174545223,
                                                  0.02856256157025901, 0.019226074516058275,
                                                  -0.03709010698840217, -0.08405826644833612,
                                                  -0.03298855398555943, -0.09948148489499548,
                                                  -0.022790401966434587, -0.03630620503764005,
                                                  -0.006813425765586863, -0.005878922358647386,
                                                  0.00036041265075565515, 0.011310742547698205,
                                                  0.007901453422695888, 0.0018729868979242416,
                                                  0.0, 1.3552527156068805e-20};
    input.baseVelocitiesTrajectory = Eigen::Map<Eigen::MatrixXd>(baseVelocitiesTemp.data(), 2, 12);

    input.jointPositions << 0.14304762524836648, 0.031516238814005136, -0.1426330272419353, -0.06098283408307653,
        -0.1341771055787397, 0.19911560020803554, 0.0328098631623236, 0.024781975683781033,
        0.01324623650231824, 0.03895569208112638, -0.5063181281080337, -0.15188637555678797,
        -0.09926268052516171, -0.35648960192960166, 0.8564996250726968, 1.3064277172088623,
        -0.05746928859305393, 0.16694140434265137, -0.1773331496644323, -0.09618105207864167,
        -0.3637387390826281, -0.12775099875607665, 0.3775743246078491, -0.06556004402010714,
        -0.9403902888298035;

    input.jointVelocities << -0.04892431424018461, 0.011816611105509, -0.030103574006318207, -0.054470913637429216,
        0.04110477832027632, 0.006149116530581617, 0.0022679536303376294, -0.01772658684486485,
        0.0018849445068382577, 0.0027670107680974943, 0.003149718526371251, 0.009372946811675064,
        -0.019232750841138426, 0.00124201362405774, 0.0019440759412122286, -0.08404139429330826,
        2.529428034093097e-05, 0.16792771220207214, -0.02287937013013993, -0.002927608219628366,
        0.018047571904430328, -0.01034426546939903, -0.023162882775068283, 2.750370144352529e-05,
        0.0011318349279463291;

    REQUIRE(mann.setInput(input));
    REQUIRE(mann.advance());
    REQUIRE(mann.isOutputValid());

    std::cerr << "base position" << std::endl;
    std::cerr << mann.getOutput().futureBasePositionTrajectory << std::endl;

    std::cerr << "joint positions" << std::endl;
    std::cerr << mann.getOutput().jointPositions.transpose() << std::endl;

    std::cerr << "joint velocities" << std::endl;
    std::cerr << mann.getOutput().jointVelocities.transpose() << std::endl;
}
