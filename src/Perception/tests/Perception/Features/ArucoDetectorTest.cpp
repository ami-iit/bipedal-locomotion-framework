/**
 * @file ArucoDetectorTest.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

// Catch2
#include <catch2/catch_test_macros.hpp>
#include <opencv2/opencv.hpp>

#include <BipedalLocomotion/Perception/Features/ArucoDetector.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <ResourceFolderPath.h>

using namespace BipedalLocomotion::Perception;
using namespace BipedalLocomotion::ParametersHandler;

TEST_CASE("Aruco Detector")
{
    std::shared_ptr<IParametersHandler> parameterHandler = std::make_shared<StdImplementation>();
    parameterHandler->setParameter("marker_dictionary", "4X4_50");
    parameterHandler->setParameter("marker_length", 0.806);
    parameterHandler->setParameter("camera_matrix", std::vector<double>{922.309448242188,0,664.546813964844,0,922.194458007813,348.770141601563,0,0,1});
    parameterHandler->setParameter("distortion_coefficients", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0});

    // // Instantiate the estimator
    ArucoDetector detector;
    REQUIRE(detector.initialize(parameterHandler));

    auto imgName = getSampleImagePath();
    auto inputImg = cv::imread(imgName);

    REQUIRE(detector.setImage(inputImg, 0.1));
    REQUIRE(detector.advance());

    cv::Mat outputImage;
    REQUIRE(detector.getImageWithDetectedMarkers(outputImage,
                                                 /*drawFrames=*/ true,
                                                 /*axisLengthForDrawing=*/ 0.3));

    // Marker 2 is detected in the sample image
    ArucoMarkerData marker2;
    REQUIRE(detector.getDetectedMarkerData(/*id=*/ 2, marker2));
    REQUIRE(marker2.id == 2);

     /* // uncomment this block to view the output image
       cv::imshow("outputImage", outputImage);
       cv::waitKey();
     */

}
