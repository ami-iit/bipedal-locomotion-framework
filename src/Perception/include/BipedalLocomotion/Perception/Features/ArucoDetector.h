/**
 * @file ArucoDetector.h
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_PERECEPTION_FEATURES_ARUCO_H
#define BIPEDAL_LOCOMOTION_PERECEPTION_FEATURES_ARUCO_H

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/Source.h>

#include <Eigen/Dense>
#include <opencv2/core.hpp>

#include <memory>
#include <unordered_map>

namespace BipedalLocomotion
{
namespace Perception
{

/**
 * Aruco marker identifiers
 */
struct ArucoMarkerData
{
    /**
     * Marker ID
     */
    int id{-1};

    /**
     * Marker corners in camera coordinates
     * in the order
     * - top left
     * - top right
     * - bottom right
     * - bottom left
     */
    std::vector<cv::Point2f> corners;

    /**
     * Pose of the marker in camera frame
     * cam_H_marker
     */
    Eigen::Matrix4d pose;
};


/**
 * Aruco detector output
 */
struct ArucoDetectorOutput
{
    std::unordered_map<int, ArucoMarkerData> markers;
    double timeNow{-1.0};
    cv::Mat imgWithMarkers;
};

class ArucoDetector : public System::Source<ArucoDetectorOutput>
{
public:
    ArucoDetector();
    ~ArucoDetector();

    /**
     * Initialize the detector
     * @note The following parameter are required by the filter:
     * - "marker_dictionary" OpenCV predefined aruco marker dictionary
     * (available options: "4X4_50", "4X4_100", "4X4_250", "4X4_1000",
     *                     "5X5_50", "5X5_100", "5X5_250", "5X5_1000",
     *                     "6X6_50", "6X6_100", "6X6_250", "6X6_1000",
     *                     "7X7_50", "7X7_100", "7X7_250", "7X7_1000",
     *                     "ARUCO_ORIGINAL")
     * options coherent with https://docs.opencv.org/master/d9/d6a/group__aruco.html#gac84398a9ed9dd01306592dd616c2c975
     * - "marker_length" marker length in m
     * - "camera_matrix" 9d vector representing the camera calbration matrix in row major order
     * - "distortion_coefficients" 5d vector containing camera distortion coefficients
     * @param[in] handlerWeak weak pointer to a ParametersHandler::IParametersHandler interface
     * @tparameter Derived particular implementation of the IParameterHandler
     * @return True in case of success, false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler) final;

    /**
     * Set image for which markers need to be detected
     * @param[in] inputImg image as OpenCV mat
     * @param[in] timeNow current time in chosen time units
     *                    it is useful for bookkeeping
     *                    or time delay synchronization
     * @return True in case of success, false otherwise
     */
    bool setImage(const cv::Mat& inputImg, double timeNow);

    /**
     * Compute one step of the detector
     * @return True in case of success, false otherwise
     */
    bool advance() final;

    /**
     * Get the detected markers' data from the current step
     * @return A struct containing a map container of detected markers.
     */
    const ArucoDetectorOutput& getOutput() const final;

    /**
     * Get the detected marker data
     * @param[in] id marker id
     * @param[in] markerData detected marker identifiers data
     * @return True in case of success, false if marker was not detected
     */
    bool getDetectedMarkerData(const int& id, ArucoMarkerData& markerData);

    /**
     * Get the image with drawn detected markers
     * @param[in] outputImg image with detected markers drawn on it
     * @param[in] drawFrames draw also estimated marker poses, set to false by default
     * @param[in] axisLengthForDrawing axis length for drawing the frame axes, 0.1 by default
     * @return True in case of success, false if no marker was detected
     */
    bool getImageWithDetectedMarkers(cv::Mat& outputImg,
                                     const bool& drawFrames = false,
                                     const double& axisLengthForDrawing = 0.1);

    /**
     * Determines the validity of the object retrieved with get()
     * @return True if the object is valid, false otherwise.
     */
    bool isOutputValid() const final;

private:
    class Impl;
    std::unique_ptr<Impl> m_pimpl;
};

} // namespace Perception
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_PERECEPTION_FEATURES_ARUCO_H
