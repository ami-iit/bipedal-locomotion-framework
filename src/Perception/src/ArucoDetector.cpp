/**
 * @file ArucoDetector.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/Conversions/CommonConversions.h>
#include <BipedalLocomotion/GenericContainer/Vector.h>
#include <BipedalLocomotion/Perception/Features/ArucoDetector.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::GenericContainer;
using namespace BipedalLocomotion::Perception;
using namespace BipedalLocomotion::Conversions;

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

class ArucoDetector::Impl
{
public:
    /**
     * clear all internal buffers
     */
    void resetBuffers();
    void estimateMarkerPose(std::vector<int>& markerIds,
                            std::vector<std::vector<cv::Point2f>>& markerCorners,
                            const double& markerLength,
                            const cv::Mat& cameraMatrix,
                            const cv::Mat& distCoeffs,
                            std::vector<cv::Vec3d>& detectedMarkerRotVecs,
                            std::vector<cv::Vec3d>& detectedMarkerTransVecs,
                            bool solvePnP = false);
    void solvePnPMarkerPose(std::vector<int>& markerIds,
                            std::vector<std::vector<cv::Point2f>>& markerCorners,
                            const double& markerLength,
                            const cv::Mat& cameraMatrix,
                            const cv::Mat& distCoeffs,
                            const double& ambiguityThreshold,
                            std::vector<cv::Vec3d>& detectedMarkerRotVecs,
                            std::vector<cv::Vec3d>& detectedMarkerTransVecs);
    bool solvePnPSingleMarkerPose(const std::vector<cv::Point2f>& corners,
                                  const double& markerLength,
                                  const cv::Mat& cameraMatrix,
                                  const cv::Mat& distCoeffs,
                                  const double& ambiguityThreshold,
                                  cv::Vec3d& rVec,
                                  cv::Vec3d& tVec);
    void getSingleMarkerObjectPoints(const double& markerLength, std::vector<cv::Vec3f>& objPoints);

    bool useSolvePnPOutlierRejection{true}; // if set to true, we call solvePnPGeneric and perform
                                            // pose estimation manually instead of using
                                            // cv::aruco::estimateMarkerPose, this will allow us to
                                            // choose the pose solution based on reprojection error
    double ambiguityThreshold{5.0}; // threshold ratio of reprojection error

    // parameters
    cv::Ptr<cv::aruco::Dictionary> dictionary; /**< container with detected markers data */
    double markerLength; /**< marker length*/
    cv::Mat cameraMatrix; /**< camera calibration matrix*/
    cv::Mat distCoeff; /**< camera distortion coefficients*/

    ArucoDetectorOutput out; /**< container with detected markers data */
    cv::Mat currentImg; /**< currently set image */
    double currentTime{-1.0}; /** time at which currentImg was set */
    std::vector<int> currentDetectedMarkerIds; /**< currently detected marker ids */
    std::vector<std::vector<cv::Point2f>> currentDetectedMarkerCorners; /**< currently detected
                                                                           marker corners */

    std::vector<cv::Vec3d> currentDetectedMarkersRotVecs; /**< currently detected marker
                                                               rotation vector */
    std::vector<cv::Vec3d> currentDetectedMarkersTransVecs; /**< currently detected marker
                                                               translation vector */

    bool initialized{false}; /**< flag to check if detector was initialized properly */

    cv::Mat R; /**< placeholder rotation matrix as cv Mat object*/
    Eigen::Matrix3d Reig; /**< placeholder rotation matrix as Eigen object*/
    Eigen::Vector3d teig; /**< placeholder translation vector as Eigen object*/
    Eigen::Matrix4d poseEig; /**< placeholder pose matrix as Eigen object*/

    /**
     * Utility map for choosing Aruco marker dictionary depending on user parameter
     */
    // TODO(traversaro): when we drop support for OpenCV < 4.7.0, we can cleanup this part
    // we can also check if there are other dictionary that should be added here
#if (CV_VERSION_MAJOR >= 5) || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
#define PREDEFINED_DICTIONARY_NAME PredefinedDictionaryType
#endif
    std::unordered_map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME>
        availableDict{{"4X4_50", cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50},
                      {"4X4_100", cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_100},
                      {"4X4_250", cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_250},
                      {"4X4_1000", cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_1000},
                      {"5X5_50", cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_5X5_50},
                      {"5X5_100", cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_5X5_100},
                      {"5X5_250", cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_5X5_250},
                      {"5X5_1000", cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_5X5_1000},
                      {"6X6_50", cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_6X6_50},
                      {"6X6_100", cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_6X6_100},
                      {"6X6_250", cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_6X6_250},
                      {"6X6_1000", cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_6X6_1000},
                      {"7X7_50", cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_7X7_50},
                      {"7X7_100", cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_7X7_100},
                      {"7X7_250", cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_7X7_250},
                      {"7X7_1000", cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_7X7_1000},
                      {"ARUCO_ORIGINAL",
                       cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_ARUCO_ORIGINAL}};
};

ArucoDetector::ArucoDetector()
    : m_pimpl(std::make_unique<Impl>())
{
}

ArucoDetector::~ArucoDetector() = default;

bool ArucoDetector::initialize(std::weak_ptr<const IParametersHandler> handler)
{
    constexpr auto printPrefix = "[ArucoDetector::initialize]";
    auto handle = handler.lock();
    if (handle == nullptr)
    {
        log()->error("{} The parameter handler has expired. Please check its scope.", printPrefix);
        return false;
    }

    std::string dictName;
    if (!handle->getParameter("marker_dictionary", dictName))
    {
        log()->error("{} The parameter handler could not find \" marker_dictionary \" in the "
                     "configuration file.",
                     printPrefix);
        return false;
    }

    if (m_pimpl->availableDict.find(dictName) == m_pimpl->availableDict.end())
    {
        log()->error("{} Undefined value set to \" marker_dictionary \" in the configuration file. "
                     "Please choose one among \n available options: \"4X4_50\", \"4X4_100\", "
                     "\"4X4_250\", \"4X4_1000\", \n \"5X5_50\", \"5X5_100\", \"5X5_250\", "
                     "\"5X5_1000\", \n \"6X6_50\", \"6X6_100\", \"6X6_250\", \"6X6_1000\", \n "
                     "\"7X7_50\", \"7X7_100\", \"7X7_250\", \"7X7_1000\", \n \"ARUCO_ORIGINAL\", "
                     "\n options coherent with v3.4.0 in "
                     "https://docs.opencv.org/3.4.1/d9/d6a/group__aruco.html",
                     printPrefix);
        return false;
    }

// In OpenCV 4.7.0 getPredefinedDictionary started returning a cv::aruco::Dictionary
// instead of a cv::Ptr<cv::aruco::Dictionary>
#if (CV_VERSION_MAJOR >= 5) || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
    m_pimpl->dictionary = cv::makePtr<cv::aruco::Dictionary>();
    *(m_pimpl->dictionary)
        = cv::aruco::getPredefinedDictionary(m_pimpl->availableDict.at(dictName));
#else
    m_pimpl->dictionary = cv::aruco::getPredefinedDictionary(m_pimpl->availableDict.at(dictName));
#endif

    if (!handle->getParameter("marker_length", m_pimpl->markerLength))
    {
        log()->error("{} The parameter handler could not find \" marker_length \" in the "
                     "configuration file.",
                     printPrefix);
        return false;
    }

    Eigen::Matrix<double, 9, 1> calibVec;
    if (!handle->getParameter("camera_matrix", calibVec))
    {
        log()->error("{} The parameter handler could not find \" camera_matrix \" in the "
                     "configuration file.",
                     printPrefix);
        return false;
    }

    Eigen::Matrix3d calibMat
        = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(calibVec.data());
    m_pimpl->cameraMatrix = cv::Mat(3, 3, CV_64F);
    cv::eigen2cv(calibMat, m_pimpl->cameraMatrix);

    Eigen::Matrix<double, 5, 1> distCoeffVec;
    if (!handle->getParameter("distortion_coefficients", distCoeffVec))
    {
        log()->error("{} The parameter handler could not find \" distortion_coefficients \" in the "
                     "configuration file.",
                     printPrefix);
        return false;
    }

    m_pimpl->distCoeff = cv::Mat(5, 1, CV_64F);
    cv::eigen2cv(distCoeffVec, m_pimpl->distCoeff);

    if (!handle->getParameter("use_solve_pnp_outlier_rejection",
                              m_pimpl->useSolvePnPOutlierRejection))
    {
        m_pimpl->useSolvePnPOutlierRejection = false;
    }

    if (m_pimpl->useSolvePnPOutlierRejection)
    {
        if (!handle->getParameter("ambiguity_threshold_reprojection_error_ratio",
                                  m_pimpl->ambiguityThreshold))
        {
            m_pimpl->ambiguityThreshold = 5.0;
            log()->warn("{} The parameter \" ambiguity_threshold_reprojection_error_ratio \" "
                        "was not found. Setting to default value: {}",
                        printPrefix,
                        m_pimpl->ambiguityThreshold);
        }
    }

    m_pimpl->initialized = true;
    return true;
}

bool ArucoDetector::setImage(const cv::Mat& inputImg, double timeNow)
{
    constexpr auto printPrefix = "[ArucoDetector::setImage]";
    if (!m_pimpl->initialized)
    {
        log()->error("{} Unable to set image. Please initialize the ArucoDetector before setting "
                     "the image.",
                     printPrefix);
        return false;
    }

    inputImg.copyTo(m_pimpl->currentImg);
    m_pimpl->currentTime = timeNow;

    return true;
}

bool ArucoDetector::advance()
{
    constexpr auto printPrefix = "[ArucoDetector::advance]";
    if (!m_pimpl->initialized)
    {
        log()->error("{} Unable to run advance(). Please initialize the ArucoDetector first.",
                     printPrefix);
        return false;
    }

    if (m_pimpl->currentImg.empty())
    {
        log()->error("{} Unable to run advance(). Please set an image first.", printPrefix);
        return false;
    }

    m_pimpl->resetBuffers();
    std::vector<std::vector<cv::Point2f>> detectedmarkerCorners;
    cv::aruco::detectMarkers(m_pimpl->currentImg,
                             m_pimpl->dictionary,
                             m_pimpl->currentDetectedMarkerCorners,
                             m_pimpl->currentDetectedMarkerIds);

    if (m_pimpl->currentDetectedMarkerIds.size() > 0)
    {
        m_pimpl->estimateMarkerPose(m_pimpl->currentDetectedMarkerIds,
                                    m_pimpl->currentDetectedMarkerCorners,
                                    m_pimpl->markerLength,
                                    m_pimpl->cameraMatrix,
                                    m_pimpl->distCoeff,
                                    m_pimpl->currentDetectedMarkersRotVecs,
                                    m_pimpl->currentDetectedMarkersTransVecs,
                                    m_pimpl->useSolvePnPOutlierRejection);

        for (std::size_t idx = 0; idx < m_pimpl->currentDetectedMarkerIds.size(); idx++)
        {
            cv::Rodrigues(m_pimpl->currentDetectedMarkersRotVecs[idx], m_pimpl->R);
            cv::cv2eigen(m_pimpl->R, m_pimpl->Reig);
            m_pimpl->teig << m_pimpl->currentDetectedMarkersTransVecs[idx](0),
                m_pimpl->currentDetectedMarkersTransVecs[idx](1),
                m_pimpl->currentDetectedMarkersTransVecs[idx](2);

            m_pimpl->poseEig = toEigenPose(m_pimpl->Reig, m_pimpl->teig);
            ArucoMarkerData markerData{m_pimpl->currentDetectedMarkerIds[idx],
                                       m_pimpl->currentDetectedMarkerCorners[idx],
                                       m_pimpl->poseEig};
            m_pimpl->out.markers[m_pimpl->currentDetectedMarkerIds[idx]] = markerData;
        }
        m_pimpl->out.timeNow = m_pimpl->currentTime;
    }

    return true;
}

const ArucoDetectorOutput& ArucoDetector::getOutput() const
{
    return m_pimpl->out;
}

bool ArucoDetector::isOutputValid() const
{
    if (!m_pimpl->initialized)
    {
        return false;
    }

    return true;
}

bool ArucoDetector::getDetectedMarkerData(const int& id, ArucoMarkerData& markerData)
{
    if (m_pimpl->out.markers.find(id) == m_pimpl->out.markers.end())
    {
        return false;
    }

    markerData = m_pimpl->out.markers.at(id);
    return true;
}

bool ArucoDetector::getImageWithDetectedMarkers(cv::Mat& outputImg,
                                                const bool& drawFrames,
                                                const double& axisLengthForDrawing)
{
    std::size_t nrDetectedMarkers = m_pimpl->currentDetectedMarkerIds.size();
    if (m_pimpl->currentImg.empty() || nrDetectedMarkers <= 0)
    {
        return false;
    }

    m_pimpl->currentImg.copyTo(outputImg);

    if (outputImg.empty())
    {
        return false;
    }

    cv::aruco::drawDetectedMarkers(outputImg,
                                   m_pimpl->currentDetectedMarkerCorners,
                                   m_pimpl->currentDetectedMarkerIds);

    if (drawFrames)
    {
        for (std::size_t idx = 0; idx < nrDetectedMarkers; idx++)
        {
            cv::drawFrameAxes(outputImg,
                              m_pimpl->cameraMatrix,
                              m_pimpl->distCoeff,
                              m_pimpl->currentDetectedMarkersRotVecs[idx],
                              m_pimpl->currentDetectedMarkersTransVecs[idx],
                              axisLengthForDrawing);
        }
    }

    return true;
}

void ArucoDetector::Impl::resetBuffers()
{
    currentDetectedMarkerCorners.clear();
    currentDetectedMarkerIds.clear();
    currentDetectedMarkersRotVecs.clear();
    currentDetectedMarkersTransVecs.clear();
    out.markers.clear();
    out.timeNow = -1.0;
}

void ArucoDetector::Impl::estimateMarkerPose(std::vector<int>& ids,
                                             std::vector<std::vector<cv::Point2f>>& corners,
                                             const double& mLength,
                                             const cv::Mat& camMat,
                                             const cv::Mat& distCoeffs,
                                             std::vector<cv::Vec3d>& detectedMarkerRotVecs,
                                             std::vector<cv::Vec3d>& detectedMarkerTransVecs,
                                             bool useSolvePnP)
{
    if (!useSolvePnP)
    {
        cv::aruco::estimatePoseSingleMarkers(corners,
                                             mLength,
                                             camMat,
                                             distCoeffs,
                                             detectedMarkerRotVecs,
                                             detectedMarkerTransVecs);

    } else
    {
        solvePnPMarkerPose(ids,
                           corners,
                           mLength,
                           camMat,
                           distCoeffs,
                           ambiguityThreshold,
                           detectedMarkerRotVecs,
                           detectedMarkerTransVecs);
    }
}

void ArucoDetector::Impl::solvePnPMarkerPose(std::vector<int>& ids,
                                             std::vector<std::vector<cv::Point2f>>& corners,
                                             const double& mLength,
                                             const cv::Mat& camMat,
                                             const cv::Mat& distCoeffs,
                                             const double& ambiguityThreshold,
                                             std::vector<cv::Vec3d>& detectedMarkerRotVecs,
                                             std::vector<cv::Vec3d>& detectedMarkerTransVecs)
{
    constexpr auto printPrefix = "[ArucoDetector::Impl::solvePnPMarkerPose]";
    if (mLength < 0)
    {
        log()->error("{} Invalid marker length.", printPrefix);
    }

    auto nrMarkers{ids.size()};

    detectedMarkerRotVecs.resize(nrMarkers);
    detectedMarkerTransVecs.resize(nrMarkers);

    std::vector<int> unambiguousIDs;
    for (auto mIdx = 0; mIdx < nrMarkers; mIdx++)
    {
        cv::Vec3d rVec, tVec;
        bool solved = solvePnPSingleMarkerPose(corners[mIdx],
                                               mLength,
                                               camMat,
                                               distCoeffs,
                                               ambiguityThreshold,
                                               rVec,
                                               tVec);

        if (solved)
        {
            unambiguousIDs.emplace_back(mIdx);
        }

        detectedMarkerRotVecs[mIdx] = rVec;
        detectedMarkerTransVecs[mIdx] = tVec;
    }

    // if the two solutions of estimated rotations
    // is too close and ambiguous, then mark the
    // the marker as not detected and remove them from outputs
    for (auto mIdx = 0; mIdx < nrMarkers; mIdx++)
    {
        if (std::find(unambiguousIDs.begin(), unambiguousIDs.end(), mIdx) == unambiguousIDs.end())
        {
            ids.erase(ids.begin() + mIdx);
            corners.erase(corners.begin() + mIdx);
            detectedMarkerRotVecs.erase(detectedMarkerRotVecs.begin() + mIdx);
            detectedMarkerTransVecs.erase(detectedMarkerTransVecs.begin() + mIdx);
        }
    }
}

void ArucoDetector::Impl::getSingleMarkerObjectPoints(const double& markerLength,
                                                      std::vector<cv::Vec3f>& objPoints)
{
    constexpr auto printPrefix = "[ArucoDetector::Impl::getSingleMarkerObjectPoints]";
    if (markerLength < 0)
    {
        log()->error("{} Invalid marker length.", printPrefix);
    }

    objPoints.clear();
    objPoints.emplace_back(cv::Vec3f(-markerLength / 2.f, markerLength / 2.f, 0.));
    objPoints.emplace_back(cv::Vec3f(markerLength / 2.f, markerLength / 2.f, 0.));
    objPoints.emplace_back(cv::Vec3f(markerLength / 2.f, -markerLength / 2.f, 0.));
    objPoints.emplace_back(cv::Vec3f(-markerLength / 2.f, -markerLength / 2.f, 0.));
}

bool ArucoDetector::Impl::solvePnPSingleMarkerPose(const std::vector<cv::Point2f>& corners,
                                                   const double& markerLength,
                                                   const cv::Mat& camMat,
                                                   const cv::Mat& distCoeffs,
                                                   const double& ambiguityThreshold,
                                                   cv::Vec3d& rVec,
                                                   cv::Vec3d& tVec)
{
    std::vector<cv::Vec3f> markerObjPoints;
    getSingleMarkerObjectPoints(markerLength, markerObjPoints);

    std::vector<cv::Mat> rvecs, tvecs;
    cv::Mat reprojErr(0, 0, CV_64FC1);
    bool useExtrinsicGuess{false};
    cv::SolvePnPMethod flags{cv::SOLVEPNP_IPPE_SQUARE};
    int solutions = cv::solvePnPGeneric(markerObjPoints,
                                        corners,
                                        camMat,
                                        distCoeffs,
                                        rvecs,
                                        tvecs,
                                        useExtrinsicGuess,
                                        flags,
                                        cv::noArray(),
                                        cv::noArray(),
                                        reprojErr);

    if (solutions > 0)
    {
        int rdepth = CV_64F;
        int tdepth = CV_64F;
        if (solutions == 1)
        {
            rvecs[0].convertTo(rVec, rdepth);
            tvecs[0].convertTo(tVec, tdepth);
        }

        if (solutions == 2)
        {
            double e1{reprojErr.at<double>(0, 0)};
            double e2{reprojErr.at<double>(1, 0)};

            if (e1 <= e2)
            {
                double ratio{e2 / e1};
                if (ratio < ambiguityThreshold)
                {
                    // ambiguous solutions
                    return false;
                }
                // choose solution with low reprojection error
                rvecs[0].convertTo(rVec, rdepth);
                tvecs[0].convertTo(tVec, tdepth);
            } else
            {
                double ratio{e1 / e2};
                if (ratio < ambiguityThreshold)
                {
                    // ambiguous solutions
                    return false;
                }
                // choose solution with low reprojection error
                rvecs[1].convertTo(rVec, rdepth);
                tvecs[1].convertTo(tVec, tdepth);
            }
        }
    }

    return true;
}
