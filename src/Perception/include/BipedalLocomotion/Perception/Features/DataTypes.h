/**
 * @file DataTypes.h
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_PERECEPTION_FEATURES_DATA_TYPES_H
#define BIPEDAL_LOCOMOTION_PERECEPTION_FEATURES_DATA_TYPES_H

#include <opencv2/opencv.hpp>

namespace BipedalLocomotion
{
namespace Perception
{

struct TimeStampedImg
{
    double ts{-1.0};
    cv::Mat img;
};

} // namespace Perception
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_PERECEPTION_FEATURES_DATA_TYPES_H

