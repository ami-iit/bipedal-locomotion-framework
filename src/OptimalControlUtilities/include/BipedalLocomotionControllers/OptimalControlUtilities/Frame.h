/**
 * @file Frame.h
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_FRAME_H
#define BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_FRAME_H

#include <iDynTree/Core/Utils.h>
#include <iDynTree/Model/Indices.h>

namespace BipedalLocomotionControllers
{

namespace OptimalControlUtilities
{

/**
 * The struct frame defines a Frame in a ControlProblemElement
 */
struct Frame
{
    iDynTree::FrameIndex indexInModel; /**< Index of the frame in the Model */

    /** Index and size of the element in the ControlProblemElement */
    iDynTree::IndexRange indexRangeInElement{iDynTree::IndexRange::InvalidRange()};
};

/**
 * FrameNames wraps a std::pair<std::string, std::string>. It contains the label
 * associated to the frame in the VariableHandler end the name of the frame in the model
 */
class FrameNames : public std::pair<std::string, std::string>
{
    // obscure first and second attribute
    using std::pair<std::string, std::string>::first;
    using std::pair<std::string, std::string>::second;

public:

    // use the std::pair constructors
    using std::pair<std::string, std::string>::pair;

    std::string& label();
    const std::string& label() const;
    std::string& nameInModel();
    const std::string& nameInModel() const;
};

} // namespace OptimalControlUtilities
} // namespace BipedalLocomotionControllers

#endif //BIPEDAL_LOCOMOTION_CONTROLLERS_OPTIMAL_CONTROL_UTILITIES_FRAME_H
