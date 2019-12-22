/**
 * @file ZMPElement.cpp
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <stdexcept>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Model/Model.h>

#include <BipedalLocomotionControllers/OptimalControlUtilities/ZMPElements.h>


using namespace BipedalLocomotionControllers::OptimalControlUtilities;

// ZMP Element
ZMPElement::ZMPElement(std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
                       const VariableHandler& handler,
                       const std::vector<FrameNames>& framesInContact)
    : ControlTask(kinDyn)
{
    m_name = "ZMP element: (Frames in contact: ";

    // resize and reset matrices
    m_A.resize(2, handler.getNumberOfVariables());
    m_A.zero();
    m_b.resize(2);
    m_b.zero();

    m_contactFramePosition.zero();

    for (const auto& frameInContact : framesInContact)
    {
        Frame frame;
        frame.indexRangeInElement = handler.getVariable(frameInContact.label());
        frame.indexInModel = m_kinDynPtr->model().getFrameIndex(frameInContact.nameInModel());

        m_framesInContact.push_back(frame);

        if (!frame.indexRangeInElement.isValid())
            throw std::runtime_error("[ZMPElement::ZMPElement] Undefined frame named "
                                     + frameInContact.label() + " in the variableHandler");

        if (frame.indexInModel == iDynTree::FRAME_INVALID_INDEX)
            throw std::runtime_error("[ZMPElement::ZMPElement] Undefined frame named "
                                     + frameInContact.nameInModel() + " in the model");

        // constant values
        m_A(0, frame.indexRangeInElement.offset + 4) = 1;
        m_A(1, frame.indexRangeInElement.offset + 5) = -1;

        m_name += "[" + frameInContact.label() + ",  " + frameInContact.nameInModel() + "] ";
    }

    m_name += ")";
}

void ZMPElement::setDesiredZMP(const iDynTree::Vector2& ZMP)
{
    m_ZMP = ZMP;
}

const iDynTree::MatrixDynSize& ZMPElement::getA()
{
    for (const auto& frame : m_framesInContact)
    {
        m_contactFramePosition = m_kinDynPtr->getWorldTransform(frame.indexInModel).getPosition();

        m_A(0, frame.indexRangeInElement.offset + 2) = m_ZMP(0) - m_contactFramePosition(0);
        m_A(1, frame.indexRangeInElement.offset + 2) = m_ZMP(1) - m_contactFramePosition(1);
    }
    return m_A;
}
