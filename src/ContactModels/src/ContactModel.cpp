/**
 * @file ContactModel.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotionControllers/ContactModels/ContactModel.h>

using namespace BipedalLocomotionControllers::ContactModels;

const iDynTree::Wrench& ContactModel::getContactWrench()
{
    computeContactWrench();
    return m_contactWrench;
}
