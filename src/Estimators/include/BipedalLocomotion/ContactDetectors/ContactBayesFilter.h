/**
 * @file ContactBayesFilter.h
 * @authors Prashanth Ramadoss
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTACT_BAYES_FILTER_H
#define BIPEDAL_LOCOMOTION_CONTACT_BAYES_FILTER_H

#include <BipedalLocomotion/ContactDetectors/ContactDetector.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/SolidShapes.h>
#include <memory>
#include <iostream>
#include <unordered_map>

namespace BipedalLocomotion
{
namespace Estimators
{

class ContactBayesCollision;
    
/**
 * Contact detection through probabilistic fusion of
 * link collision vertex distances with contact plane 
 * and link collision vertex linear velocity norms 
 * through a recursive Bayes filter
 * 
 * The transition and the likelihood probability models are
 * assumed to be known and following logistic distribution.
 * 
 * Implements the methodology described in
 * Miezal, Markus, Bertram Taetz, and Gabriele Bleser. 
 * "Real-time inertial lower body kinematics and ground contact estimation 
 * at anatomical foot points for agile human locomotion." 
 * 2017 IEEE international conference on robotics and automation (ICRA). IEEE, 2017.
 */
class ContactBayesManager : public ContactDetector
{
public:
    ContactBayesManager();
    ~ContactBayesManager();

    /**
     * Set the kinDynComputations object.
     * @param kinDyn pointer to a kinDynComputations object.
     * @return True in case of success, false otherwise.
     */
    bool setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn);
    
    /**
     * Set current time
     * @param[in] time time now in time units
     * @warning please call this before calling advance
     * this is required for updating contact switch times
     */
    void setCurrentTime(const double& timeNow);
       
    /**
     * check configured contact links with loaded URDF model
     * and load the collisions data for the desired links from the URDF
     */
    bool prepare();
    
    ContactBayesCollision getCollisionData(const std::string& linkName) const;
protected:
    /**
    * These custom parameter specifications should be specified by the derived class.
    * @param[in] handler configure the custom parameters for the detector
    * @return True in case of success, false otherwise.
    */
    virtual bool customInitialization(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler) override;

    /**
    * Update contact states based on probabilistic fusion of motion features
    * Assumes that the kinDynComputations internal state is updated by 
    * an external floating base estimator sharing the KinDynComputations resource
    * @return True in case of success, false otherwise.
    */
    virtual bool updateContactStates() override;

private:
    /**
    * Private implementation of the class
    */
    class Impl;
    std::unique_ptr<Impl> m_pimpl; /**< Pointer to implementation */
};


/**
 * Contact Bayes collision
 */
class ContactBayesCollision
{
public:        
    // populated at configure step
    std::string linkName;
    std::vector<iDynTree::Box* > boundingBoxes;   /**< collision bounding boxes associated with the link */
    std::vector< std::vector<std::string> > boxVerticesNames; /**< bounding box vertex name <link_name>_box<i>_vertex<j> */
    std::vector< std::vector<Eigen::Vector3d> > boxVertices; /**< vertex positions in link frame*/
    std::size_t nrActiveContacts{0};  /**< total number of vertices in contact */
    bool stableContact{false};  /**< if the link can be considered to be in stable contact, if less then two active vertices set to false */
    double switchTime{0.};
    
    // updated at every advance step
    Eigen::Vector3d position; /**< link position in inertial frame */
    Eigen::Matrix<double, 6, 1> velocity; /**< mixed triv. link velocity wrt inertial frame */
    std::vector< std::vector<Eigen::Vector3d> > verticesPositionsInertial; /**< vertex positions in inertial frame*/
    std::vector< std::vector<Eigen::Vector3d> > verticesVelocitiesInertial; /**< vertex mixed linear velocities in inertial frame*/
    std::vector< std::vector<double> > verticesDistancesToNearestPlane; /**< vertex distance to the nearest/local contact plane*/
    std::vector< std::vector<double> > contactProbabilities; /**< probability of contact of vertices*/
    std::vector< std::vector<double> > swingProbabilities; /**< probability of contact of vertices*/
    std::vector< std::vector<bool> > isActive; /**< probability of contact of vertices*/    
    std::vector< std::vector<double> > verticesSwitchTimes; /**< contact switch time of vertices*/
};

} // namespace Estimators
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_CONTACT_BAYES_FILTER_H
