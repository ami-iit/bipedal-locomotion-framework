/**
 * @file ContactBayesFilter.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/ContactDetectors/ContactBayesFilter.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/InertialParametersSolidShapesHelpers.h>
#include <sstream>

using namespace BipedalLocomotion::Estimators;
using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::Contacts;

class ContactBayesManager::Impl
{
public:
    std::unordered_map<std::string, ContactBayesCollision> manager;
    std::vector<std::string> contactLinkNamesFromConfig;
    std::string meshFilesPrefix{" "};
    std::shared_ptr<iDynTree::KinDynComputations> kinDyn{nullptr}; /**< Pointer to a KinDynComputations object */
    std::vector< std::vector< iDynTree::SolidShape* > > linkCollisionSolidsV;    
    
    bool assumeFlatAndAtleastOneContact{true};
    Eigen::Vector3d groundPlaneNormal{0.0, 0.0, 1.0}; // assumes inertial frame has z pointing upwards and parallel to gravity
    Eigen::Vector3d pointOnGroundPlane{0.0, 0.0, 0.0}; //assume plane is defined by level set z = 0
    
    // gains for sigmoid
    double meanContactDistance{0.034};
    double scaledDeviationContactDistance{0.00723}; // 0.004
    double meanContactVelocityNorm{0.03}; // 0.004
    double scaledDeviationContactVelocityNorm{2.0}; // 0.01
    double meanSwingVelocityNorm{0.5};
    double scaledDeviationSwingVelocityNorm{1.52};
    double contactProbabilityThreshold{0.57};
    bool useVertexContacts{true};
    
    const double divideByZeroThreshold{1e-14};
    
    double timeNow{0.};
    bool isClockSet{false};
    bool firstUpdate{true}; // used to update the ground plane height - vertex with minimum height
    bool verbose{false};
    bool noCollisionMeshes{true};
    
    bool getCollisionBoundingBoxes(const iDynTree::Model& model, 
                                   const std::string& linkName,
                                   const bool& _noCollisionMesh,
                                   const std::string& _meshFilePrefix,
                                   const std::vector< std::vector< iDynTree::SolidShape* > >& collisionSolidsV,
                                   std::vector<iDynTree::Box* >& collisionBoundingBoxes);
    
    bool updateCollisionBoxVerticesInLinkFrame(const std::vector<iDynTree::Box* >& collisionBoundingBoxes,
                                               std::vector< std::vector<iDynTree::Position> >& collisionBoxVertices);
    
    bool updateCollisionStates();
    bool updateProbabilities();
    bool updateContactsList(EstimatedContactList& contacts);
    double pointToPlaneDistance(Eigen::Ref<const Eigen::Vector3d> vertex, 
                                Eigen::Ref<const Eigen::Vector3d> pointOnPlane,
                                Eigen::Ref<const Eigen::Vector3d> planeNormal);
    
    double sigmoid(const double& value, const double& mean, const double& scale);
    double sigmoid2(const double& value, const double& mean, 
                    const double& scale, const double& m);
    
    bool isPrepared{false};
    
    bool checkKinDyn(const std::string& printPrefix)
    {
        if (kinDyn == nullptr)
        {
            std::cerr << printPrefix << " Please set KinDynComputations object before calling this method."
            << std::endl;
            return false;
        }
        return true;
    }
};

ContactBayesManager::ContactBayesManager() : m_pimpl(std::make_unique<Impl>())
{
    m_pimpl->manager.clear();
}

ContactBayesManager::~ContactBayesManager() = default;

bool ContactBayesManager::customInitialization(std::weak_ptr<IParametersHandler> handler)
{
    std::string printPrefix{"[ContactBayesManager::customInitialization] "};
    auto orignalhandle = handler.lock();
    if (orignalhandle == nullptr)
    {
        return false;
    }
    
    auto cbmHandler = orignalhandle->getGroup("ContactBayesFilter");
    auto handle = cbmHandler.lock();
    if (handle == nullptr)
    {
        return false;
    }

    if (!handle->getParameter("no_collision_meshes", m_pimpl->noCollisionMeshes))
    {
        m_pimpl->noCollisionMeshes = false;
    }
    
    if (!m_pimpl->noCollisionMeshes)
    {
        if (!handle->getParameter("urdf_mesh_files_prefix", m_pimpl->meshFilesPrefix))
        {
            std::cerr <<  printPrefix <<
            "The parameter handler could not find \" urdf_mesh_files_prefix \" in the configuration file."
            << std::endl;
            return false;
        }
    }
        
    if (!handle->getParameter("contact_links", m_pimpl->contactLinkNamesFromConfig))
    {
        std::cerr <<  printPrefix <<
        "The parameter handler could not find \" contact_links \" in the configuration file."
        << std::endl;
        return false;
    }
    
    handle->getParameter("contact_distance_mean", m_pimpl->meanContactDistance);
    handle->getParameter("contact_distance_deviation", m_pimpl->scaledDeviationContactDistance);
    handle->getParameter("contact_velocity_norm_mean", m_pimpl->meanContactVelocityNorm);
    handle->getParameter("contact_velocity_norm_deviation", m_pimpl->scaledDeviationContactVelocityNorm);
    handle->getParameter("swing_velocity_norm_mean", m_pimpl->meanSwingVelocityNorm);
    handle->getParameter("swing_velocity_norm_deviation", m_pimpl->scaledDeviationSwingVelocityNorm);
    handle->getParameter("contact_probability_threshold", m_pimpl->contactProbabilityThreshold);
    handle->getParameter("verbose", m_pimpl->verbose);
    handle->getParameter("use_individual_vertex_contacts", m_pimpl->useVertexContacts);
    return true;
}

bool ContactBayesManager::setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    std::string printPrefix{"[ContactBayesManager::setKinDyn] "};
    if (kinDyn == nullptr || !kinDyn->isValid())
    {
        std::cerr <<  printPrefix << "Invalid KinDynComputations object." << std::endl;
        return false;
    }
            
    m_pimpl->kinDyn = kinDyn;
        
    return true;
}

void ContactBayesManager::setCurrentTime(const double& timeNow)
{
    m_pimpl->timeNow = timeNow;
    m_pimpl->isClockSet = true;
}

bool ContactBayesManager::prepare()
{
    std::string printPrefix{"[ContactBayesManager::prepare] "};
    if (!m_pimpl->checkKinDyn(printPrefix))
    {
        return false;
    }
    
    iDynTree::Model model = m_pimpl->kinDyn->model();
    m_pimpl->linkCollisionSolidsV = model.collisionSolidShapes().getLinkSolidShapes();
    
    if (m_pimpl->contactLinkNamesFromConfig.size() == 0)
    {
        std::cerr << printPrefix << "No contact links mentioned in configuration. Nothing to prepare." << std::endl;
            return false;
    }
    
    for (auto& linkName : m_pimpl->contactLinkNamesFromConfig)
    {
        ContactBayesCollision cCollision;
        cCollision.linkName = linkName;
        cCollision.position.setZero();
        cCollision.velocity.setZero();
        if (!m_pimpl->getCollisionBoundingBoxes(model, linkName,
                                                m_pimpl->noCollisionMeshes,
                                                m_pimpl->meshFilesPrefix,
                                                m_pimpl->linkCollisionSolidsV,
                                                cCollision.boundingBoxes))
        {
            std::cerr << printPrefix << "Could not retrieve link collision bounding box for link: " 
            << linkName << std::endl;
            return false;
        }
        
        // get bounding box vertices in link frame
        std::vector<std::vector<iDynTree::Position> > boxVerticesDyn;
        if (!m_pimpl->updateCollisionBoxVerticesInLinkFrame(cCollision.boundingBoxes,
                                                            boxVerticesDyn))
        {
            std::cerr << printPrefix << "Could not compute bounding box vertices in link frame for link: " 
            << linkName << std::endl;
            return false;
        }
        
        // add additional frames to the link and update the robot model
        // assuming that these additional frames are appended only at the end of
        // the existing frames vector
        auto nrBoxes = boxVerticesDyn.size();
        cCollision.boxVertices.resize(nrBoxes);
        cCollision.boxVerticesNames.resize(nrBoxes);
        cCollision.verticesPositionsInertial.resize(nrBoxes);
        cCollision.verticesVelocitiesInertial.resize(nrBoxes);
        cCollision.contactProbabilities.resize(nrBoxes);
        cCollision.swingProbabilities.resize(nrBoxes);
        cCollision.verticesDistancesToNearestPlane.resize(nrBoxes);
        cCollision.isActive.resize(nrBoxes);
        cCollision.verticesSwitchTimes.resize(nrBoxes);
        for (int idx = 0; idx < nrBoxes; idx++)
        {            
            auto nrVertices = boxVerticesDyn[idx].size();
            cCollision.boxVertices[idx].resize(nrVertices);
            cCollision.boxVerticesNames[idx].resize(nrVertices);
            cCollision.verticesPositionsInertial[idx].resize(nrVertices);
            cCollision.verticesVelocitiesInertial[idx].resize(nrVertices);
            cCollision.contactProbabilities[idx].resize(nrVertices);
            cCollision.swingProbabilities[idx].resize(nrVertices);
            cCollision.verticesDistancesToNearestPlane[idx].resize(nrVertices);
            cCollision.isActive[idx].resize(nrVertices);
            cCollision.verticesSwitchTimes[idx].resize(nrVertices);
            
            for (int jdx = 0; jdx < nrVertices; jdx++)
            {
                auto link_H_vertex = iDynTree::Transform(iDynTree::Rotation::Identity(),
                                                         boxVerticesDyn[idx][jdx]);
                std::string frameName = linkName + "_collision_box" + std::to_string(idx) + "_vertex"  + std::to_string(jdx);
                if (!model.addAdditionalFrameToLink(linkName, frameName, link_H_vertex))
                {
                    std::cerr << printPrefix << "Could not add vertex " << frameName << " frame to link: " 
                    << linkName << std::endl;
                    return false;
                }
                
                cCollision.boxVertices[idx][jdx] = iDynTree::toEigen(boxVerticesDyn[idx][jdx]);
                cCollision.boxVerticesNames[idx][jdx] = frameName;
                cCollision.verticesPositionsInertial[idx][jdx].setZero();
                cCollision.verticesVelocitiesInertial[idx][jdx].setZero();
                cCollision.contactProbabilities[idx][jdx] = 0.5;
                cCollision.swingProbabilities[idx][jdx] = 0.5;
                cCollision.verticesDistancesToNearestPlane[idx][jdx] = 0.0;
                cCollision.isActive[idx][jdx] = true;
                cCollision.verticesSwitchTimes[idx][jdx] = 0.0;
            }
        }
        
        m_pimpl->manager[linkName] = cCollision;        
    }
    
    if (!m_pimpl->kinDyn->loadRobotModel(model))
    {
        std::cerr << printPrefix << "Could not update robot model with new collision frames. " << std::endl;
        return false;
    }

    m_pimpl->isPrepared = true;
    return true;
}

bool ContactBayesManager::Impl::getCollisionBoundingBoxes(const iDynTree::Model& model, 
                                                          const std::string& linkName,
                                                          const bool& _noCollisionMesh,
                                                          const std::string& _meshFilePrefix,
                                                          const std::vector< std::vector< iDynTree::SolidShape* > >& collisionSolidsV,
                                                          std::vector<iDynTree::Box* >& collisionBoundingBoxes)
{
    std::string printPrefix{"[ContactBayesManager::Impl::getCollisionBoundingBoxes] "};
    int nrLinks = model.getNrOfLinks();
    
    if (model.getLinkIndex(linkName) == iDynTree::LINK_INVALID_INDEX)
    {
        std::cerr << printPrefix << "Link specified in the configuration is not available in the model: " 
        << linkName << std::endl;
        return false;
    }        
    
    for (int linkIdx = 0; linkIdx < nrLinks; linkIdx++)
    {
        std::string modelLinkName = model.getLinkName(linkIdx);
        if (modelLinkName != linkName)
        {
            continue;
        }
        
        std::vector<iDynTree::SolidShape*> solidsV = collisionSolidsV[linkIdx];        
        collisionBoundingBoxes.resize(solidsV.size());
        for (int solidIdx = 0; solidIdx < solidsV.size(); solidIdx++)
        {
            auto& solid = solidsV[solidIdx];
            if (solid->isBox())
            {
                collisionBoundingBoxes[solidIdx] = solid->asBox();   
            }
            else
            {                    
                if (solid->isCylinder() ||
                    solid->isSphere() ||
                    solid->isExternalMesh())
                {                    
                    if (solid->isExternalMesh())
                    {
                        if (_noCollisionMesh)
                        {
                            std::cerr << printPrefix << "Option for loading external mesh is disabled."
                            << linkName << std::endl; 
                            return false;
                        }
                        
                        // expects ROS_PACKAGE_PATH to point to iCub package w
                        std::string meshPath{" "}; 
                        meshPath = solid->asExternalMesh()->getFileLocationOnLocalFileSystem();
                        
                        if (meshPath == " ")
                        {
                            auto urdfFileName = solid->asExternalMesh()->getFilename();
                            // split string
                            std::istringstream iss(urdfFileName);
                            std::vector<std::string> collapsedFileName;
                            std::string part;
                            while (std::getline(iss, part, ':'))
                            {
                                collapsedFileName.push_back(part);
                            }
                            
                            if (collapsedFileName.size() != 2)
                            {
                                std::cerr << printPrefix << "Error parsing mesh file name for link: "
                                << linkName << std::endl; 
                            }
                            meshPath = _meshFilePrefix + collapsedFileName[1];
                        }
                                                    
                        // set mesh path
                        solid->asExternalMesh()->setFilename(meshPath);
                    }
                    
                    iDynTree::Box box;
                    if (!iDynTree::computeBoundingBoxFromShape(*solid, box))
                    {
                        std::cerr << printPrefix << "Could not convert mesh to bounding box for link: "
                        << linkName << std::endl; 
                        return false;
                    }                    
                    
                    collisionBoundingBoxes[solidIdx] = box.clone()->asBox();                        
                }
            }
        }
    }
    
    return true;
}

double ContactBayesManager::Impl::sigmoid(const double& value, 
                                          const double& mean, 
                                          const double& scale)
{
    // https://en.wikipedia.org/wiki/Logistic_distribution
    return 0.5 + 0.5*std::tanh( (value - mean) / (2*scale) );
}

double ContactBayesManager::Impl::sigmoid2(const double& value, 
                                          const double& mean, 
                                          const double& scale,
                                          const double& m)
{
    // Real-time inertial lower body kinematics and ground contact estimation
    // at anatomical foot points for agile human locomotion
    // Markus Miezal, Bertram Taetz and Gabriele Bleser
    return 0.5 + (m*0.5*std::tanh((value - mean)*scale));
}

double ContactBayesManager::Impl::pointToPlaneDistance(Eigen::Ref<const Eigen::Vector3d> vertex, 
                                                       Eigen::Ref<const Eigen::Vector3d> pointOnPlane,
                                                       Eigen::Ref<const Eigen::Vector3d> planeNormal)
{
    std::string printPrefix{"[ContactBayesManager::Impl::pointToPlaneDistance] "};
    double norm = planeNormal.norm();
    if (norm < divideByZeroThreshold)
    {
        std::cerr << printPrefix << "plane normal is not a unit vector." << std::endl;
        return -1.0;
    }
    
    // normalize plane normal vector
    Eigen::Vector3d unitNormal = planeNormal/norm;
    Eigen::Vector3d vec = vertex - pointOnPlane;
    return std::abs(vec.dot(unitNormal));    
}

bool ContactBayesManager::Impl::updateCollisionBoxVerticesInLinkFrame(const std::vector<iDynTree::Box *>& collisionBoundingBoxes, 
                                                                      std::vector<std::vector<iDynTree::Position> >& collisionBoxVertices)
{
    auto nrBoxes = collisionBoundingBoxes.size();
    collisionBoxVertices.resize(nrBoxes);
    for (std::size_t idx = 0; idx < nrBoxes; idx++)
    {
        auto box = collisionBoundingBoxes[idx];
        auto verticesDyn = iDynTree::computeBoxVertices(*box);
        
        collisionBoxVertices[idx] = verticesDyn;
    }
    return true;
}

bool ContactBayesManager::Impl::updateCollisionStates()
{
    std::string printPrefix{"[ContactBayesManager::Impl::updateCollisionStates] "};
    double minHeightVertex{std::numeric_limits<double>::max()};
    
    for (auto& linkName : contactLinkNamesFromConfig)
    {
        auto& cCollision = manager.at(linkName);
        auto linkIdx = kinDyn->model().getLinkIndex(linkName);
        bool ok = kinDyn->getFrameVel(linkName, cCollision.velocity);
        if (linkIdx == iDynTree::LINK_INVALID_INDEX || !ok)
        {
            std::cerr << printPrefix << "Could not update internal collision state for link: " 
            << linkName << std::endl;
            continue;
        }
                        
        auto linkPose = kinDyn->getWorldTransform(linkName);
        cCollision.position = iDynTree::toEigen(linkPose.getPosition());
                
        Eigen::Vector3d vLF = cCollision.velocity.head<3>();
        Eigen::Matrix3d omegaLFCross = iDynTree::skew(cCollision.velocity.tail<3>());
        Eigen::Matrix3d w_R_link = iDynTree::toEigen(linkPose.getRotation());
        auto nrBoxes{cCollision.boxVertices.size()};
               
        for (std::size_t idx = 0; idx < nrBoxes; idx++)
        {
            auto nrVertices{cCollision.boxVertices[idx].size()};
            for (std::size_t jdx = 0; jdx < nrVertices; jdx++)
            {
                // vertex spatial position and linear velocities
                cCollision.verticesPositionsInertial[idx][jdx] = (w_R_link*cCollision.boxVertices[idx][jdx]) + cCollision.position;
                cCollision.verticesVelocitiesInertial[idx][jdx] = vLF + omegaLFCross*cCollision.verticesPositionsInertial[idx][jdx];
                
                if (assumeFlatAndAtleastOneContact)
                {
                    // get point with lowest spatial height
                    if (cCollision.verticesPositionsInertial[idx][jdx](2) < minHeightVertex)
                    {
                        minHeightVertex = cCollision.verticesPositionsInertial[idx][jdx](2);                        
                    }                                        
                }
            }
        }
    }
    
    // update ground plane height
    if (assumeFlatAndAtleastOneContact && firstUpdate)
    {
       pointOnGroundPlane(2) = minHeightVertex;
       firstUpdate = false;
    }
    
    // update vertex to ground plane distance
    for (auto& linkName : contactLinkNamesFromConfig)
    {
        auto& cCollision = manager.at(linkName);
        auto nrBoxes{cCollision.boxVertices.size()};
               
        for (std::size_t idx = 0; idx < nrBoxes; idx++)
        {
            auto nrVertices{cCollision.boxVertices[idx].size()};
            for (std::size_t jdx = 0; jdx < nrVertices; jdx++)
            {
                // get distance to ground plane
                cCollision.verticesDistancesToNearestPlane[idx][jdx] = pointToPlaneDistance(cCollision.verticesPositionsInertial[idx][jdx],
                                                                                            pointOnGroundPlane,
                                                                                            groundPlaneNormal);
            }
        }
    }
    
    return true;
}

bool ContactBayesManager::Impl::updateProbabilities()
{
    for (auto& linkName : contactLinkNamesFromConfig)
    {
        auto& cCollision = manager.at(linkName);
        
        // reset number of active contacts to zero
        cCollision.nrActiveContacts = 0;
        
        auto nrBoxes{cCollision.boxVertices.size()};
        for (std::size_t idx = 0; idx < nrBoxes; idx++)
        {
            auto nrVertices{cCollision.boxVertices[idx].size()};
            for (std::size_t jdx = 0; jdx < nrVertices; jdx++)
            {   
                // prior
                double& onBelief = cCollision.contactProbabilities[idx][jdx];
                double& offBelief = cCollision.swingProbabilities[idx][jdx];
                
                // transition probabilities
                double distance{cCollision.verticesDistancesToNearestPlane[idx][jdx]};
                
                // transition from 1 to 0 and 0 to 0 (both have same models)
                double offTransitionProb = sigmoid(distance, meanContactDistance, scaledDeviationContactDistance);
                
                // transition from 1 to 1 and 0 to 1 (both have same models)
                double onTransitionProb{1 - offTransitionProb};
                if (verbose)
                {
                    std::cout << "Name: " << cCollision.boxVerticesNames[idx][jdx] 
                            << " Distance: " << distance
                            << " onTransitionProb: " << onTransitionProb 
                            << " offTransitionProb: " << offTransitionProb << std::endl;
                }
                
                // prediction
                double onPrediction = onTransitionProb*onBelief + onTransitionProb*offBelief;
                double offPrediction = offTransitionProb*onBelief + offTransitionProb*offBelief;
                
                // likelihoods
                double velocityNorm{cCollision.verticesVelocitiesInertial[idx][jdx].norm()};
                double onLikelihood = 1 - sigmoid(velocityNorm, meanContactVelocityNorm, scaledDeviationContactVelocityNorm);
                
                
                double offLikelihood = sigmoid(velocityNorm, meanSwingVelocityNorm, scaledDeviationSwingVelocityNorm);
                
                if (verbose)
                {
                    std::cout << "Name: " << cCollision.boxVerticesNames[idx][jdx] 
                            << " velocityNorm: " << velocityNorm
                            << " onLikelihood: " << onLikelihood 
                            << " offLikelihood: " << offLikelihood 
                            << std::endl;
                }
                
                cCollision.contactProbabilities[idx][jdx] = onLikelihood*onPrediction;
                cCollision.swingProbabilities[idx][jdx] = offLikelihood*offPrediction;                
                               
                double normalizer = 1/(cCollision.contactProbabilities[idx][jdx] + cCollision.swingProbabilities[idx][jdx]);
                               
                if (normalizer > divideByZeroThreshold)
                {
                    cCollision.contactProbabilities[idx][jdx] *= normalizer;
                    cCollision.swingProbabilities[idx][jdx] *= normalizer;
                }
                
                bool prevVertexContact {cCollision.isActive[idx][jdx]};
                if (cCollision.contactProbabilities[idx][jdx] > contactProbabilityThreshold)
                {
                    cCollision.isActive[idx][jdx] = true;
                    cCollision.nrActiveContacts++;
                }
                else
                {
                    cCollision.isActive[idx][jdx] = false;
                }
                
                if (prevVertexContact != cCollision.isActive[idx][jdx])
                {
                    cCollision.verticesSwitchTimes[idx][jdx] = timeNow;
                }
                
                if (verbose)
                {                    
                    std::cout << "Name: " << cCollision.boxVerticesNames[idx][jdx] << 
                    " distance: " << cCollision.verticesDistancesToNearestPlane[idx][jdx] <<
                    " vNorm: " << velocityNorm <<
                    " status: " << cCollision.isActive[idx][jdx] <<
                    " on probabilities: " << cCollision.contactProbabilities[idx][jdx] <<
                    " off probabilities: " << cCollision.swingProbabilities[idx][jdx] << std::endl;
                }
            }
        }
        
        bool prevContact{cCollision.stableContact};
        cCollision.nrActiveContacts > 0 ? cCollision.stableContact = true : cCollision.stableContact = false;

        if (prevContact != cCollision.stableContact)
        {            
            cCollision.switchTime = timeNow;
        }

    }

    return true;
}

bool ContactBayesManager::Impl::updateContactsList(EstimatedContactList& contactsList)
{
    for (auto& linkName : contactLinkNamesFromConfig)
    {
        auto& cCollision = manager.at(linkName);
        if (!useVertexContacts)
        {
            EstimatedContact contact;
            contact.name = linkName;
            contact.index = kinDyn->getFrameIndex(linkName);
            contact.isActive = cCollision.stableContact;
            contact.switchTime = cCollision.switchTime;
            contactsList[linkName] = contact;
        }        
        else
        {
            auto nrBoxes{cCollision.boxVertices.size()};        
            for (std::size_t idx = 0; idx < nrBoxes; idx++)
            {
                auto nrVertices{cCollision.boxVertices[idx].size()};
                for (std::size_t jdx = 0; jdx < nrVertices; jdx++)
                {
                    EstimatedContact contact;
                    contact.name = cCollision.boxVerticesNames[idx][jdx];
                    contact.index = kinDyn->getFrameIndex(contact.name);
                    contact.isActive = cCollision.isActive[idx][jdx];   
                    contact.switchTime = cCollision.verticesSwitchTimes[idx][jdx];
                    contactsList[contact.name] = contact;
                }
            }
        }
    }
    return true;
}

bool ContactBayesManager::updateContactStates()
{
    std::string printPrefix{"[ContactBayesManager::updateContactStates] "};
    if (!m_pimpl->checkKinDyn(printPrefix))
    {
        return false;
    }
    
    if (!m_pimpl->isClockSet)
    {
        std::cerr << printPrefix << "Please set current clock time before calling this method." << std::endl;
        return false;
    }
    
    if (!m_pimpl->isPrepared)
    {
        std::cerr << printPrefix << "Please run prepare() before calling this method." << std::endl;
        return false;
    }
    
    bool ok = m_pimpl->updateCollisionStates();
    ok = ok && m_pimpl->updateProbabilities();
    ok = ok && m_pimpl->updateContactsList(m_contactStates);
    
    m_pimpl->isClockSet = false;
    return true;
}


ContactBayesCollision ContactBayesManager::getCollisionData(const std::string& linkName) const
{
    std::string printPrefix{"[ContactBayesManager::getCollisionData] "};
    if (m_pimpl->manager.find(linkName) == m_pimpl->manager.end())
    {
        std::cerr << printPrefix << "Please run prepare() before calling this method." << std::endl;
        return ContactBayesCollision();
    }
    auto cCollision = m_pimpl->manager.at(linkName);

    return m_pimpl->manager.at(linkName);
}

