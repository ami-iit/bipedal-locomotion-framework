/**
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_PERCEPTION_PCL_PROCESSOR_H
#define BIPEDAL_LOCOMOTION_PERCEPTION_PCL_PROCESSOR_H

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>

namespace BipedalLocomotion {

namespace Perception {

namespace Features {


struct PCLProcessorParameters
{
    std::array<double, 3> voxelSizeDownsampling{0.05, 0.05, 0.05}; /**< voxel grid leaf size (x, y , z) in meters for downsampling based on centroid of all the points  */
    int nrPointsForOutlierEstimation{100}; /**< number of neighboring points to analyze for statistical outlier removal filter*/
    double multiplierForOutlierStdDev{1.0}; /**< multiplier for standard deviation (k-sigma) from the mean distance of neighboring points to the query point for statistical outlier removal. any points larger than k-sigma from the mean will be removed*/

    double spatialClusterTolerance{0.02}; /**< spatial distance in meters to consider the Euclidean clustering of points (L2 euclidean norm) */
    int minNrPointsInCluster{10}; /**< minimum number of points required to detect a cluster */
    int maxNrPointsInCluster{1000};  /**< maximum number of points allowed in a detected cluster */
};

/**
 * A wrapper class to do a basic PCL processing for
 * perception aided locomotion. The available operations,
 * - Downsample a point cloud
 * - Remove outliers
 * - Extract spatial clusters of point cloud
 * - Transform a point cloud
 */
template <class PointType>
class PointCloudProcessor
{
public:
    PointCloudProcessor();
    ~PointCloudProcessor() = default;

    bool initialize(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler);

    /**
     * Downsample the point cloud using voxel grid filter
     * See https://pcl.readthedocs.io/projects/tutorials/en/latest/voxel_grid.html#voxelgrid
     */
    bool downsample(const typename pcl::PointCloud<PointType>::Ptr inCloud,
                    typename pcl::PointCloud<PointType>::Ptr outCloud);

    /**
     * Remove outliers from point cloud based on nearest neighbors
     * See https://pcl.readthedocs.io/projects/tutorials/en/latest/statistical_outlier.html#statistical-outlier-removal
     */
    bool removeOutliers(const typename pcl::PointCloud<PointType>::Ptr inCloud,
                        typename pcl::PointCloud<PointType>::Ptr outCloud);

    /**
     * Extract Euclidean clustering based point cloud clusters
     * See https://pcl.readthedocs.io/projects/tutorials/en/latest/cluster_extraction.html#cluster-extraction
     */
    bool extractClusters(const typename pcl::PointCloud<PointType>::Ptr inCloud,
                         std::vector<pcl::PointIndices>& clusterIndices,
                         std::vector<typename pcl::PointCloud<PointType>::Ptr>& cloudClusters);

    /**
     * Transform point cloud
     * See https://pcl.readthedocs.io/projects/tutorials/en/latest/matrix_transform.html
     */
    bool transform(const typename pcl::PointCloud<PointType>::Ptr inCloud,
                   Eigen::Ref<const Eigen::Matrix4f> transform,
                   typename pcl::PointCloud<PointType>::Ptr outCloud);

    /**
     * Transform point cloud
     * See https://pcl.readthedocs.io/projects/tutorials/en/latest/matrix_transform.html
     */
    bool transform(const typename pcl::PointCloud<PointType>::Ptr inCloud,
                   const Eigen::Affine3f& transform,
                   typename pcl::PointCloud<PointType>::Ptr outCloud);


private:
    bool checkInitialization();
    void updateInternalParameters();

    bool m_initialized{false};
    pcl::VoxelGrid<PointType> m_voxelGridFilter;
    pcl::StatisticalOutlierRemoval<PointType> m_outlierRemover;
    typename pcl::search::KdTree<PointType>::Ptr m_kdTree;
    pcl::EuclideanClusterExtraction<PointType> m_clusterExtractor;
    PCLProcessorParameters m_params;
};

template <class PointType>
PointCloudProcessor<PointType>::PointCloudProcessor()
{
}

template <class PointType>
bool PointCloudProcessor<PointType>::initialize(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
{
    constexpr std::string_view logPrefix = "[PointCloudProcessor::initialize] ";
    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        return false;
    }

    if (!ptr->getParameter("downsample_voxel_size", m_params.voxelSizeDownsampling))
    {
        log()->warn("{} Parameter \"downsample_voxel_size\" not available in the configuration."
                    "Using default name \"(0.05, 0.05, 0.05)\".", logPrefix);
    }

    if (!ptr->getParameter("nr_points_for_outlier_estimation", m_params.nrPointsForOutlierEstimation))
    {
        log()->warn("{} Parameter \"nr_points_for_outlier_estimation\" not available in the configuration."
                    "Using default name \"100\".", logPrefix);
    }

    if (!ptr->getParameter("std_dev_multiplier_for_outlier_estimation", m_params.multiplierForOutlierStdDev))
    {
        log()->warn("{} Parameter \"std_dev_multiplier_for_outlier_estimation\" not available in the configuration."
                    "Using default name \"1.0\".", logPrefix);
    }

    if (!ptr->getParameter("spatial_cluster_tolerance", m_params.spatialClusterTolerance))
    {
        log()->warn("{} Parameter \"spatial_cluster_tolerance\" not available in the configuration."
                    "Using default name \"0.02\".", logPrefix);
    }

    if (!ptr->getParameter("min_points_for_clustering", m_params.minNrPointsInCluster))
    {
        log()->warn("{} Parameter \"min_points_for_clustering\" not available in the configuration."
                    "Using default name \"10\".", logPrefix);
    }

    if (!ptr->getParameter("max_points_for_clustering", m_params.maxNrPointsInCluster))
    {
        log()->warn("{} Parameter \"max_points_for_clustering\" not available in the configuration."
                    "Using default name \"1000\".", logPrefix);
    }

    m_kdTree = pcl::make_shared< pcl::search::KdTree<PointType> >();
    updateInternalParameters();
    m_initialized = true;
    return true;
}


template <class PointType>
void PointCloudProcessor<PointType>::updateInternalParameters()
{
    m_voxelGridFilter.setLeafSize(m_params.voxelSizeDownsampling[0],
                                  m_params.voxelSizeDownsampling[1],
                                  m_params.voxelSizeDownsampling[2]);

    m_outlierRemover.setMeanK(m_params.nrPointsForOutlierEstimation);
    m_outlierRemover.setStddevMulThresh(m_params.multiplierForOutlierStdDev);

    m_clusterExtractor.setClusterTolerance(m_params.spatialClusterTolerance);
    m_clusterExtractor.setMinClusterSize(m_params.minNrPointsInCluster);
    m_clusterExtractor.setMaxClusterSize(m_params.maxNrPointsInCluster);
}

template <class PointType>
bool PointCloudProcessor<PointType>::checkInitialization()
{
    if (m_initialized != true)
    {
        log()->error("[PointCloudProcessor::checkInitialization] "
                     "Processor was not initialized.");
        return false;
    }

    return true;
}

template <class PointType>
bool PointCloudProcessor<PointType>::downsample(const typename pcl::PointCloud<PointType>::Ptr inCloud,
                                                typename pcl::PointCloud<PointType>::Ptr outCloud)
{
    if (!checkInitialization())
    {
        return false;
    }

    if (inCloud == nullptr)
    {
        log()->error("[PointCloudProcessor::downsample] Invalid pointer");
        return false;
    }

    if ((inCloud->size() / m_params.voxelSizeDownsampling[0]) >= std::numeric_limits<int>::max())
    {
        log()->warn("[PointCloudProcessor::downsample] "
                    " Input point cloud size too large for given voxel size x"
                    " This might cause overflow. Returning without any action.");
        return false;
    }

    if ((inCloud->size() / m_params.voxelSizeDownsampling[1]) >= std::numeric_limits<int>::max())
    {
        log()->warn("[PointCloudProcessor::downsample] "
                    " Input point cloud size too large for given voxel size y"
                    " This might cause overflow. Returning without any action.");
        return false;
    }

    if ((inCloud->size() / m_params.voxelSizeDownsampling[2]) >= std::numeric_limits<int>::max())
    {
        log()->warn("[PointCloudProcessor::downsample] "
                    " Input point cloud size too large for given voxel size z"
                    " This might cause overflow. Returning without any action.");
        return false;
    }

    if (inCloud->size() <= m_params.nrPointsForOutlierEstimation)
    {
        log()->error("[PointCloudProcessor::downsample] "
                     "Input point cloud size less than minimium required points for downsampling.");
        return false;
    }

    m_voxelGridFilter.setInputCloud(inCloud);
    m_voxelGridFilter.filter(*outCloud);
    return true;
}

template <class PointType>
bool PointCloudProcessor<PointType>::removeOutliers(const typename pcl::PointCloud<PointType>::Ptr inCloud,
                                                    typename pcl::PointCloud<PointType>::Ptr outCloud)
{
    if (!checkInitialization())
    {
        return false;
    }

    if (inCloud == nullptr)
    {
        log()->error("[PointCloudProcessor::removeOutliers] Invalid pointer");
        return false;
    }

    if (inCloud->size() < m_params.nrPointsForOutlierEstimation)
    {
        log()->error("[PointCloudProcessor::removeOutliers] "
                     "Input point cloud size less than minimium required points for outlier estimation.");
        return false;
    }

    m_outlierRemover.setInputCloud(inCloud);
    m_outlierRemover.filter(*outCloud);
    return true;
}

template <class PointType>
bool PointCloudProcessor<PointType>::extractClusters(const typename pcl::PointCloud<PointType>::Ptr inCloud,
                                                     std::vector<pcl::PointIndices>& clusterIndices,
                                                     std::vector<typename pcl::PointCloud<PointType>::Ptr>& cloudClusters)
{
    if (!checkInitialization())
    {
        return false;
    }

    if (inCloud == nullptr)
    {
        log()->error("[PointCloudProcessor::extractClusters] Invalid pointer");
        return false;
    }

    if (inCloud->size() < m_params.minNrPointsInCluster)
    {
        log()->error("[PointCloudProcessor::extractClusters] "
                     "Input point cloud size less than minimium required points for clustering.");
        return false;
    }

    cloudClusters.clear();
    clusterIndices.clear();
    m_kdTree->setInputCloud(inCloud);
    m_clusterExtractor.setSearchMethod(m_kdTree);
    m_clusterExtractor.setInputCloud(inCloud);
    m_clusterExtractor.extract(clusterIndices);

    for (auto it = clusterIndices.begin(); it != clusterIndices.end(); ++it)
    {
        auto clusteredCloud =  pcl::make_shared<pcl::PointCloud<PointType>>();
        for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            clusteredCloud->push_back ((*inCloud)[*pit]);
        }
        clusteredCloud->width = clusteredCloud->size();
        clusteredCloud->height = 1;
        clusteredCloud->is_dense = false;

        cloudClusters.emplace_back(clusteredCloud);
    }
    return true;
}

template <class PointType>
bool PointCloudProcessor<PointType>::transform(const typename pcl::PointCloud<PointType>::Ptr inCloud,
                                               Eigen::Ref<const Eigen::Matrix4f> transform,
                                               typename pcl::PointCloud<PointType>::Ptr outCloud)
{
    if (!checkInitialization())
    {
        return false;
    }

    if (inCloud == nullptr)
    {
        log()->error("[PointCloudProcessor::transform] Invalid pointer");
        return false;
    }

    pcl::transformPointCloud(*inCloud, *outCloud, transform);

    return true;
}

template <class PointType>
bool PointCloudProcessor<PointType>::transform(const typename pcl::PointCloud<PointType>::Ptr inCloud,
                                               const Eigen::Affine3f& transform,
                                               typename pcl::PointCloud<PointType>::Ptr outCloud)
{
    if (!checkInitialization())
    {
        return false;
    }

    if (inCloud == nullptr)
    {
        log()->error("[PointCloudProcessor::transform] Invalid pointer");
        return false;
    }

    pcl::transformPointCloud(*inCloud, *outCloud, transform);

    return true;
}


} // namespace Features
} // namespace Perception
} // namespace BipedalLocomotion

#endif

