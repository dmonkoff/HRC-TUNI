//
// Created by antti on 23.5.2018.
//

#ifndef PROJECT_FILTER_HPP
#define PROJECT_FILTER_HPP
#include "cluster.hpp"
#include "safety_map.hpp"
#include "depth_camera.hpp"
#include "pcl_utils.hpp"
#include "types.hpp"
#include "work_object.hpp"

#include <pcl/segmentation/extract_clusters.h>

namespace filter
{

/// Using 2D maps
void filter_organized_cloud(const cv::Mat &depth, const cv::Mat &color, std::vector<int> & cluster_idx,
                                    pcl::PointCloud<PointL>::Ptr &cloud, const DepthCamera * cam,
                                    const MatrixDouble sensor_to_base, const MatrixDouble link_locations, SafetyMap * sm,
                                    cv::Mat & robot_working_zone, const float cloud_difference_threshold,
                                    const std::vector<WorkObject*> & work_objects);

int find_anomalies(pcl::PointCloud<PointL>::Ptr anomalies_cloud_, const pcl::PointCloud<PointL>::Ptr cloud_,
                   const std::vector<pcl::PointIndices> & inliers,
                   const std::vector<std::vector<double> > &  link_locations,
                   const std::vector<int> & cluster_idx,
                   SafetyMap * sm, cv::Mat & safety_boundary, const std::vector<WorkObject*> & work_objects,
                   Cluster * nearest_object);

template <typename PointType>
void euclidean_clustering(typename pcl::PointCloud<PointType>::Ptr input_cloud, std::vector<pcl::PointIndices> & inliers,
                          const float cluster_tolerance, const float min_cluster_size)
{
    pcl::EuclideanClusterExtraction<PointType> ec;
    ec.setClusterTolerance (cluster_tolerance);
    ec.setMinClusterSize (min_cluster_size);
    ec.setMaxClusterSize (std::numeric_limits<int>::max());
    ec.setInputCloud (input_cloud);
    ec.extract(inliers);
}

}

#endif //PROJECT_FILTER_HPP
