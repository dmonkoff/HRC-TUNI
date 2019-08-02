#ifndef PCL_PCL_UTILS_HPP
#define PCL_PCL_UTILS_HPP

#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace pcl_utils {
	const float nan_point = std::numeric_limits<float>::quiet_NaN();
	template<class PointT>
	bool is_valid_point(const PointT * point) {
		if (!pcl_isfinite (point->x) ||
			!pcl_isfinite (point->y) ||
			!pcl_isfinite (point->z)) {
			return false;
		}
		return true;
	}
	template<class PointT>
	bool is_valid_point(const PointT  point) {
		if (!pcl_isfinite (point.x) ||
			!pcl_isfinite (point.y) ||
			!pcl_isfinite (point.z)) {
			return false;
		}
		return true;
	}

	template<class PointT>
	typename pcl::PointCloud<PointT>::Ptr initialize_cloud(const unsigned int width, const unsigned int height){
		typename pcl::PointCloud<PointT>::Ptr cloud;
		cloud.reset(new pcl::PointCloud<PointT>);
		cloud->height = height;
		cloud->width = width;
		cloud->points.resize(cloud->height * cloud->width);
		cloud->is_dense = false;
		return cloud;
	}

	template<class PointT>
	void set_to_nan(typename pcl::PointCloud<PointT>::Ptr input) {
		for (size_t i = 0; i < input->size(); ++i) {
			input->at(i).x = input->at(i).y = input->at(i).z = nan_point;
		}
	}

	template<class PointT>
	typename pcl::PointCloud<PointT>::Ptr copy_cloud(typename pcl::PointCloud<PointT>::Ptr input,
			const std::vector<pcl::PointIndices> & inliers, int width, int heigth){

		typename pcl::PointCloud<PointT>::Ptr cloud;
		cloud = pcl_utils::initialize_cloud<PointT>(width, heigth);
		for (size_t k = 0; k < inliers.size(); ++k) {
			for (size_t i = 0; i < inliers[k].indices.size(); ++i) {
				cloud->at(inliers[k].indices[i]).x = input->at(inliers[k].indices[i]).x;
				cloud->at(inliers[k].indices[i]).y = input->at(inliers[k].indices[i]).y;
				cloud->at(inliers[k].indices[i]).z = input->at(inliers[k].indices[i]).z;
				cloud->at(inliers[k].indices[i]).r = 0;
				cloud->at(inliers[k].indices[i]).g = 255;
				cloud->at(inliers[k].indices[i]).b = 0;
			}
		}

		return cloud;
	}

}

#endif //PCL_PCL_UTILS_HPP

