#ifndef SENSOR_DEPTHCAMERA_HPP
#define SENSOR_DEPTHCAMERA_HPP

#include <sensor.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZRGBL PointL;
typedef pcl::PointXYZRGBA PointA;

class DepthCamera: public Sensor {

public:

    DepthCamera();
    ~DepthCamera();
    void initialize(const sensor_msgs::CameraInfo::ConstPtr& camera_matrix_input);
	void create_lookup_table();
    bool set_crop_box( std::vector<double> p_mins, std::vector<double> p_maxs );
	void get_crop_box( std::vector<double> & p_mins, std::vector<double>& p_maxs );
    void get_lookups(cv::Mat & X, cv::Mat & Y) const;
	bool is_outside_cropbox(double x, double y, double z) const;

	template <typename PointType>
	void create_cloud(const cv::Mat &depth, const cv::Mat &color, typename pcl::PointCloud<PointType>::Ptr &cloud) const
	{
		const float badPoint = std::numeric_limits<float>::quiet_NaN();
		for(int r = 0; r < depth.rows; ++r)
		{
			PointType *itP = &cloud->points[r * depth.cols];
			const uint16_t *itD = depth.ptr<uint16_t>(r);
			const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);
			const float y = lookup_Y_.at<float>(0, r);
			const float *itX = lookup_X_.ptr<float>();

			for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itD, ++itC, ++itX)
			{
				register const float depthValue = *itD / 1000.0f;
				// Check for invalid measurements
				if(*itD == 0) {
					// not valid
					itP->x = itP->y = itP->z = badPoint;
					itP->rgba = 0;
					continue;
				}

				itP->z = depthValue;
				itP->x = *itX * depthValue;
				itP->y = y * depthValue;
				itP->b = itC->val[0];
				itP->g = itC->val[1];
				itP->r = itC->val[2];
				itP->a = 255;

			}
		}
	}

	template<typename PointType>
	void filter_using_cropbox(typename pcl::PointCloud<PointType>::Ptr & cloud ) const {
		const float badPoint = std::numeric_limits<float>::quiet_NaN();
		for (typename pcl::PointCloud<PointType>::iterator it = cloud->begin(); it != cloud->end(); ++it) {
			if (is_outside_cropbox(it->x, it->y, it->z))
				it->x = it->y = it->z = badPoint;
		}
	}

private:
	cv::Mat lookup_X_, lookup_Y_;
	std::vector<double> p_mins_, p_maxs_;
};










#endif //SENSOR_DEPTHCAMERA_HPP
