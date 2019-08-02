#ifndef ROS_ROS_UTILS_HPP
#define ROS_ROS_UTILS_HPP

#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>

namespace ros_utils {
	namespace tf {
		void get_tf(const std::string source_frame, std::vector<std::string> link_names,
					std::vector<geometry_msgs::TransformStamped>& link_locations,
									tf2_ros::Buffer * buffer,
					ros::Time time_instance = ros::Time(0));
		void convertTF2vector(const geometry_msgs::Transform tf, std::vector<std::vector<double> >& transformation);
		void vector2EigenMatrix(const std::vector<std::vector<double> >& transformation, Eigen::Matrix4f& eigen_matrix);
	}
}

#endif //ROS_ROS_UTILS_HPP
