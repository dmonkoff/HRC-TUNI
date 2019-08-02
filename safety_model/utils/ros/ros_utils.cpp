#include <ros_utils.hpp>
#include <Eigen/src/Core/Matrix.h>
#include <tf/LinearMath/Transform.h>

namespace ros_utils {

    namespace tf {
        void get_tf(const std::string source_frame, std::vector<std::string> link_names,
                        std::vector<geometry_msgs::TransformStamped>& link_locations,
                                    tf2_ros::Buffer * buffer,
                        ros::Time time_instance) {

            ros::Rate rate(120.0);
            for (size_t k = 0; k < link_names.size(); ++k) {
                std::string end_point = link_names[k];
                geometry_msgs::TransformStamped transformStamped;
                while (ros::ok()){
                    try {
                        transformStamped = buffer->lookupTransform(source_frame, end_point, time_instance);
                    }
                    catch (tf2::TransformException &ex) {
                        ROS_WARN("%s",ex.what());
                        rate.sleep();
                        continue;
                    }
                    break;
                }
                link_locations[k] = transformStamped;
            }
            return;
        }

        void convertTF2vector(const geometry_msgs::Transform tf, std::vector<std::vector<double> >& transformation)
        {
            transformation.resize(4, std::vector<double>(4));
            Eigen::Matrix3f temp = Eigen::Quaternionf(tf.rotation.w, tf.rotation.x, tf.rotation.y, tf.rotation.z).toRotationMatrix();

            /// rotation
            transformation[0][0] = temp(0,0);
            transformation[0][1] = temp(0,1);
            transformation[0][2] = temp(0,2);
            transformation[1][0] = temp(1,0);
            transformation[1][1] = temp(1,1);
            transformation[1][2] = temp(1,2);
            transformation[2][0] = temp(2,0);
            transformation[2][1] = temp(2,1);
            transformation[2][2] = temp(2,2);

            /// translation
            transformation[0][3] = tf.translation.x;
            transformation[1][3] = tf.translation.y;
            transformation[2][3] = tf.translation.z;

            transformation[3][0] = transformation[3][1] = transformation[3][2] = 0.0;
            transformation[3][3] = 1.0;
        }

        void vector2EigenMatrix(const std::vector<std::vector<double> >& transformation, Eigen::Matrix4f& eigen_matrix)
        {
            eigen_matrix = Eigen::Matrix4f::Identity();
            /// rotation
            eigen_matrix(0,0) = transformation[0][0];
            eigen_matrix(0,1) = transformation[0][1];
            eigen_matrix(0,2) = transformation[0][2];
            eigen_matrix(1,0) = transformation[1][0];
            eigen_matrix(1,1) = transformation[1][1];
            eigen_matrix(1,2) = transformation[1][2];
            eigen_matrix(2,0) = transformation[2][0];
            eigen_matrix(2,1) = transformation[2][1];
            eigen_matrix(2,2) = transformation[2][2];

            /// translation
            eigen_matrix(0,3) = transformation[0][3];
            eigen_matrix(1,3) = transformation[1][3];
            eigen_matrix(2,3) = transformation[2][3];
        }
	}
}
