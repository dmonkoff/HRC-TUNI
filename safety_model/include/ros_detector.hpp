#ifndef PROJECT_ROS_DETECTOR_HPP
#define PROJECT_ROS_DETECTOR_HPP

#include "depth_camera.hpp"
#include "safety_map.hpp"
#include "work_object.hpp"
#include "ur5_kinematics.hpp"
#include "filter.hpp"

#include <pcl/common/transforms.h>
#include <std_msgs/String.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/JointState.h>
#include "unity_msgs/NearestObject.h"
#include <unity_msgs/ManipulatedObject.h>
#include "unity_msgs/MarkerDataArray.h"

struct DetectorParams {
    bool viz;
    bool lookup_initialized;
    float cloud_diff_threshold;
    float cluster_tolerance;
    double interaction_button_threshold;
    int min_cluster_size;
    float robot_zone_rad;
    float safety_zone_rad;
    int anomalies_threshold;
    int map_2D_scale;
    MatrixDouble sensor_to_base_;
};


class ROSDetector
{


  public:
    ROSDetector();
    ~ROSDetector();
    void initialize();
    void get_workobject_controlpoints(std::vector<double> joint_values,
                                      std::vector<std::vector<double> > & control_points);
    void sensor_callback(const sensor_msgs::Image::ConstPtr& scene_depth,
                         const sensor_msgs::Image::ConstPtr& scene_color,
                         const sensor_msgs::CameraInfo::ConstPtr& camera_info_scene_depth,
                         const sensor_msgs::CameraInfo::ConstPtr& camera_info_scene_dummy);
    void cb_joint_state(const sensor_msgs::JointStateConstPtr& msg);
    void cb_object_listener(unity_msgs::ManipulatedObject object_msg);
    void cb_system_mode_listener(const std_msgs::StringConstPtr& str);
    void cb_marker_listener(unity_msgs::MarkerDataArray marker_array_msg);

    void visualize_state(const std::vector<pcl::PointIndices> & inliers);
    void publish_clusters(int n_anomalies);


  private:
    ros::NodeHandle nh_priv_;
    ros::NodeHandle nh_;
    tf2_ros::Buffer buffer_;

    ros::Subscriber sub_joint_state_;
    ros::Subscriber sub_object_manipulation_;
    ros::Subscriber sub_safety_system_mode_;
    ros::Subscriber sub_marker_buttons_;
    ros::Publisher pub_anomalies_;
    ros::Publisher pub_filtered_;
    ros::Publisher pub_3D_map_;
    ros::Publisher pub_nearest_obj_;

    /// camera subscriber parameters
    tf2_ros::TransformListener* tf_listener_;
    message_filters::Synchronizer<ExactSyncPolicy> *sync_exact_;
    image_transport::ImageTransport it_;
    image_transport::SubscriberFilter *sub_scene_depth_, *sub_scene_color_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> *sub_camera_info_depth_, *sub_camera_info_color_;

    DetectorParams params_;
    DepthCamera * dc_;
    SafetyMap * sm_;
    Ur5Kinematics * ur5_kin_;
    Cluster * nearest_object_;

    std::vector<WorkObject*> work_objects_;
    std::vector<double> joint_positions_;
    ros::Time joint_stamp_;
    boost::mutex joint_lock_;
    bool robot_carrying_object_;
    std::string safety_mode_;



    pcl::PointCloud<PointL>::Ptr cloud_;
    pcl::PointCloud<PointL>::Ptr filtered_cloud_;
    pcl::PointCloud<PointL>::Ptr anomalies_cloud_;
};






#endif //PROJECT_ROS_DETECTOR_HPP
