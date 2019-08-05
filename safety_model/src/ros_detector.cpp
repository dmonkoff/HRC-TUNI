

#include "ros_detector.hpp"

// #include <pcl/visualization/cloud_viewer.h>


ROSDetector::ROSDetector() : nh_priv_("~"),
                             nh_(""),
                             it_(nh_),
                             robot_carrying_object_(false),
                             safety_mode_("normal")
{
    image_transport::TransportHints hints_("raw");
    sub_scene_depth_ = new image_transport::SubscriberFilter(it_, "/kinect2/sd/image_depth_rect", 1, hints_);
    sub_scene_color_ = new image_transport::SubscriberFilter(it_, "/kinect2/sd/image_color_rect", 1, hints_);
    sub_camera_info_depth_ = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_, "/kinect2/sd/camera_info", 1);
    sub_camera_info_color_ = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_, "/kinect2/sd/camera_info", 1);
    sync_exact_ = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(1),
            *sub_scene_depth_, *sub_scene_color_, *sub_camera_info_depth_, *sub_camera_info_color_);
    sync_exact_->registerCallback(boost::bind(&ROSDetector::sensor_callback, this, _1, _2, _3, _4));
    tf_listener_ = new tf2_ros::TransformListener(buffer_);
    sub_joint_state_ = nh_.subscribe("/joint_states", 1, &ROSDetector::cb_joint_state, this);
    sub_object_manipulation_ = nh_.subscribe<unity_msgs::ManipulatedObject>("/unity/manipulated_object", 1, &ROSDetector::cb_object_listener, this);
    sub_safety_system_mode_ = nh_.subscribe<std_msgs::String>("/unity/system_mode", 1, &ROSDetector::cb_system_mode_listener, this);
    sub_marker_buttons_ = nh_.subscribe<unity_msgs::MarkerDataArray>("/unity/interaction_markers", 10, &ROSDetector::cb_marker_listener, this);

    dc_ = new DepthCamera();
    sm_ = new SafetyMap();
    ur5_kin_ = new Ur5Kinematics;
    nearest_object_ = new Cluster;

    anomalies_cloud_ = pcl_utils::initialize_cloud<PointL>(D_WIDTH, D_HEIGHT);
    cloud_ = pcl_utils::initialize_cloud<PointL>(D_WIDTH, D_HEIGHT);
    pub_anomalies_ =  nh_.advertise<pcl::PointCloud<PointL>>("/unity/anomalies", 1);
    pub_filtered_ =  nh_.advertise<pcl::PointCloud<PointL>>("/unity/filtered", 1);
    pub_3D_map_ =  nh_.advertise<pcl::PointCloud<PointL>>("/unity/3D_map", 1);
    pub_nearest_obj_ = nh_.advertise<unity_msgs::NearestObject> ("/unity/nearest_object",10);
}

ROSDetector::~ROSDetector()
{
    ROS_INFO_STREAM("Destroying node");
}


void ROSDetector::cb_system_mode_listener(const std_msgs::StringConstPtr& str)
{
    safety_mode_ = str->data;
}


void ROSDetector::cb_marker_listener(unity_msgs::MarkerDataArray marker_array_msg) {
    bool confirm_button = false;
    bool dead_man_button = false;
    for (size_t i = 0; i < marker_array_msg.markers.size(); ++i) {
        if ( marker_array_msg.markers[i].id == "confirm" and marker_array_msg.markers[i].data > params_.interaction_button_threshold )
            confirm_button = true;
        if ( marker_array_msg.markers[i].id == "dead_man" and marker_array_msg.markers[i].data > params_.interaction_button_threshold )
            dead_man_button = true;
    }

    if ( dead_man_button and confirm_button )
    {
        ROS_INFO_STREAM("HEREEE");
        params_.lookup_initialized = false;
    }
}


void ROSDetector::cb_object_listener(unity_msgs::ManipulatedObject object_msg) {

    if ( object_msg.id == 0 ) {
        robot_carrying_object_ = false;
    } else {
        robot_carrying_object_ = true;
    }
}



void ROSDetector::cb_joint_state(const sensor_msgs::JointStateConstPtr& msg)
{
    joint_lock_.lock();
    joint_positions_ = msg->position;
    joint_stamp_ = msg->header.stamp;
    joint_lock_.unlock();
}


void ROSDetector::get_workobject_controlpoints(std::vector<double> joint_values,
                                               std::vector<std::vector<double> > & control_points)
{
    std::vector<double> tcp;
    std::vector<std::vector<double> > tool_reference_frame;
    ur5_kin_->get_tf(joint_values, "onrobot_rg2", tool_reference_frame);
    work_objects_[0]->get_control_points(control_points, tool_reference_frame);
}


void ROSDetector::sensor_callback(const sensor_msgs::Image::ConstPtr& scene_depth,
                                  const sensor_msgs::Image::ConstPtr& scene_color,
                                  const sensor_msgs::CameraInfo::ConstPtr& camera_info_scene_depth,
                                  const sensor_msgs::CameraInfo::ConstPtr& dummy)
{

    if (safety_mode_ == "force_mode")
        return;

    ros::Time s_time = ros::Time::now();
    nearest_object_->reset();
    cv::Mat depth, color;
    if ( !params_.lookup_initialized )
        dc_->initialize(camera_info_scene_depth);

    dc_->read_img(scene_depth, depth);
    dc_->read_img(scene_color, color);

    double time_diff = std::fabs(scene_depth->header.stamp.toSec() - joint_stamp_.toSec());
    if (time_diff > 0.005)
        ROS_WARN("Frame timestamp difference was: %f",time_diff);
    std::vector<std::vector<double> > link_locations;
    joint_lock_.lock();
    ur5_kin_->get_links_XYZ_corrected(joint_positions_, link_locations,"robotiq_85_gripper");
    joint_lock_.unlock();

    if ( !params_.lookup_initialized ) {
        sm_->create_3D_map(dc_, depth, color, params_.sensor_to_base_,
                           params_.cluster_tolerance, params_.min_cluster_size);
        // filter::filter_robot_from_cloud(sm_->get_3D_map(), link_locations, params_.robot_zone_rad);
        filtered_cloud_.reset(new pcl::PointCloud<PointL>);
        sm_->create_2D_map(dc_, params_.map_2D_scale);
        params_.lookup_initialized = true;
        return;
    }

    std::vector<int> cluster_idx;
    cv::Mat robot_workspace = sm_->get_2D_zone(link_locations, params_.robot_zone_rad, 1);
    if (robot_carrying_object_)
    {
        std::vector<std::vector<double> > woc;
        get_workobject_controlpoints(joint_positions_, woc);
        cv::Mat object_workspace = sm_->get_2D_zone(woc, params_.robot_zone_rad, 1);
        cv::bitwise_or(robot_workspace,object_workspace,robot_workspace);
    }

    filter::filter_organized_cloud(depth, color, cluster_idx,
                                   cloud_, dc_,
                                   params_.sensor_to_base_,
                                   link_locations, sm_,
                                   robot_workspace,
                                   params_.cloud_diff_threshold,
                                   work_objects_);


    /// Euclidean clustering to get rid off noise
    pcl::copyPointCloud(*cloud_, cluster_idx, *filtered_cloud_);
    std::vector<pcl::PointIndices> inliers;
    filter::euclidean_clustering<PointL>(filtered_cloud_,inliers,params_.cluster_tolerance,params_.min_cluster_size);


    /// iterate over points left from the clustering
    int n_anomalies = 0;
    cv::Mat safety_area = sm_->get_2D_zone(link_locations, params_.robot_zone_rad + params_.safety_zone_rad, 1);
    if (robot_carrying_object_)
    {
        std::vector<std::vector<double> > woc;
        get_workobject_controlpoints(joint_positions_, woc);
        cv::Mat object_workspace = sm_->get_2D_zone(woc, params_.robot_zone_rad + params_.safety_zone_rad, 1);
        cv::bitwise_or(safety_area, object_workspace, safety_area);
    }
    n_anomalies = filter::find_anomalies(anomalies_cloud_, cloud_, inliers, link_locations, cluster_idx,
                                         sm_, safety_area, work_objects_, nearest_object_);

    this->publish_clusters(n_anomalies);
    if (params_.viz)
        visualize_state(inliers);
    ros::Time e_time = ros::Time::now();
    double dur_ros = (e_time - s_time).toNSec() * 1e-9;
    ROS_INFO("FPS: %f; Frame difference: %f; # Difference pixels: %zu; # Anomalies: %u; # Clusters: %zu",
              1/dur_ros, time_diff, cluster_idx.size(), n_anomalies, inliers.size());
    //ROS_INFO("FPS: %f; Difference pixels: %u; ", 1/dur_ros,cluster_idx.size());
}


void ROSDetector::publish_clusters(int n_anomalies)
{
    unity_msgs::NearestObject nearest_object;
    Cluster::vec3d p = nearest_object_->get_cluster_center();
    nearest_object.coordinates.x = p.x;
    nearest_object.coordinates.y = p.y;
    nearest_object.coordinates.z = p.z;
    if ( n_anomalies > params_.anomalies_threshold)
    {
        nearest_object.distance = 0.0;
        ROS_WARN("Anomalies detected !!");
    }
    else
    {
        nearest_object.distance = std::numeric_limits<double>::max();
    }
    pub_nearest_obj_.publish(nearest_object);
}



void ROSDetector::visualize_state(const std::vector<pcl::PointIndices> & inliers)
{
    filtered_cloud_ = pcl_utils::copy_cloud<PointL>(filtered_cloud_, inliers, D_WIDTH, D_HEIGHT);
    anomalies_cloud_->header.frame_id = "base";
    filtered_cloud_->header.frame_id = "base";
    pub_anomalies_.publish(anomalies_cloud_);
    pub_filtered_.publish(filtered_cloud_);
    pub_3D_map_.publish(sm_->get_3D_map());
    sm_->visualize_2D_Map();
}


void ROSDetector::initialize()
{

    std::vector<double> cropbox = {-1.0, -1.0, -0.1, 0.4, 1.0, 1.0};
    nh_priv_.param("cloud_diff_threshold", params_.cloud_diff_threshold, 0.02f);
    nh_priv_.param("cluster_tolerance", params_.cluster_tolerance, 0.008f);
    nh_priv_.param("min_cluster_size", params_.min_cluster_size, 250);
    nh_priv_.param("safety_map_scale", params_.map_2D_scale, 100);
    nh_priv_.param("anomalies_threshold", params_.anomalies_threshold, 10);
    nh_priv_.param("visualize", params_.viz, true);
    nh_priv_.getParam("workspace_limits", cropbox);
    if ( !dc_->set_crop_box({cropbox[0],cropbox[1],cropbox[2]},{cropbox[3],cropbox[4],cropbox[5]}) )
        ROS_ERROR("Bad values for cropbox");


    /// global parameters loaded from $(find unity_msgs)/configs/config.yaml
    nh_.param("dynamic_workspace_size", params_.robot_zone_rad, 0.25f);
    nh_.param("safety_area_offset", params_.safety_zone_rad, 0.02f);
    nh_.param("interaction_button_thres", params_.interaction_button_threshold, 0.023);



    float width, height;
    nh_.getParam("work_object_width", width);
    WorkObject * workObject = new WorkObject;
    workObject->set_control_point(width, height, CYLINDER);
    work_objects_.push_back(workObject);


    std::vector<geometry_msgs::TransformStamped> link;
    link.resize(1);
    ros_utils::tf::get_tf("base", {"kinect2_link"}, link, &buffer_, ros::Time(0));
    ros_utils::tf::convertTF2vector(link[0].transform, params_.sensor_to_base_);
    for ( const std::vector<double> &v : params_.sensor_to_base_ )
    {
        for ( double x : v ) std::cout << x << ' ';
        std::cout << std::endl;
    }

    ROS_INFO_STREAM("########################################");
    ROS_INFO_STREAM("Cloud difference threshold: " << params_.cloud_diff_threshold);
    ROS_INFO_STREAM("Cluster tolerance: " << params_.cluster_tolerance);
    ROS_INFO_STREAM("Minimum cluster size: " << params_.min_cluster_size);
    ROS_INFO_STREAM("Robot zone radius: " << params_.robot_zone_rad);
    ROS_INFO_STREAM("Safety zone radius: " << params_.safety_zone_rad);
    ROS_INFO_STREAM("Safety map scale: " << params_.safety_zone_rad);
    ROS_INFO_STREAM("Interaction button threshold: " << params_.interaction_button_threshold);
    ROS_INFO("Cropbox: min(x,y,z) max(x,y,z): (%f,%f,%f) (%f,%f,%f)", cropbox[0],cropbox[1],cropbox[2],
             cropbox[3],cropbox[4],cropbox[5]);
    ROS_INFO_STREAM("########################################");
}