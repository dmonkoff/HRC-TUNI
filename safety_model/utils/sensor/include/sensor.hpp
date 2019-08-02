#ifndef SENSOR_SENSOR_HPP
#define SENSOR_SENSOR_HPP

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <image_transport/subscriber_filter.h>

// OpenCV
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>




class Sensor {

public:

    Sensor();
    virtual ~Sensor();
    void read_intrinsic_camera_info(const sensor_msgs::CameraInfo::ConstPtr msg_camera_info, cv::Mat &camera_intrinsic) const;
    void read_img(const sensor_msgs::Image::ConstPtr msg_image, cv::Mat &image) const;


    /// setters and getters
    void set_intrinsic(cv::Mat &camera_mat);
    void set_image_size(const size_t width, const size_t height);
    cv::Mat get_intrinsic() const;
    cv::Size get_image_size() const;


private:
    cv::Mat intrinsic_camera_matrix_;
    cv::Size size_;
};


#endif //SENSOR_SENSOR_HPP
