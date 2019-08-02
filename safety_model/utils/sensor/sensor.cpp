#include <sensor.hpp>

Sensor::Sensor() {
    intrinsic_camera_matrix_ = cv::Mat::zeros(3, 3, CV_64F);
}

Sensor::~Sensor(){}

void Sensor::read_img(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) const {
    cv_bridge::CvImageConstPtr pCvImage;
    pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
    pCvImage->image.copyTo(image);
}

void Sensor::read_intrinsic_camera_info(const sensor_msgs::CameraInfo::ConstPtr msg_camera_info, cv::Mat &camera_intrinsic) const {

    double *itK = camera_intrinsic.ptr<double>(0, 0);
    for(size_t i = 0; i < 9; ++i, ++itK) {
        *itK = msg_camera_info->K[i];
    }
}

void Sensor::set_intrinsic(cv::Mat &camera_mat) {
    intrinsic_camera_matrix_ = camera_mat;
}

void Sensor::set_image_size(const size_t width, const size_t height){
    size_.width = width;
    size_.height = height;
}


cv::Mat Sensor::get_intrinsic() const {
    return intrinsic_camera_matrix_;
}

cv::Size Sensor::get_image_size() const {
    return size_;
}