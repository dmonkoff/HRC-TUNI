#include <depth_camera.hpp>

DepthCamera::DepthCamera(){}

DepthCamera::~DepthCamera(){}


void DepthCamera::initialize(const sensor_msgs::CameraInfo::ConstPtr& camera_matrix_input)
{
    cv::Mat camera_matrix_output = cv::Mat::zeros(3, 3, CV_64F);
    this->read_intrinsic_camera_info(camera_matrix_input, camera_matrix_output);
    this->set_intrinsic(camera_matrix_output);
    this->set_image_size(camera_matrix_input->width, camera_matrix_input->height);
    this->create_lookup_table();
}

bool DepthCamera::is_outside_cropbox(double x, double y, double z) const {
    if ( (x < p_mins_[0] || y < p_mins_[1] || z < p_mins_[2]) ||
         (x > p_maxs_[0] || y > p_maxs_[1] || z > p_maxs_[2])) {
        return true;
    }
    return false;
}

void DepthCamera::create_lookup_table() {
    cv::Size img_size = get_image_size();
    cv::Mat camera_matrix = get_intrinsic();
    const float fx = 1.0f / camera_matrix.at<double>(0, 0);
    const float fy = 1.0f / camera_matrix.at<double>(1, 1);
    const float cx = camera_matrix.at<double>(0, 2);
    const float cy = camera_matrix.at<double>(1, 2);
    float *it;

    lookup_Y_ = cv::Mat(1, img_size.height, CV_32F);
    it = lookup_Y_.ptr<float>();
    for(size_t r = 0; r < img_size.height; ++r, ++it)
    {
        *it = (r - cy) * fy;
    }

    lookup_X_ = cv::Mat(1, img_size.width, CV_32F);
    it = lookup_X_.ptr<float>();
    for(size_t c = 0; c < img_size.width; ++c, ++it)
    {
        *it = (c - cx) * fx;
    }
}


void DepthCamera::get_lookups(cv::Mat & X, cv::Mat & Y) const {
    lookup_X_.copyTo(X);
    lookup_Y_.copyTo(Y);
}

bool DepthCamera::set_crop_box( std::vector<double> p_mins, std::vector<double> p_maxs ) {

    if (p_mins[0] >= p_maxs[0] or p_mins[1] >= p_maxs[1] or p_mins[2] >= p_maxs[2])
    {
        std::cerr << "Incorrect boundaries for the cropbox" << std::endl;
        return false;
    }

    p_mins_ = p_mins;
    p_maxs_ = p_maxs;
    return true;
}

void DepthCamera::get_crop_box( std::vector<double> & p_mins, std::vector<double> & p_maxs ) {
    p_mins = p_mins_;
    p_maxs = p_maxs_;
}
