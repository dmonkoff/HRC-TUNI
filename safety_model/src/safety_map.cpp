
#include "safety_map.hpp"

SafetyMap::SafetyMap()
{
    map_3D_ = nullptr;
}
SafetyMap::~SafetyMap() {}


void SafetyMap::create_3D_map(const DepthCamera * cam, const cv::Mat& depth, const cv::Mat& color,
                              MatrixDouble transformation, const float cluster_tolerance,
                              const float min_number_inliers)
{
    cv::Size img_size = cam->get_image_size();
    map_3D_ = pcl_utils::initialize_cloud<PointA>((unsigned int)img_size.width, (unsigned int)img_size.height);
    map_3D_->header.frame_id = "base";
    Eigen::Matrix4f trans;
    ros_utils::tf::vector2EigenMatrix(transformation, trans);
    cam->create_cloud<PointA>(depth, color, map_3D_);
    pcl::transformPointCloud(*map_3D_, *map_3D_, trans);
    cam->filter_using_cropbox<PointA>(map_3D_);
}

void SafetyMap::create_2D_map(DepthCamera * cam, int scale)
{
    std::vector<double> p_min;
    std::vector<double> p_max;
    cam->get_crop_box(p_min, p_max);
    map_2D_scale_ = scale;
    map_2D_size_.width = int(fabs(p_max[0] - p_min[0]) * scale) + 100; // this is the robots x-direction (ncols in image)
    map_2D_size_.height = int(fabs(p_max[1] - p_min[1]) * scale) + 100; // y -directions
    map_2D_middle_point_.x = map_2D_size_.width / 2;
    map_2D_middle_point_.y = map_2D_size_.height / 2;
    map_2D_ = cv::Mat::ones(map_2D_size_.height, map_2D_size_.width, CV_64FC1);
    map_2D_ *= -1.0f;
    ROS_INFO("2D Map dimensions (W,H) (center_x, center_y): (%u,%u) (%u,%u)", map_2D_.cols, map_2D_.rows, map_2D_middle_point_.x, map_2D_middle_point_.y);
    if ( map_3D_ == nullptr )
        return;

    for (auto it = map_3D_->begin(); it != map_3D_->end(); ++it) {
        pcl::PointXYZ temp;
        if (!pcl_utils::is_valid_point(pcl::PointXYZ(it->x, it->y, it->z)))
            continue;
        temp.x = it->x * map_2D_scale_ + map_2D_middle_point_.x;
        temp.y = it->y * map_2D_scale_ + map_2D_middle_point_.y;
        temp.z = it->z;
        map_2D_.at<double>(temp.y, temp.x) = temp.z;
    }
}


void SafetyMap::update_safety_maps(pcl::PointXYZ * p, unsigned int r, unsigned int c)
{
    // ROS_INFO_STREAM(map_3D_->width * r + c);
    map_3D_->points[map_3D_->width * r + c].x = p->x;
    map_3D_->points[map_3D_->width * r + c].y = p->y;
    map_3D_->points[map_3D_->width * r + c].z = p->z;
    map_3D_->points[map_3D_->width * r + c].r = 0;
    map_3D_->points[map_3D_->width * r + c].g = 0;
    map_3D_->points[map_3D_->width * r + c].b = 255;
    map_2D_.at<double>(p->y * map_2D_scale_ + map_2D_middle_point_.y,
                       p->x * map_2D_scale_ + map_2D_middle_point_.x) = p->z;
}


void SafetyMap::update_safety_maps(pcl::PointXYZ * p, unsigned int r, unsigned int c, double value)
{
    // ROS_INFO_STREAM(map_3D_->width * r + c);
    map_3D_->points[map_3D_->width * r + c].x = p->x;
    map_3D_->points[map_3D_->width * r + c].y = p->y;
    map_3D_->points[map_3D_->width * r + c].z = p->z;
    map_3D_->points[map_3D_->width * r + c].r = 0;
    map_3D_->points[map_3D_->width * r + c].g = 0;
    map_3D_->points[map_3D_->width * r + c].b = 255;
    map_2D_.at<double>(p->y * map_2D_scale_ + map_2D_middle_point_.y,
                       p->x * map_2D_scale_ + map_2D_middle_point_.x) = value;
}

double SafetyMap::check_distance(const pcl::PointXYZ * p, int r, int c)
{

    /// todo: what if map_3D_ is nan
    if ( !pcl_utils::is_valid_point<PointA>(map_3D_->points[map_3D_->width * r + c]) )
        return 666;

    return sqrt((map_3D_->points[map_3D_->width * r + c].x - p->x)*(map_3D_->points[map_3D_->width * r + c].x - p->x) +
                (map_3D_->points[map_3D_->width * r + c].y - p->y)*(map_3D_->points[map_3D_->width * r + c].y - p->y) +
                (map_3D_->points[map_3D_->width * r + c].z - p->z)*(map_3D_->points[map_3D_->width * r + c].z - p->z));
}


pcl::PointCloud<PointA>::Ptr SafetyMap::get_3D_map()
{
    return map_3D_;
}

void SafetyMap::visualize_2D_Map()
{
    cv::Mat viz = map_2D_.clone();
    viz += 1.0f; // this is to compensate the negative -1 at line 47
    if (viz.rows > 1000 or viz.cols > 1000)
        cv::resize(viz, viz, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);
    else if ( viz.rows < 400 or viz.cols < 400)
        cv::resize(viz, viz, cv::Size(), 2, 2, cv::INTER_LINEAR);
    double min, max;
    cv::minMaxLoc(viz, &min, &max);
    cv::Mat adjMap;
    cv::convertScaleAbs(viz, adjMap, 255 / max);
    cv::imshow("2D_map",adjMap);
    cv::waitKey(1);
}


cv::Mat SafetyMap::get_2D_zone(const MatrixDouble &link_locations, double dynamic_workspace_size, int offset)
{
    cv::Mat blank_img(map_2D_size_.height, map_2D_size_.width, CV_8UC1, cv::Scalar(0));
    for (size_t k = 0; k < link_locations.size(); ++k)
    {
        std::vector<cv::Point2d> c_points;
        geometry::points_in_circum<cv::Point2d>(dynamic_workspace_size,
                                                cv::Point2d(link_locations[k][0],
                                                            link_locations[k][1]),
                                                // params_.robot_dynamic_workspace_limits.pmax,
                                                // params_.robot_dynamic_workspace_limits.pmin,
                                                c_points,
                                                50);

        for (size_t i = 0; i < c_points.size(); ++i)
        {
            cv::Point2d temp(map_2D_middle_point_.x + c_points[i].x * map_2D_scale_, // m -> mm
                             map_2D_middle_point_.y + c_points[i].y * map_2D_scale_);

            if ( temp.x < offset or temp.y < offset or temp.x > blank_img.cols - offset or temp.y > blank_img.rows - offset)
                continue;


            blank_img.at<uchar>(temp) = 255;
        }
    }
    /// Find contours
    ros::Time s_time = ros::Time::now();
    std::vector<cv::Vec4i> hierarchy;
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours( blank_img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
    /// merge separate contours
    std::vector<std::vector<cv::Point> > points( 1 );
    for (size_t i = 0; i < contours.size(); ++i) {
        for (size_t j = 0; j < contours[i].size(); ++j) {
            points[0].push_back(contours[i][j]);
        }
    }
    std::vector<std::vector<cv::Point> >hull( points.size() );
    for( int i = 0; i < points.size(); i++ ) {
        convexHull( cv::Mat(points[i]), hull[i], false ); }

    cv::Mat drawing = cv::Mat::zeros( blank_img.size(), CV_8UC1 );
    // cv::Mat boundary = cv::Mat::zeros( blank_img.size(), CV_8UC1 );
    for( int i = 0; i< points.size(); i++ ) {
        cv::Scalar color = cv::Scalar( 255 );
        drawContours( drawing, hull, i, color, -1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );
        // drawContours( boundary, hull, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );
    }

    // find_non_zeros(boundary, boundary_points);
    // opencv_utils::findNonZeros(boundary,boundary_points);
    // cv::Mat test;
    // std::string moro;
    // opencv_utils::findNonZeros(test, moro);
    // opencv_utils::homo();
    return drawing;
}


bool SafetyMap::point_on_surface(PointL * itP, cv::Mat & drawing)
{
    cv::Point2d temp(map_2D_middle_point_.x + itP->x * map_2D_scale_, // m -> mm
                     map_2D_middle_point_.y + itP->y * map_2D_scale_);

    if (temp.x > drawing.cols or temp.y > drawing.rows )
        return false;

    if ( 0 < (int)drawing.at<uchar>(temp) ) {
        //std::cout << "Under safety area" << std::endl;
        return true;
    }
    return false;
}