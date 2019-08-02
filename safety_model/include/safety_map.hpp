#ifndef PROJECT_SAFETY_MAP_HPP
#define PROJECT_SAFETY_MAP_HPP
#include "depth_camera.hpp"
#include "pcl_utils.hpp"
#include "ros_utils.hpp"
#include "types.hpp"
#include "helper_functions.hpp"
#include <pcl/common/transforms.h>

template <typename T>
struct Vec2 {
    T x;
    T y;
};

class SafetyMap
{
  public:

    SafetyMap();
    ~SafetyMap();
    void create_3D_map(const DepthCamera * cam, const cv::Mat& depth, const cv::Mat& color,
                       MatrixDouble transformation, const float cluster_tolerance,
                       const float min_number_inliers);

    void create_2D_map(DepthCamera * cam, int scale);
    void update_safety_maps(pcl::PointXYZ * p, unsigned int r, unsigned int c);
    void update_safety_maps(pcl::PointXYZ * p, unsigned int r, unsigned int c, double value);
    double check_distance(const pcl::PointXYZ * p, int r, int c);
    cv::Mat get_2D_zone(const MatrixDouble &link_locations, double dynamic_workspace_size, int offset);
    pcl::PointCloud<PointA>::Ptr get_3D_map();

    void visualize_2D_Map();
    bool point_on_surface(PointL * itP, cv::Mat & drawing);


    template <typename PointType>
    void update_safety_map(PointType * p)
    {
        map_2D_.at<double>(p->y * map_2D_scale_ + map_2D_middle_point_.y,
                           p->x * map_2D_scale_ + map_2D_middle_point_.x) = p->z;
    }

    template <typename PointType>
    double check_distance(PointType * p)
    {
        return fabs((map_2D_.at<double>(p->y * map_2D_scale_ + map_2D_middle_point_.y,
                                        p->x * map_2D_scale_ + map_2D_middle_point_.x) - p->z));
    }



  private:

    cv::Size map_2D_size_;
    Vec2<int> map_2D_middle_point_;
    int map_2D_scale_;
    cv::Mat map_2D_;
    pcl::PointCloud<PointA>::Ptr map_3D_;
};


#endif //PROJECT_SAFETY_MAP_HPP
