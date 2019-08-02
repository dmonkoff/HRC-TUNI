//
// Created by antti on 8.6.2018.
//



#include "work_object.hpp"
#include "filter.hpp"

WorkObject::WorkObject(){};
WorkObject::~WorkObject(){};
void WorkObject::get_control_points(std::vector<std::vector<double>> & control_points, WorkObject::DoubleMatrix & reference_frame)
{
    if ( type_ == CYLINDER)
    {
        // std::vector<std::vector<double> > control_points;
        control_points.resize(2);
        Vec3 c1_robot_base, c2_robot_base;
        c1_robot_base.x = reference_frame[0][0]*control_points_[0].x + reference_frame[0][1]*control_points_[0].y +
                          reference_frame[0][2]*control_points_[0].z + reference_frame[0][3];
        c1_robot_base.y = reference_frame[1][0]*control_points_[0].x + reference_frame[1][1]*control_points_[0].y +
                          reference_frame[1][2]*control_points_[0].z + reference_frame[1][3];
        c1_robot_base.z = reference_frame[2][0]*control_points_[0].x + reference_frame[2][1]*control_points_[0].y +
                          reference_frame[2][2]*control_points_[0].z + reference_frame[2][3];

        c2_robot_base.x = reference_frame[0][0]*control_points_[1].x + reference_frame[0][1]*control_points_[1].y +
                          reference_frame[0][2]*control_points_[1].z + reference_frame[0][3];
        c2_robot_base.y = reference_frame[1][0]*control_points_[1].x + reference_frame[1][1]*control_points_[1].y +
                          reference_frame[1][2]*control_points_[1].z + reference_frame[1][3];
        c2_robot_base.z = reference_frame[2][0]*control_points_[1].x + reference_frame[2][1]*control_points_[1].y +
                          reference_frame[2][2]*control_points_[1].z + reference_frame[2][3];
        control_points[0] = {c1_robot_base.x, c1_robot_base.y, c1_robot_base.z};
        control_points[1] = {c2_robot_base.x, c2_robot_base.y, c2_robot_base.z};
        // control_points.push_back(c1_robot_base);
        // control_points.push_back(c2_robot_base);
    }
}

void WorkObject::set_control_point(float width, float height, int type)
{
    /// with cylinder two control points are made to both side of the cylinder
    if (type == CYLINDER) {
        control_points_.resize(2);
        Vec3 c1;
        Vec3 c2;
        c1.x = 0.0;
        c1.y = width / 2;
        c1.z = 0.0;
        c2.x = 0.0;
        c2.y = (-1) * (width / 2);
        c2.z = 0.0;
        control_points_[0] = c1;
        control_points_[1] = c2;
        type_ = CYLINDER;
    } else {
        std::cerr << "Unknown object type" << std::endl;
    }

}

WorkObject::Vec3 WorkObject::get_pick_locations()
{
    return pick_location_;
}

WorkObject::Vec3 WorkObject::get_drop_locations()
{
    return drop_location_;
}

void WorkObject::set_manipulated(bool manipulated)
{
    manipulated_ = true;
}


void WorkObject::set_pick_location(Vec3 loc)
{
    pick_location_ = loc;
}
void WorkObject::set_drop_location(Vec3 loc)
{
    drop_location_ = loc;
}

bool WorkObject::get_manipulated()
{
    return manipulated_;
}

void WorkObject::set_reference_frame(WorkObject::DoubleMatrix & reference)
{
    tool_reference_frame_ = reference;
}

bool WorkObject::point_inside_object(pcl::PointXYZ * test_p, float squared_radius)
{
    /*
    if ( type_ == CYLINDER )
    {
        std::vector<Vec3> control_points;
        this->get_control_points(control_points);
        double squared_cylinder_length = (control_points[0].x - control_points[1].x) * (control_points[0].x - control_points[1].x) +
                                         (control_points[0].y - control_points[1].y) * (control_points[0].y - control_points[1].y) +
                                         (control_points[0].z - control_points[1].z) * (control_points[0].z - control_points[1].z);


        if (filter::cylinder_test(&control_points[0], &control_points[1], (float)squared_cylinder_length, squared_radius, test_p) != -1.0f)
        {
            return true;
        }

    }*/
    return false;
}