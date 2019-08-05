#include <work_object.hpp>
// #include <filter.hpp>
WorkObject::WorkObject(){};
WorkObject::~WorkObject(){};
void WorkObject::get_control_points(std::vector<std::vector<double>> & control_points, WorkObject::DoubleMatrix & reference_frame)
{
    if ( type_ == CYLINDER)
    {
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