//
// Created by antti on 8.6.2018.
//

#ifndef PROJECT_WORK_OBJECT_HPP
#define PROJECT_WORK_OBJECT_HPP
#include <iostream>
#include <vector>
#include <pcl/point_types.h>

enum ObjectType{
    SPHERE,
    CYLINDER
};



class WorkObject {
public:
    typedef std::vector<std::vector<double> > DoubleMatrix;
    struct Vec3 {
        double x;
        double y;
        double z;
    };

    WorkObject();
    ~WorkObject();
    void set_reference_frame(WorkObject::DoubleMatrix & reference);
    void get_control_points(std::vector<std::vector<double>> & control_points, WorkObject::DoubleMatrix & reference_frame);
    WorkObject::Vec3 get_pick_locations();
    WorkObject::Vec3 get_drop_locations();
    void set_control_point(float width, float height, int type);
    void set_manipulated(bool manipulated);
    bool get_manipulated();
    void set_pick_location(WorkObject::Vec3 loc);
    void set_drop_location(WorkObject::Vec3 loc);
    bool point_inside_object(pcl::PointXYZ * test_p, float squared_radius);

private:
    WorkObject::Vec3 pick_location_;
    WorkObject::Vec3 drop_location_; /// in the future these could be vectors holding multiple pick and place locations
    bool manipulated_;
    ObjectType type_;
    std::vector<WorkObject::Vec3> control_points_;
    DoubleMatrix tool_reference_frame_;
};



#endif //PROJECT_WORK_OBJECT_HPP
