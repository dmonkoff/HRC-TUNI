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
    void get_control_points(std::vector<std::vector<double>> & control_points, WorkObject::DoubleMatrix & reference_frame);
    void set_control_point(float width, float height, int type);

private:
    WorkObject::Vec3 pick_location_;
    WorkObject::Vec3 drop_location_; /// in the future these could be vectors holding multiple pick and place locations
    bool manipulated_;
    ObjectType type_;
    std::vector<WorkObject::Vec3> control_points_;
};



#endif //PROJECT_WORK_OBJECT_HPP
