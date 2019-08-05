#ifndef PROJECT_CLUSTER_HPP
#define PROJECT_CLUSTER_HPP

class Cluster
{
public:
    template <typename T>
    struct Vec3 {
        T x;
        T y;
        T z;
    };
    typedef Vec3<double> vec3d;

    Cluster();
    ~Cluster();
    void reset();
    void set_cluster_center(double x, double y, double z);
    vec3d get_cluster_center();

private:
    // vec3d points_[1000];
    int cluster_size_;
    vec3d center_p_;

};


#endif //PROJECT_CLUSTER_HPP
