//
// Created by antti on 25.7.2018.
//

#include "cluster.hpp"

Cluster::Cluster() : cluster_size_(0) {

    // points_.reserve(10000);

}
Cluster::~Cluster() {}



void Cluster::set_cluster_center(double x, double y, double z)
{
    center_p_.x = x;
    center_p_.y = y;
    center_p_.z = z;
}


Cluster::vec3d Cluster::get_cluster_center()
{
    return center_p_;
}

void Cluster::reset()
{
    center_p_.x = 0.0;
    center_p_.y = 0.0;
    center_p_.z = 0.0;
}