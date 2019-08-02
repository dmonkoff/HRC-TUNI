#include "filter.hpp"

void filter::filter_organized_cloud(const cv::Mat &depth, const cv::Mat &color, std::vector<int> & cluster_idx,
                                    pcl::PointCloud<PointL>::Ptr &cloud, const DepthCamera * cam,
                                    const MatrixDouble sensor_to_base, const MatrixDouble link_locations, SafetyMap * sm,
                                    cv::Mat & robot_working_zone, const float cloud_difference_threshold,
                                    const std::vector<WorkObject*> & work_objects)
{
    const float badPoint = std::numeric_limits<float>::quiet_NaN();
    cv::Mat lookupY;
    cv::Mat lookupX;
    cam->get_lookups(lookupX, lookupY);
    int idx_counter = 0;

    pcl::PointXYZ temp_pp;

    double x1 = sensor_to_base[0][0];
    double x2 = sensor_to_base[0][1];
    double x3 = sensor_to_base[0][2];
    double x4 = sensor_to_base[0][3];

    double y1 = sensor_to_base[1][0];
    double y2 = sensor_to_base[1][1];
    double y3 = sensor_to_base[1][2];
    double y4 = sensor_to_base[1][3];

    double z1 = sensor_to_base[2][0];
    double z2 = sensor_to_base[2][1];
    double z3 = sensor_to_base[2][2];
    double z4 = sensor_to_base[2][3];
// #pragma omp parallel for
    for(int r = 0; r < depth.rows; ++r)
    {
        PointL *itP = &cloud->points[r * depth.cols];
        const uint16_t *itD = depth.ptr<uint16_t>(r);
        const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);

        const float y = lookupY.at<float>(0, r);
        const float *itX = lookupX.ptr<float>();

        for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itD, ++itC, ++itX, ++idx_counter) //itI
        {

            register const float depthValue = *itD / 1000.0f;
            itP->x = *itX * depthValue;
            itP->y = y * depthValue;
            itP->z = depthValue;

            // TODO: remove temp_pp
            temp_pp.x = itP->x*x1 + itP->y*x2 + itP->z*x3 + x4;
            temp_pp.y = itP->x*y1 + itP->y*y2 + itP->z*y3 + y4;
            temp_pp.z = itP->x*z1 + itP->y*z2 + itP->z*z3 + z4;
            itP->x = temp_pp.x;
            itP->y = temp_pp.y;
            itP->z = temp_pp.z;
            itP->b = itC->val[0];
            itP->g = itC->val[1];
            itP->r = itC->val[2];
            itP->a = 255;
            /// Check for invalid measurements (zero depth, robot is current location and robot in the initial locations)

            /*if (*itD == 0 ) {
                itP->x = itP->y = itP->z = badPoint;
                itP->rgba = 0;
                continue;
            }*/
            // std::cout << "11" << std::endl;
            /// Check nans
            if ( !(pcl_utils::is_valid_point<PointL>(itP))) {
                itP->x = itP->y = itP->z = badPoint;
                itP->rgba = 0;
                // sm->update_safety_maps(&temp_pp,r,c,-1.0);
                continue;
            }

            /// Check if we are outside the cropbox
            if (cam->is_outside_cropbox(itP->x, itP->y, itP->z) ){
                itP->x = itP->y = itP->z = badPoint;
                itP->rgba = 0;
                continue;
            }

            if ( sm->point_on_surface(itP, robot_working_zone)) {
                /// do not set itP to nan because the value will be copied to 2Dmap (pointer)
                sm->update_safety_map<PointL>(itP);
                continue;
            }


            /// finally if the difference is big enough save the point for futher processing
            if ( sm->check_distance<PointL>(itP) > cloud_difference_threshold)
                cluster_idx.push_back(idx_counter);



            /*
            /// check the zones related to robot
            bool robot_point = false;
            for (size_t i = 0; i < link_locations.size(); ++i)
            {
                begin.x = link_locations[i][0];
                begin.y = link_locations[i][1];
                begin.z = link_locations[i][2];

                if (sphere_test(&begin, &temp_pp, robot_zone_rad_squared)) {
                    // sm->update_safety_maps(&temp_pp, r, c);
                    robot_point = true;
                    break;
                }

                if ( i == link_locations.size()-1)
                    break;

                end.x = link_locations[i+1][0];
                end.y = link_locations[i+1][1];
                end.z = link_locations[i+1][2];

                float squared_cylinder_length = (begin.x - end.x)*(begin.x - end.x) +
                                                (begin.y - end.y)*(begin.y - end.y) +
                                                (begin.z - end.z)*(begin.z - end.z);
                if ( cylinder_test(&begin, &end, squared_cylinder_length, robot_zone_rad_squared, &temp_pp) != -1.0f )
                {
                    // sm->update_safety_maps(&temp_pp, r, c);
                    robot_point = true;
                    break;
                }
            }

            /// lets skip this points because it belongs to the robot
            if ( robot_point )
                continue;

            // if (!sm->is_visited(&temp_pp, r, c))
            //    continue;
            /// check if carrying and object, if so add to map
            for (size_t j = 0; j < work_objects.size(); ++j) {
                if ( work_objects[j]->get_manipulated() && work_objects[j]->point_inside_object(&temp_pp, robot_zone_rad_squared) )
                {
                    sm->update_safety_maps(&temp_pp, r, c);
                }
            }

            /// check if new point
            // double dist = sm->check_distance(&temp_pp);
            double dist = sm->check_distance(&temp_pp, r, c);
            if ( dist > cloud_difference_threshold) {
                cluster_idx.push_back(idx_counter);
            }*/
        }
    }
}


int filter::find_anomalies(pcl::PointCloud<PointL>::Ptr anomalies_cloud_, const pcl::PointCloud<PointL>::Ptr cloud_,
                           const std::vector<pcl::PointIndices> & inliers,
                           const std::vector<std::vector<double> > &  link_locations,
                           const std::vector<int> & cluster_idx,
                           SafetyMap * sm, cv::Mat & safety_boundary, const std::vector<WorkObject*> & work_objects,
                           Cluster * nearest_object)
{
    int n_anomalies = 0;
    int biggest_cluster_size = 0;
    pcl_utils::set_to_nan<PointL>(anomalies_cloud_);
    for (size_t k = 0; k < inliers.size(); ++k) {
        for (size_t i = 0; i < inliers[k].indices.size(); ++i) {

            double temp_x = 0;
            double temp_y = 0;
            double temp_z = 0;
            int counter = 0;
            if ( sm->point_on_surface(&cloud_->at(cluster_idx[inliers[k].indices[i]]), safety_boundary))
            {
                anomalies_cloud_->at(cluster_idx[inliers[k].indices[i]]).x = cloud_->at(cluster_idx[inliers[k].indices[i]]).x;
                anomalies_cloud_->at(cluster_idx[inliers[k].indices[i]]).y = cloud_->at(cluster_idx[inliers[k].indices[i]]).y;
                anomalies_cloud_->at(cluster_idx[inliers[k].indices[i]]).z = cloud_->at(cluster_idx[inliers[k].indices[i]]).z;
                anomalies_cloud_->at(cluster_idx[inliers[k].indices[i]]).r = 0;
                anomalies_cloud_->at(cluster_idx[inliers[k].indices[i]]).g = 255;
                anomalies_cloud_->at(cluster_idx[inliers[k].indices[i]]).b = 0;

                temp_x += cloud_->at(cluster_idx[inliers[k].indices[i]]).x;
                temp_y += cloud_->at(cluster_idx[inliers[k].indices[i]]).y;
                temp_z += cloud_->at(cluster_idx[inliers[k].indices[i]]).z;
                ++counter;
                ++n_anomalies;
            }


            if ( counter > biggest_cluster_size )
            {
                nearest_object->set_cluster_center(temp_x, temp_y, temp_z);
                biggest_cluster_size = counter;
            }
        }
    }
    return n_anomalies;
}
