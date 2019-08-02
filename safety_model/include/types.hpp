//
// Created by antti on 23.5.2018.
//

#ifndef PROJECT_TYPES_HPP
#define PROJECT_TYPES_HPP

// #include <message_filters/syn
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactSyncPolicy;
typedef std::vector<std::vector<double> > MatrixDouble;


#define D_WIDTH 512
#define D_HEIGHT 424


#endif //PROJECT_TYPES_HPP
