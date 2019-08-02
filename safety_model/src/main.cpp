#include "ros_detector.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "change_detector");
    ros::NodeHandle node_priv("~");
    ROSDetector rd;
    rd.initialize();
    ros::spin();

    return 0;
}