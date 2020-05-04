// ROS
#include <calibration/utils.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <opencv2/core/eigen.hpp>
#include <sensor_msgs/PointCloud.h>
#include <pcl_ros/point_cloud.h>

// PCL
#include <pcl/registration/transformation_estimation_dual_quaternion.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d_omp.h>


using namespace std;
typedef pcl::PointXYZRGB PointP;
typedef pcl::Normal PointN;
typedef pcl::PointCloud<PointN> NCloud;
typedef pcl::PointCloud<PointP> PCloud;

void euclideanClustering(PCloud::Ptr cloud, std::vector<pcl::PointIndices> * inliers,
                         float clusterTorelance = 0.03, unsigned int minClustSize = 40,
                         unsigned int maxClustSize = (std::numeric_limits<int>::max)() )
{
    pcl::EuclideanClusterExtraction<PointP> ec;
    ec.setClusterTolerance (clusterTorelance);
    ec.setMinClusterSize (minClustSize);
    ec.setMaxClusterSize (maxClustSize);
    ec.setInputCloud (cloud);
    ec.extract(*inliers);
}

void computeNormals(PCloud::Ptr cloud, NCloud::Ptr cloud_normals, const float nrad = 0.01, bool radiusSearch = true )
{
    pcl::NormalEstimationOMP<PointP, PointN> ne;
    ne.setViewPoint(0,0,0);
    ne.setInputCloud(cloud);
    if ( radiusSearch)
        ne.setRadiusSearch(nrad);
    else
        ne.setKSearch(nrad);
    ne.compute(*cloud_normals);
}


Eigen::Matrix4f estimateTransformation(PCloud sensor, PCloud robot) {
    /*
     * Calculates the transformation to map sensor points to robot frame
     */

    Eigen::Matrix4f trans;
    Eigen::Matrix4f trans_inverse;
    pcl::registration::TransformationEstimationSVD <PointP, PointP> transformation_estimator;
    transformation_estimator.estimateRigidTransformation(sensor, robot, trans);
    transformation_estimator.estimateRigidTransformation(robot, sensor, trans_inverse);
    cout << "Transformation: " << endl << trans << endl << endl;
    cout << "Inverse transformation: " << endl << trans_inverse << endl << endl;


    PCloud transformed_p;
    pcl::transformPointCloud(sensor, transformed_p, trans.matrix());
    float mse_error = 0.0f;
    for (size_t t = 0; t < transformed_p.size(); ++t)
        mse_error += (transformed_p.at(t).getVector3fMap() - robot.at(t).getVector3fMap()).squaredNorm();
    mse_error = mse_error / transformed_p.size();
    cout << "MSE error: " << mse_error << endl;
    return trans;
}


void visualizeCloud(PCloud::Ptr cloud, string wname)
{
    if (cloud->size() < 1)
    {
        ROS_INFO_STREAM("No points in the cloud");
        return;
    }

    pcl::visualization::PCLVisualizer viewer(wname);
    viewer.addPointCloud(cloud);
    viewer.spin();
    viewer.close();
}



class KinectCalib
{
public:
    KinectCalib() : topicColor_("/kinect2/hd/image_color"),
                    topicCloud_("kinect2/sd/points"),
                    counter_(0),
                    sensor2robot_tf_(Eigen::Matrix4f::Identity()),
                    tool_offset_(Eigen::Matrix4f::Identity()),
                    save_path_(ros::package::getPath("calibration"))

    {
        subCloud_ = nh_.subscribe<PCloud>(topicCloud_, 1, &KinectCalib::cbSensor, this);
        currentCLoudView_.reset(new PCloud);
        centroidsXYZ_.reset(new PCloud);
        robotXYZ_.reset(new PCloud);
        tf_listener_ = new tf2_ros::TransformListener(buffer_);
        this->init_node();
        save_path_ += "/data/";
        ROS_INFO_STREAM("Tool offset matrix: ");
        cout << tool_offset_ << endl;
        ROS_INFO_STREAM("Save path: " << save_path_);
        ROS_INFO_STREAM("Base frame: " << base_frame_name_);
        ROS_INFO_STREAM("Tool0 frame: " << tool0_frame_name_);
    }
    ~KinectCalib() {}

    void init_node()
    {
        std::vector<double> tool0_off_temp = {0.00373779, 0.0217116, 0.0671781};
        nh_.param<std::string>("/calibration_kinect2robot/base_frame", base_frame_name_, "base");
        nh_.param<std::string>("/calibration_kinect2robot/tool0_frame", tool0_frame_name_, "tool0");
        nh_.getParam("/calibration_kinect2robot/tool0_offset", tool0_off_temp);
        tool_offset_(0, 3) = tool0_off_temp[0];
        tool_offset_(1, 3) = tool0_off_temp[1];
        tool_offset_(2, 3) = tool0_off_temp[2];
    }

    void cbSensor(const PCloud::ConstPtr& msg) {
        currentCLoudView_ = msg->makeShared();
    }

    PCloud::Ptr getAverageCloud(size_t n_clouds = 5)
    {
        vector<PCloud::Ptr> clouds;
        clouds.resize(n_clouds);
        ros::Rate r(5);
        // collect consecutive frames
        for ( size_t i = 0; i < n_clouds; ++i ) {
            PCloud::Ptr c(new PCloud);
            *c = *currentCLoudView_;
            clouds[i] = c;
            ros::spinOnce();
            r.sleep();
        }

        PCloud::Ptr output(new PCloud);
        for ( size_t t = 0; t < clouds[0]->size(); ++t ) {
            Eigen::Vector3f xyz;
            Eigen::Vector3i rgb;
            xyz[0] = xyz[1] = xyz[2] = rgb[0] = rgb[1] = rgb[2] = 0;
            for (size_t i = 0; i < n_clouds; ++i) {
                xyz += clouds[i]->at(t).getVector3fMap();
                rgb += clouds[i]->at(t).getRGBVector3i();
            }
            xyz = xyz/n_clouds;
            rgb = rgb/n_clouds;
            if ( xyz[0] != xyz[0] ) // check NANs
                continue;

            PointP p;
            p.x = xyz[0];
            p.y = xyz[1];
            p.z = xyz[2];
            p.r = (unsigned char)rgb[0];
            p.g = (unsigned char)rgb[1];
            p.b = (unsigned char)rgb[2];
            output->push_back(p);
        }
        return output;
    }


    PointP locateObject(){
        PCloud::Ptr cloudPtr( new PCloud);
        PCloud::Ptr coloredPart( new PCloud);
        cloudPtr = getAverageCloud();

        visualizeCloud(cloudPtr, "average");
        std::vector<int> dummy;
        pcl::removeNaNFromPointCloud(*cloudPtr,*cloudPtr,dummy);
        PointP * p(new PointP);
        Eigen::Vector3f lab;
        for (size_t i = 0; i < cloudPtr->size(); ++i)
        {
            p->x = cloudPtr->at(i).x;
            p->y = cloudPtr->at(i).y;
            p->z = cloudPtr->at(i).z;
            p->rgb = cloudPtr->at(i).rgb;
            lab = RGB2Lab(p->getRGBVector3i());

            int L = lab[0];
            int a = lab[1];
            int b = lab[2];

            // for green value a < -10
            // for red a > 20
            if ( a > 5 )
            {
                cloudPtr->at(i).r = 255;
                cloudPtr->at(i).g = 0;
                cloudPtr->at(i).b = 0;
                coloredPart->push_back(*p);
            }
        }
        visualizeCloud(coloredPart, "colored");

        // discard small clusters
        pcl::PointCloud<PointP>::Ptr segmentedCloud( new pcl::PointCloud<PointP>);
        std::vector<pcl::PointIndices> clusterIdx;
        euclideanClustering(coloredPart, &clusterIdx, 0.04, 120, 400000);
        pcl::copyPointCloud<PointP>(*coloredPart, clusterIdx, *segmentedCloud);
        visualizeCloud(segmentedCloud, "segmented");

        // calculate normals
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>(coloredPart->width, coloredPart->height));
        computeNormals(coloredPart,cloud_normals,0.02,true);

        pcl::PointCloud<PointP>::Ptr cluster( new pcl::PointCloud<PointP>);
        pcl::PointCloud<pcl::Normal>::Ptr clusterNormals( new pcl::PointCloud<pcl::Normal>);
        math::StdDeviation sd;

        double minVariance = std::numeric_limits<double>::max();
        size_t objectID;
        for (unsigned int i = 0; i < clusterIdx.size(); ++i) {
            pcl::copyPointCloud<PointP>(*coloredPart, clusterIdx.at(i), *cluster);
            pcl::copyPointCloud(*cloud_normals, clusterIdx.at(i), *clusterNormals);

            std::vector<double> x;
            std::vector<double> y;
            std::vector<double> z;
            for (size_t ii = 0; ii < clusterNormals->size(); ++ii) {
                if (isnan(clusterNormals->at(ii).normal_x)||isnan(clusterNormals->at(ii).normal_y)||isnan(clusterNormals->at(ii).normal_z))
                    continue;
                x.push_back(clusterNormals->at(ii).normal_x);
                y.push_back(clusterNormals->at(ii).normal_y);
                z.push_back(clusterNormals->at(ii).normal_z);
            }
            sd.SetValues(x);
            double xx = sd.CalculateVariane();

            sd.SetValues(y);
            double yy = sd.CalculateVariane();

            sd.SetValues(z);
            double zz = sd.CalculateVariane();
            if ( minVariance > (xx+yy+zz)) {
                minVariance = xx+yy+zz;
                objectID = i;
            }

            std::stringstream ss;
            std::stringstream ss2;
            ss << (i + 1) * 10;
            ss2 << (i + 1) * 10 + 1;
            std::string str = ss.str();
            std::string str2 = ss2.str();
        }
        PCloud::Ptr object( new PCloud);
        pcl::copyPointCloud(*coloredPart, clusterIdx.at(objectID), *object);

        PointP point = PointP(0, 0, 255);
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*object, centroid);
        cout << "Centroid value: " << "x:" << centroid(0) << " y:" << centroid(1) << " z:"<< centroid(2) << endl;
        point.x =centroid(0);
        point.y =centroid(1);
        point.z =centroid(2);
        point.r = 0;
        point.g = 0;
        point.b = 255;
        object->push_back(point);
        visualizeCloud(object, "results");
        return point;
    }
    void printInfo()
    {
        cout << "Number of points: " << centroidsXYZ_->size() <<  endl;
        cout << "Robot position: " << endl;
        cout << getRobotTCP() << endl;
    }

    void popOne()
    {
        centroidsXYZ_->erase(centroidsXYZ_->end());
        robotXYZ_->erase(robotXYZ_->end());
    }

    Eigen::Matrix4f getRobotTCP()
    {
        Eigen::Matrix4f link_locations;
        // get_tf("base", "tool0_controller", link_locations, &buffer_, ros::Time(0));
        get_tf(base_frame_name_, tool0_frame_name_, link_locations, &buffer_, ros::Time(0));
        Eigen::Matrix4f tcpPose = link_locations * tool_offset_;
        return tcpPose;
    }

    void findMarker()
    {
        PointP sensor_p = this->locateObject();
        PointP robot_p;
        Eigen::Matrix4f tcpPose = getRobotTCP();
        robot_p.x = tcpPose(0,3); robot_p.y = tcpPose(1,3); robot_p.z = tcpPose(2,3);
        ROS_INFO("Robot point (x,y,z):  (%.4f, %.4f, %.4f)", robot_p.x, robot_p.y, robot_p.z);
        ROS_INFO("Sensor point (x,y,z): (%.4f, %.4f, %.4f)", sensor_p.x, sensor_p.y, sensor_p.z);
        centroidsXYZ_->push_back(sensor_p);
        robotXYZ_->push_back(robot_p);
    }

    void calculateTransformation()
    {
        if (robotXYZ_->size() < 3 or centroidsXYZ_->size() < 3)
        {
            ROS_INFO_STREAM("Must have at least 3 calibration points");
            return;
        }
        sensor2robot_tf_ = estimateTransformation(*centroidsXYZ_, *robotXYZ_);
        saveEigenMatrix(save_path_ + "kinect2base_tf.txt", sensor2robot_tf_);

        Eigen::Matrix3f R;
        R << static_cast<double>(sensor2robot_tf_(0,0)), static_cast<double>(sensor2robot_tf_(0,1)), static_cast<double>(sensor2robot_tf_(0,2)),
             static_cast<double>(sensor2robot_tf_(1,0)), static_cast<double>(sensor2robot_tf_(1,1)), static_cast<double>(sensor2robot_tf_(1,2)),
             static_cast<double>(sensor2robot_tf_(2,0)), static_cast<double>(sensor2robot_tf_(2,1)), static_cast<double>(sensor2robot_tf_(2,2));
        Eigen::Quaternionf q(R);
        cout << "Rotation [x y z w]: " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        cout << "Translation [x y z]: " << sensor2robot_tf_(0,3) << " " << sensor2robot_tf_(1,3) << " " << sensor2robot_tf_(2,3) <<  endl;
     }

private:
    ros::NodeHandle nh_;
    ros::Subscriber subColor_;
    ros::Subscriber subCloud_;

    int counter_;
    string base_frame_name_;
    string tool0_frame_name_;
    pcl::PointCloud<PointP>::Ptr currentCLoudView_;
    pcl::PointCloud<PointP>::Ptr centroidsXYZ_;
    pcl::PointCloud<PointP>::Ptr robotXYZ_;
    const string topicCloud_;
    const string topicColor_;
    Eigen::Matrix4f sensor2robot_tf_;
    Eigen::Matrix4f tool_offset_;
    string save_path_;

    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener* tf_listener_;
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "kinect_to_robotarm_node");
    ROS_INFO("Started kinect_to_robotarm_node");
    KinectCalib cal;
    ros::Rate r(10);
    int keyboardEvent = -1;
    while (ros::ok())
    {
        keyboardEvent = getch();
        if ( keyboardEvent == 'c')
        {
            cout << " pressed, finding marker!" << endl;
            cal.findMarker();
        }
        if ( keyboardEvent == 's')
        {
            cout << " pressed, calculating transformation!" << endl;
            cal.calculateTransformation();
        }
        if ( keyboardEvent == 'i')
        {
            cout << " pressed, printing info!" << endl;
            cal.printInfo();
        }
        if ( keyboardEvent == 'd')
        {
            cout << " pressed, deleting last added point!" << endl;
            cal.popOne();
        }
        ros::spinOnce();
        r.sleep();
    }

    ROS_INFO("Stopped kinect_to_robotarm_node");
    return 0;
}


