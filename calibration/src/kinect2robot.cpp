// ROS
#include <calibration/utils.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_broadcaster.h>
#include <opencv2/core/eigen.hpp>

// PCL
#include <pcl/registration/transformation_estimation_dual_quaternion.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/visualization/pcl_visualizer.h>

// Owm
// #include <toolbox/cloudToolbox.hh>
// #include <toolbox/helperFunctions.h>
// #include <helperFunctions.hpp>
// #include <pcl_utils.hpp>
// #include <ros_utils.hpp>
// #include <toolbox/estimate_tf.hpp>
#include <pcl/filters/filter.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d_omp.h>


using namespace std;
typedef pcl::PointXYZRGB PointP;
typedef pcl::Normal PointN;
typedef pcl::PointCloud<PointN> NCloud;
typedef pcl::PointCloud<PointP> PCloud;


/*
 * Remember to set the tool offset
 * Check that tool0_controller is in the center of the endeffector
 * Start pose: -0.0019143263446252945, -1.8660934607135218, -1.2928875128375452, -2.857654396687643, -0.003704849873678029, -0.2666247526751917
 */

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
    cout << "Number of clusters: " << inliers->size() << endl;
}

void computeNormals(PCloud::Ptr cloud, NCloud::Ptr cloud_normals, const float nrad = 0.01, bool radiusSearch = true )
{
    cout << "---Normal estimation---" << endl;
    pcl::NormalEstimationOMP<PointP, PointN> ne;
    ne.setViewPoint(0,0,0);
    ne.setInputCloud(cloud);
    if ( radiusSearch)
    {
        cout << "Using radius search." << endl;
        ne.setRadiusSearch(nrad);
    }
    else
    {
        cout << "Using k-means search." << endl;
        ne.setKSearch(nrad);
    }

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
    std::cout << "MSE error: " << mse_error << std::endl;
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
        tool_offset_(0, 3) = 0.00373779;
        tool_offset_(1, 3) = 0.0217116;
        tool_offset_(2, 3) = 0.0671781;
        save_path_ += "/data/";

        cout << "Tool offset matrix:" << endl;
        cout << tool_offset_ << endl;
        ROS_INFO_STREAM("Save path: " << save_path_);
    }

    ~KinectCalib() {}


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
        /// every cloud has the same size
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

        cout << "Got average cloud!" << endl;
        visualizeCloud(cloudPtr, "average");
        std::vector<int> dummy;
        cout << "1" << endl;
        pcl::removeNaNFromPointCloud(*cloudPtr,*cloudPtr,dummy);
        cout << "2" << endl;
        // pcl_utils::removeNearPoints<PointP>(cloudPtr, 0.30);
        cout << "3" << endl;
        PointP * p(new PointP);
        Eigen::Vector3f lab;
        for (size_t i = 0; i < cloudPtr->size(); ++i)
        {
            p->x = cloudPtr->at(i).x;
            p->y = cloudPtr->at(i).y;
            p->z = cloudPtr->at(i).z;
            p->rgb = cloudPtr->at(i).rgb;
            //cout << p.x << " " << p.y << " " << p.z << endl;
            lab = RGB2Lab(p->getRGBVector3i());

            int L = lab[0];
            int a = lab[1];
            int b = lab[2];

            // for green value a < -10
            // for red a > 20
            if ( a > 5 )
            {
                //cout << "Found green" << endl;
                cloudPtr->at(i).r = 255;
                cloudPtr->at(i).g = 0;
                cloudPtr->at(i).b = 0;
                coloredPart->push_back(*p);
            }
        }
        cout << "Number of points in colored cloud: " << coloredPart->size() << endl;
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
            //pcl::io::savePCDFileASCII("/home/antti/Desktop/joujou.pcd",*cloud_normals);
            pcl::copyPointCloud<PointP>(*coloredPart, clusterIdx.at(i), *cluster);
            cout << "Cluster number: " << i << endl;
            //v->spinUntilClosed(cluster,"1");
            //pcl::io::savePCDFileASCII("/home/antti/Desktop/joujou.pcd",*coloredPart);
            pcl::copyPointCloud(*cloud_normals, clusterIdx.at(i), *clusterNormals);
            //pcl::io::savePCDFileASCII("/home/antti/Desktop/joujou.pcd",*clusterNormals);
            //cout << clusterNormals->size() << endl;

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
            cout << "xx " << xx << endl;

            sd.SetValues(y);
            double yy = sd.CalculateVariane();
            cout << "yy " << yy << endl;

            sd.SetValues(z);
            double zz = sd.CalculateVariane();
            cout << "zz " << zz << endl;
            if ( minVariance > (xx+yy+zz)) {
                minVariance = xx+yy+zz;
                objectID = i;
            }
            cout << "Cluster size: " << cluster->size() << endl;
            std::stringstream ss;
            std::stringstream ss2;
            ss << (i + 1) * 10;
            ss2 << (i + 1) * 10 + 1;
            std::string str = ss.str();
            std::string str2 = ss2.str();
            //v->spinUntilClosedWithNormals(cluster, clusterNormals, str, str2);
        }
        cout << "Object ID:" << objectID << endl;
        PCloud::Ptr object( new PCloud);
        pcl::copyPointCloud(*coloredPart, clusterIdx.at(objectID), *object);
        //v->spinUntilClosed(object,"Selected object");


        PointP point = PointP(0, 0, 255);
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*object, centroid);
        cout << "Centroid value: " << "x:" << centroid(0) << " y:" << centroid(1) << " z:"<< centroid(2) << endl;
        point.x =centroid(0);
        point.y =centroid(1);
        point.z =centroid(2);
        // centroidsXYZ_->push_back(point);
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

    }


    Eigen::Matrix4f getRobotTCP()
    {
        Eigen::Matrix4f link_locations;
        get_tf("base", "tool0_controller", link_locations, &buffer_, ros::Time(0));
        Eigen::Matrix4f tcpPose= link_locations * tool_offset_;
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
    }


    void testTransformation() {

        Eigen::Matrix4f Tm;
        loadEigenMatrix(save_path_ + "kinect2base_tf.txt", Tm);
        tf::Vector3 origin;
        origin.setValue(static_cast<double>(Tm(0,3)),static_cast<double>(Tm(1,3)),static_cast<double>(Tm(2,3)));
        tf::Matrix3x3 tf3d;
        tf3d.setValue(static_cast<double>(Tm(0,0)), static_cast<double>(Tm(0,1)), static_cast<double>(Tm(0,2)),
                      static_cast<double>(Tm(1,0)), static_cast<double>(Tm(1,1)), static_cast<double>(Tm(1,2)),
                      static_cast<double>(Tm(2,0)), static_cast<double>(Tm(2,1)), static_cast<double>(Tm(2,2)));

        tf::Quaternion tfqt;
        tf3d.getRotation(tfqt);

        tf::Transform transform;
        transform.setOrigin(origin);
        transform.setRotation(tfqt);
        static tf::TransformBroadcaster br;
        while (ros::ok())
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base", "kinect2_link"));
    }


private:
    ros::NodeHandle nh_;
    ros::Subscriber subColor_;
    ros::Subscriber subCloud_;
    ros::ServiceClient serMoveArm_;

    cv::Mat currentRGB_;
    int counter_;
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
    ros::NodeHandle node_priv("~");
    ROS_INFO("Started kinect_to_robotarm_node");
    KinectCalib cal;
    ros::Rate r(10);
    int keyboardEvent = -1;
    while (ros::ok())
    {

        // cout << "keyboard pressed: " << getch() << endl;
        keyboardEvent = getch();
        if ( keyboardEvent == 'c')
        {
            cout << "Finding marker" << endl;
            cal.findMarker();
        }
        if ( keyboardEvent == 's')
        {
            cout << " pressed, saving points" << endl;
            cal.calculateTransformation();
        }
        if ( keyboardEvent == 'i')
        {
            cout << " pressed, printing point informations" << endl;
            cal.printInfo();
        }
        if ( keyboardEvent == 't')
        {
            cout << " pressed, testing transformation" << endl;
            cal.testTransformation();
        }
        if ( keyboardEvent == 'd')
        {
            cout << " pressed, deleting last added point" << endl;
            cal.popOne();
        }

        ros::spinOnce();// Handle ROS events
        r.sleep();
    }

    ROS_INFO("Stopped kinect_to_robotarm_node");
    return 0;
}


