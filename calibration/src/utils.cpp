//
// Created by antti on 5.9.2019.
//

#include <calibration/utils.h>


Eigen::Vector3f RGB2Lab (const Eigen::Vector3i& colorRGB)
{
    // for sRGB   -> CIEXYZ see http://www.easyrgb.com/index.php?X=MATH&H=02#text2
    // for CIEXYZ -> CIELAB see http://www.easyrgb.com/index.php?X=MATH&H=07#text7

    double R, G, B, X, Y, Z;

    // map sRGB values to [0, 1]
    R = colorRGB[0] / 255.0;
    G = colorRGB[1] / 255.0;
    B = colorRGB[2] / 255.0;

    // linearize sRGB values
    if (R > 0.04045)
        R = pow ( (R + 0.055) / 1.055, 2.4);
    else
        R = R / 12.92;

    if (G > 0.04045)
        G = pow ( (G + 0.055) / 1.055, 2.4);
    else
        G = G / 12.92;

    if (B > 0.04045)
        B = pow ( (B + 0.055) / 1.055, 2.4);
    else
        B = B / 12.92;

    // postponed:
    //    R *= 100.0;
    //    G *= 100.0;
    //    B *= 100.0;

    // linear sRGB -> CIEXYZ
    X = R * 0.4124 + G * 0.3576 + B * 0.1805;
    Y = R * 0.2126 + G * 0.7152 + B * 0.0722;
    Z = R * 0.0193 + G * 0.1192 + B * 0.9505;

    // *= 100.0 including:
    X /= 0.95047;  //95.047;
    //    Y /= 1;//100.000;
    Z /= 1.08883;  //108.883;

    // CIEXYZ -> CIELAB
    if (X > 0.008856)
        X = pow (X, 1.0 / 3.0);
    else
        X = 7.787 * X + 16.0 / 116.0;

    if (Y > 0.008856)
        Y = pow (Y, 1.0 / 3.0);
    else
        Y = 7.787 * Y + 16.0 / 116.0;

    if (Z > 0.008856)
        Z = pow (Z, 1.0 / 3.0);
    else
        Z = 7.787 * Z + 16.0 / 116.0;

    Eigen::Vector3f colorLab;
    colorLab[0] = static_cast<float> (116.0 * Y - 16.0);
    colorLab[1] = static_cast<float> (500.0 * (X - Y));
    colorLab[2] = static_cast<float> (200.0 * (Y - Z));

    return colorLab;
}

int getch() {
    static struct termios oldt, newt;
    tcgetattr( STDIN_FILENO, &oldt);           // save old settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON);                 // disable buffering
    newt.c_cc[VMIN] = 0;
    newt.c_cc[VTIME] = 0;
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings
    int c = getchar();  // read character (non-blocking)

    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
    return c;
}


void get_tf(const std::string source_frame, std::string link_names, Eigen::Matrix4f& link_locations,
            tf2_ros::Buffer * buffer,
            ros::Time time_instance)
{
    ros::Rate rate(120.0);
    geometry_msgs::TransformStamped transformStamped;
    while (ros::ok())
    {
        try
        {
            transformStamped = buffer->lookupTransform(source_frame, link_names, time_instance);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            rate.sleep();
            continue;
        }
        break;
    }
    convertTF2EigenMatrix(transformStamped.transform, link_locations);
}

void convertTF2EigenMatrix(const geometry_msgs::Transform tf, Eigen::Matrix4f& eigen_matrix) {

    eigen_matrix = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f mat3_temp = Eigen::Quaternionf(tf.rotation.w, tf.rotation.x, tf.rotation.y, tf.rotation.z).toRotationMatrix();
    eigen_matrix.block(0,0,3,3) = mat3_temp;
    eigen_matrix(0,3) = tf.translation.x;
    eigen_matrix(1,3) = tf.translation.y;
    eigen_matrix(2,3) = tf.translation.z;
    eigen_matrix(3,3) = 1.0;
}


void saveEigenMatrix(std::string path, Eigen::Matrix4f trans)
{
    std::fstream matrixOut(path.c_str(), std::ios::out);
    matrixOut << trans;
    matrixOut.close();
    return;
}


bool loadEigenMatrix(std::string path, Eigen::Matrix4f &trans)
{
    std::cout << "Loading from: " << path << std::endl;
    std::string line;
    std::fstream input_file (path.c_str(), std::ios::in);
    if ( !input_file ) {throw std::string("Couldn't load the estimation matrix..");}
    for (unsigned int i = 0; i < 4; ++i)
    {
        for (unsigned int j = 0; j < 4; ++j)
        {
            input_file >> line;
            trans(i,j) = atof(line.c_str());
        }
    }
    return true;
}
