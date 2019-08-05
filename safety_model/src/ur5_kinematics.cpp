//
// Created by antti on 23.5.2018.
//

#include "ur5_kinematics.hpp"


Ur5Kinematics::Ur5Kinematics() : n_joints_(6) {
    as_ = {0, -0.425, -0.39225, 0, 0, 0};
    ds_ = {0.089159, 0, 0, 0.10915, 0.09465, 0.0823};
    alphas_ = {M_PI / 2.0f, 0, 0, M_PI / 2.0f, -M_PI / 2.0f, 0};
    joint_threshold_ = 0.0001;

    /// extra links
    tf_upper_arm_link_.resize(4, std::vector<double>(4,0.0));
    tf_corrected_forearm_link_.resize(4, std::vector<double>(4,0.0));
    tf_robotiq_85_gripper_tip_.resize(4, std::vector<double>(4,0.0));
    tf_onrobot_rg2_tip_.resize(4, std::vector<double>(4,0.0));

    tf_upper_arm_link_[0][0] = tf_upper_arm_link_[1][1] = tf_upper_arm_link_[2][2] = tf_upper_arm_link_[3][3] = 1.0;
    tf_upper_arm_link_[0][3] = tf_upper_arm_link_[1][3] = 0.0;
    tf_upper_arm_link_[2][3] = 0.136;

    tf_corrected_forearm_link_[0][0] = tf_corrected_forearm_link_[1][1] = tf_corrected_forearm_link_[2][2] = tf_corrected_forearm_link_[3][3] = 1.0;
    tf_corrected_forearm_link_[0][3] = tf_corrected_forearm_link_[1][3] = 0.0;
    tf_corrected_forearm_link_[2][3] = 0.136;

    tf_robotiq_85_gripper_tip_[0][0] = tf_robotiq_85_gripper_tip_[1][1] = tf_robotiq_85_gripper_tip_[2][2] = tf_robotiq_85_gripper_tip_[3][3] = 1.0;
    tf_robotiq_85_gripper_tip_[0][3] = tf_robotiq_85_gripper_tip_[1][3] = 0.0;
    tf_robotiq_85_gripper_tip_[2][3] = 0.16; /// tool0 to almost gripper tip

    tf_onrobot_rg2_tip_[0][0] = tf_onrobot_rg2_tip_[1][1] = tf_onrobot_rg2_tip_[2][2] = tf_onrobot_rg2_tip_[3][3] = 1.0;
    tf_onrobot_rg2_tip_[0][3] = tf_onrobot_rg2_tip_[1][3] = 0.0;
    tf_onrobot_rg2_tip_[2][3] = 0.186;

    print2DVector(tf_upper_arm_link_);
    std::cout << std::endl;
    print2DVector(tf_corrected_forearm_link_);
}

Ur5Kinematics::~Ur5Kinematics() {}

bool Ur5Kinematics::valid_input(const std::vector<double> & joint_vals)
{

    /// Exepction
    if ( joint_vals.size() > n_joints_) {
        std::cout << "Invalid joint values" << std::endl;
        return false;
    }
    return true;

}


std::vector<std::vector<double> > Ur5Kinematics::DH(double a, double d, double alpha, double theta) {


    // double dh_[4][4];
    std::vector<std::vector<double> > dh_;
    dh_.resize(4, std::vector<double>(4));
    dh_[0][0]  = cos(theta);
    dh_[0][1]  = -sin(theta) * cos(alpha);
    dh_[0][2]  = sin(theta) * sin(alpha);
    dh_[0][3]  = a * cos(theta);

    dh_[1][0]  = sin(theta);
    dh_[1][1]  = cos(theta) * cos(alpha);
    dh_[1][2]  = -cos(theta) * sin(alpha);
    dh_[1][3]  = a * sin(theta);

    dh_[2][0]  = 0;
    dh_[2][1]  = sin(alpha);
    dh_[2][2]  = cos(alpha);
    dh_[2][3]  = d;

    dh_[3][0]  = 0;
    dh_[3][1]  = 0;
    dh_[3][2]  = 0;
    dh_[3][3]  = 1;


    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            if ( fabs(dh_[i][j]) < joint_threshold_)
                dh_[i][j] = 0.0;
        }
    }
    return dh_;
}

std::vector<std::vector<double> > Ur5Kinematics::vector_multiplication(std::vector<std::vector<double> > v1,
                                                                       std::vector<std::vector<double> > v2)
{
    std::vector<std::vector<double> > output;
    output.resize(4, std::vector<double>(4));
    for (size_t i = 0; i < v2.size(); ++i) {
        for (size_t j = 0; j < v1.size(); ++j) {
            output[j][i] = v1[j][0]*v2[0][i] + v1[j][1]*v2[1][i] + v1[j][2]*v2[2][i] + v1[j][3]*v2[3][i];
        }
    }
    return output;
}

std::vector<double> Ur5Kinematics::get_translation(std::vector<std::vector<double> > T) {
    std::vector<double> output;
    output.resize(3);
    output[0] = T[0][3];
    output[1] = T[1][3];
    output[2] = T[2][3];
    return output;
}

void Ur5Kinematics::get_links_XYZ_corrected(const std::vector<double> & joint_vals,
                                            std::vector<std::vector<double> >& output, std::string tool)
{
    if (!valid_input(joint_vals))
        return;
    std::vector<std::vector<double> > T01 = DH(as_[0], ds_[0], alphas_[0], joint_vals[0]);
    std::vector<std::vector<double> > T12 = DH(as_[1], ds_[1], alphas_[1], joint_vals[1]);
    std::vector<std::vector<double> > T23 = DH(as_[2], ds_[2], alphas_[2], joint_vals[2]);
    std::vector<std::vector<double> > T34 = DH(as_[3], ds_[3], alphas_[3], joint_vals[3]);
    std::vector<std::vector<double> > T45 = DH(as_[4], ds_[4], alphas_[4], joint_vals[4]);
    std::vector<std::vector<double> > T56 = DH(as_[5], ds_[5], alphas_[5], joint_vals[5]);
    std::vector<std::vector<double> > T02 = vector_multiplication(T01, T12);
    std::vector<std::vector<double> > T03 = vector_multiplication(T02, T23);
    std::vector<std::vector<double> > T04 = vector_multiplication(T03, T34);
    std::vector<std::vector<double> > T05 = vector_multiplication(T04, T45);
    std::vector<std::vector<double> > T06 = vector_multiplication(T05, T56);

    if (tool.empty()) {
        output.resize(9, std::vector<double>(3, 0.0));
        std::vector<std::vector<double> > T_upper_arm_link = vector_multiplication(T01, tf_upper_arm_link_);
        std::vector<std::vector<double> > T_forearm_link = vector_multiplication(T02, tf_corrected_forearm_link_);

        output[0] = {0.0,0.0,0.0};
        output[1] = this->get_translation(T01);
        output[2] = this->get_translation(T_upper_arm_link);
        output[3] = this->get_translation(T_forearm_link);
        output[4] = this->get_translation(T02);
        output[5] = this->get_translation(T03);
        output[6] = this->get_translation(T04);
        output[7] = this->get_translation(T05);
        output[8] = this->get_translation(T06);
    }
    else if( tool == "robotiq_85_gripper")
    {
        output.resize(9, std::vector<double>(3, 0.0));
        std::vector<std::vector<double> > T_upper_arm_link = vector_multiplication(T01, tf_upper_arm_link_);
        std::vector<std::vector<double> > T_forearm_link = vector_multiplication(T02, tf_corrected_forearm_link_);
        std::vector<std::vector<double> > T_gripper_link = vector_multiplication(T06, tf_robotiq_85_gripper_tip_);
        output[0] = {0.0,0.0,0.0};
        output[1] = this->get_translation(T01);
        output[2] = this->get_translation(T_upper_arm_link);
        output[3] = this->get_translation(T_forearm_link);
        output[4] = this->get_translation(T02);
        output[5] = this->get_translation(T03);
        output[6] = this->get_translation(T04);
        output[7] = this->get_translation(T05);
        output[8] = this->get_translation(T_gripper_link); // safely skip tool0 location, its along the same axis
    } else {
        std::cout << "Unknown tool!" << std::endl;
    }


}

void Ur5Kinematics::get_tf(const std::vector<double> & joint_vals, std::string frame_name,
                           std::vector<std::vector<double> >& output)
{

    if (!valid_input(joint_vals))
        return;

    if ( frame_name == "robotiq_85_gripper" )
    {
        std::vector<std::vector<double> > T01 = DH(as_[0], ds_[0], alphas_[0], joint_vals[0]);
        std::vector<std::vector<double> > T12 = DH(as_[1], ds_[1], alphas_[1], joint_vals[1]);
        std::vector<std::vector<double> > T23 = DH(as_[2], ds_[2], alphas_[2], joint_vals[2]);
        std::vector<std::vector<double> > T34 = DH(as_[3], ds_[3], alphas_[3], joint_vals[3]);
        std::vector<std::vector<double> > T45 = DH(as_[4], ds_[4], alphas_[4], joint_vals[4]);
        std::vector<std::vector<double> > T56 = DH(as_[5], ds_[5], alphas_[5], joint_vals[5]);
        std::vector<std::vector<double> > T02 = vector_multiplication(T01, T12);
        std::vector<std::vector<double> > T03 = vector_multiplication(T02, T23);
        std::vector<std::vector<double> > T04 = vector_multiplication(T03, T34);
        std::vector<std::vector<double> > T05 = vector_multiplication(T04, T45);
        std::vector<std::vector<double> > T06 = vector_multiplication(T05, T56);
        output = vector_multiplication(T06, tf_robotiq_85_gripper_tip_);
    }
    else if ( frame_name == "onrobot_rg2" )
    {
        std::vector<std::vector<double> > T01 = DH(as_[0], ds_[0], alphas_[0], joint_vals[0]);
        std::vector<std::vector<double> > T12 = DH(as_[1], ds_[1], alphas_[1], joint_vals[1]);
        std::vector<std::vector<double> > T23 = DH(as_[2], ds_[2], alphas_[2], joint_vals[2]);
        std::vector<std::vector<double> > T34 = DH(as_[3], ds_[3], alphas_[3], joint_vals[3]);
        std::vector<std::vector<double> > T45 = DH(as_[4], ds_[4], alphas_[4], joint_vals[4]);
        std::vector<std::vector<double> > T56 = DH(as_[5], ds_[5], alphas_[5], joint_vals[5]);
        std::vector<std::vector<double> > T02 = vector_multiplication(T01, T12);
        std::vector<std::vector<double> > T03 = vector_multiplication(T02, T23);
        std::vector<std::vector<double> > T04 = vector_multiplication(T03, T34);
        std::vector<std::vector<double> > T05 = vector_multiplication(T04, T45);
        std::vector<std::vector<double> > T06 = vector_multiplication(T05, T56);
        output = vector_multiplication(T06, tf_onrobot_rg2_tip_);
    }
    else
    {
        std::cerr << "Unknown frame" << std::endl;
    }
}
