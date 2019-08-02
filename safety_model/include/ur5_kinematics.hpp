//
// Created by antti on 23.5.2018.
//

#ifndef PROJECT_UR5_KINEMATICS_HPP
#define PROJECT_UR5_KINEMATICS_HPP

#include <math.h>
#include <vector>
#include <string>
#include <iostream>
class Ur5Kinematics
{

public:
    Ur5Kinematics();
    ~Ur5Kinematics();

    std::vector<std::vector<double> > DH(double a, double d, double alpha, double theta);
    std::vector<std::vector<double> > vector_multiplication(std::vector<std::vector<double> > v1,
                                                            std::vector<std::vector<double> > v2);
    void forward_kinematics(const std::vector<double> joint_vals, std::vector<double> & output);
    void get_links_XYZ(std::vector<double> joint_vals, std::vector<std::vector<double> >& output);
    std::vector<double> get_translation(std::vector<std::vector<double> > T);
    // void inverse_kinematics();
    void get_links_XYZ_corrected(const std::vector<double> & joint_vals, std::vector<std::vector<double> >& output,
                                                std::string tool="");
    void get_tf(const std::vector<double> & joint_vals, std::string frame_name,
                std::vector<std::vector<double> >& output);

    bool valid_input(const std::vector<double> & joint_vals);


    template <typename T>
    void print2DVector(T Vec)
    {
        for (int i = 0; i < Vec.size(); ++i) {
            for (int j = 0; j < Vec[0].size(); ++j) {
                std::cout << Vec[i][j] << ' ';
            }
            std::cout << std::endl;
        }
    }


private:
    std::vector<std::string> link_names;
    std::vector<double> as_;
    std::vector<double> ds_;
    std::vector<double> alphas_;
    std::vector<std::vector<double>> tf_upper_arm_link_;
    std::vector<std::vector<double>> tf_corrected_forearm_link_;
    std::vector<std::vector<double>> tf_robotiq_85_gripper_tip_;
    std::vector<std::vector<double>> tf_onrobot_rg2_tip_;
    double joint_threshold_;
    const int n_joints_;
};




#endif //PROJECT_UR5_KINEMATICS_HPP
