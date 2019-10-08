//
// Created by antti on 5.9.2019.
//

#ifndef CALIBRATION_UTILS_H
#define CALIBRATION_UTILS_H

#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/buffer.h>
#include <fstream>
#include <termio.h>


int getch();
Eigen::Vector3f RGB2Lab (const Eigen::Vector3i& colorRGB);
void convertTF2EigenMatrix(const geometry_msgs::Transform tf, Eigen::Matrix4f& eigen_matrix);
void get_tf(const std::string source_frame, std::string link_names, Eigen::Matrix4f& link_locations,
            tf2_ros::Buffer * buffer,
            ros::Time time_instance);

void saveEigenMatrix(std::string path, Eigen::Matrix4f trans);
bool loadEigenMatrix(std::string path, Eigen::Matrix4f &trans);

namespace math
{
    class StdDeviation
    {
    private:
        int size;
        std::vector<double> value;
        double mean;

    public:
        double CalculateMean()
        {
            double sum = 0;
            for(int i = 0; i < size; i++)
                sum += value[i];
            return (sum / size);
        }

        double CalculateVariane()
        {
            mean = CalculateMean();
            double temp = 0;
            for(int i = 0; i < size; i++)
            {
                temp += (value[i] - mean) * (value[i] - mean) ;
            }
            return temp / size;
        }

        double CalculateSampleVariane()
        {
            mean = CalculateMean();
            double temp = 0;
            for(int i = 0; i < size; i++)
            {
                temp += (value[i] - mean) * (value[i] - mean) ;
            }
            return temp / (size - 1);
        }

        int SetValues(std::vector<double> p)
        {
            size = p.size();
            value.clear();
            mean = 0;
            for(int i = 0; i < size; i++)
                value.push_back(p[i]);
            return 0;
        }

        double GetStandardDeviation()
        {
            return sqrt(CalculateVariane());
        }

        double GetSampleStandardDeviation()
        {
            return sqrt(CalculateSampleVariane());
        }
    };

    template<typename T, typename A>
    float medianOfVec( std::vector<T,A> const& v_input ) {

        std::vector<T> v(v_input);
        const size_t vec_size = v.size();
        std::sort(v.begin(),v.end());
        // The median of the elements is computed accordingly, i.e.
        // if the count of the numbers is even, the average of the
        // middle and one more than middle element is computed otherwise
        // the middle element’s value is computed.
        float result = vec_size % 2 ? (v[v.size()/2 - 1] + v[v.size()/2]) / 2 : v[v.size()/2];
        return result;
    }

    template<typename T, typename A>
    float meanOfVec( std::vector<T,A> const& v_input ) {

        float sum = 0.0f;
        for (typename std::vector<T>::const_iterator it = v_input.begin(); it != v_input.end(); ++it)
            sum += *it;
        return (sum / v_input.size());
    }

    template<typename T, typename A>
    float meanOfSubVec( std::vector<T,A> const& v_input, int ending = -1 ) {

        typename std::vector<T>::const_iterator end_ptr;
        if ( ending == -1 ){
            end_ptr = v_input.end();
        } else {
            end_ptr = v_input.begin() + ending;
        }

        float sum = 0.0f;
        int counter = 0;
        for (typename std::vector<T>::const_iterator it = v_input.begin(); it != end_ptr; ++it) {
            sum += *it;
            counter += 1;
        }

        return (sum / counter);
    }
}


#endif //CALIBRATION_UTILS_H
