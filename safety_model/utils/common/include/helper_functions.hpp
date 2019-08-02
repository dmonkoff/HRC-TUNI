#ifndef HELPERFUNCTIONS_H
#define HELPERFUNCTIONS_H
#define PI 3.14159265

namespace common_utils
{
    template<typename T>
    T clip(const T& n, const T& lower, const T& upper) {
        return std::max(lower, std::min(n, upper));
    }

};

namespace geometry {
    template <typename T>
    void points_in_circum(double radius, T center_point, std::vector<T>& points, size_t n_points = 25) {
            for (size_t i = 0; i < n_points; ++i) {
                T new_point;
                new_point.x = cos(2*PI/n_points*i)*radius+center_point.x;
                new_point.y = sin(2*PI/n_points*i)*radius+center_point.y;
                points.push_back(new_point);
            }
    }

    template <typename T>
    void points_in_circum(double radius, const T& center_point, const T& upper, const T& lower, std::vector<T>& points,
                          size_t n_points = 25)
    {
        for (size_t i = 0; i < n_points; ++i)
        {
            T new_point;
            new_point.x = cos(2*PI/n_points*i)*radius+center_point.x;
            new_point.y = sin(2*PI/n_points*i)*radius+center_point.y;
            new_point.x = common_utils::clip<double>(new_point.x, lower.x, upper.x);
            new_point.y = common_utils::clip<double>(new_point.y, lower.y, upper.y);
            points.push_back(new_point);
        }
    }
}



#endif //HELPERFUNCTIONS_H
