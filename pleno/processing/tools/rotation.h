#pragma once

#include <Eigen/Core>
#include <cmath>

/**
 *
 * Apply a rotation to a transformation.
 *
 * matrix: A transformation matrix.
 * d: A rotation in axis-angle representation (radian)
 *
*/
inline void apply_rotation(Eigen::Matrix2d& matrix, const double theta)
{
    matrix <<
    std::cos(theta), -std::sin(theta),
    std::sin(theta),  std::cos(theta);
}

/**
 *
 * Apply a rotation to a transformation.
 *
 * matrix: A transformation matrix.
 * d: A rotation in axis-angle representation (radian)
 *
*/
inline double get_rotation_angle(const Eigen::Matrix2d& matrix)
{
    return std::atan2( matrix(1, 0), matrix(0, 0) );
}


//take an angle, verify if it's included in [-2*PI; 2*PI]
//if so it returns the angle
//if not it add/sub 2 * PI to the angle
template<typename T>
inline T restrict_to_circle(T rad)
{
    auto still_superior_to_2pi = [](T& r)
    {
        if (r > T(2 * M_PI))
        {
            r -= T(2 * M_PI);
            return true;
        }

        return false;
    };

    auto still_inferior_to_2pi = [](T& r)
    {
        if (r < -T(2 * M_PI))
        {
            r += T(2 * M_PI);
            return true;
        }

        return false;
    };

    while (still_superior_to_2pi(rad)){}

    while (still_inferior_to_2pi(rad)){}

    return rad;
};

template<typename T> inline T deg_to_rad(const T angle){ return( angle * M_PI / T(180)); };
template<typename T> inline T rad_to_deg(const T angle){ return(T(180) * angle / M_PI); };
