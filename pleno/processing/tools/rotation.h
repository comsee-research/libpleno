#pragma once

#include <Eigen/Core>
#include <cmath>

#include <libv/geometry/rotation.hpp> 

/**
 *
 * Apply a rotation to a transformation.
 *
 * matrix: A transformation matrix.
 * d: A rotation in axis-angle representation (radian)
 *
*/
inline void apply_rotation(Eigen::Matrix2d& matrix, double theta)
{
    matrix <<
    std::cos(theta), -std::sin(theta),
    std::sin(theta),  std::cos(theta);
}

using v::apply_rotation;
using v::apply_small_rotation;
using v::rotation_orthogonalize;


/**
 * returns a rotation matrix given 3 angles (radian) according to lie algebra and Taylor expansions
**/
inline Eigen::Matrix3d rotation(const double a, const double b, const double g)
{
    double theta2 = a * a + b * b + g * g + std::numeric_limits<double>::epsilon();

    double theta = std::sqrt(theta2);

    const double factor = std::sin(theta) / theta2;
    const double t = std::tan(0.5 * theta);

    Eigen::Matrix3d matrix;
    matrix <<
    -(b * b + g * g)  *  t, a * b * t - g * theta, a * g * t + b * theta,
     a * b * t + g * theta,  -(a * a + g * g) * t, b * g * t - a * theta,
     a * g * t - b * theta, b * g * t + a * theta,  -(a * a + b * b) * t;

    matrix *= factor;
    matrix += Eigen::Matrix3d::Identity();

    return matrix;
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
        if (r > T(2.0 * M_PI))
        {
            r -= T(2.0 * M_PI);
            return true;
        }

        return false;
    };

    auto still_inferior_to_2pi = [](T& r)
    {
        if (r < -T(2.0 * M_PI))
        {
            r += T(2.0 * M_PI);
            return true;
        }

        return false;
    };

    while (still_superior_to_2pi(rad));
    while (still_inferior_to_2pi(rad));

    return rad;
};

template<typename T> inline T deg_to_rad(const T angle){ return( angle * M_PI / T(180.0)); };
template<typename T> inline T rad_to_deg(const T angle){ return(T(180.0) * angle / M_PI); };
