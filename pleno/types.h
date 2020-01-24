#pragma once

#include <Eigen/StdVector>

#include <opencv2/opencv.hpp> //cv::Mat
#include <array>

#include "geometry/pose.h"

// Custom types
template<class T>
using AlignedVector = std::vector<T, Eigen::aligned_allocator<T>>;

using P2D = Eigen::Vector2d;
using P2DS = AlignedVector<P2D>;
using P3D = Eigen::Vector3d;
using P3DS = AlignedVector<P3D>;

template<std::size_t N> class Pose_;
using Poses = AlignedVector<Pose_<3>>;
using Poses3D = AlignedVector<Pose_<3>>;
using Poses2D = AlignedVector<Pose_<2>>;

template<std::size_t N> class Ray_;
using Rays = AlignedVector<Ray_<3>>;
using Rays3D = AlignedVector<Ray_<3>>;
using Rays2D = AlignedVector<Ray_<2>>;

using Quad   = std::array<P3D, 4>;
using Quad3D = std::array<P3D, 4>;
using Quad2D = std::array<P2D, 4>;

using Real = double;
using Index = int;

using Image 				= cv::Mat;
struct ImageWithInfo 		{ Image img; Real fnumber; };

struct LineCoefficients 	{ Real m; Real c; };
struct GaussianCoefficients { Real mu_x; Real mu_y; Real sigma; Real alpha; };
struct CircleCoefficients 	{ Real x0; Real y0; Real r; };

struct Peak 				{ Real angle; Real magnitude; };
using Peaks 				= AlignedVector<Peak>;

struct CalibrationPose 		{ Pose pose; Index frame; };
using CalibrationPoses 		= AlignedVector<CalibrationPose>;

struct FocalLength 			{ Real f; };

struct RGBA					{ Real r,g,b,a; };


//----------------------------------
template<int I>
struct CharlInt{
    static const int value = I;
};

static const CharlInt<0> _0;