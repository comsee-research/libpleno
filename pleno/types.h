#pragma once

#include <Eigen/StdVector>

#include <opencv2/opencv.hpp> //cv::Mat
#include <array>

#include "geometry/pose.h"

//helper type
template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;

// Custom types
template<class T>
using AlignedVector = std::vector<T, Eigen::aligned_allocator<T>>;

using P2D 	= Eigen::Vector2d;
using P2DS 	= AlignedVector<P2D>;
using P3D 	= Eigen::Vector3d;
using P3DS 	= AlignedVector<P3D>;

template<std::size_t N> class Pose_;
using Poses 	= AlignedVector<Pose_<3u>>;
using Poses3D 	= AlignedVector<Pose_<3u>>;
using Poses2D 	= AlignedVector<Pose_<2u>>;

/* Holder for rotation */
template<std::size_t N> struct Rotation_ {
	typename Pose_<N>::Matrix * R;
};
using Rotation2D 	= Rotation_<2u>;
using Rotation3D	= Rotation_<3u>;
using Rotation 		= Rotation_<3u>;

/* Holder for translation */
template<std::size_t N> struct Translation_ {
	typename Pose_<N>::Vector * t;
};
using Translation2D	= Translation_<2u>;
using Translation3D	= Translation_<3u>;
using Translation	= Translation_<3u>;

template<std::size_t N> class Ray_;
using Rays 		= AlignedVector<Ray_<3u>>;
using Rays3D 	= AlignedVector<Ray_<3u>>;
using Rays2D 	= AlignedVector<Ray_<2u>>;

using Quad   = std::array<P3D, 4u>;
using Quad3D = std::array<P3D, 4u>;
using Quad2D = std::array<P2D, 4u>;

struct Corner {
	enum : std::size_t {
		TL = 0u, 
		TR = 1u,
		BR = 2u,
		BL = 3u
	};
};

using Real = double;
using Index = int;

using Image 				= cv::Mat;
using Images 				= AlignedVector<cv::Mat>;
struct ImageWithInfo 		{ Image img; Real fnumber; Index frame = -1; };

struct LineCoefficients 	{ Real m; Real c; };
struct GaussianCoefficients { Real mu_x; Real mu_y; Real sigma; Real alpha; };
struct CircleCoefficients 	{ Real x0; Real y0; Real r; };

struct Peak 				{ Real angle; Real magnitude; };
using Peaks 				= AlignedVector<Peak>;

struct CalibrationPose 		{ Pose pose; Index frame; };
using CalibrationPoses 		= AlignedVector<CalibrationPose>;

#include "processing/tools/rmse.h"

struct PoseWithError 		{ Pose pose; RMSE rmse; };
using PosesWithError 		= AlignedVector<PoseWithError>;

struct FocalLength 			{ Real f; };
using FocalLengths 			= AlignedVector<FocalLength>;

struct RGBA					{ Real r,g,b,a; };

struct BlurProportionalityCoefficient { Real kappa; };
