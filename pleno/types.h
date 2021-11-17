#pragma once

#include <Eigen/StdVector>

#include <opencv2/opencv.hpp> //cv::Mat
#include <array>

//helper type
template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;

// Custom types
template<class T>
using AlignedVector = std::vector<T, Eigen::aligned_allocator<T>>;

template <std::size_t N>
using PnD = Eigen::Matrix<double, static_cast<int>(N), 1>;
template <std::size_t N>
using PnDS = AlignedVector<PnD<N>>;

using P2D 		= PnD<2ul>; //Eigen::Vector2d;
using P2DS 		= AlignedVector<P2D>;

using Pixel		= PnD<2ul>; //Eigen::Vector2d;
using Pixels	= AlignedVector<Pixel>; //Eigen::Vector2d;

using P3D 		= PnD<3ul>; //Eigen::Vector3d;
using P3DS 		= AlignedVector<P3D>;

using Point 	= PnD<3ul>;
using Points	= AlignedVector<Point>;

using P4D 		= PnD<4ul>; //Eigen::Vector4d;
using P4DS 		= AlignedVector<P4D>;

using HomogeneousPoint = PnD<4ul>;
using HomogeneousPoints = AlignedVector<HomogeneousPoint>;

using Vector = Eigen::Matrix<double, Eigen::Dynamic, 1>;
using Matrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;

template<std::size_t N> class Pose_;
using Poses 	= AlignedVector<Pose_<3ul>>;
using Poses3D 	= AlignedVector<Pose_<3ul>>;
using Poses2D 	= AlignedVector<Pose_<2ul>>;

template<std::size_t N>
using Rotation_ 	= Eigen::Matrix<double, static_cast<int>(N), static_cast<int>(N)>;

using Rotation 		= Rotation_<3ul>;
using Rotation3D	= Rotation_<3ul>;
using Rotation2D 	= Rotation_<2ul>;

template<std::size_t N> 
using Translation_ 	= Eigen::Matrix<double, static_cast<int>(N), 1>;

using Translation	= Translation_<3ul>;
using Translation3D	= Translation_<3ul>;
using Translation2D	= Translation_<2ul>;

template<std::size_t N> class Ray_;
using Rays 		= AlignedVector<Ray_<3ul>>;
using Rays3D 	= AlignedVector<Ray_<3ul>>;
using Rays2D 	= AlignedVector<Ray_<2ul>>;

using Quad   = std::array<P3D, 4ul>;
using Quad3D = std::array<P3D, 4ul>;
using Quad2D = std::array<P2D, 4ul>;

struct Corner {
	enum : std::size_t {
		TL = 0ul, 
		TR = 1ul,
		BR = 2ul,
		BL = 3ul
	};
};

using Real = double;
using Index = int;

struct Depth 		{ Real z = 0.; };
struct InverseDepth { Real z = 0.; };
struct VirtualDepth { Real v = 0.; };

struct xyz 
{
	Real x = 0., y = 0., z = 0.;
};

using XYZs = AlignedVector<xyz>;

enum Axis { X = 0, Y = 1, Z = 2, XY = 4, XZ = 5, YZ = 6, XYZ = 7 };

struct IndexPair {
	std::size_t k, l;
	
	IndexPair(std::size_t k_ = 0ul, std::size_t l_ = 0ul) : k{k_}, l{l_} {}
};

using NeighborsIndexes 		= std::vector<IndexPair>;

using Image 				= cv::Mat;
using Images 				= AlignedVector<Image>;
using IndexedImages			= std::unordered_map<Index, Image>;
struct ImageWithInfo 		{ Image img; Real fnumber; Index frame = -1; };

struct LineCoefficients 	{ Real m; Real c; };
struct GaussianCoefficients { Real mu_x; Real mu_y; Real sigma; Real alpha; };
struct CircleCoefficients 	{ Real x0; Real y0; Real r; };

struct Peak 				{ Real angle; Real magnitude; };
using Peaks 				= AlignedVector<Peak>;

struct CalibrationPose;
using CalibrationPoses 		= AlignedVector<CalibrationPose>;

#include "processing/tools/error.h"

struct PoseWithError;
using PosesWithError 		= AlignedVector<PoseWithError>;

struct FocalLength 			{ Real f; };
using FocalLengths 			= AlignedVector<FocalLength>;

struct BlurProportionalityCoefficient { Real kappa; };
