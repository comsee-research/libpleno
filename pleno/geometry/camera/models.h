#pragma once

#include <Eigen/Dense>

#include <libv/geometry/camera_model.hpp> ////UnifiedCameraModel

#include "geometry/ray.h"
#include "geometry/pose.h"
#include "geometry/sensor.h"

////////////////////////////////////////////PinholeCameraModel//////////////////////////////////////
struct PinholeCameraModel
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double focal; // The focal length (mm)
    // double k, l; // scale parameters (pixels/mm)
    // double skew; // TODO ADD SKEW
    P2D center; // The principal point (pixel)

    Sensor _sensor;

    // PinholeCameraModel(const double fl, const double sk, const double sl, const P2D& ctr);
    PinholeCameraModel(const double f, const Sensor& s);

    virtual ~PinholeCameraModel();

    const Sensor& sensor() const;
    Sensor& sensor();

    bool project(const P3D&, P2D&) const;
    bool project(const P3DS&, P2DS&) const;

    bool raytrace(const P2D& pixel, Ray3D& ray) const;
};

std::ostream& operator<<(std::ostream& o, const PinholeCameraModel& pcm);

////////////////////////////////////////////ThinLensCameraModel/////////////////////////////////////
class ThinLensCameraModel
{
    double _f; // The focal length
    double _aperture; // aperture of the lens

    Pose _pose;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW	
	double& f() { return _f; }
	double f() const { return _f; }
	
	double& aperture() { return _aperture; }
	double aperture() const { return _aperture; }
	
	Pose& pose() { return _pose; }
	const Pose& pose() const { return _pose; }
	
    double diameter() const;
	
    bool project(const P3D&, P3D&) const;
    bool project(const P3DS& p3d_cam, P3DS& p3d_out) const;
    bool raytrace(const Ray3D& ray_in, Ray3D& ray_out) const;
    bool raytrace(const Rays3D& rays_in, Rays3D& rays_out) const;

    // https://physicssoup.wordpress.com/2015/04/26/how-do-object-distance-and-focal-length-affect-depth-of-field/
    // double depthOfField(){};     \displaystyle C = (S - d_i)(\tan \theta_1 + \tan \theta_2)
};

std::ostream& operator<<(std::ostream& o, const ThinLensCameraModel& tcm);

//////////////////////////////////////OrthographicCameraModel///////////////////////////////////////
struct OrthographicCameraModel
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double focal; // The focal length (mm)

    // OrthographicCameraModel(const double fl, const double sk, const double sl, const P2D& ctr);
    OrthographicCameraModel(const double f, const Sensor& s);

    virtual ~OrthographicCameraModel();

    const Sensor& sensor() const;
    Sensor& sensor();

    bool project(const P3D&, P2D&) const;
    bool raytrace(const P2D& pixel, Ray3D& ray) const;

private:
    Sensor _sensor;
};

std::ostream& operator<<(std::ostream& o, const OrthographicCameraModel& ocm);
