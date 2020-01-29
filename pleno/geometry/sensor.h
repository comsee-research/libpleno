#pragma once

#include "types.h"

#include "geometry/pose.h"
#include "io/cfg/sensor.h"

class Sensor
{
    Pose pose_;
    double scale_; // size of a pixel (mm)
    size_t width_; // width of the sensor (pixel)
    size_t height_; // height of the sensor (pixel)

public:
    Sensor(size_t w, size_t h, double s);
    Sensor(const SensorConfig& config = {});
    ~Sensor();

    const Pose& pose() const;
          Pose& pose();

    double scale() const;
    double& scale();

    size_t width() const;
    size_t& width();

    size_t height() const;
    size_t& height();


    // the plane equation coefficients
    Eigen::Matrix<double, 4, 1> plane() const;
    // the plane equation coefficients in WORLD coordinate system
    Eigen::Matrix<double, 4, 1> planeInWorld() const;

	template<typename T>
    T pxl2metric(const T& p) const;
    template<typename T>
    T metric2pxl(const T& p) const;
    
};

std::ostream& operator<<(std::ostream& o, const Sensor& s);
