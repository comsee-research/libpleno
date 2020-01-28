#include "sensor.h"

#include <libv/geometry/plane_equation.hpp>

Sensor::Sensor(size_t w, size_t h, double s)
: _scale(s), _width(w), _height(h)
{}

Sensor::Sensor(const SensorConfig& config)
	: _pose{config.pose()}, _scale{config.scale()}, _width{config.width()}, _height{config.height()} 
{
}

Sensor::~Sensor()
{}

const Pose& Sensor::pose() const
{
	return _pose;
}
Pose& Sensor::pose()
{
	return _pose;
}

double Sensor::scale() const
{
	return _scale;
}
double& Sensor::scale()
{
	return _scale;
}

size_t Sensor::width() const
{
	return _width;
}
size_t& Sensor::width()
{
	return _width;
}

size_t Sensor::height() const
{
	return _height;
}
size_t& Sensor::height()
{
	return _height;
}
          
// the plane coefficients
Eigen::Matrix<double, 4, 1> Sensor::plane() const
{
    return v::plane_from_3_points(P3D{0.0, 0.0, 0.0},
                                  P3D{double(_width), 0.0, 0.0},
                                  P3D{double(_width), double(_height), 0.0});
};

// the plane coefficients in WORLD coordinate system
Eigen::Matrix<double, 4, 1> Sensor::planeInWorld() const
{
    return v::plane_from_3_points(from_coordinate_system_of(_pose, P3D{0.0, 0.0, 0.0}),
                                  from_coordinate_system_of(_pose, P3D{double(_width), 0.0, 0.0}),
                                  from_coordinate_system_of(_pose, P3D{double(_width), double(_height), 0.0}));
};

template<typename T>
T Sensor::pxl2metric(const T& p) const
{
    return (p * _scale);
};
template<typename T>
T Sensor::metric2pxl(const T& p) const
{
    return (p / _scale);
};

template double Sensor::pxl2metric(const double&) const;
template P2D Sensor::pxl2metric(const P2D&) const;
template P3D Sensor::pxl2metric(const P3D&) const;

template double Sensor::metric2pxl(const double&) const;
template P2D Sensor::metric2pxl(const P2D&) const;
template P3D Sensor::metric2pxl(const P3D&) const;


std::ostream& operator<<(std::ostream& o, const Sensor& s)
{
    o << "Dimensions (pixels) = [" << s.width() << ", " << s.height() << "]" << std::endl
      << "Scale (pixel/mm) = " << s.scale() << std::endl
      << "Pose = {" << std::endl << s.pose() << "}";

    return o;
}
