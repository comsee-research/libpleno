#include "sensor.h"

Sensor::Sensor(size_t w, size_t h, double s)
: scale_{s}, width_{w}, height_{h}
{}

Sensor::Sensor(const SensorConfig& config)
	: pose_{config.pose()}, scale_{config.scale()}, width_{config.width()}, height_{config.height()} 
{
}

Sensor::~Sensor()
{}

const Pose& Sensor::pose() const
{
	return pose_;
}
Pose& Sensor::pose()
{
	return pose_;
}

double Sensor::scale() const
{
	return scale_;
}
double& Sensor::scale()
{
	return scale_;
}

size_t Sensor::width() const
{
	return width_;
}
size_t& Sensor::width()
{
	return width_;
}

size_t Sensor::height() const
{
	return height_;
}
size_t& Sensor::height()
{
	return height_;
}
          
// the plane coefficients
PlaneCoefficients Sensor::plane() const
{
    return plane_from_3_points(	P3D{0.0, 0.0, 0.0},
    							P3D{double(width_), 0.0, 0.0},
                                P3D{double(width_), double(height_), 0.0}
    );
};

// the plane coefficients in WORLD coordinate system
PlaneCoefficients Sensor::planeInWorld() const
{
    return plane_from_3_points(from_coordinate_system_of(pose_, P3D{0.0, 0.0, 0.0}),
                               from_coordinate_system_of(pose_, P3D{double(width_), 0.0, 0.0}),
                               from_coordinate_system_of(pose_, P3D{double(width_), double(height_), 0.0})
   	);
};

template<typename T>
T Sensor::pxl2metric(const T& p) const
{
    return (p * scale_);
};
template<typename T>
T Sensor::metric2pxl(const T& p) const
{
    return (p / scale_);
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
