#include "camera.h"

//Ctor/Dtor
Camera::Camera(const Sensor& s, const Pose& p) : sensor_{s}, pose_{p} {}	
Camera::~Camera() {}

//Accessors	
const Sensor& Camera::sensor() const { return sensor_; }
Sensor& Camera::sensor() { return sensor_; }

Pose& Camera::pose() { return pose_; }
const Pose& Camera::pose() const { return pose_; }

//Space convertion	(Image Space / Sensor Space)
void Camera::uv2xy(P2D& puv) const { puv[0] = sensor_.width()-1 - puv[0]; puv[1] = sensor_.height()-1 - puv[1];}
void Camera::xy2uv(P2D& pxy) const { uv2xy(pxy); }

//Helper functions
bool Camera::hit_the_sensor(const P2D& p) const {
	return p[0] >= 0.0 and p[1] >= 0.0 and p[0] < sensor_.width() and p[1] < sensor_.height();
};
