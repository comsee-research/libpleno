#pragma once

#include "types.h"

#include "camera.h"
#include "pinhole.h"

#include "geometry/sensor.h"
#include "geometry/ray.h"

////////////////////////////////////////////OrthographicCamera//////////////////////////////////////
class OrthographicCamera final : public PinholeCamera
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//Ctor/Dtor
    OrthographicCamera(double f, const Sensor& s);
    ~OrthographicCamera() override;
    
//Project and Raytrace
    bool project(const P3D& /*p3d_cam*/, P2D& /*pixel*/) const override;
    bool raytrace(const P2D& pixel, Ray3D& ray) const override;
};
