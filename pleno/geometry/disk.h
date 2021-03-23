#pragma once

#include <cmath>

struct Disk
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    P2D center;
    double radius;
    
    Disk(const P2D& c, double r) : center{c}, radius{r} {}
    Disk(double u, double v, double r) : center{P2D{u,v}}, radius{r} {}

    //verify is a point is present on the disk
    bool is_inside(const P2D& p) const
    {
        return ((p-center).norm() <= radius);
    };
};

/*
 *  @Brief is_on_disk test if a point is on a parametetrized disk
 *  a point in the disk coordinate system (in the plane of the disk)
 *  disk_diameter the diameter of the disk
**/
template<typename T>
bool is_on_disk(const T& p, double disk_diameter)
{
    return ((p.template head<2>()).norm() <= disk_diameter / 2.0 );
}
