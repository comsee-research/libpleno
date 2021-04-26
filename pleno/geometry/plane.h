#pragma once

#include "types.h"

using PlaneCoefficients = Eigen::Matrix<double, 4, 1>; //Coefficients (a,b,c,d) such as ax + by + cz = d 

/**
	Compute the signed distance between the plane and the point with a²+b²+c²=1
**/
inline double compute_signed_distance(const PlaneCoefficients& normalized_plane_eqn, const P3D& p3d)
{
  Eigen::Matrix<double, 1, 4> tp1;
  tp1[0] = p3d[0];
  tp1[1] = p3d[1];
  tp1[2] = p3d[2];
  tp1[3] = 1;
  return (tp1 * normalized_plane_eqn);
}

/**
	Compute the distance between the plane and the point with a²+b²+c²=1
**/
inline double compute_distance(const PlaneCoefficients& normalized_plane_eqn, const P3D& p3d)
{
  return std::fabs(compute_signed_distance(normalized_plane_eqn,p3d));
}

/**
	Compute a plane equation from 3 3D points: ax + by + cz = d
**/
inline PlaneCoefficients plane_from_3_points(const P3D& p1, const P3D& p2, const P3D& p3)
{
    PlaneCoefficients eq;
    eq << ((p2-p1).cross(p3-p1)).normalized(), 0;
    eq(3) = - compute_signed_distance(eq,p1);
    return eq;
}
