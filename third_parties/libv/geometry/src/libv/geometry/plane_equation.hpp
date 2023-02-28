/**

\file
\author Clement Deymier (2013)
\copyright 2013 Institut Pascal

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef LIBV_GEOMETRY_PLANE_EQUATION_HPP
#define LIBV_GEOMETRY_PLANE_EQUATION_HPP

#include <numeric>
#include <Eigen/Core>
#include <type_traits>

#include "global.hpp"

namespace v {
namespace geometry {
/// \addtogroup geometry
/// \{

/**

Compute the signed distance between the plane and the point with a²+b²+c²=1

*/
template <typename type_>
type_ compute_signed_distance(const Eigen::Matrix<type_, 4, 1> & normalized_plane_eqn, const Eigen::Matrix<type_, 3, 1> & p3d)
{
  Eigen::Matrix<type_, 1, 4> tp1;
  tp1[0] = p3d[0];
  tp1[1] = p3d[1];
  tp1[2] = p3d[2];
  tp1[3] = 1;
  return (tp1 * normalized_plane_eqn);
}

/**

Compute the distance between the plane and the point with a²+b²+c²=1

*/
template <typename type_>
type_ compute_distance(const Eigen::Matrix<type_, 4, 1> & normalized_plane_eqn, const Eigen::Matrix<type_, 3, 1> & p3d)
{
  return std::abs(compute_signed_distance(normalized_plane_eqn,p3d));
}

/**

Compute and orthonormal basis with a given 3D vector as first vector.

*/
template <typename type_>
Eigen::Matrix<type_, 3, 3> build_orthobasis(const Eigen::Matrix<type_, 3, 1> & ve)
{
  Eigen::Matrix<type_, 3, 1> v2=ve.unitOrthogonal();
  Eigen::Matrix<type_, 3, 3> rot;
  rot << ve.normalized(),v2,ve.cross(v2).normalized();
  return rot;
}

/**

Compute a plane equation from 3 3d points

*/
template <typename type_>
Eigen::Matrix<type_, 4, 1> plane_from_3_points(const Eigen::Matrix<type_, 3, 1> & p1,const Eigen::Matrix<type_, 3, 1> & p2, const Eigen::Matrix<type_, 3, 1> & p3)
{
    Eigen::Matrix<type_, 4, 1> eq;
    eq << ((p2-p1).cross(p3-p1)).normalized(), 0;
    eq(3) = -compute_signed_distance(eq,p1);
    return eq;
}

template <typename type_=double>
struct PlaneEquationFrom3DPoints
{

  static_assert(std::is_floating_point<type_>::value,
      "this class can only by used with float, double or long double types");

  Eigen::Matrix<type_, 3, 1> mean;
  Eigen::Matrix<type_, 4, 1> equation;

  template<class Iterator>
  const Eigen::Matrix<type_, 4, 1>& compute_plane(Iterator begin, Iterator end)
  {
    const size_t nb_points_3d = end - begin;
    //! Compute vector mean
    mean.setZero();
    mean = std::accumulate(begin,end,mean) / type_(nb_points_3d);

    //! Build matrix for SVD with point minus the mean
    Eigen::Matrix<type_, 3, Eigen::Dynamic> matrix_to_decompose(3,nb_points_3d);

    for(Iterator it = begin ; it != end ; ++it)
      matrix_to_decompose.col(it - begin) = (*it - mean);

    return solve(matrix_to_decompose);
  }

private:

  const Eigen::Matrix<type_, 4, 1>& solve(Eigen::Matrix<type_, 3, Eigen::Dynamic>& matrix_to_decompose);

};

/// \}
}}

#endif
