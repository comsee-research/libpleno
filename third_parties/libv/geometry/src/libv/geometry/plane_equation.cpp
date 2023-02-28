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

#include <Eigen/Geometry>
#include <Eigen/SVD>

#include "plane_equation.hpp"

namespace v {
namespace geometry {


/**

Decompose the matrix and extract the column of U with the smallest values (often the U(:,2))

*/
template<class type_>
const Eigen::Matrix<type_, 4, 1>&
PlaneEquationFrom3DPoints<type_>::solve(Eigen::Matrix<type_, 3, Eigen::Dynamic>& matrix_to_decompose)
{
  Eigen::JacobiSVD<Eigen::Matrix<type_, 3, Eigen::Dynamic> > svd_solver;
  //! Make SVD and compute normal
  svd_solver.compute(matrix_to_decompose, Eigen::ComputeFullU);

  //! Build result equation
  for(size_t i = 0 ; i < 3 ; ++i)
    equation(i) = svd_solver.matrixU()(i,2);
  equation(3) = 0;
  equation.normalize();
  equation(3) = -compute_signed_distance(equation,mean);

  return equation;
}

template struct LIBV_GEOMETRY_EXPORT PlaneEquationFrom3DPoints<long double>;
template struct LIBV_GEOMETRY_EXPORT PlaneEquationFrom3DPoints<double>;
template struct LIBV_GEOMETRY_EXPORT PlaneEquationFrom3DPoints<float>;

}}
