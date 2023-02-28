/**

\file
\author Clement Deymier (2012)
\author Alexis Wilhelm (2013)
\copyright 2012-2013 Institut Pascal

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

#ifndef LIBV_GEOMETRY_CUBIC_SPLINE_HPP
#define LIBV_GEOMETRY_CUBIC_SPLINE_HPP

#include <libv/core/assert.hpp>
#include <vector>
#include <Eigen/Core>

#include "global.hpp"

namespace v {
namespace geometry {
/// \addtogroup geometry
/// \{

/**

A 1-dimension spline.

*/
template<class Vector, class Scalar>
struct CubicSpline
{
  /**

  Add a control point.

  */
  void push_back
  ( const Vector &value ///< A new control point.
  )
  {
    control_points_.push_back(value);
  }

  /**

  Recompute polynomes coefficients.

  */
  void recompute_coefs()
  {
    V_PRECONDITION(size() >= 4)
    precomputed_coefs_.resize(size() - 3);

    for(size_t i = 0; i < size() - 3; ++i)
    {
      static const Scalar c6m1 = Scalar(1. / 6.);
      precomputed_coefs_[i][0] = c6m1 * ( -1 *(*this)[i] +3 *(*this)[i+1] -3 *(*this)[i+2] +1 *(*this)[i+3] );
      precomputed_coefs_[i][1] = c6m1 * ( +3 *(*this)[i] -6 *(*this)[i+1] +3 *(*this)[i+2] );
      precomputed_coefs_[i][2] = c6m1 * ( -3 *(*this)[i] +3 *(*this)[i+2] );
      precomputed_coefs_[i][3] = c6m1 * ( +1 *(*this)[i] +4 *(*this)[i+1] +1 *(*this)[i+2] );
    }

  }

  /**

  Evaluate the \f$n\f$-th polynomial at a date \f$t\f$.

  \returns The value of this spline at \f$n+t\f$.

  */
  Vector operator()
  ( size_t n ///< The index of a polynomial.
  , Scalar t ///< A date.
  ) const
  {
    V_PRECONDITION(n < size())
    V_PRECONDITION(t >= 0 && t <= 1)
    return precomputed_coefs_[n][3] + t * (precomputed_coefs_[n][2] + t * (precomputed_coefs_[n][1] + t * precomputed_coefs_[n][0]));
  }

  /**

  Evaluate the derivative of the \f$n\f$-th polynomial at a date \f$t\f$.

  \returns The slope of this spline at \f$n+t\f$.

  */
  Vector eval_df
  ( size_t n ///< The index of a polynomial.
  , Scalar t ///< A date.
  ) const
  {
    V_PRECONDITION(n < size())
    V_PRECONDITION(t >= 0 && t <= 1)
    return precomputed_coefs_[n][2] + t * ( Scalar(2) * precomputed_coefs_[n][1] + t * Scalar(3) * precomputed_coefs_[n][0] );
  }

  /**

  Evaluate this spline at a date \f$t\f$.

  \returns The value of this spline at \f$t\f$.

  */
  Vector operator()
  ( Scalar t ///< A date.
  ) const
  {
    V_PRECONDITION(t >= 0 && t <= t_validity())
    const size_t n = size_t(t);
    return (*this)(n, t - Scalar(n));
  }

  /**

  Evaluate the derivative of this spline at a date \f$t\f$.

  \returns The slope of this spline at \f$t\f$.

  */
  Vector eval_df
  ( Scalar t ///< A date.
  ) const
  {
    V_PRECONDITION(t >= 0 && t <= t_validity())
    const size_t n = size_t(t);
    return eval_df(n, t - Scalar(n));
  }

  /**

  The number of control points.

  */
  size_t size() const
  {
    return control_points_.size();
  }

  /**

  The \f$i\f$-th control point.

  \returns A control point.

  */
  const Vector &operator[]
  ( size_t i ///< The index of a control point.
  ) const
  {
    V_PRECONDITION(i < size())
    return control_points_[i];
  }

  /// \copydoc operator[]()
  Vector &operator[]
  ( size_t i ///< The index of a control point.
  )
  {
    V_PRECONDITION(i < size())
    return control_points_[i];
  }

  /**

  Upper bound of the domain of this spline.
  The lower bound is zero.

  \return The upper bound of the domain of this spline.

  */
  size_t t_validity() const
  {
    return size_t(std::max(int(control_points_.size()) - 3, 0));
  }

private :

  std::vector<Vector, Eigen::aligned_allocator<Vector> > control_points_;
  std::vector<Eigen::Matrix<Vector, 4, 1>, Eigen::aligned_allocator<Eigen::Matrix<Vector, 4, 1> > > precomputed_coefs_;
};

/// \}
}}

#endif
