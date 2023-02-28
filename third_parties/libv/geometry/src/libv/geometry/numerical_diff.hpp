/**

\file
\author Alexis Wilhelm (2013)
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

#ifndef LIBV_GEOMETRY_NUMERICAL_DIFF_HPP
#define LIBV_GEOMETRY_NUMERICAL_DIFF_HPP

#include <unsupported/Eigen/NumericalDiff>

#include "global.hpp"

namespace v {
namespace geometry {
/// \addtogroup geometry
/// \{

/// Helper class for NumericalDiff.
template<class _derived, class _scalar, int _inputs, int _values>
class NumericalDiff_Functor
{
  const int inputs_, values_;

public:

  enum {InputsAtCompileTime = _inputs, ValuesAtCompileTime = _values};

  /// The scalar type.
  typedef _scalar Scalar;

  /// The input type.
  typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;

  /// The value type.
  typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;

  /// The jacobian type.
  typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

  /// Constructor.
  NumericalDiff_Functor(int inputs, int values)
    : inputs_(inputs)
    , values_(values)
  {
  }

  /// The input count.
  int inputs(void) const
  {
    return inputs_;
  }

  /// The value count.
  int values(void) const
  {
    return values_;
  }

  /// Apply this functor.
  int operator()(const InputType &inputs, ValueType &values) const
  {
    return static_cast<const _derived &>(*this)(inputs, values);
  }
};

/**

Base class for functors to be used with Eigen's non-linear optimization module.
This is easier to use than \c Eigen::NumericalDiff.
Your functor only need to derive from this base class.

\tparam _derived Your functor.
\tparam _scalar The scalar type.
\tparam _inputs The size of the input, or \c Eigen::Dynamic if it is not known at compile time.
\tparam _values The value count, or \c Eigen::Dynamic if it is not known at compile time.

\extends NumericalDiff_Functor

*/
template<class _derived, class _scalar, int _inputs, int _values>
class NumericalDiff
  : public Eigen::NumericalDiff<NumericalDiff_Functor<_derived, _scalar, _inputs, _values> >
{
  typedef NumericalDiff_Functor<_derived, _scalar, _inputs, _values> Functor;

public:

  /// Default constructor.
  NumericalDiff(_scalar epsilon = 0)
    : Eigen::NumericalDiff<Functor>(Functor(_inputs, _values), epsilon)
  {
  }

  /// Constructor.
  NumericalDiff(int inputs, int values, _scalar epsilon = 0)
    : Eigen::NumericalDiff<Functor>(Functor(inputs, values), epsilon)
  {
  }
};

/// \}
}}

#endif
