/**

\file
\author Alexis Wilhelm (2012)
\copyright 2012 Institut Pascal

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

#ifndef LIBV_CORE_MISCMATH_HPP
#define LIBV_CORE_MISCMATH_HPP

#include <algorithm>
#include <cmath>

#include "global.hpp"

namespace v {
namespace core {
namespace pow_ {

template<int n, class T>
struct pow
{
  static T eval(T x)
  {
    return x * pow<n - 1, T>::eval(x);
  }
};

template<class T>
struct pow<0, T>
{
  static T eval(T)
  {
    return 1;
  }
};

}

/// \addtogroup core_math
/// \{

/// Compute the n-th power of a number.
/// \tparam n An integer.
/// \param x A number.
/// \returns \f$ x^n \f$
/// \note GCC optimizes the consecutive multiplications with the binary exponentiation method, so we do not need to do so ourselves.
template<int n, class T> T
pow(T x)
{
  return pow_::pow<n, T>::eval(x);
}

/// Compute the n-th power of a number.
/// \param x A number.
/// \param n A positive integer.
/// \returns \f$ x^n \f$
/// \note We import the standard pow() function in this namespace, so you can use v::pow() for both integral and real exponents.
template<class T> T
pow(T x, int n)
{
  T y = 1;
  while(n)
  {
    if(n % 2)
    {
      y *= x;
      n -= 1;
    }
    x *= x;
    n /= 2;
  }
  return y;
}

using std::pow;

/// Clamp a number between two bounds.
/// \param x The number to be clamped.
/// \param min The lower bound.
/// \param max The upper bound.
/// \returns \f$\left\{\begin{array}{ll} min & \mathrm{if}\ x < min \\ max & \mathrm{if}\ x > max \\ x & \mathrm{otherwise} \end{array}\right.\f$
template<class T> T
clamp(T x, T min, T max)
{
  return std::max(std::min(x, max), min);
}

/// Round to the nearest integer.
/// \param x The number to be rounded.
/// \return The nearest integer to \e x.
template<class T> T
round(T x)
{
  return std::floor(x + .5);
}

/// \copydoc round
inline long
lround(double x)
{
  return long(round(x));
}

/// Wrap a number between \f$-a\f$ and \f$+a\f$.
/// \param x The number to be wrapped.
/// \param a A positive number.
/// \returns \f$ y \in [-a,+a[: y \equiv x\ (\mathrm{mod}\ 2 a) \f$
template<class T> T
wrap_centered(T x, T a)
{
  return std::fmod(std::fmod(x + a, 2 * a) + 2 * a, 2 * a) - a;
}

/// Wrap a number between \f$-\pi\f$ and \f$+\pi\f$.
/// \param x The number to be wrapped.
/// \returns \f$ y \in [-\pi,+\pi[: y \equiv x\ (\mathrm{mod}\ 2 \pi) \f$
template<class T> T
wrap_pi(T x)
{
  return wrap_centered(x, T(M_PI));
}

/**

Round an integer to the next power of two.

\return
The smallest power of two greater than \e x.

*/
template
<class _integer_type ///< A built-in integer type.
>
_integer_type ceil_pow2
( _integer_type x ///< An integer.
)
{
  --x;
  for(size_t n = 1; n < sizeof(x) * 8; n *= 2)
  {
    x |= x >> n;
  }
  ++x;
  return x;
}

/// \}
}}

#endif
