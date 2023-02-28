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

#ifndef LIBV_CORE_TEST_HPP
#define LIBV_CORE_TEST_HPP

#include <boost/format.hpp>

#include "macros.hpp"

namespace v {
namespace core {
/// \addtogroup logger
/// \{

 /**

An error message, printed when a test fails.

*/
#define V_TEST_MESSAGE(_test, _message)\
  __FILE__ ":" V_STRINGIFY(__LINE__) ": " _test " failed because " _message "."

/**

If \e test is \c false, print \e message and call \c exit().

*/
inline void test_message
( bool test ///< The result of the test.
, const boost::format &message ///< A message printed if the test failed.
)
{
  if(!test) throw std::runtime_error(str(message));
}

/**

Check that two values are equal.

*/
#define V_TEST_EQUAL(a, b) v::test_equal(a, b, V_TEST_MESSAGE(#a " = " #b, "%1% ≠ %2%"))

/**

Helper function for #V_TEST_EQUAL.

*/
template<class T1, class T2> void test_equal
( T1 first ///< The first value
, T2 second ///< The second value.
, const char *format ///< The message printed when the test fails.
)
{
  test_message(first == second, boost::format(format) % first % second);
}

/**

Check that a value is less than an other.

*/
#define V_TEST_LT(a, b) v::test_lt(a, b, V_TEST_MESSAGE(#a " < " #b, "%1% ≥ %2%"))

/**

Helper function for #V_TEST_LT.

*/
template<class T1, class T2> void test_lt
( T1 first ///< The first value
, T2 second ///< The second value.
, const char *format ///< The message printed when the test fails.
)
{
  test_message(first < second, boost::format(format) % first % second);
}

/// \}
}}

#endif
