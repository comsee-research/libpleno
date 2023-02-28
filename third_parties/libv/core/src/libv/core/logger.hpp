/**

\file
\author Alexis Wilhelm (2012, 2015)
\copyright 2012, 2015 Institut Pascal

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

#ifndef LIBV_CORE_LOGGER_HPP
#define LIBV_CORE_LOGGER_HPP

#include <sstream>

#include "macros.hpp"

namespace v {
namespace core {
/// \addtogroup logger
/// \{

/**

Construct a Logger if the macro \e condition is defined, else do nothing.

\param condition
A macro that must be defined to enable this logger.
For example, if \e condition is \c ENABLE_LOG, this logger will not generate any code unless you define the \e ENABLE_LOG macro (with \c \#define in a source file or with \c -DENABLE_LOG on your compiler's command line).

\param stream
The stream the logger will output to.
This will often be \c std::cerr.

\note
The magic here lies in the test with \c sizeof and #V_STRINGIFY.
If the macro \e ENABLE_LOG is undefined, then \c V_STRINGIFY(ENABLE_LOG) expands to \c "ENABLE_LOG", and \c sizeof() returns 11, which is greater than \c sizeof("1").
Now if the macro is defined with \c -DENABLE_LOG, \e ENABLE_LOG expands to \c 1, so \c V_STRINGIFY(ENABLE_LOG) expands to \c "1", and \c sizeof() returns 2, which is equal to \c sizeof("1").

*/
#define V_NEW_LOGGER_IF(condition, stream)\
  V_NEW_LOGGER_IF_(sizeof(V_STRINGIFY(condition)) <= sizeof("1"), stream)

/// Construct a Logger if the macro \e condition is undefined, else do nothing.
/// \see V_NEW_LOGGER_IF
#define V_NEW_LOGGER_IF_NOT(condition, stream)\
  V_NEW_LOGGER_IF_(sizeof(V_STRINGIFY(condition)) > sizeof("1"), stream)

/// Helper macro for #V_NEW_LOGGER_IF and #V_NEW_LOGGER_IF_NOT.
#define V_NEW_LOGGER_IF_(condition, stream)\
  v::Logger<(condition)>((stream), __FILE__ ":" V_STRINGIFY(__LINE__) ": ")

/// The default Logger.
#define V_DEBUG\
  V_NEW_LOGGER_IF_NOT(NDEBUG, v::log_stderr)

/**

Print the value of an expression.

Use this when debugging your code.
Example:
\code
  V_DUMP(2 + 3) // will print: src/file.cpp:99: 2 + 3 = 5
\endcode

*/
#define V_DUMP(x)\
  V_NEW_LOGGER_IF_(true, v::log_stderr) << #x " = " << (x);

LIBV_CORE_EXPORT const char *shorten_file_name(const char *);

/// A logger which forwards all messages to an \c std::ostringstream, and then prints them all at once with a user-provided logging function.
template<bool enabled>
struct Logger
: std::ostringstream
{
  /// Initialize a new logger with a logging function.
  Logger(void (*write)(std::ostringstream *), const char *prefix)
  : write_(write)
  {
    *this << shorten_file_name(prefix);
  }

  /// Commit the message to the underlying logging function.
  ~Logger(void)
  {
    *this << std::endl;
    write_(this);
  }

private:

  void (*const write_)(std::ostringstream *);
};

/// A phony Logger which does nothing.
template<>
struct Logger<false>
{
  /// Do nothing.
  Logger(void (*)(std::ostringstream *), const char *)
  {
  }

  /// Do nothing.
  template<class T> Logger &
  operator << (const T &)
  {
    return *this;
  }
};

#define V_PROFILER(var) v::Profiler(var){__FILE__ ":" V_STRINGIFY(__LINE__) ": " #var ":"}

struct LIBV_CORE_EXPORT Profiler
{
  Profiler(const char *id);
  ~Profiler();
  void start();
  void stop();
private:
  const char *const id_;
  long begin_;
};

LIBV_CORE_EXPORT void log_stdout(std::ostringstream *);
LIBV_CORE_EXPORT void log_stderr(std::ostringstream *);

/// \}
}}

#endif
