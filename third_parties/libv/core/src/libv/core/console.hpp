/**

\file
 Color in console
\author Morgan SLADE (2015)
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
#ifndef __LIBV__CONSOLE_HPP__
#define __LIBV__CONSOLE_HPP__

#include <string>
#include <boost/lexical_cast.hpp>
#include <libv/core/global.hpp>

namespace v{
namespace core {
  struct LIBV_CORE_EXPORT ConsoleColor
  {
    template<class T> std::string operator()(const T& o) const
    { return std::string("\033[") + boost::lexical_cast<std::string>(o) + "m"; }
    
    std::string operator()() const { return "\033[0m"; }
    std::string reset() const { return "\033[0m"; }
    std::string red() const { return "\033[31m"; }
    std::string black() const { return "\033[30m"; }
    std::string green() const { return "\033[32m"; }
    std::string white() const { return "\033[37m"; }
    std::string cyan() const { return "\033[36m"; }
    std::string yellow() const{ return "\033[33m"; }
    std::string blue() const{ return "\033[34m"; }
    std::string magenta() const{ return "\033[35m"; }
    std::string bright_red() const { return "\033[31;1m"; }
    std::string bright_black() const { return "\033[30;1m"; }
    std::string bright_green() const { return "\033[32;1m"; }
    std::string bright_white() const { return "\033[37;1m"; }
    std::string bright_cyan() const { return "\033[36;1m"; }
    std::string bright_yellow() const{ return "\033[33;1m"; }
    std::string bright_blue() const{ return "\033[34;1m"; }
    std::string bright_magenta() const{ return "\033[35;1m"; }

    std::string bold() const { return this->operator()(1);}
    std::string underline() const { return this->operator()(4);}
    std::string background() const { return this->operator()(7);}
    std::string strike() const { return this->operator()(9);}
    std::string italic() const { return "\033[3m"; }
    
    std::string red(const std::string& str)     const { return red()     + str + reset(); }
    std::string magenta(const std::string& str) const { return magenta() + str + reset(); }
    std::string yellow(const std::string& str)  const { return yellow()  + str + reset(); }
    std::string green(const std::string& str)  const  { return green()   + str + reset(); }
    std::string cyan (const std::string& str)  const  { return cyan()    + str + reset(); }
    std::string blue (const std::string& str)  const  { return blue()    + str + reset(); }
    std::string black(const std::string& str)  const  { return black()   + str + reset(); }
    std::string white(const std::string& str)  const  { return white()   + str + reset(); }

    std::string bright_red(const std::string& str)     const { return bright_red()     + str + reset(); }
    std::string bright_magenta(const std::string& str) const { return bright_magenta() + str + reset(); }
    std::string bright_yellow(const std::string& str)  const { return bright_yellow()  + str + reset(); }
    std::string bright_green(const std::string& str)  const  { return bright_green()   + str + reset(); }
    std::string bright_cyan (const std::string& str)  const  { return bright_cyan()    + str + reset(); }
    std::string bright_blue (const std::string& str)  const  { return bright_blue()    + str + reset(); }
    std::string bright_black(const std::string& str)  const  { return bright_black()   + str + reset(); }
    std::string bright_white(const std::string& str)  const  { return bright_white()   + str + reset(); }

    std::string bold(const std::string& str)   const  { return bold()   + str + reset(); }
    std::string strike(const std::string& str)  const { return strike()  + str + reset(); }
    std::string italic(const std::string& str) const  { return italic() + str + reset(); }
    std::string underline(const std::string& str)  const  { return underline()   + str + reset(); }
    std::string background(const std::string& str)  const { return background()  + str + reset(); }
    
  };

  LIBV_CORE_EXPORT extern const ConsoleColor console_color;
}
}
#endif  //__LIBV__CONSOLE_HPP__
