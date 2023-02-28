/**

\file
\author Datta Ramadasan
//==============================================================================
//         Copyright 2015 INSTITUT PASCAL UMR 6602 CNRS/Univ. Clermont II
//
//          Distributed under the Boost Software License, Version 1.0.
//                 See accompanying file LICENSE.txt or copy at
//                     http://www.boost.org/LICENSE_1_0.txt
//==============================================================================

*/

#ifndef __LIBS_COLOR_CONSOLE_HPP__
#define __LIBS_COLOR_CONSOLE_HPP__

#include <string>
#include "../string/string_utiliy.hpp"

  struct Couleur
  {

    template<class T> std::string operator()(const T& o) const
    { return "\033[" + lma::to_string(o) + "m"; }

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
    std::string bold() const { return this->operator()(1);}
    std::string underline() const { return this->operator()(4);}
    std::string background() const { return this->operator()(7);}
    std::string barre() const { return this->operator()(9);}
    std::string italic() const { return "\033[3m"; }
    std::string yellow(const std::string& str) const { return yellow() + str + reset(); }
    std::string green(const std::string& str) const { return green() + str + reset(); }
    std::string red(const std::string& str) const { return red() + str + reset(); }
  };

  static const Couleur color = Couleur();

#endif
