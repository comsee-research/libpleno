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

#ifndef __MODULES_TIME_TICTOC_HPP__
#define __MODULES_TIME_TICTOC_HPP__

#include "time.hpp"
#include <iostream>
#include <string>
#include "../color/console.hpp"

namespace utils
{
  template<bool Use> struct Tic
  {
    static const bool use = true;
    double t;
    std::string name;
    Tic(std::string name_ =""):name(name_)
    {
      tic();
    }

    inline void tic()
    {
      t = now();
    }

    inline double toc() const
    {
      return now() - t;
    }

    inline std::ostream& disp(std::ostream& o = std::cout , const std::string& str="") const
    {
      double d = toc();
      o << color.white() << color.bold() << " [TIC] " << str << " " << name << " : \t\t" << d << " sec." << color.reset() << std::endl;
      return o;
    }
  };

  template<> struct Tic<false>
  {

    static const bool use = false;
    Tic(){}
    Tic(std::string) {}
    void tic() { }
    double toc() { return 0; }
    double disp(std::ostream& = std::cout , const std::string& ="") { return 0;}
  };

  template<bool Use> struct CTic
  {
    static const bool use = true;
    double t;
    std::string name;
    CTic(std::string name_ =""):name(name_)
    {
      tic();
    }

    inline void tic()
    {
      t = read_cycles();
    }

    inline double toc() const
    {
      return read_cycles() - t;
    }

    inline std::ostream& disp(std::ostream& o = std::cout , const std::string& str="") const
    {
      double d = toc();
      o << color.white() << color.bold() << " [CTIC] " << str << " " << name << " : " << d << " cycles." << color.reset() << std::endl;
      return o;
    }
  };

  template<> struct CTic<false>
  {
    static const bool use = false;
    CTic(std::string ="") {}
    void tic() { }
    double toc() { return 0; }
    std::ostream& disp(std::ostream& o= std::cout, const std::string& ="") { return o; }
  };
}

#endif
