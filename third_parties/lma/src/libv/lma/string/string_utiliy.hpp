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

#ifndef __MODULE__STRING_UTILIY_HPP__
#define __MODULE__STRING_UTILIY_HPP__

#include <boost/lexical_cast.hpp>
#include <string>
#include <sstream>

namespace lma
{
template<class T>
std::string to_string(const T& obj)
{
  return boost::lexical_cast<std::string>(obj);
//   return boost::format(obj).str();
//   std::stringstream str;
//   str << obj;
//   return str.str();
}

template<class T>
const char* to_char(const T& t)
{
  return to_string(t).c_str();
}
}

#endif // STRING_UTILIY_HPP
