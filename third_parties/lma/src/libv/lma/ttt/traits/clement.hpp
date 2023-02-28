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

#ifndef __MODULES_TTT_TRAITS_CLEMENT_HPP__
#define __MODULES_TTT_TRAITS_CLEMENT_HPP__

struct clement_type{};

inline void clement( clement_type ){}

template<class ... T> inline void plz_no_warning(const T &... ){}

#endif