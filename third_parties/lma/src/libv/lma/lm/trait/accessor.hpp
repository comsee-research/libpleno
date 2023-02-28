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

#ifndef __OPTIMISATION2_TRAIT_ACCESSOR_HPP__
#define __OPTIMISATION2_TRAIT_ACCESSOR_HPP__

#include <cstddef>
#include <Eigen/Core>
#include <libv/core/tag.hpp>
#include <libv/lma/ttt/traits/naming.hpp>
#include "size.hpp"


namespace lma
{
  template<class T, class Enable = void> struct BackUp
  {
    T& ref;
    const T back;
    BackUp(T& obj_):ref(obj_),back(ref){}
    void restore(){ref = back;}
    ~BackUp(){restore();}
  };

  template<class T, size_t I> struct ChooseBackUp
  {
    typedef BackUp<T> Result;
    
    static Result create_back_up(T& obj)
    {
      return Result(obj);
    }
  };

  template<size_t I, class T> typename ChooseBackUp<T,I>::Result back_up(T& obj)
  {
    return ChooseBackUp<T,I>::create_back_up(obj);
  }
}// eon

#endif
