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

#ifndef __LIBV_LMA_NUMERIC_MEDIANE_HPP__
#define __LIBV_LMA_NUMERIC_MEDIANE_HPP__

#include <algorithm>

namespace lma
{
  template<class T, class A, template<class,class> class Container> 
  T mediane(Container<T,A> container)
  {
    if (container.end()==container.begin()) return T();
    size_t mid = (container.end() - container.begin()) / 2;
    std::nth_element(container.begin(), container.begin()+mid, container.end());
    return *(container.begin() + mid);
  }

  template<class T, class A, template<class,class> class Container, class Compare> 
  T mediane(Container<T,A> container, Compare compare)
  {
    if (container.end()==container.begin()) return T();
    size_t mid = (container.end() - container.begin()) / 2;
    std::nth_element(container.begin(), container.begin()+mid, container.end(),compare);
    return *(container.begin() + mid);
  }
}

#endif
