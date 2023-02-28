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

#ifndef __LIBV_LMA_NUMERIC_AD_CT_VERBOSE_HPP__
#define __LIBV_LMA_NUMERIC_AD_CT_VERBOSE_HPP__

#include "adct.hpp"
#include <iostream>

namespace adct
{
  inline std::ostream& disp(const double& d, std::ostream& o)
  {
    return o << "d(" << d << ")";
  }
  
  inline std::ostream& disp(const Addition& , std::ostream& o)
  {
    return o << "Addition";
  }
  
  inline std::ostream& disp(const Multiply& , std::ostream& o)
  {
    return o << "Multiply";
  }
  
  inline std::ostream& disp(const Minus& , std::ostream& o)
  {
    return o << "Minus";
  }
  
  inline std::ostream& disp(const Substract& , std::ostream& o)
  {
    return o << "Substract";
  }
  
  inline std::ostream& disp(const Divide& , std::ostream& o)
  {
    return o << "Divide";
  }
  
  inline std::ostream& disp(const DivideScalar& , std::ostream& o)
  {
    return o << "DivideScalar";
  }
  
  inline std::ostream& disp(const ScalarDivide& , std::ostream& o)
  {
    return o << "ScalarDivide";
  }
  
  inline std::ostream& disp(const MultiplyScalar& , std::ostream& o)
  {
    return o << "MultiplyScalar";
  }
  
  inline std::ostream& disp(const Sqrt& , std::ostream& o)
  {
    return o << "Sqrt";
  }
  
  inline std::ostream& disp(const Sin& , std::ostream& o)
  {
    return o << "Sin";
  }
  
  inline std::ostream& disp(const Cos& , std::ostream& o)
  {
    return o << "Cos";
  }
  
  
  template<class A, class B, class Op> std::ostream& disp(const Binary<A,B,Op>& binary, std::ostream& o)
  {
    return disp(binary.a,disp(binary.b,disp(Op(),o) << "<") << ",") << ">" ;
  }
  
  template<class A, class Op> std::ostream& disp(const Unary<A,Op>& unary, std::ostream& o)
  {
    return disp(unary.a,disp(Op(),o) << "<") << ">";
  }
  
  template<class T, int N> std::ostream& disp(const Ad<T,N>& /*ad*/, std::ostream& o)
  {
    return o << "Ad<>" ;
  }

  template<class A> std::ostream& disp(const Expr<A>& expr, std::ostream& o = std::cout)
  {
    return disp(expr.cast(),o);
  }
}

template<class T, int N> std::ostream& operator<<(std::ostream& o, const adct::Ad<T,N>& ad)
{
  return o << "[" << ad.value() << "," << ad.infinite().transpose() << "]";
}

#endif

