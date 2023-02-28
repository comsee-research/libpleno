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

#ifndef __LMA_OPTIMISATION2_CONTOAINRE_ADCT_TOON_HPP__
#define __LMA_OPTIMISATION2_CONTOAINRE_ADCT_TOON_HPP__

#ifdef USE_TOON

#include <libv/lma/numeric/ad/ct/adct.hpp>

namespace lma
{
  template<int ddl, int dim, class Float, int size>
  std::array<adct::Ad<Float,ddl,Toon>,size> to_adct(const TooN::Vector<size,Float>& mat, boost::fusion::pair<Toon,Float>)
  {
    std::array<adct::Ad<Float,ddl,Toon>,size> array;
    for(size_t i = 0 ; i < size ; ++i)
      array[i] = adct::Ad<Float,ddl,Toon>(mat[i],i+dim);
    return array;
  }
  
  template<int ddl, int dim, class Float, int size>
  std::array<adct::Ad<Float,ddl,Toon>,size> to_adct(const Eigen::Matrix<Float,size,1>& mat, boost::fusion::pair<Toon,Float>)
  {
    std::array<adct::Ad<Float,ddl,Toon>,size> array;
    for(size_t i = 0 ; i < size ; ++i)
      array[i] = adct::Ad<Float,ddl,Toon>(mat[i],i+dim);
    return array;
  }

  template<int ddl, int dim, class Float> 
  std::array<adct::Ad<Float,ddl,Toon>,1> to_adct(const Float& flt, boost::fusion::pair<Toon,Float>, typename boost::enable_if<boost::is_floating_point<Float>>::type* =0)
  {
    return {adct::Ad<Float,ddl,Toon>(flt,dim)};
  }
}

#endif

#endif