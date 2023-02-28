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

#ifndef __LMA_OPTIMISATION2_CONTAINER_ADCT_EIGEN_HPP__
#define __LMA_OPTIMISATION2_CONTAINER_ADCT_EIGEN_HPP__

#include <boost/fusion/support/pair.hpp>
#include <boost/fusion/include/pair.hpp>

#include <libv/lma/numeric/ad/ct/adct.hpp>
#include <libv/lma/lm/container/tag.hpp>

namespace lma
{
  template<int ddl, int dim, class Float, int size> 
  std::array<adct::Ad<Float,ddl,Eig>,size> to_adct(const Eigen::Matrix<Float,size,1>& mat, boost::fusion::pair<Eig,Float>)
  {
    std::array<adct::Ad<Float,ddl,Eig>,size> array;
    for(size_t i = 0 ; i < size ; ++i)
      array[i] = adct::Ad<Float,ddl,Eig>(mat[i],i+dim);
    return array;
  }
  
  template<int ddl, int dim, class Float> 
  std::array<adct::Ad<Float,ddl,Eig>,1> to_adct(const Float& flt, boost::fusion::pair<Eig,Float>, typename boost::enable_if<boost::is_floating_point<Float>>::type* =0)
  {
#pragma GCC diagnostic push
#ifdef __clang__
#pragma GCC diagnostic ignored "-Wmissing-braces" // the line below is correct
#endif
    return {adct::Ad<Float,ddl,Eig>(flt,dim)};
#pragma GCC diagnostic pop
  }
}

#endif
