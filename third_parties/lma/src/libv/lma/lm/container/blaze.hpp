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

#ifndef __LIBV_LMA_OPTIMISATION2_CONTAINER_BLAZE_HPP__
#define __LIBV_LMA_OPTIMISATION2_CONTAINER_BLAZE_HPP__

#ifdef USE_BLAZE

#include "tag.hpp"
#include <blaze/Blaze.h>
#include <libv/lma/ttt/traits/naming.hpp>
#include <libv/lma/ttt/traits/wrap.hpp>
#include <libv/lma/string/string_utiliy.hpp>
#include <boost/fusion/include/pair.hpp>
#include "container.hpp"
#include <libv/lma/ttt/traits/unroll1.hpp>
#include <boost/mpl/if.hpp>

namespace std
{
  template<int I, int J, size_t N, class Float> inline const Float& get(const blaze::StaticVector<Float,N>& mat)
  {
    static_assert( I < N , "std::get<I,J>(blaze)");
    static_assert( J == 0 , "std::get<I,J>(toon)");
    return mat[I];
  }

  template<int I, int J, size_t N, size_t M, class Float> inline const Float& get(const blaze::StaticMatrix<Float,N,M>& mat)
  {
    static_assert( I < N , "std::get<I,J>(blaze)");
    static_assert( I < M , "std::get<I,J>(blaze)");
    return mat(I,J);
  }


  template<int I, int J, size_t N, class Float> inline Float& get(blaze::StaticVector<Float,N>& mat)
  {
    static_assert( I < N , "std::get<I,J>(blaze)");
    static_assert( J == 0 , "std::get<I,J>(blaze)");
    return mat[I];
  }

  template<int I, int J, size_t N, size_t M, class Float> inline Float& get(blaze::StaticMatrix<Float,N,M>& mat)
  {
    static_assert( I < N , "std::get<I,J>(blaze)");
    static_assert( I < M , "std::get<I,J>(blaze)");
    return mat(I,J);
  }
}

namespace lma
{
  template<std::size_t I, std::size_t J, class flt> struct ContainerOption<boost::fusion::pair<Blaze,flt>,I,J>
  {
    typedef flt Float;
    typedef typename 
      boost::mpl::if_c<
                        J==1,
                        blaze::StaticVector<Float,I>,
                        blaze::StaticMatrix<Float,I,J>
                      >::type Matrix;

    static Matrix Zero() { return Matrix(0); }
    typedef blaze::DynamicMatrix<Float> MatrixDD;
    typedef blaze::DynamicVector<Float> MatrixD1;
  };
  
  template<class Float, int N> struct Size<blaze::StaticVector<Float,N>> { enum { value = N }; };
  template<class Float, int N> struct Size<blaze::StaticMatrix<Float,N,1>> { enum { value = N }; };

  template<class Float, int Rows, int Cols> struct Rows<blaze::StaticMatrix<Float,Rows,Cols>>
  {
    enum { value = Rows };
  };

  
  template<class Float, int Rows, int Cols> struct Cols<blaze::StaticMatrix<Float,Rows,Cols>>
  {
    enum { value = Cols };
  };


  template<class Float, std::size_t I, std::size_t J>
  bool is_invalid(const blaze::StaticMatrix<Float,I,J>& mat)
  {
    for(int i = 0 ; i < I ; ++i)
      for(int j = 0 ; j < J ; ++j)
      if (is_invalid(mat(i,j)))
        return true;
    return false;
  }
  
  template<class Float>
  bool is_invalid(const blaze::DynamicVector<Float>& mat)
  {
    for(size_t i = 0 ; i < mat.size() ; ++i)
      if (is_invalid(mat[i]))
        return true;
    return false;
  }

  template<class Float, std::size_t N> const Float& at(const blaze::StaticMatrix<Float,N,N>& m, int i, int j)
  {
    return m(i,j);
  }

  template<class Float, std::size_t N> Float& at(blaze::StaticMatrix<Float,N,N>& m, int i, int j)
  {
    return m(i,j);
  }

  template<class Float, std::size_t N> Float& at(blaze::StaticVector<Float,N>& m, int i, int j)
  {
    plz_no_warning(j);
    assert(j==0);
    return m[i];
  }

  template<class Float, std::size_t N> const Float& at(const blaze::StaticVector<Float,N>& m, int i, int j)
  {
    plz_no_warning(j);
    assert(j==0);
    return m[i];
  }


  template<class Float> void set_zero(blaze::DynamicMatrix<Float>& mat)
  {
    mat.reset();
  }

  template<class Float, size_t I> void set_zero(blaze::StaticVector<Float,I>& mat)
  {
    mat.reset();
  }

  template<class Float, size_t I, size_t J> void set_zero(blaze::StaticMatrix<Float,I,J>& mat)
  {
    mat.reset();
  }

  template<class Float, size_t I>
  const Float* get_ptr(const blaze::StaticVector<Float,I>& m)
  {
    return m.data();
  }

  template<class Float, size_t I, size_t J, bool b>
  auto transpose(const blaze::StaticMatrix<Float,I,J,b>& mat)
  {
    return blaze::trans(mat);
  }

  template<class Float> void inverse_in_place(blaze::StaticMatrix<Float,3,3>& mat)
  {
    const Float 
      a = mat(0,0),
      b = mat(0,1),
      c = mat(0,2),
      e = mat(1,1),
      f = mat(1,2),
      i = mat(2,2),
      f2 = f*f,
      fc = f*c,
      ae = a*e,
      b_2 = b*b,
      c_2 = c*c,
      det = 1.0 / (ae*i + b*fc * 2.0 - c_2*e - f2*a - i*b_2);

    mat(0,0) = (e*i - f2) * det;
    mat(1,0) = mat(0,1) = (fc - b*i) * det;
    mat(1,1) = (a*i - c_2) * det;
    mat(2,1) = mat(1,2) = (b*c - a*f) * det;
    mat(2,0) = mat(0,2) = (b*f - e*c) * det;
    mat(2,2) = (ae - b_2) * det;
  }
}


namespace ttt
{
  template<> struct Name<lma::Blaze> 
  { static std::string name() { return "Blaze"; } };
  
  template<class Float, size_t I, size_t J>
  struct Name<blaze::StaticMatrix<Float,I,J>> 
  { static std::string name() { return "Blaze::Matrix<" + lma::to_string(I) + "," + lma::to_string(J) + "," + ttt::name<Float>() + ">"; } };

  template<class Float, size_t I>
  struct Name<blaze::StaticVector<Float,I>> 
  { static std::string name() { return "Blaze::Vector<" + lma::to_string(I) + "," + ttt::name<Float>() + ">"; } };
}

#endif // #ifdef USE_BLAZE

#endif
