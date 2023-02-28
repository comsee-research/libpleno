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

#ifndef __LIBV_LMA_OPTIMISATION2_CONTAINER_TOON_HPP__
#define __LIBV_LMA_OPTIMISATION2_CONTAINER_TOON_HPP__

#ifdef USE_TOON

#include "tag.hpp"
#include <TooN/TooN.h>
#include <TooN/Lapack_Cholesky.h>
#include <libv/lma/ttt/traits/naming.hpp>
#include <libv/lma/ttt/traits/wrap.hpp>
#include <libv/lma/string/string_utiliy.hpp>
#include <boost/fusion/include/pair.hpp>
#include "container.hpp"
#include <libv/lma/ttt/traits/unroll1.hpp>
#include <boost/mpl/if.hpp>


namespace std
{
  template<int I, int J, int N, class Float> inline const Float& get(const TooN::Vector<N,Float>& mat)
  {
    static_assert( I < N , "std::get<I,J>(toon)");
    static_assert( J == 0 , "std::get<I,J>(toon)");
    return mat[I];
  }

  template<int I, int J, int N, int M, class Float> inline const Float& get(const TooN::Matrix<N,M,Float>& mat)
  {
    static_assert( I < N , "std::get<I,J>(toon)");
    static_assert( I < M , "std::get<I,J>(toon)");
    return mat(I,J);
  }


  template<int I, int J, int N, class Float> inline Float& get(TooN::Vector<N,Float>& mat)
  {
    static_assert( I < N , "std::get<I,J>(toon)");
    static_assert( J == 0 , "std::get<I,J>(toon)");
    return mat[I];
  }

  template<int I, int J, int N, int M, class Float> inline Float& get(TooN::Matrix<N,M,Float>& mat)
  {
    static_assert( I < N , "std::get<I,J>(toon)");
    static_assert( I < M , "std::get<I,J>(toon)");
    return mat(I,J);
  }
}


namespace lma
{
  template<std::size_t I, std::size_t J, class flt> struct ContainerOption<boost::fusion::pair<Toon,flt>,I,J>
  {
    typedef flt Float;
    typedef typename 
      boost::mpl::if_c<
                        J==1,
                        TooN::Vector<I,Float>,
                        TooN::Matrix<I,J,Float>
                      >::type Matrix;

    static TooN::Operator<TooN::Internal::Zero> Zero() { return TooN::Zeros; }
    typedef TooN::Matrix<TooN::Dynamic,TooN::Dynamic,Float> MatrixDD;
    typedef TooN::Vector<TooN::Dynamic,Float> MatrixD1;
  };
  

  template<class Float, int N, class _> struct Size<TooN::Vector<N,Float,_>> { enum { value = N }; };
  template<class Float, int N, class _> struct Size<TooN::Matrix<N,1,Float,_>> { enum { value = N }; };

  template<class JA, class JB> inline void jtj(TooN::Vector<1>& result, const JA& Ja, const JB& Jb)
  {
    result[0] += transpose(Ja) * Jb;
  }

  template<int I, class E, class Tag, class Float> inline void jte(TooN::Vector<I,Float>& result, const TooN::Matrix<1,I,Float>& jacob, const E& erreur, ttt::wrap<Tag> const&)
  {
    result.as_row() -= transpose(jacob) * make_view(erreur,ttt::wrap<Tag>());
  }

  template<class Float, int Rows, int Cols> struct Rows<TooN::Matrix<Rows,Cols,Float>>
  {
    enum { value = Rows };
  };

  
  template<class Float, int Rows, int Cols> struct Cols<TooN::Matrix<Rows,Cols,Float>>
  {
    enum { value = Cols };
  };

  template<class Float, int Rows> struct Rows<TooN::Vector<Rows,Float>>
  {
    enum { value = Rows };
  };

  template<class Float, int Rows> struct Cols<TooN::Vector<Rows,Float>>
  {
    enum { value = 1 };
  };

  
  template<class Float, size_t N> TooN::Vector<int(N),const Float,TooN::Reference> make_view(const std::array<Float,N>& residual, ttt::wrap<boost::fusion::pair<Toon,Float>>)
  {
    return TooN::Vector<int(N),const Float,TooN::Reference>(residual.data());
  }

  template<class Float, int N, class _> Float& at(TooN::Matrix<N,N,Float,_>& m, int i, int j)
  {
    return m(i,j);
  }
  
  template<class Float, int N, class _> Float& at(TooN::Vector<N,Float,_>& m, int i, int j)
  {
    plz_no_warning(j);
    assert(j==0);
    return m[i];
  }
  
  template<class Float, int N, class _> const Float& at(const TooN::Matrix<N,N,Float,_>& m, int i, int j)
  {
    return m(i,j);
  }
  
  template<class Float, int N, class _> const Float& at(const TooN::Vector<N,Float,_>& m, int i, int j)
  {
    plz_no_warning(j);
    assert(j==0);
    return m[i];
  }
  
  template<class Float, int N, class _> void damp_diag(TooN::Matrix<N,N,Float,_>& m, const Float& lambda)
  {
    for(int i = 0 ; i < N ; ++i)
      m(i,i) += lambda;
  }
  
  template<class Float, class _> void damp_diag(TooN::Vector<1,Float,_>& m, const Float& lambda)
  {
    m[0] += lambda;
  }
  
  template<class Float, std::size_t X, std::size_t Y> struct Blocker<boost::fusion::pair<Toon,Float>,X,Y>
  {
    static auto view(TooN::Matrix<TooN::Dynamic,TooN::Dynamic,Float>& m, std::size_t j, std::size_t k)
    -> decltype(m.slice(j,k,X,Y))
    {
      return m.slice(j,k,X,Y);
    }
  };
  
  template<class Float, int I, int J> void set_zero(TooN::Matrix<I,J,Float>& mat)
  {
    mat = TooN::Zeros;
  }

  template<class Float, int I> void set_zero(TooN::Vector<I,Float>& vec)
  {
    vec = TooN::Zeros;
  }
  
  template<class Float, int I, int J>
  auto transpose(const TooN::Matrix<I,J,Float>& mat) -> decltype(mat.T())
  {
    return mat.T();
  }
  
  template<class Float, int I> 
  auto dot(const TooN::Vector<I,Float>& a, const TooN::Vector<I,Float>& b) -> decltype(a*b)
  {
    return a * b;
  }
  
  template<class Float, int I>
  Float squared_norm(const TooN::Vector<I,const Float,TooN::Reference>& m)
  {
    return m * m;
  }
  
  template<class Float, int I>
  auto squared_norm(const TooN::Vector<I,Float>& m) -> decltype(m*m)
  {
    return m * m;
  }
  
  template<class Float, int I>
  const Float* get_ptr(const TooN::Vector<I,Float>& m)
  {
    return m.get_data_ptr();
  }
  
  template<class Float, int I>
  const Float* get_ptr(const TooN::Matrix<I,1,Float>& m)
  {
    return m.get_data_ptr();
  }
  
  template<class Float, int I, int J>
  bool is_invalid(const TooN::Matrix<I,J,Float>& mat)
  {
    for(int i = 0 ; i < I ; ++i)
      for(int j = 0 ; j < J ; ++j)
      if (is_invalid(mat(i,j)))
        return true;
    return false;
  }
  
  template<class Float>
  bool is_invalid(const TooN::Vector<TooN::Dynamic,Float>& mat)
  {
    for(int i = 0 ; i < mat.size() ; ++i)
      if (is_invalid(mat[i]))
        return true;
    return false;
  }
  
  template<class Float>
  void ldlt_solve(TooN::Vector<TooN::Dynamic,Float>& x, const TooN::Matrix<TooN::Dynamic,TooN::Dynamic,Float>& a, const TooN::Vector<TooN::Dynamic,Float>&  b)
  {
//     x = TooN::Lapack_Cholesky<TooN::Dynamic,Float>(a).backsub(b);
    Eigen::Map<const Eigen::Matrix<Float,Eigen::Dynamic,Eigen::Dynamic>> a1(a.get_data_ptr(),b.size(),b.size());
    Eigen::Map<Eigen::Matrix<Float,Eigen::Dynamic,1>> x1(x.get_data_ptr(),b.size());
    Eigen::Map<const Eigen::Matrix<Float,Eigen::Dynamic,1>> b1(b.get_data_ptr(),b.size());
    x1 = a1.template selfadjointView<Eigen::Lower>().llt().solve(b1);
  }
  
  template<class Float, int I> TooN::Matrix<I,I,Float> inverse(TooN::Matrix<I,I,Float> mat)
  {
//     return TooN::Cholesky<I>(mat).get_inverse();
    TooN::Matrix<I,I,Float> res;
    invert_symmetric_matrix<Float,I>(mat.get_data_ptr(),res.get_data_ptr());
    return res;
  }
  
  template<class Float, int I> void inverse_in_place(TooN::Matrix<I,I,Float>& mat)
  {
    mat = inverse(mat);
  }
  
  template<class Float> void inverse_in_place(TooN::Matrix<3,3,Float>& mat)
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
  template<> struct Name<lma::Toon> 
  { static std::string name() { return "TooN"; } };
  
  template<class Float, int I, int J>
  struct Name<TooN::Matrix<I,J,Float>> 
  { static std::string name() { return "TooN::Matrix<" + lma::to_string(I) + "," + lma::to_string(J) + "," + ttt::name<Float>() + ">"; } };

  template<class Float, int I>
  struct Name<TooN::Vector<I,Float>> 
  { static std::string name() { return "TooN::Vector<" + lma::to_string(I) + "," + ttt::name<Float>() + ">"; } };
}

#endif // #ifdef USE_TOON

#endif
