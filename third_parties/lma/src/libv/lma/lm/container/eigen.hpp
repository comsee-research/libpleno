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

#ifndef __OPTIMISATION2_CONTAINER_EIGEN_HPP__
#define __OPTIMISATION2_CONTAINER_EIGEN_HPP__

#include "tag.hpp"
#include <Eigen/Dense>
// #include <Eigen/SparseCholesky>
#include <libv/lma/lm/trait/accessor.hpp>
#include <libv/lma/lm/trait/adl.hpp>
#include <libv/lma/ttt/traits/naming.hpp>
#include <libv/lma/ttt/traits/wrap.hpp>
#include <libv/lma/string/string_utiliy.hpp>
#include <boost/fusion/include/pair.hpp>
#include "residual.hpp"
#include <libv/lma/lm/container/container.hpp>


namespace std
{
  template<int I, int J, int N, int M, class Float> inline const Float& get(const Eigen::Matrix<Float,N,M>& mat)
  {
    static_assert( I < N , "std::get<I>(eigen)");
    static_assert( J < M , "std::get<I>(eigen)");
    return mat(I,J);
  }

  template<int I, int J, int N, class Float> inline const Float& get(const Eigen::Matrix<Float,I,1>& mat)
  {
    static_assert( I < N , "std::get<I,J>(toon)");
    static_assert( J == 0 , "std::get<I,J>(toon)");
    return mat[I];
  }

  template<int I, int N, class Float> inline const Float& get(const Eigen::Matrix<Float,N,1>& mat)
  {
    static_assert( I < N , "std::get<I,J>(toon)");
    return mat[I];
  }



  template<int I, int J, int N, int M, class Float> inline Float& get(Eigen::Matrix<Float,N,M>& mat)
  {
    static_assert( I < N , "std::get<I>(eigen)");
    static_assert( J < M , "std::get<J>(eigen)");
    return mat(I,J);
  }

  template<int I, int J, int N, class Float> inline Float& get(Eigen::Matrix<Float,I,1>& mat)
  {
    static_assert( I < N , "std::get<I,J>(toon)");
    static_assert( J == 0 , "std::get<I,J>(toon)");
    return mat[I];
  }
}

namespace lma
{
  template<std::size_t I, std::size_t J, class flt> struct ContainerOption<boost::fusion::pair<Eig,flt>,I,J>
  {
    typedef flt Float;
    typedef Eigen::Matrix<Float,I,J> Matrix;
    static EIGEN_STRONG_INLINE const typename Eigen::DenseBase<Matrix>::ConstantReturnType Zero() { return Matrix::Zero(); }
    typedef Eigen::Matrix<Float,Eigen::Dynamic,Eigen::Dynamic> MatrixDD;
    typedef Eigen::Matrix<Float,Eigen::Dynamic,1> MatrixD1;
  };

  template<class Float, int Rows_, int Cols_> struct Rows<Eigen::Matrix<Float,Rows_,Cols_>>
  {
    enum { value = Rows_ };
  };

  template<class Float, int Rows_, int Cols_> struct Cols<Eigen::Matrix<Float,Rows_,Cols_>>
  {
    enum { value = Cols_ };
  };

////////////////////////////////////////////////////////

////////////////////////////////////////////////////////
  template<class T, size_t I, int K> struct ChooseBackUp<Eigen::Matrix<T,K,1>,I>
  {
    typedef Eigen::Matrix<T,K,1> Obj;
    typedef BackUp<T> Result;
    
    static Result create_back_up(Obj& obj)
    {
      return Result(obj(I));
    }
  };
  
  template<class T, int I, int J> struct Size< Eigen::Matrix<T,I,J> > { static const std::size_t value = I; };
  
  template<class Float, int N, int M> inline Float& at(Eigen::Matrix<Float,N,M>& m, int i, int j)
    { static_assert(N==M,"at(Eigen::Matrix<F,N,M>): N!=M");assert(i<N && j<N);return m(i,j); }

  template<class Float, int N, int M> inline const Float& at(const Eigen::Matrix<Float,N,M>& m, int i, int j)
    { static_assert(N==M,"at(Eigen::Matrix<F,N,M>): N!=M");assert(i<N && j<N);return m(i,j); }
  
  template<class Float, size_t N> Eigen::Map<const Eigen::Matrix<Float,int(N),1>> make_view(const std::array<Float,N>& residual, ttt::wrap<boost::fusion::pair<Eig,Float>>)
  {
    return Eigen::Map<const Eigen::Matrix<Float,N,1>>(residual.data());
  }
  
  template<class Float, int N> void damp_diag(Eigen::Matrix<Float,N,N>& m, const Float& lambda)
  {
    for(int i = 0 ; i < N ; ++i)
      m(i,i) += lambda;
  }
  
  template<class Float, int N> const Eigen::Matrix<Float,N,1>& make_view(const Eigen::Matrix<Float,N,1>& residual, ttt::wrap<boost::fusion::pair<Eig,Float>>)
  {
    return residual;
  }

  template<class Float, std::size_t X, std::size_t Y> struct Blocker<boost::fusion::pair<Eig,Float>,X,Y>
  {
    template<class Derived> static Eigen::Block<Derived,X,Y> view(Eigen::MatrixBase<Derived>& m, std::size_t j, std::size_t k)
    {
      return Eigen::Block<Derived,X,Y>(m.derived(), j, k);
    }
  };
  

  template<class T, int I, int J>
  bool is_invalid(const Eigen::Matrix<T,I,J>& mat)
  {
    for(int k = 0 ; k < mat.size() ; ++k)
      if (is_invalid(mat(k)))
        return true;
    return false;
  }

  template<class T, int I, int J>
  bool is_symmetric(const Eigen::Matrix<T,I,J>& mat)
  {
    for(int i = 0 ; i < I ; ++i)
      for(int j = 0 ; j < J ; ++j)
        if (mat(i,j) != mat(j,i))
          return false;
    return true;
  }
  

  
  template<class Float, int I>
  const Float* get_ptr(const Eigen::Matrix<Float,I,1>& m)
  {
    return m.data();
  }
  
  template<class Derived>
  EIGEN_STRONG_INLINE typename Eigen::NumTraits<typename Eigen::internal::traits<Derived>::Scalar>::Real 
  squared_norm(const Eigen::MatrixBase<Derived>& m)
  {
    return m.squaredNorm();
  }
  
  template<class Derived>
  inline typename Eigen::NumTraits<typename Eigen::internal::traits<Derived>::Scalar>::Real 
  norm(const Eigen::MatrixBase<Derived>& m)
  {
    return m.norm();
  }
  
  template<class Float, size_t N> 
  Eigen::Map<const Eigen::Matrix<Float,N,1>> array_to_eigen(const std::array<Float,N>& array)
  {
    return Eigen::Map<const Eigen::Matrix<Float,N,1>>(array.data());
  }
  
  template<class Float, size_t N> 
  Float squared_norm(const std::array<Float,N>& res)
  { return squared_norm(array_to_eigen(res));}

  template<class Float, size_t N> 
  Float norm(const std::array<Float,N>& res)
  { return norm(array_to_eigen(res));}


  template<class Derived, class T, size_t N>
  void cwise_product(std::array<T,N>& a, const Eigen::MatrixBase<Derived>& b, const std::array<T,N>& c)
  {
    auto x = (b.cwiseProduct(array_to_eigen(c))).eval();
    for(size_t i = 0 ; i < N ; ++i)
      a[i] = x[i];
  }
  
  template<class T, int N>
  void cwise_product(Eigen::Matrix<T,N,1>& a, const Eigen::Matrix<T,N,1>& b, const Eigen::Matrix<T,N,1>& c)
  {
    a = b.cwiseProduct(c);
  }
  
  template<class T>
  void cwise_product(double& a, const Eigen::Matrix<T,1,1>& b, const double& c)
  {
    a = b[0] * c;
  }


////////////////////////////////////////////////////////

  template<class T, int I> void apply_increment(Eigen::Matrix<T, I, 1>& obj, const T *delta, const Adl&)
  {
	  for (int i = 0; i < I; i++)
		  obj[i] += delta[i];
  }

  template<class T> void apply_increment(Eigen::Matrix<T, 1, 1>& obj, const T& delta, const Adl&)
  {
	  obj[0] += delta;
  }

  template<class T, int K, int I> void apply_small_increment(Eigen::Matrix<T, K, 1>& obj, T h, v::numeric_tag<I>, const Adl&)
  {
	  static_assert(I < K, " apply_small_increment : I < K ");
	  obj[I] += h;
  }

////////////////////////////////////////////////////////


  template<class Float, int I> Eigen::Matrix<Float,I,I> inverse(const Eigen::Matrix<Float,I,I>& mat)
  {
    return mat.inverse();
  }
  
  template<class Float, int I, int J> void inverse_in_place(Eigen::Matrix<Float,I,J>& mat)
  {
    static_assert(I==J,"inverse_in_place(Matrix<F,I,J>: I!=J");
    mat = mat.inverse().eval();
  }
  
  template<class Float, int I, int J>
  const Eigen::Transpose<const Eigen::Matrix<Float,I,J>> transpose(const Eigen::Matrix<Float,I,J>& mat)
  {
    return mat.transpose();
  }
  
  template<class Derived, class OtherDerived>
  auto dot(const Eigen::MatrixBase<Derived>& a, const Eigen::MatrixBase<OtherDerived>& b)
  {
    return a.dot(b);
  }
  
  template<class Float, int I, int J> void set_zero(Eigen::Matrix<Float,I,J>& mat)
  {
    mat.setZero();
  }
  
  template<class Float>
  void ldlt_solve(Eigen::Matrix<Float,Eigen::Dynamic,1>& x, const Eigen::Matrix<Float,Eigen::Dynamic,Eigen::Dynamic>& a, const Eigen::Matrix<Float,Eigen::Dynamic,1>&  b)
  {
    assert(a.cols()!=0 && a.rows()!=0 && b.size()!=0);
    x = a.template selfadjointView<Eigen::Upper>().ldlt().solve(b);
  }
}

namespace ttt
{
  template<> struct Name<lma::Eig> { static std::string name() { return "Eigen"; } };
  template<class T, int I, int J> struct Name<Eigen::Matrix<T,I,J>> { static std::string name() { return "Eigen<" + ttt::name<T>() + "," + lma::to_string(I) + "," + lma::to_string(J) + ">";}};
}

#endif
