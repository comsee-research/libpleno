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

#ifndef __MODULES_NUMERIC_AD_CD_ADCT_HPP__
#define __MODULES_NUMERIC_AD_CD_ADCT_HPP__

#include <Eigen/Core>
#include <boost/type_traits/is_floating_point.hpp>
#include <boost/utility/enable_if.hpp>

#ifdef USE_TOON
#include <TooN/TooN.h>
#include <libv/lma/lm/container/tag.hpp>
#endif
#include <libv/lma/ttt/traits/naming.hpp>

namespace adct
{
  template<class T> struct Expr
  {
    T const& cast() const {return static_cast<T const &>(*this); }
  };

  template<class T, int N, class Default=void> class Ad : public Expr<Ad<T,N,Default>>
  {
    public:
      typedef Eigen::Matrix<T, N, 1/*, Eigen::DontAlign*/> Array;
      Ad(const T& val = T()):value_(val),infinite_(Array::Zero()) {}
      Ad(const T& val , int i):value_(val),infinite_(Array::Zero()) { infinite_[i] = 1; }
      template<class X> Ad(const Expr<X>& expr):value_(expr.cast().value()),infinite_(expr.cast().infinite()) {}
      
      const T&     value()    const { return value_; }
      const Array& infinite() const { return infinite_; }

    private:
      T     value_;
      Array infinite_;
  };


#ifdef USE_TOON
  template<class T, int N> class Ad<T,N,lma::Toon> : public Expr<Ad<T,N,lma::Toon>>
  {
    public:
      typedef TooN::Vector<N,T> Array;
      Ad(const T& val = T()):value_(val),infinite_(TooN::Zeros) {}
      Ad(const T& val , int i):value_(val),infinite_(TooN::Zeros) { infinite_[i] = 1; }
      template<class X> Ad(const Expr<X>& expr):value_(expr.cast().value()),infinite_(expr.cast().infinite()) {}
      
      const T&     value()    const { return value_; }
      const Array& infinite() const { return infinite_; }

    private:
      T     value_;
      Array infinite_;
  };
#endif


  template<class T, class = void> struct Traits { typedef const T& type; };

  template<class A, class Op> struct Unary : Expr<Unary<A,Op>>
  {
    typename Traits<A>::type a;
    Unary(const A& a_):a(a_){}
    auto value()    const { return Op::value(a); }
    auto infinite() const { return Op::infinite(a); }
  };
  
  template<class A, class B, class Op> struct Binary : Expr<Binary<A,B,Op>>
  {
    typename Traits<A>::type a;
    typename Traits<B>::type b;
    Binary(const A& a_, const B& b_):a(a_),b(b_){}
    auto value()    const { return Op::value(a,b); }
    auto infinite() const { return Op::infinite(a,b); }
  };

  template<class T> struct Traits<T, typename boost::enable_if< boost::is_floating_point<T> >::type > { typedef T type; };
  template<class A, class B, class Op>  struct Traits<Binary<A,B,Op>> { typedef Binary<A,B,Op> type; };
  template<class A, class Op>           struct Traits<Unary<A,Op>>    { typedef Unary<A,Op> type; };
  
  struct Minus
  {
    template<class A> static auto value   (const A& a) { return - a.value();}
    template<class A> static auto infinite(const A& a) { return - a.infinite();}
  };
  
  struct Sqrt
  {
    template<class A> static auto value   (const A& a) { return sqrt(a.value()); }
    template<class A> static auto infinite(const A& a) { return a.infinite() / (2.0 * sqrt(a.value())); }
  };
  
  struct Cos
  {
    template<class A> static auto value   (const A& a) { return cos(a.value()); }
    template<class A> static auto infinite(const A& a) { return - sin(a.value()) * a.infinite(); }
  };
  
  struct Sin
  {
    template<class A> static auto value   (const A& a) { return sin(a.value()); }
    template<class A> static auto infinite(const A& a) { return cos(a.value()) * a.infinite(); }
  };
  
  struct Addition
  {
    template<class A, class B> static auto value   (const A& a, const B& b) { return a.value() + b.value() ;}
    template<class A, class B> static auto infinite(const A& a, const B& b) { return a.infinite() + b.infinite() ;}
  };
  
  struct Substract
  {
    template<class A, class B> static auto value   (const A& a, const B& b) { return a.value() - b.value() ;}
    template<class A, class B> static auto infinite(const A& a, const B& b) { return a.infinite() - b.infinite() ;}
  };

  struct Multiply
  {
    template<class A, class B> static auto value   (const A& a, const B& b) { return a.value() * b.value() ;}
    template<class A, class B> static auto infinite(const A& a, const B& b) { return a.value() * b.infinite() + a.infinite() * b.value(); }
  };
  
  struct AdditionScalar
  {
    template<class A> static auto value   (const A& a, double b) { return a.value() + b ;}
    template<class A> static auto infinite(const A& a, double  ) { return a.infinite() ;}
  };
  
  struct SubstractScalar
  {
    template<class A> static auto value   (const A& a, double b) { return a.value() - b ;}
    template<class A> static auto infinite(const A& a, double  ) { return a.infinite() ;}
  };
  
  struct ScalarSubstract
  {
    template<class A> static auto value   (double b, const A& a) { return b - a.value() ;}
    template<class A> static auto infinite(double  , const A& a) { return -a.infinite() ;}
  };
  
  struct MultiplyScalar
  {
    template<class A> static auto value   (const A& a, double b) { return a.value() * b ;}
    template<class A> static auto infinite(const A& a, double b) { return a.infinite() * b;}
  };
  
  struct DivideScalar
  {
    template<class A> static auto value   (const A& a, double b) { return a.value() / b ;}
    template<class A> static auto infinite(const A& a, double b) { return a.infinite() / b;}
  };
  
  struct ScalarDivide
  {
    static auto sqr(double t) { return t * t; }
    template<class B> static auto value   (double a, const B& b) { return a / b.value() ;}
    template<class B> static auto infinite(double a, const B& b) { return - b.infinite() * a / sqr(b.value()); }
  };

  struct Divide
  {
    template<class A, class B> static auto value(const A& a, const B& b) { return a.value() / b.value() ; }
    template<class A, class B> static auto infinite(const A& a, const B& b)
    {
      const double val_inv = 1.0 / b.value();
      return (a.infinite() - a.value() * val_inv * b.infinite()) * val_inv;
    }
  };

  template<class A, class B> auto operator+(const Expr<A>& a, const Expr<B>& b) { return Binary<A,B,Addition>(a.cast(),b.cast()); }
  template<class A, class B> auto operator-(const Expr<A>& a, const Expr<B>& b) { return Binary<A,B,Substract>(a.cast(),b.cast()); }
  template<class A, class B> auto operator*(const Expr<A>& a, const Expr<B>& b) { return Binary<A,B,Multiply>(a.cast(),b.cast()); }
  template<class A, class B> auto operator/(const Expr<A>& a, const Expr<B>& b) { return Binary<A,B,Divide>(a.cast(),b.cast()); }

  template<class A>          auto operator+(const Expr<A>& a, double b)         { return Binary<A,double,AdditionScalar>(a.cast(),b); }
  template<class A>          auto operator+(double b, const Expr<A>& a)         { return a+b; }
  template<class A>          auto operator-(const Expr<A>& a, double b)         { return Binary<A,double,SubstractScalar>(a.cast(),b); }
  template<class A>          auto operator-(double b, const Expr<A>& a)         { return Binary<double,A,ScalarSubstract>(b,a.cast()); }

  template<class A>          auto operator*(const Expr<A>& a, double b)         { return Binary<A,double,MultiplyScalar>(a.cast(),b); }
  template<class A>          auto operator*(double b, const Expr<A>& a)         { return a*b; }
  template<class A>          auto operator/(const Expr<A>& a, double b)         { return Binary<A,double,DivideScalar>(a.cast(),b); }
  template<class B>          auto operator/(double a, const Expr<B>& b)         { return Binary<double,B,ScalarDivide>(a,b.cast()); }
  
  template<class A>          auto operator-(const Expr<A>& a)                   { return Unary<A,Minus>(a.cast()); }
  template<class A>          auto sqrt     (const Expr<A>& a)                   { return Unary<A,Sqrt>(a.cast()); }
  template<class A>          auto cos      (const Expr<A>& a)                   { return Unary<A,Cos>(a.cast()); }
  template<class A>          auto sin      (const Expr<A>& a)                   { return Unary<A,Sin>(a.cast()); }
  
  template<class A, class B> bool operator>(const Expr<A>& a, const Expr<B>& b) { return a.cast().value() > b.cast().value() ; }
  template<class A, class B> bool operator<(const Expr<A>& a, const Expr<B>& b) { return a.cast().value() < b.cast().value() ; }
}

#include <libv/lma/ttt/traits/naming.hpp>
#include <libv/lma/string/string_utiliy.hpp>

namespace ttt
{
  template<class T, int N> struct Name<adct::Ad<T,N>>
  {
    static std::string name(){ return std::string("Ad<") + ttt::name<T>() + "," + lma::to_string(N) + ">"; }
  };
  
}

#endif
