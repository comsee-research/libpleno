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

#ifndef __NUMERIC_AD_RT_AD_HPP__
#define __NUMERIC_AD_RT_AD_HPP__

#ifdef USE_TOON

#include <iostream>
#include <Eigen/Dense>
#include <TooN/TooN.h>
#include <libv/lma/ttt/traits/naming.hpp>
#include <cmath>

namespace AdRt
{
//   Eigen::Matrix<T,N,1>
  template<class T, std::size_t N > struct Ad
  {
    typedef T type;
    static const std::size_t dim = N;

    typedef TooN::Vector<N,T> Array;
    T value;
    Array infinite;

    Ad():value(0) {}

    Ad(type value_) : value(value_)
    {
//       infinite.fill(0);
      infinite = TooN::Zeros;
    }

    Ad(type value_, std::size_t k)
    {
      value = value_;
//       infinite.fill(0);
      infinite = TooN::Zeros;
      infinite[k] = type(1);
    }

    Ad(type value_, const Array& infinite_): value(value_),infinite(infinite_) {}

//     bool operator<(type value_) const
//     {
//       return value < value_;
//     }
//     
//     bool operator>(type value_) const
//     {
//       return value > value_;
//     }
// 
//     bool operator<=(type value_) const
//     {
//       return value <= value_;
//     }
// 
//     bool operator<=(const Ad& ad) const
//     {
//       return value <= ad.value;
//     }
//     
    bool operator>(const Ad& ad) const
    {
      return value > ad.value;
    }
    
    Ad<T,N>& operator+=(const Ad<type, dim>& st) { *this = *this + st; return *this; }
//     Ad<T,N>& operator-=(const Ad<type, dim>& st) { return *this -= st; }
//     Ad<T,N>& operator*=(const Ad<type, dim>& st) { return *this *= st; }
//     Ad<T,N>& operator/=(const Ad<type, dim>& st) { return *this /= st;}
  };

  template<class T, std::size_t N> bool operator==(const Ad<T,N>& st, const T& scalar)
  {
    return st.value == scalar;
  }

  template<class T, std::size_t N> bool operator!=(const Ad<T,N>& st, const T& scalar)
  {
    return st.value != scalar;
  }

  //! Arithmetic for Ad
  template<class T, std::size_t N> const Ad<T,N>& operator+(const Ad<T,N>& st)
  {
    return st;
  }

  template<class T, std::size_t N> Ad<T,N> operator-(const Ad<T,N>& st)
  {
    return Ad<T,N>(-st.value,-st.infinite);
  }

  template<class T, std::size_t N> Ad<T,N> operator+(const Ad<T,N>& s, const Ad<T,N>& t)
  {
    return Ad<T,N>(s.value + t.value, s.infinite + t.infinite);
  }

  template<class T, std::size_t N> Ad<T,N> operator+(const Ad<T,N>& s, const T& value)
  {
    return Ad<T,N>(s.value + value, s.infinite);
  }

  template<class T, std::size_t N> Ad<T, N> operator+(const T& value, const Ad<T, N>& s)
  {
    return s + value;
  }

  template<class T, std::size_t N> Ad<T,N> operator-(const Ad<T,N>& s, const Ad<T,N>& t)
  {
    return Ad<T,N>(s.value - t.value, s.infinite - t.infinite);
  }

  template<class T, std::size_t N> Ad<T,N> operator-(const Ad<T,N>& s, const T& value)
  {
    return Ad<T,N>(s.value - value, s.infinite);
  }

  template<class T, std::size_t N> Ad<T,N> operator-(const T& value, const Ad<T,N>& s)
  {
    return -s + value;
  }

  template<class T, std::size_t N> Ad<T,N> operator*(const Ad<T,N>& s, const Ad<T,N>& t)
  {
    return Ad<T,N>(s.value * t.value , s.value * t.infinite + s.infinite * t.value );
  }

  template<class T, std::size_t N> Ad<T,N> operator*(const Ad<T,N>& s, const T& value)
  {
    return Ad<T,N>(s.value * value, s.infinite * value);
  }

  template<class T, std::size_t N> Ad<T,N> operator*(const T& value, const Ad<T,N>& s)
  {
    return s * value;
  }

  template<class T, std::size_t N> Ad<T,N> operator/(const Ad<T,N>& s, const Ad<T,N>& t)
  {
    return Ad<T,N>(s.value / t.value, (s.infinite - s.value / t.value * t.infinite ) / t.value);
  }

  template<class T, std::size_t N> Ad<T,N> operator/(const T& value, const Ad<T,N>& s)
  {
    return Ad<T,N>(value / s.value, - value * s.infinite / ( s.value * s.value ) );
  }

  template<class T, std::size_t N> Ad<T,N> operator/(const Ad<T,N>& s, const T& value)
  {
    return Ad<T,N>(s.value / value, s.infinite / value);
  }

//! trigonometric and others functions


  template <class T, std::size_t N> Ad<T, N> abs(const Ad<T, N>& f)
  {
    return Ad<T,N>(f.value < T(0.0) ? -f : f);
  }

  template <class T, std::size_t N> Ad<T, N> log(const Ad<T, N>& f)
  {
    return Ad<T, N>(log(f.value),f.infinite / f.value);
  }

  template <class T, std::size_t N> Ad<T, N> exp(const Ad<T, N>& f)
  {
    Ad<T, N> g;
    g.value = exp(f.value);
    g.infinite = g.value * f.infinite;
    return g;
  }

  template <class T, std::size_t N> Ad<T, N> sqrt(const Ad<T, N>& f)
  {
    Ad<T, N> g;
    g.value = std::sqrt(f.value);
    g.infinite = f.infinite / (T(2.0) * g.value);
    return g;
  }

  template <class T, std::size_t N> Ad<T, N> cos(const Ad<T, N>& f)
  {
    using std::sin;using std::cos;
    Ad<T, N> g;
    g.value = cos(f.value);
    T sin_a = sin(f.value);
    g.infinite = - sin_a * f.infinite;
    return g;
  }

  template <class T, std::size_t N> Ad<T, N> acos(const Ad<T, N>& f)
  {
    return Ad<T, N>(acos(f.value),- T(1.0) / sqrt(T(1.0) - f.value * f.value) * f.infinite);
  }

  template <class T, std::size_t N> Ad<T, N> sin(const Ad<T, N>& f)
  {
    using std::sin;using std::cos;
    Ad<T, N> g;
    g.value = sin(f.value);
    T cos_a = cos(f.value);
    g.infinite = cos_a * f.infinite;
    return g;
  }

  template <class T, std::size_t N> Ad<T, N> asin(const Ad<T, N>& f)
  {
    return Ad<T, N>(asin(f.a),T(1.0) / sqrt(T(1.0) - f.a * f.a) * f.v);
  }


}//! eon AdRt

  template<class T, std::size_t N> std::ostream& operator<<(std::ostream& o, const AdRt::Ad<T,N>& st)
  {
    return o << " Ad<" << ttt::name<T>() << "," << N << ":value = " << st.value << " ; [" << st.infinite << "]";
  }

#endif // #ifdef USE_TOON

#endif
