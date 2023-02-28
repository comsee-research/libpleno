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


#ifndef __LIBV_LMA_LMA_HPP__
#define __LIBV_LMA_LMA_HPP__

#include <libv/lma/lm/solver/solver2.hpp>
#include <libv/lma/lm/solver/verbose.hpp>

namespace lma
{
  typedef LdltTag       <Eig,double>     DENSE_;
  typedef PcgTag        <Eig,double>     SPARSE_;
  typedef LdltTag       <Eig,double,1>   DENSE_SCHUR_;
  typedef PcgTag        <Eig,double,1>   SPARSE_SCHUR_;
  typedef ImplPcg       <Eig,double,1>   IMPLICIT_SCHUR_;
  typedef LdltTag       <Eig,double,2>   DENSE_SCHUR2_;
  typedef PcgTag        <Eig,double,2>   SPARSE_SCHUR2_;
  typedef ImplPcg       <Eig,double,2>   IMPLICIT_SCHUR2_;
  
  
  typedef LdltTag       <Eig,double>     EIGEN_DENSE_;
  typedef PcgTag        <Eig,double>     EIGEN_SPARSE_;
  typedef LdltTag       <Eig,double,1>   EIGEN_DENSE_SCHUR_;
  typedef PcgTag        <Eig,double,1>   EIGEN_SPARSE_SCHUR_;
  typedef ImplPcg       <Eig,double,1>   EIGEN_IMPLICIT_SCHUR_;
  typedef LdltTag       <Eig,double,2>   EIGEN_DENSE_SCHUR2_;
  typedef PcgTag        <Eig,double,2>   EIGEN_SPARSE_SCHUR2_;
  typedef ImplPcg       <Eig,double,2>   EIGEN_IMPLICIT_SCHUR2_;

  
  static const DENSE_                     DENSE = {};
  static const SPARSE_                    SPARSE = {};
  static const DENSE_SCHUR_               DENSE_SCHUR = {};
  static const SPARSE_SCHUR_              SPARSE_SCHUR = {};
  static const IMPLICIT_SCHUR_            IMPLICIT_SCHUR = {};
  static const DENSE_SCHUR2_              DENSE_SCHUR2 = {};
  static const SPARSE_SCHUR2_             SPARSE_SCHUR2 = {};
  static const IMPLICIT_SCHUR2_           IMPLICIT_SCHUR2 = {};
  
  static const EIGEN_DENSE_               EIGEN_DENSE = {};
  static const EIGEN_SPARSE_              EIGEN_SPARSE = {};
  static const EIGEN_DENSE_SCHUR_         EIGEN_DENSE_SCHUR = {};
  static const EIGEN_SPARSE_SCHUR_        EIGEN_SPARSE_SCHUR = {};
  static const EIGEN_IMPLICIT_SCHUR_      EIGEN_IMPLICIT_SCHUR = {};
  static const EIGEN_DENSE_SCHUR2_        EIGEN_DENSE_SCHUR2 = {};
  static const EIGEN_SPARSE_SCHUR2_       EIGEN_SPARSE_SCHUR2 = {};
  static const EIGEN_IMPLICIT_SCHUR2_     EIGEN_IMPLICIT_SCHUR2 = {};

#ifdef USE_TOON
  typedef LdltTag       <Toon,double>    TOON_DENSE_;
  typedef PcgTag        <Toon,double>    TOON_SPARSE_;
  typedef LdltTag       <Toon,double,1>  TOON_DENSE_SCHUR_;
  typedef PcgTag        <Toon,double,1>  TOON_SPARSE_SCHUR_;
  typedef ImplPcg       <Toon,double,1>  TOON_IMPLICIT_SCHUR_;
  typedef LdltTag       <Toon,double,2>  TOON_DENSE_SCHUR2_;
  typedef PcgTag        <Toon,double,2>  TOON_SPARSE_SCHUR2_;
  typedef ImplPcg       <Toon,double,2>  TOON_IMPLICIT_SCHUR2_;
  static const TOON_DENSE_               TOON_DENSE;
  static const TOON_SPARSE_              TOON_SPARSE;
  static const TOON_DENSE_SCHUR_         TOON_DENSE_SCHUR;
  static const TOON_SPARSE_SCHUR_        TOON_SPARSE_SCHUR;
  static const TOON_IMPLICIT_SCHUR_      TOON_IMPLICIT_SCHUR;
  static const TOON_DENSE_SCHUR2_        TOON_DENSE_SCHUR2;
  static const TOON_SPARSE_SCHUR2_       TOON_SPARSE_SCHUR2;
  static const TOON_IMPLICIT_SCHUR2_     TOON_IMPLICIT_SCHUR2;
#endif
}

#endif