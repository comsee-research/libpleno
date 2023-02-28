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

#ifndef __LMA_OPT2_SOLVER_POLICIES_HPP__
#define __LMA_OPT2_SOLVER_POLICIES_HPP__

#include <libv/lma/lm/algo/levmar.hpp>
#include <libv/lma/lm/ba/pcg.hpp>
#include <libv/lma/lm/ba/ldlt.hpp>
#include <libv/lma/lm/algo/global/global1.hpp>
#include <libv/lma/lm/algo/schur_complement/explicit_schur.hpp>
#include <libv/lma/lm/algo/schur_complement/implicit_schur.hpp>

namespace lma
{
  
 template<class A, class B> using pair = bf::pair<A,B>;
  
 template<class MatrixTag = Eig, class Float=double, size_t I=0> struct LdltTag{};
  
  template<class MatrixTag, class Float, size_t I=0> struct PcgTag : PcgConfig {
    PcgTag(){}
    PcgTag(double seuil_, size_t max_iteration_, size_t f=-1):PcgConfig(seuil_,max_iteration_,f) {}
  };
  
  template<class MatrixTag, class Float, size_t I> struct ImplPcg : PcgConfig {
    ImplPcg(){}
    ImplPcg(double seuil_, size_t max_iteration_, size_t f=-1):PcgConfig(seuil_,max_iteration_,f) {}
  };
  
  template<class Container, size_t nb_class, class Tag> struct SelectAlgo;

  template<class Container, size_t nb_class, class MatrixTag, class Float, size_t K> struct SelectAlgo <Container,nb_class,ImplPcg<MatrixTag,Float,K>> 
    : LevMar<ImplicitSchur<Container,PCG,bf::pair<MatrixTag,Float>,v::numeric_tag<K>>> { };
  
  template<class Container, size_t nb_class, class MatrixTag, class Float, size_t K> struct SelectAlgo <Container,nb_class,LdltTag<MatrixTag,Float,K>> 
    : LevMar<ExplicitSchur<Container,LDLT,bf::pair<MatrixTag,Float>,v::numeric_tag<K>>> { };
  
  template<class Container, size_t nb_class, class MatrixTag, class Float, size_t K> struct SelectAlgo <Container,nb_class,PcgTag<MatrixTag,Float,K>> 
    : LevMar<ExplicitSchur<Container,PCG,bf::pair<MatrixTag,Float>,v::numeric_tag<K>>> { };
  
  template<class Container, size_t nb_class, class MatrixTag, class Float> struct SelectAlgo <Container,nb_class,LdltTag<MatrixTag,Float,0>>
    : LevMar<Global<Container,LDLT,bf::pair<MatrixTag,Float>>> { };
  
  template<class Container, size_t nb_class, class MatrixTag, class Float> struct SelectAlgo <Container,nb_class,PcgTag<MatrixTag,Float,0>>
    : LevMar<Global<Container,PCG,bf::pair<MatrixTag,Float>>> { };
}

#endif