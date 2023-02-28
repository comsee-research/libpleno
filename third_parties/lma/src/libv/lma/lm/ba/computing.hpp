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

#ifndef __OPTIMISATION2_BA_COMPUTING_HPP__
#define __OPTIMISATION2_BA_COMPUTING_HPP__

#include "../container/container.hpp"
#include "container.hpp"

#include "../omp/omp.hpp"
#include <libv/lma/time/tictoc.hpp>

namespace lma {

  template<class A, class T> void prod(VectorColumn<A,T>& a, const Table<A,A,T>& u, const VectorColumn<A,T>& b)
  {
//     std::cout << " prod 1 " << std::endl;
    if (a.size()==0) a.resize(b.size());

    for(auto i = u.first() ; i < u.size() ; ++i)
     for(auto j = u.first(i) ; j < u.size(i) ; ++j)
        a(i) += u(i,j) * b(i);
  }
  
  template<class A, class T> void prod(VectorColumn<A,T>& a, const Table<A,A,T,Diagonal>& u, const VectorColumn<A,T>& b)
  {
//     std::cout << " prod 2 " << std::endl;
    if (a.size()==0) a.resize(b.size());
    for(auto i = u.first() ; i < u.size() ; ++i)
      a(i) += u(i) * b(i);
  }
  
  //! A += AB * B
  template<class A, class B, class T> void prod(VectorColumn<A,T>& vectora, const Table<A,B,T>& table, const VectorColumn<B,T>& vectorb)
  {
//     std::cout << " prod 3 " << std::endl;
    if (vectora.empty()) vectora.resize(table.size());
//     #pragma omp parallel for if(use_omp())
    for(auto i = table.first() ; i < table.size() ; ++i)
      for(auto js = table.first(i) ; js < table.size(i) ; ++js)
        vectora(i) += table(i,js) * vectorb(table.indice(i,js));
  }

  //! A += BAt * B
  template<class A, class B, class T> void prod(VectorColumn<A,T>& vectora, const Table<B,A,T>& table, const VectorColumn<B,T>& vectorb)
  {
//     std::cout << " prod 4 " << std::endl;
    
    if (vectora.empty()) vectora.resize(table.indice.maxb());
//     #pragma omp parallel for if(use_omp())
    for(auto i = table.first() ; i < table.size() ; ++i)
      for(auto j = table.first(i) ; j < table.size(i) ; ++j)
        vectora(table.indice(i,j)) += transpose(table(i,j)) * vectorb(i);
  }

  //! A -= BAt * B
  template<class A, class B, class T> void prod_minus(VectorColumn<A,T>& vectora, const Table<B,A,T>& table, const VectorColumn<B,T>& vectorb)
  {
    if (vectora.empty()) vectora.resize(table.indice.maxb());
//     #pragma omp parallel for if(use_omp())
    for(auto i = table.first() ; i < table.size() ; ++i)
      for(auto j = table.first(i) ; j < table.size(i) ; ++j)
        vectora(table.indice(i,j)) -= transpose(table(i,j)) * vectorb(i);
  }

  //! A -= AB * B
  template<class A, class B, class T> void prod_minus(VectorColumn<A,T>& vectora, const Table<A,B,T>& table, const VectorColumn<B,T>& vectorb)
  {
    if (vectora.empty()) vectora.resize(table.size());
//     #pragma omp parallel for if(use_omp())
    for(auto i = table.first() ; i < table.size() ; ++i)
      for(auto j = table.first(i) ; j < table.size(i) ; ++j)
        vectora(i) -= table(i,j) * vectorb(table.indice(i,j));
  }

  //!   Y< A,B > = W< A,B > * V< B,B >;
  template<class A, class B, class T> void prod(Table<A,B,T>& y_ab, const Table<A,B,T>& t_ab, const Table<B,B,T,Diagonal>& t_bb)
  {
//     std::cout << " prod 5 " << std::endl;
    if (y_ab.size()==0) y_ab.resize(t_ab.indice);
//     #pragma omp parallel for if(use_omp())
    for(auto i = t_ab.first() ; i < t_ab.size() ; ++i)
      for(auto j = t_ab.first(i) ; j < t_ab.size(i) ; ++j)
        y_ab(i,j) += t_ab(i,j) * t_bb(t_ab.indice(i,j));
  }

  //! Sp = S * p
  template<class A, class B, class T> void prod_trig_sup(VectorColumn<A,T>& vectora, const Table<A,B,T>& table, const VectorColumn<B,T>& vectorb)
  {
//     std::cout << " prod_trig_sup " << std::endl;
    if (vectora.size()==0) vectora.resize(table.size());

//     #pragma omp parallel for if(use_omp())
    for(auto i = table.first() ; i < table.size() ; ++i)
      vectora(i) += table(i,table.first(i)) * vectorb(i());

//     #pragma omp parallel for if(use_omp())
    for(auto i = table.first() ; i < table.size() ; ++i)
      for(auto js = table.first(i) + 1 ; js < table.size(i) ; ++js)
      {
        assert(i()<=table.indice(i,js)());// continue;
        vectora(i) += table(i,js) * vectorb(table.indice(i,js));
        vectora(table.indice(i,js)) += transpose(table(i,js)) * vectorb(i());
      }
  }
  
  template<class A, class T> void prod_trig_sup(VectorColumn<A,T>& vectora, const Table<A,A,T,Diagonal>& table, const VectorColumn<A,T>& vectorb)
  {
    if (vectora.size()==0) vectora.resize(table.size());

    for(auto i = table.first() ; i < table.size() ; ++i)
      vectora(i) += table(i) * vectorb(i);
  }

  //! AB += AB2
  template<class A, class B, class T> void plus(Table<A,B,T>& tAB, const Table<A,B,T>& tAB2)
  {
//     #pragma omp parallel for if(use_omp())
    for(auto ia = tAB2.first() ; ia < tAB2.size() ; ++ia)
      for(auto iab = tAB2.first(ia) ; iab < tAB2.size(ia) ; ++iab)
        tAB(ia,iab) += tAB2(ia,iab);
  }

  template<class A, class T> void minus_prod_(VectorColumn<A,T>& a, const Table<A,A,T,Diagonal>& b, const VectorColumn<A,T>& c)
  {
    for(auto i = a.first() ; i < a.size() ; ++i)
      a(i) = b(i)*(c(i) - a(i));
  }
  
  struct A__B_prod_C
  {
    template<class A, class B, class C> void operator()(A& a, const B& b, const C& c) const
    {
      prod(a,b,c);
    }
  };


  //vector<A> -= table<A,B> * vector<B>();
  struct A__minus_B_prod_C
  {
    template<class A, class B, class C> void operator()(A& a, const B& b, const C& c) const
    { 
      prod_minus(a,b,c);
    }
  };
  
  struct A__B_prod_C_minus_A
  {
    template<class A, class B, class C> void operator()(A& a, const B& b, const C& c) const
    { 
      minus_prod_(a,b,c);
    }
  };

  
  struct AABB
  {
    template<class A, class B, class C> void operator()(A& a, const B& b, const C& c) const
    {
//       std::cout << ttt::name<A>() << " = " << ttt::name<B>() << " * " << ttt::name<C>() << std::endl;
      apply(a,b,c);
    }
    
    template<class A, class B, class C> void apply(A& a, const B& b, const C& c) const
    {
      prod(a,b,c);
    }
    
    template<class A, class T, class _> void apply(VectorColumn<A,T>& a, const Table<A,A,T,_>& b, const VectorColumn<A,T>& c) const
    {
      prod_trig_sup(a,b,c);
    }
  };
}// eon

#endif
