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

#ifndef __OPTIMISATION2_BA_COMPUTE_S_HPP__
#define __OPTIMISATION2_BA_COMPUTE_S_HPP__

#include "container.hpp"
#include <libv/lma/time/tictoc.hpp>
#include <libv/lma/lm/omp/omp.hpp>

namespace lma
{

  template<class A, class B, class C, class Bundle, class U, class T>
  void S__U_WY_Init1(Table<A,C,T>& tAC, const Table<A,B,T>& , const Table<C,B,T>& , const Bundle& bundle, const U& u)
  {
//     std::cout << " init 1 " << tAC.name() << std::endl;
    const auto& BA = ttt::at<A,B>(bundle.vab_map);
    const auto& BC = ttt::at<C,B>(bundle.vab_map);

    for(auto i = u.indice.first() ; i < u.indice.size() ; ++i)
      for(auto j = u.indice.first(i) ; j < u.indice.size(i) ; ++j)
        tAC.indice.add(i,u.indice(i,j));

    for(auto ib = BA.first() ; ib < BA.size() ; ++ib)
      for(auto iba = BA.first(ib) ; iba < BA.size(ib) ; ++iba)
      {
        auto a1 = BA(ib,iba);
        for(auto ibc = BC.first(ib) ; ibc < BC.size(ib) ; ++ibc)
          tAC.indice.add(a1,BC(ib,ibc));
      }
  }
  
  template<class A, class B, class C, class Bundle, class Vec, class Vab, class T>
  void S__U_WY_Init2(Table<A,C,T>& tAC, const Table<A,B,T>& , const Table<C,B,T>& , const Bundle& bundle, Vec& vec, const Vab& vab)
  {
//     std::cout << " init 2 AC : " << tAC.name() << std::endl;
    const auto& BA = ttt::at<A,B>(bundle.vab_map);
    const auto& BC = ttt::at<C,B>(bundle.vab_map);

    const auto& BA2 = ttt::at<A,B>(vab);
    const auto& BC2 = ttt::at<C,B>(vab);
    
    for(auto ib = BA.first() ; ib < BA.size() ; ++ib)
      for(auto iba = BA.first(ib) ; iba < BA.size(ib) ; ++iba)
        for(auto ibc = BC.first(ib) ; ibc < BC.size(ib) ; ++ibc)
        {
          auto a1 = BA(ib,iba);
          auto a2 = BC(ib,ibc);
          auto is = tAC.indice.get(a1,a2);
          auto r1 = BA2.reverse(ib,iba);
          auto r2 = BC2.reverse(ib,ibc);
          vec.liste_op.emplace_back(a1(),a2(),r1(),is(),r2());
        }
  }
  
  template<class A, class T>
  void copy_indice(Table<A,A,T,Symetric>& tAC, const Table<A,A,T,Symetric>& u)
  {
    for(auto i = u.first() ; i < u.size() ; ++i)
      for(auto j = u.first(i) ; j < u.size(i) ; ++j)
        if (i <= u.indice(i,j))
          tAC.indice.add(i,u.indice(i,j));
  }
  
  template<class A, class T>
  void copy_indice(Table<A,A,T,Symetric>& tAC, const Table<A,A,T,Diagonal>& u)
  {
    for(auto i = u.first() ; i < u.size() ; ++i)
      tAC.indice.add(i,i);
  }
  
  template<class A, class B, class Bundle, class Flt, class T, class _1, class _2, class _3>
  void S__U_WY_Init1(Table<A,A,T,Symetric>& tAC, const Table<A,B,T,_1>& , const Table<A,B,T,_2>& , const Bundle& bundle, const Table<A,A,Flt,_3>& u)
  {
    const auto& BA  = ttt::at<A,B>(bundle.vab_map);

    copy_indice(tAC,u);
    
    for(auto ib = BA.first() ; ib < BA.size() ; ++ib)
      for(auto iba = BA.first(ib) ; iba < BA.size(ib) ; ++iba)
      {
        auto a1 = BA(ib,iba);
        for(auto ibc = iba ; ibc < BA.size(ib) ; ++ibc)
        {
          tAC.indice.add(a1,BA(ib,ibc));
        }
      }
  }
  
  template<class A, class B, class Bundle, class Vec, class Vab, class T, class _1, class _2>
  void S__U_WY_Init2(Table<A,A,T,Symetric>& tAC, const Table<A,B,T,_1>& , const Table<A,B,T,_2>& , const Bundle& bundle, Vec& vec, const Vab& vab)
  {
//     std::cout << " init 2 AA : " << tAC.name() << std::endl;
    const auto& BA  = ttt::at<A,B>(bundle.vab_map);
    const auto& RBA = ttt::at<A,B>(vab);

    std::vector<std::map<Indice<A>,std::vector<bf::vector<int,int,int,int,int>>>> v(tAC.size()());
    
    for(auto ib = BA.first() ; ib < BA.size() ; ++ib)
      for(auto iba = BA.first(ib) ; iba < BA.size(ib) ; ++iba)
      {
        auto r1 = RBA.reverse(ib,iba);
        auto a1 = BA(ib,iba);
        for(auto ibc = iba ; ibc < BA.size(ib) ; ++ibc)
        {
          auto a2 = BA(ib,ibc);
          auto is = tAC.indice.get(a1,a2);
          auto r2 = RBA.reverse(ib,ibc);
// //           vec.liste_op.emplace_back(a1(),a2(),r1(),is(),r2());
          v[a1()][a2].emplace_back(a1(),a2(),r1(),is(),r2());
          assert( a1() <= a2() );
        }
      }
      
      for(auto& x : v)
        for(auto& q : x)
          for(auto& p : q.second)
            vec.liste_op.push_back(p);
  }
  
  template<class A, class B, class C, class Vec>
  void S__U_WY(A& AC, const B& AB, const C& CB, const Vec& vec)
  {
//     if(use_omp())
//     {
//       #pragma omp parallel for
//       for(size_t i = 0 ; i < vec.liste_op.size() ; ++i)
//       {
//         auto& x = vec.liste_op[i];
//         AC(bf::at_c<0>(x),bf::at_c<3>(x)) -= AB(bf::at_c<0>(x),bf::at_c<2>(x))*CB(bf::at_c<1>(x),bf::at_c<4>(x)).transpose();
//       }
//     }
//     else
//     {
      for(auto& x : vec.liste_op)
        AC(bf::at_c<0>(x),bf::at_c<3>(x)) -= AB(bf::at_c<0>(x),bf::at_c<2>(x))*transpose(CB(bf::at_c<1>(x),bf::at_c<4>(x)));
//     }
//     auto ia  = bf::at_c<0>(x);auto ic  = bf::at_c<1>(x);auto iab = bf::at_c<2>(x);auto iac = bf::at_c<3>(x);auto icb = bf::at_c<4>(x);
//     AC(ia,iac) -= AB(ia,iab)*CB(ic,icb).transpose();
  }
  
  template<class Tie> struct AA_INIT_S_1_AA
  {
    Tie tie;
    AA_INIT_S_1_AA(Tie tie_):tie(tie_){}
    
    template<class A, class B, class C> void operator()(A& a, const B& b, const C& c)
    {
      auto& u = bf::at_key<bf::pair<typename A::Id1,typename A::Id2>>(std::get<1>(tie));
      S__U_WY_Init1(a,b,c,std::get<0>(tie),u);
    }
  };

  template<class Tie> struct AA_INIT_S_2_AA
  {
    Tie tie;
    AA_INIT_S_2_AA(Tie tie_):tie(tie_){}
    
    template<class A, class B, class C> void operator()(A& a, const B& b, const C& c)
    {
      auto& vec = bf::at_key<bf::pair<typename A::Id1,typename A::Id2>>(bf::at_key<typename B::Id2>(std::get<0>(tie)));
      S__U_WY_Init2(a,b,c,std::get<2>(tie),vec,std::get<1>(tie));
    }
  };
  
  template<class V> struct AA_COMPUTE_S_AA
  {
    const V& v;
    AA_COMPUTE_S_AA(const V& v_):v(v_){}
    template<class A, class B, class C> void operator()(A& a, const B& b, const C& c) const
    {S__U_WY(a,b,c,bf::at_key<bf::pair<typename A::Id1,typename A::Id2>>(bf::at_key<typename B::Id2>(v)));}
  };

}// eon

#endif
