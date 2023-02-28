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

#ifndef __OPTIMISATION2_ALGO_LM_MANY_CLASSES_SCHUR_COMPLEMENT_SCHUR_COMPLEMENT5_HPP__
#define __OPTIMISATION2_ALGO_LM_MANY_CLASSES_SCHUR_COMPLEMENT_SCHUR_COMPLEMENT5_HPP__

#include "../levmar.hpp"
#include <libv/lma/lm/ba/initialize.hpp>
#include <libv/lma/lm/ba/compute_s.hpp>
#include <libv/lma/lm/bundle/make_sparse_indice.hpp>
#include "schur.hpp"
#include "implicit_schur.hpp"
#include <boost/fusion/include/pair.hpp>
#include <libv/lma/lm/ba/tuple_to_mat.hpp>

namespace lma
{
  template<class BA, class ExplictSCHUR, class ImplicitSCHUR> struct BABYS
  {
    const BA& ba;
    const ExplictSCHUR& eschur;
    const ImplicitSCHUR& ischur;

    typedef typename ExplictSCHUR::TypeS Hessian;
    typedef typename ExplictSCHUR::OptimizeKeys OptimizeKeys;

    const typename ExplictSCHUR::TupleS& A() const { return eschur.s; }
    const typename ImplicitSCHUR::TupleResiduUs& B() const { return ischur.bs; }

    BABYS(const BA& ba_, const ExplictSCHUR& schur_, const ImplicitSCHUR& ischur_):ba(ba_),eschur(schur_),ischur(ischur_){}

    template<class AP, class P> void prodAP(AP& ap, const P& p) const
    {
//       mpl::for_each<Hessian,ttt::wrap_>(prod_ap_p(ap,eschur.s,p));
//       typedef typename MetaProd<typename TrigSup<Hessian>::type,typename ImplicitSCHUR::TupleResiduUs>::type keys;
//       std::cout << std::endl << ttt::name<keys>() << std::endl;
//       for_each<keys>(std::tie(ap,eschur.s,p),AABB());

      for_each<MetaProd<typename AddTranspose<Hessian>::type,typename ImplicitSCHUR::TupleResiduUs>>(std::tie(ap,eschur.s,p),AABB());
    }
  };

  template<class VAB> struct CopyReverseIndice
  {
    const VAB& vab;
    CopyReverseIndice(const VAB& vab_):vab(vab_){}

    template<template<class,class> class Pair, class Key, class Value> void operator()(Pair<Key,Value>& pair) const
    {
      auto& sics = pair.second;
      bf::for_each(sics,CopyReverseIndice<VAB>(vab));
    }

    template<template<class,class> class Pair, class Key, class A, class B> void operator()(Pair<Key,SIC2<A,B>>& pair) const
    {
      auto& sics_ab = pair.second;
      const auto& sic_ba = bf::at_key<B>(bf::at_key<A>(vab));

     // trop dur de savoir ce que ca fait et si c'est vraiment utile...
      for(auto i = sic_ba.first() ; i < sic_ba.size() ; ++i)
        for(auto j = sic_ba.first(i) ; j < sic_ba.size(i) ; ++j)
          sics_ab.add(sic_ba(i,j),i,j);
    }
  };

  template<class BA, class NumericTag, class KeyUs> struct SchurExplicit
  {
    typedef typename BA::Keys Keys;

    // K_ est le nombre de famille à mettre dans KeyUs
    static const size_t K = mpl::size<Keys>::value - Size<NumericTag>::value;
    
    typedef typename For<K,mpl::size<Keys>::value,Keys,PushBack>::type KeyVs;
    
//     typedef typename MakeH<typename BA::Float,KeyUs>::type TypeS;
    typedef typename ListS<typename BA::ListeHessien,KeyUs>::type TypeS;
    
    typedef typename br::as_map< TypeS >::type TupleS;

    struct Vec { std::vector< bf::vector<int,int,int,int,int> > liste_op; };

    typedef typename br::as_map< 
				 typename mpl::transform<
							  typename mpl::transform<TypeS,br::first<mpl::_1>>::type,
							  bf::pair<mpl::_1,Vec>
							>::type
			       >::type PrecalcVector;

    typedef KeyUs OptimizeKeys;
    typedef typename MakeSparseIndiceContainer2<Keys>::type VABMap;
    
    // tuple de vector des indices pré-calculés pour le calcul de S.
    
    typedef typename br::as_map<typename mpl::transform<KeyVs,bf::pair<mpl::_1,PrecalcVector>>::type>::type TUPLEVECTORINDICE;
    TUPLEVECTORINDICE tuple_v;
    VABMap svab;
    TupleS s;

    const TupleS& A() const { return s; }
    
    SchurExplicit()
    {
//       std::cout << " schur : " << ttt::name<TypeS>() << std::endl;
    }

    template<class Indices> void init(const Indices& vab)
    {
      bf::for_each(svab,CopyReverseIndice<Indices>(vab));
    }
  };

  struct UpdateIndice
  {
    template<class Pair> void operator()(Pair& pair) const
    {
      pair.second.indice.update();
      pair.second.resize(pair.second.indice);
    }
  };

  template<class Bdl, class NormEq, class MatrixTag_, class K> class ExplicitSchur : ImplicitSchur<Bdl,NormEq,MatrixTag_,K>
  {
    public:
    typedef MatrixTag_ MatrixTag;
    typedef ImplicitSchur<Bdl,NormEq,MatrixTag_,K> parent;
    typedef Bdl Bundle;
    typedef utils::Tic<false> Tic;


    typedef Bas<Bundle,MatrixTag> Ba;
    typedef typename Ba::Hessian Hessian;
    typedef typename Ba::Keys Keys;
    typedef typename parent::SchurCont ParentSchur;
    typedef SchurExplicit<Ba,K,typename ParentSchur::KeyUs> SchurCont;
    
    typedef typename ParentSchur::TypeVs TypeVs;
    typedef typename ParentSchur::TypeWs TypeWs;
    typedef typename ParentSchur::ListResidu ResiduUs;
    
    SchurCont schur_;

    typedef typename SchurCont::KeyVs KeyVs;

    template<class Config> ExplicitSchur(Config config):parent(config){}

    void init(Bundle& bundle_, Ba& ba_)
    {
      parent::init(bundle_,ba_);
      bf::for_each(schur_.s,detail::ResizeInterInit<Bdl>(bundle_));
      schur_.init(bundle_.vab_map);
      auto tie1 = std::tie(bundle_, ba_.h);
      for_each<MetaProd<TypeWs,Transpose<TypeWs>>>(std::tie(schur_.s,parent::schur_.ys,ba_.h),AA_INIT_S_1_AA<decltype(tie1)>(tie1));
      
      bf::for_each(schur_.s,UpdateIndice());
      /*bf::for_each(schur_.s,[](auto& pair)
      {
        pair.second.indice.update();
        pair.second.resize(pair.second.indice);
      });*/
      auto tie2 = std::tie(schur_.tuple_v,schur_.svab, bundle_);
      for_each<MetaProd<TypeWs,Transpose<TypeWs>>>(std::tie(schur_.s,parent::schur_.ys,ba_.h),AA_INIT_S_2_AA<decltype(tie2)>(tie2));
    }

    void init_zero()
    {
      parent::init_zero();
      bf::for_each(schur_.s,detail::SetZero());//! facultatif ?
    }

    using parent::save_h;
    using parent::reload_h;
    using parent::get_vs;

    void solve(Ba& ba_, Bundle& bundle_)
    {
      parent::inv_v(ba_.h);
      parent::compute_y(ba_);
      parent::compute_b(ba_);
      compute_s(ba_,bundle_);
      compute_delta_a(ba_);
      parent::compute_delta_b(ba_);
    }

    void compute_s(Ba& ba_, Bundle& )
    {
      Tic tic("Compute S");
      bf::for_each(schur_.s,AssignSame<Hessian>(ba_.h));
//       bf::for_each(schur_.s,[&](auto& pair)     
//       {
//         std::cout << " Assign " << pair.second.name() << std::endl;
//         AssignSame<Hessian>(ba_.h)(pair);
//         
// //         auto a = to_mat<typename SchurCont::OptimizeKeys,typename SchurCont::TypeS>(schur_.s,size_tuple<mpl::size<typename SchurCont::OptimizeKeys>::value>(ba_.delta));
//         
// //         std::cout << " S = U \n " << a << std::endl;
//       }
//       );
      
//       typedef typename SchurCont::OptimizeKeys aa;
//       typedef typename SchurCont::TypeS bb;
//       auto a = to_mat<typename SchurCont::OptimizeKeys,typename SchurCont::TypeS>(schur_.s,size_tuple<mpl::size<typename SchurCont::OptimizeKeys>::value>(ba_.delta));
      
//       std::cout << " S = U \n " << a << std::endl;
      // S = Y * W
      for_each<MetaProd<TypeWs,Transpose<TypeWs>> >(std::tie(schur_.s,parent::schur_.ys,ba_.h),AA_COMPUTE_S_AA<typename SchurCont::TUPLEVECTORINDICE>(schur_.tuple_v));

//       a = to_mat<typename SchurCont::OptimizeKeys,typename SchurCont::TypeS>(schur_.s,size_tuple<mpl::size<typename SchurCont::OptimizeKeys>::value>(ba_.delta));
      
//       std::cout << " S -= YW\n " << a << std::endl;

      tic.disp();
    }

    void compute_delta_a(Ba& ba_)
    {
      Tic tic("Compute DA");
      parent::norm_eq(BABYS<Ba,SchurCont,ParentSchur>(ba_,schur_,parent::schur_),ba_.delta,MatrixTag());
      tic.disp();
    }
  };
}// eon

namespace ttt
{
  template<class A, class B, class C, int K> struct Name<lma::ExplicitSchur<A,B,C,v::numeric_tag<K>>> { static std::string name(){ return "ExplicitSchur<" + ttt::name<B>() + "," + ttt::name<C>() + "," + lma::to_string(K) + ">"; } };
}

#endif
