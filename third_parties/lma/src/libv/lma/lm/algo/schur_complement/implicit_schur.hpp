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

#ifndef __OPTIMISATION2_ALGO_LM_MANY_CLASSES_SCHUR_COMPLEMENT_IMPLICIT_SCHUR2_HPP__
#define __OPTIMISATION2_ALGO_LM_MANY_CLASSES_SCHUR_COMPLEMENT_IMPLICIT_SCHUR2_HPP__

#include "../levmar.hpp"
#include <libv/lma/lm/ba/initialize.hpp>
#include "schur.hpp"

#include <boost/mpl/remove_if.hpp>

namespace lma
{
  template<class TypeS, class AP, class H, class P, class Ys> struct PRODAPPHS
  {
    AP& ap;
    const H& h;
    const P& p;
    const Ys& ys;
    
    PRODAPPHS(AP& ap_, const H& h_, const P& p_, const Ys& ys_):ap(ap_),h(h_),p(p_),ys(ys_){}
    
    template<class V> void operator()(ttt::wrap<V>)
    {
      mpl::for_each<TypeS,ttt::wrap<mpl::_1>>(prod_ap_ph<V>(ap,h,p,ys));
    }
  };

  
  template<class BA, class SCHUR> struct BABY2
  {
    typedef typename SCHUR::KeyUs OptimizeKeys;
    typedef typename ListS<typename BA::ListeHessien,typename SCHUR::KeyUs>::type TypeS;// TODO en fait, on a besoin que des clés, et pas de la map complète avec les containers
    
    typedef typename SCHUR::TupleResiduUs TupleResiduUs;
    typedef typename SCHUR::TupleResiduVs TupleResiduVs;
    const BA& ba_;
    const SCHUR& schur_;

    const typename BA::Hessian& A() const { return ba_.h; }
    const TupleResiduUs& B() const { return schur_.bs; }

    BABY2(const BA& ba__, const SCHUR& schur__):ba_(ba__),schur_(schur__){}

    template<class AP, class P> void prodAP(AP& ap, const P& p) const
    {
      for_each<MetaProd<typename AddTranspose<TypeS>::type,TupleResiduUs>>(std::tie(ap,ba_.h,p),AABB());

      mpl::for_each<typename SCHUR::KeyVs,ttt::wrap<mpl::_1>>(PRODAPPHS<TypeS,AP,typename BA::Hessian,P,typename SCHUR::TupleWs>(ap,ba_.h,p,schur_.ys)); // S -= YWT * p
    }
  };


// ######################################################################################
// ######################################################################################

  template<class BA,class K> void set_zero(ImplicitSchurContainer<BA,K>& schur)
  {
    bf::for_each(schur.ys,detail::SetZero());
    bf::for_each(schur.bs,detail::SetZero());
  }

  template<class BDL, class BA, class K> void initialize(const BDL& bundle, ImplicitSchurContainer<BA,K>& schur)
  {
    bf::for_each(schur.ys,detail::ResizeInter<BDL>(bundle));
  }

  
  
  template<class Bdl, class NormEq, class MatrixTag_, class K> struct ImplicitSchur
  {
    typedef MatrixTag_ MatrixTag;
    typedef Bdl Bundle;
    typedef utils::Tic<false> Tic;
    typedef Bas<Bundle,MatrixTag> Ba;
    typedef typename Ba::Hessian Hessian;

    typedef ImplicitSchurContainer<Ba,K> SchurCont;
    SchurCont schur_;

    typedef typename SchurCont::TypeVs TypeVs;
    typedef typename SchurCont::TypeWs TypeWs;
    typedef typename SchurCont::ListResidu ResiduUs;
    typedef typename SchurCont::ListResiduVs ResiduVs;
    NormEq norm_eq;
    
    template<class Config>
    ImplicitSchur(Config config):norm_eq(config) {}

	  const typename SchurCont::TupleVs& get_vs() const { return schur_.save_vs;}
	
    void init(Bundle& bundle_, Ba&)
    {
      initialize(bundle_,schur_);
    }

    void init_zero()
    {
      set_zero(schur_);
    }

    void save_h(const Hessian& h)
    {
      bf::for_each(schur_.save_vs,AssignSame<Hessian>(h));
    }

    void reload_h(Hessian& h, double prev_lambda)
    {
      for_each<MetaUnary<typename SchurCont::DiagUs>>(std::tie(h),LambdaDiag(prev_lambda));
      for_each<MetaBinary<typename SchurCont::DiagVs>>(std::tie(h,schur_.save_vs),OpAssignSame());
    }

    void solve(Ba& ba_, const Bundle&)
    {
      inv_v(ba_.h);
      compute_y(ba_);
      compute_b(ba_);
      compute_delta_a(ba_);
      compute_delta_b(ba_);
    }

    void inv_v(Hessian& h)
    {
      Tic tic("Inv V");
      for_each<MetaUnary<typename SchurCont::DiagVs>>(std::tie(h),Inv());
      tic.disp();
    }

    void compute_y(const Ba& ba_)
    {
      Tic tic("Compute Y");
      // Y = W * V
      for_each<MetaProd<TypeWs,TypeVs>>(std::tie(schur_.ys,ba_.h,ba_.h),A__B_prod_C());
      tic.disp();
    }

    void compute_b(Ba& ba_)
    {
      Tic tic("Compute B");
      //b = ea - y * eb;
      schur_.bs = ba_.jte;
      for_each<MetaProd<TypeWs,ResiduVs>>(std::tie(schur_.bs,schur_.ys,ba_.jte),A__minus_B_prod_C());
      tic.disp();
    }

    void compute_delta_a(Ba& ba_)
    {
      Tic tic("Compute DA");
      norm_eq(BABY2<Ba,SchurCont>(ba_,schur_),ba_.delta,MatrixTag());
      tic.disp();
    }

    void compute_delta_b(Ba& ba_)
    {
      Tic tic("Compute DB");
// -------------------------------------------- //
//       delta_b = V * (eb - Wt * delta_a)      //
// -------------------------------------------- //
//    delta_b = Wt * delta_a
      for_each<MetaProd<Transpose<TypeWs>,ResiduUs>>(std::tie(ba_.delta,ba_.h,ba_.delta),A__B_prod_C());
      
//    delta_b = V * (eb - delta_b)      
      for_each<MetaProd<TypeVs,ResiduVs>>(std::tie(ba_.delta,ba_.h,ba_.jte),A__B_prod_C_minus_A());
      tic.disp();
    }
  };

   
}// eon

namespace ttt
{
  template<class A, class B, class C, int I> struct Name<lma::ImplicitSchur<A,B,C,v::numeric_tag<I>>> { static std::string name(){ return "ImplicitSchur<" + ttt::name<B>() + "," + ttt::name<C>() + "," + lma::to_string(I) + ">"; } };
}

#endif
