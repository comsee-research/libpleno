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

#ifndef __OPTIMISATION2_ALGO_LM_MANY_CLASSES_GLOBAL_GLOBAL1_HPP__
#define __OPTIMISATION2_ALGO_LM_MANY_CLASSES_GLOBAL_GLOBAL1_HPP__

#include "../levmar.hpp"
#include <libv/lma/lm/ba/isdiagonal1f.hpp>
#include <libv/lma/lm/ba/meta_prod.hpp>

namespace lma
{


  
  template<class BA> struct HDJ
  {
    const BA& ba;
    HDJ(const BA& ba_):ba(ba_){}

    typedef typename BA::Hessian Hessian;
    typedef typename BA::Vectors Vectors;
    
    const Hessian& A() const { return ba.h; }
    const Vectors& B() const { return ba.jte; }

    typedef typename BA::Keys OptimizeKeys;
    
    template<class AP, class P> void prodAP(AP& ap, const P& p) const
    {
      for_each<MetaProd<typename AddTranspose<typename BA::ListeHessien>::type,Vectors>>(std::tie(ap,ba.h,p),AABB());
    }
  };

  template<class BA> struct IsDiagonal1F<HDJ<BA>>
  {
    static const bool value = br::size<typename BA::Keys>::value == 1;
  };

  template<class Bdl, class NormEq, class T> class Global
  {
    public:
      typedef Bdl Bundle;
      typedef T MatrixTag;
      typedef Bas<Bundle,MatrixTag> Ba;
      typedef typename Ba::Hessian Hessian;
      NormEq norm_eq;
      
    public:

      typedef typename Ba::Keys Keys;

      template<class Config>
      Global(Config config):norm_eq(config){}

      void init(Bundle& , Ba& ){}

      void init_zero(){}

      void solve(Ba& ba_, Bundle&)
      {
        norm_eq(HDJ<Ba>(ba_),ba_.delta,MatrixTag());
      }

      void save_h(Hessian&){}

      void reload_h(Hessian& h, double prev_lambda)
      {
        //! H - eye(H)*lambda (permet de ne pas sauvegarder entièrement H)
        for_each<MetaUnary<typename Ba::ListeDiag>>(std::tie(h),LambdaDiag(prev_lambda));
      }
  };

}// eon

namespace ttt
{
  template<class A, class B, class C> struct Name<lma::Global<A,B,C>> { static std::string name(){ return "Global<" /*+ ttt::name<A>() + ","*/ + ttt::name<B>() + "," + ttt::name<C>() + ">"; } };
}
#endif
