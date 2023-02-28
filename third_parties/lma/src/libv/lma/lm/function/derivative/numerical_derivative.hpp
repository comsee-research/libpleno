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

#ifndef __OPTIMISATION2_FUNCTION_DERIVATIVE_NUMERICAL_DERIVATIVE_HPP__
#define __OPTIMISATION2_FUNCTION_DERIVATIVE_NUMERICAL_DERIVATIVE_HPP__

#include "detail_derivative.hpp"
#include <libv/lma/ttt/traits/unroll1.hpp>
#include <boost/fusion/include/vector.hpp>
#include <boost/fusion/include/vector_tie.hpp>
#include <libv/lma/lm/trait/accessor.hpp>

template<class T >
struct Container;

namespace lma
{
  //! calcul de dérivée numérique avec déroulage static sur les blocks, puis sur les paramètres de chaque block, puis sur chaque résidu

    
  namespace detail
  {
    template<class Float, std::size_t I, std::size_t Fin> struct TupleDerivator;
    template<class Float, std::size_t Fin> struct TupleDerivator<Float,Fin,Fin>
    {
      template<class A, class B, class C, class D> static void compute(const A&, const B&, const C&, const D&){}
      template<class A, class B, class C> static void compute(const A&, const B&, const C&){}
    };

    template<class Float, std::size_t K, size_t I, std::size_t F> struct TupleDerivatorInternal;
    template<class Float, std::size_t K, size_t F> struct TupleDerivatorInternal<Float,K,F,F>
    {
      template<class Fonction, class Result, class Tuple, class R1> static void compute(const Fonction&, Result&, Tuple&, const R1&){}
      template<class Fonction, class Result, class Tuple> static void compute(const Fonction&, Result&, Tuple&){}
    };

    template<class Float, size_t I, class TIE> struct FinalUnroll
    {
      TIE& tie;
      FinalUnroll(TIE& tie_):tie(tie_){}
      
      template<size_t K, class A, class D> inline void compute(A& a, const Float& b, const Float& c, const D& d) const
      {
      	static_assert( K==0, "K==0" );
      	std::get<K,I>(a) = ( b - c ) * d;
        //a(K,I) = ( b - c ) * d;
      }

      template<size_t K, class A, class B, class C, class D> inline void compute(A& a, const B& b, const C& c, const D& d) const
      {
	       //a(K,I) = ( std::get<K>(b) - std::get<K>(c) ) * d;
         std::get<K,I>(a) = ( std::get<K>(b) - std::get<K>(c) ) * d;
      }
      
      template<size_t K> inline void operator()(Int<K> const &) const
      {
	       compute<K>(bf::at_c<0>(tie),bf::at_c<1>(tie),bf::at_c<2>(tie),bf::at_c<3>(tie));
      }
    };

    template<class Float, std::size_t K, size_t I, std::size_t Fin> struct TupleDerivatorInternal
    {
      // forward derivative
      template<class Fonction, class Result, class Tuple, class R> static void compute(const Fonction& fonction, Result& result, Tuple& tuple, const R& r1)
      {
        static const Float h = Float(2.0)*std::sqrt( std::numeric_limits<Float>::epsilon() );
        static const Float _h = Float(1.0) / h ;
        //! to_ref : on crée une référence vers l'objet contenu dans le tuple (uniquement pour simplifier l'écriture)
        //! -> to_ref renvoie T& que l'objet contenu soit T ou T*
        //! at -> renvoie at_c<I>(tuple) ou at_c<I>(map).second

        auto& ref_objet = ttt::to_ref(bf::at_c<K>(tuple));
        auto& jacob = bf::at_c<K>(result).second;

        BOOST_MPL_ASSERT((boost::is_reference<decltype(ref_objet)>));
        assert( (&ref_objet == &ttt::to_ref(bf::at_c<K>(tuple))) );

        auto backup = back_up<I>(ref_objet);
        detail::internal_apply_small_increment(ref_objet,h,v::numeric_tag<I>());
        typedef typename Fonction::ErreurType Residu;
        Residu r2;
        bool b2 = fonction(tuple,r2);
        backup.restore();

        if(b2) // functor evaluation didn't failed
        {
          auto tie = bf::vector_tie(jacob,r2,r1,_h);
          ttt::unroll<0,Size<R>::value>(FinalUnroll<Float,I,decltype(tie)>(tie));
          TupleDerivatorInternal<Float,K,I+1,Fin>::template compute(fonction,result,tuple,r1);
          return;
        }

        set_zero(jacob);
      }
      
      // central derivative
      template<class Fonction, class Result, class Tuple> static void compute(const Fonction& fonction, Result& result, Tuple& tuple)
      {
        static const Float h = Float(2.0)*std::sqrt( std::numeric_limits<Float>::epsilon() );
        static const Float _h = Float(1.0) / (2.0*h) ;
        //! to_ref : on crée une référence vers l'objet contenu dans le tuple (uniquement pour simplifier l'écriture)
        //! -> to_ref renvoie T& que l'objet contenu soit T ou T*
        //! at -> renvoie at_c<I>(tuple) ou at_c<I>(map).second

        auto& ref_objet = ttt::to_ref(bf::at_c<K>(tuple));
        auto& jacob = bf::at_c<K>(result).second;

        BOOST_MPL_ASSERT((boost::is_reference<decltype(ref_objet)>));
        assert( (&ref_objet == &ttt::to_ref(bf::at_c<K>(tuple))) );

        typedef typename Fonction::ErreurType Residu;
        Residu r2;
        
        auto backup = back_up<I>(ref_objet);
        detail::internal_apply_small_increment(ref_objet,h,v::numeric_tag<I>());
        bool b2 = fonction(tuple,r2);
        backup.restore();
        if (b2)
        {
          detail::internal_apply_small_increment(ref_objet,-h,v::numeric_tag<I>());
          Residu r1;
          bool b1 = fonction(tuple,r1);
          backup.restore();
          
          if(b1)
          {
            auto tie = bf::vector_tie(jacob,r2,r1,_h);
            ttt::unroll<0,Size<Residu>::value>(FinalUnroll<Float,I,decltype(tie)>(tie));
            TupleDerivatorInternal<Float,K,I+1,Fin>::template compute(fonction,result,tuple);
            return;
          }
        }
        set_zero(jacob);
      }
    };

    template<class Float, std::size_t I, std::size_t Fin> struct TupleDerivator
    {
      // numerical central derivative
      template<class Fonction, class Result, class Tuple> static void compute(const Fonction& fonction, Result& result, Tuple& tuple)
      {
        static const size_t F = Size<decltype(ttt::to_ref(bf::at_c<I>(tuple)))>::value;
        TupleDerivatorInternal<Float,I,0,F>::template compute(fonction,result,tuple);
        TupleDerivator<Float,I+1,Fin>::template compute(fonction,result,tuple);
      }

      // numerical foward derivative
      template<class Fonction, class Result, class Tuple, class R1> static void compute(const Fonction& fonction, Result& result, Tuple& tuple, const R1& r1)
      {
        static const size_t F = Size<decltype(ttt::to_ref(bf::at_c<I>(tuple)))>::value;
        TupleDerivatorInternal<Float,I,0,F>::template compute(fonction,result,tuple,r1);
        TupleDerivator<Float,I+1,Fin>::template compute(fonction,result,tuple,r1);
      }
    };
  }// eon detail


  template<class Tag> struct NumericalDerivator
  {
    //! le tuple en entrée ne doit pas être constant car il sera modifié, et remis à l'état d'origine
    template<class Fonctor, class Tuple, class Jacob>
    static void derive(const Function<Fonctor>& fonctor, Tuple tuple, Jacob& result)
    {
      detail::TupleDerivator<typename Tag::second_type, 0,br::size<Jacob>::value>::template compute(fonctor,result,tuple);
    }

    template<class Fonctor, class Tuple, class Jacob, class Residual>
    static void derive(const Function<Fonctor>& fonctor, Tuple tuple, Jacob& result, const Residual& residual)
    {
      detail::TupleDerivator<typename Tag::second_type, 0,br::size<Jacob>::value>::template compute(fonctor,result,tuple,residual);
    }
  };

}// eon

namespace ttt
{
  template<class T> struct Name<lma::NumericalDerivator<T>> { static std::string name(){ return "NumericalDerivator<" + ttt::name<T>() + ">"; } };
}

#endif

