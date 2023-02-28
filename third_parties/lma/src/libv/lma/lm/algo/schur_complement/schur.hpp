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

#ifndef __OPTIMISATION2_ALGO_LM_MANY_CLASSES_SCHUR_COMPLEMENT_HPP__
#define __OPTIMISATION2_ALGO_LM_MANY_CLASSES_SCHUR_COMPLEMENT_HPP__

#include <libv/lma/ttt/traits/wrap.hpp>
#include <libv/lma/global.hpp>
#include <libv/lma/lm/ba/computing.hpp>
#include <libv/lma/lm/ba/make_type.hpp>
#include <libv/lma/lm/ba/create_hessian.hpp>
#include <libv/lma/lm/omp/omp.hpp>
#include <boost/mpl/advance.hpp>
#include <boost/mpl/remove_if.hpp>
#include <boost/mpl/contains.hpp>
#include <boost/mpl/assert.hpp>
#include <boost/type_traits.hpp>
#include <boost/fusion/include/as_map.hpp>
#include <libv/core/tag.hpp>

namespace lma
{

  template<class VTYPE, class Ap, class H, class P, class Y> struct ProdApPH
  {
    Ap& ap;
    const H& h;
    const P& p;
    const Y& y;
    ProdApPH(Ap& ap_, const H& h_, const P& p_, const Y& y_):ap(ap_),h(h_),p(p_),y(y_){}

    template<class T> void operator()(T){}
    
    template<class Key1,class Key2, class Value, template<class,class> class Pair> void operator()(ttt::wrap<Pair<Pair<Key1,Key2>,Value>>,
      typename boost::disable_if<
          mpl::or_<
                    mpl::not_<br::has_key<H,Pair<Key2,VTYPE>>>,
                    mpl::not_<br::has_key<Y,Pair<Key1,VTYPE>>>
                  >
            >::type * = 0)
    {
      {
        auto& a1 = bf::at_key<Key1>(ap);
        const auto& pr2 = bf::at_key<Key2>(p);
        const auto& y1= bf::at_key<Pair<Key1,VTYPE>>(y);
        const auto& w2 = bf::at_key<Pair<Key2,VTYPE>>(h);
        const auto& v = bf::at_key<Pair<VTYPE,VTYPE>>(h);

        typedef typename boost::remove_reference<decltype(v)>::type V;

        VectorColumn< VTYPE, typename V::MatrixTag > v0;

// 	std::cout << " [1] : " << a1.name() << " = " << w2.name() << " " << pr2.name() << " " << y1.name() << std::endl;
        // v0 = Wt * pr
        prod(v0,w2,pr2);
        // a = - Y * v0 , (Y = W * V)
        prod_minus(a1,y1,v0);

        auto& a2 = bf::at_key<Key2>(ap);
        const auto& pr1 = bf::at_key<Key1>(p);
        const auto& y2= bf::at_key<Pair<Key2,VTYPE>>(y);
        const auto& w1 = bf::at_key<Pair<Key1,VTYPE>>(h);

        typedef typename boost::remove_reference<decltype(v)>::type V;

        VectorColumn< VTYPE, typename V::MatrixTag > v1;

// 	std::cout << " [2] : " << a2.name() << " = " << w1.name() << " " << pr1.name() << " " << y2.name() << std::endl;
        // v1 = Wt * pr
        prod(v1,w1,pr1);
        // a = - Y * v1, (Y = W * V)
        prod_minus(a2,y2,v1);
      }
    }

    //! cas où on est sur la diagonale
    template<class Key1, class Value, template<class,class> class Pair> void operator()(ttt::wrap<Pair<Pair<Key1,Key1>,Value>>,
      typename boost::disable_if<boost::mpl::not_<br::has_key<H,Pair<Key1,VTYPE>>>>::type * = 0)
    {
      const auto& pr = bf::at_key<Key1>(p);
      const auto& u = bf::at_key<Pair<Key1,Key1>>(h);

      auto& a = bf::at_key<Key1>(ap);
      auto& w = bf::at_key<Pair<Key1,VTYPE>>(h);
      auto& y0 = bf::at_key<Pair<Key1,VTYPE>>(y);

      auto& v = bf::at_key<Pair<VTYPE,VTYPE>>(h);

      if (a.size()==0) a.resize(pr.size());
      typedef typename boost::remove_reference<decltype(v)>::type V;
      VectorColumn< VTYPE, typename V::MatrixTag > v0;
      VectorColumn< Key1, typename V::MatrixTag > v1;
      v0.resize(v.size());
      v1.resize(u.size());

//       std::cout << " [0] : " << a.name() << " = " << w.name() << " " << pr.name() << " " << y0.name() << std::endl;
      //v0 = Wt * pr
      prod(v0,w,pr);

      //v1 = Y * v0 , ( Y = W * V )
      prod(v1,y0,v0);

      // a -= u1
//       #pragma omp parallel for if(use_omp())
      for(auto i = w.first() ; i < w.size() ; ++i)
        a(i) -= v1(i);
    }
  };


  template<class VTYPE, class AP, class H, class P, class Y> ProdApPH<VTYPE,AP,H,P,Y> prod_ap_ph(AP& ap, const H& h, const P& p, const Y& y){ return ProdApPH<VTYPE,AP,H,P,Y>(ap,h,p,y); }
  
  
  template<class Float, class I, class List, class J, class Result> struct UnrollW2 : 
    mpl::push_back<
		    Result,
		    typename MakeTupleTable<
					    typename mpl::at<List,I>::type,
					    typename mpl::at<List,J>::type,
					    Float
					  >::type
		  >::type {};
  
  template<class Float, class K, class List, class Int, class Result> struct UnrollW1_ :
    For<K::value,mpl::size<List>::value,List,UnrollW2<Float,Int,mpl::_1,mpl::_2,mpl::_3>,Result> {};

  template<class Float, class A> using UnrollW1 = UnrollW1_<Float,A,mpl::_1,mpl::_2,mpl::_3>;
  
  template<class L, class Int, class Result> struct PushBack_ : 
    mpl::push_back<Result,typename mpl::at<L,Int>::type> {};
    
  typedef PushBack_<mpl::_1,mpl::_2,mpl::_3> PushBack;
  
  template<class BA, class NumericTag> struct ImplicitSchurContainer
  {
    typedef typename BA::Keys Keys;
    typedef typename BA::MatrixTag MatrixTag;
    
    // K_ est le nombre de famille à mettre dans Vs
    // K est la position de Vs
    static const size_t K_ = Size<NumericTag>::value;
    static const size_t N = mpl::size<Keys>::value;
    static const size_t K = N - K_;
    
    static_assert( (N > K) , "Nombre de paramètres dans Schur >= Nombre de paramètre dans le problème");

    typedef typename For<K,N,Keys,PushBack>::type KeyVs;
    typedef typename For<0,K,Keys,PushBack>::type KeyUs;
    
    typedef typename For<0,K,Keys,UnrollW1<MatrixTag,mpl::int_<K>>>::type TypeWs_;
    typedef typename 
                    mpl::remove_if<
                                    TypeWs_,
                                    mpl::not_<
                                              mpl::contains<typename BA::ListeHessien,mpl::_1>
                                            >
                                  >::type TypeWs;
    typedef typename br::as_map<TypeWs>::type TupleWs;
    
    typedef typename mpl::transform< KeyUs, VectorToPairStruct<mpl::_1,MatrixTag> >::type ListResidu;
    typedef typename mpl::transform< KeyVs, VectorToPairStruct<mpl::_1,MatrixTag> >::type ListResiduVs;
    typedef typename br::as_map<ListResidu>::type TupleResiduUs;
    typedef typename br::as_map<ListResiduVs>::type TupleResiduVs;

    typedef typename mpl::transform<KeyVs,ToTable<Single<mpl::_1>,MatrixTag>>::type TypeVs;
    typedef typename br::as_map<TypeVs>::type TupleVs;
  
    typedef typename mpl::transform< KeyUs, pair_>::type DiagUs;
    typedef typename mpl::transform< KeyVs, pair_>::type DiagVs;
    
    ImplicitSchurContainer()
    {
//        std::cout << "Parametres : " << ttt::name<ListeParametre>() << std::endl;
//        std::cout << "Us         : " << ttt::name<Us>() << std::endl;
//        std::cout << "Vs         : " << ttt::name<Vs>() << std::endl;
       //std::cout << "Ws         : " << ttt::name<TypeWs>() << std::endl;
//        std::cout << "ResiduUs   : " << ttt::name<TupleResiduUs>() << std::endl;
//        std::cout << "TypeVs     : " << ttt::name<TypeVs>() << std::endl;
//        std::cout << "TupleVs    : " << ttt::name<TupleVs>() << std::endl;
    }
    
    TupleWs ys;
    TupleResiduUs bs;
    TupleVs save_vs;
  };
}// eon

#endif

