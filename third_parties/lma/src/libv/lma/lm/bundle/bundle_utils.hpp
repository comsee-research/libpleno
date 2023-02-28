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

#ifndef __OPTIMISATION2_BUNDLE_BUNDLE_UTILS_HPP__
#define __OPTIMISATION2_BUNDLE_BUNDLE_UTILS_HPP__

#include "../function/function.hpp"
#include <libv/lma/color/console.hpp>

#include <boost/type_traits/remove_pointer.hpp>
#include <boost/fusion/mpl.hpp>
#include <boost/fusion/include/find.hpp>
#include <boost/fusion/include/for_each.hpp>
#include <boost/fusion/include/pop_back.hpp>
#include <boost/fusion/include/make_tuple.hpp>
#include <boost/fusion/include/io.hpp>
#include <boost/mpl/vector.hpp>
#include <boost/fusion/include/equal_to.hpp>
#include <boost/fusion/include/as_vector.hpp>
#include "make_sparse_indice.hpp"
#include "../container/map_container.hpp"
#include <libv/lma/ttt/fusion/copy_f.hpp>
#include <libv/lma/ttt/mpl/naming.hpp>

#include <libv/lma/ttt/traits/unroll1.hpp>
#include <tuple>

namespace lma
{
  
  template<class ID, class Scalar> ttt::Indice<ID> to_indice(const Scalar& value){ return ttt::Indice<ID>(value); }

  namespace detail
  {
    template<class T> struct FonctorPairTic { typedef bf::pair<T,TIC<T,T>> type;};

    template<int K, class OBS, int I, int F> struct AddWithAll
    {

      template<class Key, class Bdl>
      static void update(Bdl& bundle, const ttt::Indice<Key>& i, const ttt::Indice<Key>& j)
      {
        //! bundle.sparse_indice est utilisé uniquement pour remplir la hessienne : hessien() "
        assert(i < j && "solver.add(F(), A a0, A a1,...) -> &a0 < &a1 , essayer solver.add(F(),&a0,&a1,...) au lieu de solver.add(F(),&a1,&a0,...)");
        bundle.template indices<Key,Key>().add(i,j);
      }
      
      template<class Key1, class Key2, class Bdl>
      static void update(Bdl& bundle, const ttt::Indice<Key1>& i, const ttt::Indice<Key2>& j)
      {
        //! bundle.sparse_indice est utilisé uniquement pour remplir la hessienne : hessien() "
        bundle.template indices<Key2,Key1>().add(j,i);
//         bf::at_c<I>(bf::at_c<K>(bundle.spi2.template at<OBS>().back().second)) = 
        bundle.template indices<Key1,Key2>().add(i,j);
      }

      template<class Bdl, class Indice, class Tuple>
      static void add(Bdl& bundle, const Indice& indice, const Tuple& tuple)
      {
        update(bundle,indice,boost::fusion::at_c<I>(tuple));
        AddWithAll<K,OBS,I+1,F>::add(bundle,indice,tuple);
      }
      
      
      template<class Key, class Bdl>
      static void update2(Bdl& bundle, const ttt::Indice<Key>& i, const ttt::Indice<Key>& j)
      {
        //! bundle.sparse_indice est utilisé uniquement pour remplir la hessienne : hessien() "
          bf::at_c<I>(bf::at_c<K>(bundle.spi2.template at<OBS>().back().second)) =  bundle.template indices<Key,Key>().get(i,j);
      }
      
      template<class Key1, class Key2, class Bdl>
      static void update2(Bdl& bundle, const ttt::Indice<Key1>& i, const ttt::Indice<Key2>& j)
      {
        //! bundle.sparse_indice est utilisé uniquement pour remplir la hessienne : hessien() "
        bf::at_c<I>(bf::at_c<K>(bundle.spi2.template at<OBS>().back().second)) = bundle.template indices<Key1,Key2>().get(i,j);
      }

      template<class Bdl, class Indice, class Tuple>
      static void add2(Bdl& bundle, const Indice& indice, const Tuple& tuple)
      {
        update2(bundle,indice,boost::fusion::at_c<I>(tuple));
        AddWithAll<K,OBS,I+1,F>::add2(bundle,indice,tuple);
      }
    };

    template<int K, class Obs, int F> struct AddWithAll<K,Obs,F,F>
      {
        template<class Bdl, class Indice, class Tuple> static void add(Bdl&, const Indice&, const Tuple&){}
        template<class Bdl, class Indice, class Tuple> static void add2(Bdl&, const Indice&, const Tuple&){}
      };

    template<class Obs, int I, int F> struct AddTupleIterate
    {
      template<class Bdl, class Tuple1, class Tuple2>
      static void add(Bdl& bundle, const Tuple1& original, const Tuple2& tuple)
      {
        static const std::size_t S = boost::fusion::result_of::size<Tuple2>::value;
        AddWithAll<I,Obs,0,S>::add(bundle,boost::fusion::at_c<I>(original),tuple);
        AddTupleIterate<Obs,I+1,F>::add(bundle,original,boost::fusion::pop_front(tuple));
      }
      
      template<class Bdl, class Tuple1, class Tuple2>
      static void add2(Bdl& bundle, const Tuple1& original, const Tuple2& tuple)
      {
        static const std::size_t S = boost::fusion::result_of::size<Tuple2>::value;
        AddWithAll<I,Obs,0,S>::add2(bundle,boost::fusion::at_c<I>(original),tuple);
        AddTupleIterate<Obs,I+1,F>::add2(bundle,original,boost::fusion::pop_front(tuple));
      }
    };

    template<class Obs, int F> struct AddTupleIterate<Obs,F,F>
    { 
      template<class Bdl, class Tuple1, class Tuple2> static void add(Bdl&, const Tuple1&, const Tuple2&) { }
      template<class Bdl, class Tuple1, class Tuple2> static void add2(Bdl&, const Tuple1&, const Tuple2&) { }
    };

    template<class Bdl, class Tuple> struct AddSelf
    {
      Bdl& bundle;
      const Tuple& tuple;
      AddSelf(Bdl& bundle_, const Tuple& tuple_):bundle(bundle_),tuple(tuple_){}
      template<size_t I> void operator()(Int<I> const &)
      {
        auto indice = bf::at_c<I>(tuple);
        typedef typename decltype(indice)::IdType Key;
        bundle.template indices<Key,Key>().add(indice,indice);
      }
    };
    
    struct AddTuple
    {
      template<class Obs, class Bdl, class Tuple> static void add(Bdl& bundle, const Tuple& tuple)
      {
        static const std::size_t S= boost::fusion::result_of::size<Tuple>::value;// nombre d'indices
        
        // association de chaque élément i du tuple avec les autres j tel que i <= j
        ttt::unroll<0,S>(AddSelf<Bdl,Tuple>(bundle,tuple));
        AddTupleIterate<Obs,0,S>::add(bundle,tuple,boost::fusion::pop_front(tuple));
      }
      
      template<class Obs, class Bdl, class Tuple> static void add2(Bdl& bundle, const Tuple& tuple)
      {
        static const std::size_t S= boost::fusion::result_of::size<Tuple>::value;// nombre d'indices
        
        // association de chaque élément i du tuple avec les autres j tel que i <= j
        AddTupleIterate<Obs,0,S>::add2(bundle,tuple,boost::fusion::pop_front(tuple));
      }
    };

    template<bool obs> struct DispName
    {
      std::ostream& stream;

      DispName(std::ostream& stream_):stream(stream_){}

      template<template<class,class> class Pair, class Key,class Value> void operator()(const Pair<Key,Value>& obj) const
      {
        if (obs)
        stream << " " << color.cyan() << ttt::name<Key>() << "(" << color.bold() << obj.second.size() << color.reset() << color.cyan() << ")" << color.reset() << std::endl;
        else
        stream << " " << color.cyan() << ttt::name<Key>() << "(" << obj.second.size() << ")" << color.reset() << std::endl;
      }
    };


    struct Print6
    {
      std::ostream& o;
      Print6( std::ostream& o_):o(o_){}

      template<class A, class B, class C> void operator()(const boost::fusion::pair<A,SIC<B,C>>& pair) const
      {
        pair.second.disp(o);
      }

      template<class Key> void operator()(const boost::fusion::pair<Key,TIC<Key>>& obj) const
      {
        obj.second.disp(o);
      }

      template<class A, class B> void operator()(const TIC<A,B>& obj) const
      {
        obj.disp(o);
      }

      template<template<class,class> class Pair, class K, class V> void operator()( const Pair<K,V>& pair_map) const
      {
        this->operator()(pair_map.second);
      }

      template<class T> void operator()( const T& map) const
      {
        boost::fusion::for_each(map,*this);
      }
    };

  }// eon detail

  template<class T> struct ToPair { typedef boost::fusion::pair<T,T> type;};
  template<class Liste> struct TransformToPair : mpl::transform< Liste, ToPair<mpl::_1> >::type {};

  typedef boost::fusion::pair<mpl::_1,br::as_map<mpl::_2>> FunctorPairKeyMap;

  template<class L1, class L2, class F> struct MapMapMaker : br::as_map<typename mpl::transform<L1,L2,F>::type> {};

  template<class F, class Apply> struct CreateListArgApply : 
    mpl::transform<
                    typename Function<F>::ParametersType,
                    mpl::apply<Apply,mpl::_1>
                  > {};

  struct MapFromTuple
  {
    template<template<class,class> class Pair, class Key, class Value, class B> void operator()(Pair<Key,Value>& a, const B& b) const
    {
      a.second = b;
    }
  };
}// eon lma



#endif
