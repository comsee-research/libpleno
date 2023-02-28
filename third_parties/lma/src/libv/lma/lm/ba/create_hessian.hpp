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

#ifndef __LMA_OPT2_BA_CREATE_HESSIAN_HPP__
#define __LMA_OPT2_BA_CREATE_HESSIAN_HPP__

#include "make_type.hpp"

#include <libv/lma/ttt/mpl/for.hpp>
#include <libv/lma/lm/function/function.hpp>
#include <libv/lma/ttt/fusion/pair.hpp>
#include <libv/lma/ttt/mpl/naming.hpp>
#include <libv/lma/ttt/mpl/cat.hpp>
#include <boost/mpl/unique.hpp>
#include <boost/mpl/count_if.hpp>
#include <boost/mpl/if.hpp>
#include <boost/mpl/or.hpp>
#include <boost/mpl/and.hpp>
#include <boost/mpl/transform.hpp>
#include <boost/mpl/replace.hpp>
#include <boost/mpl/copy_if.hpp>
#include <boost/type_traits/is_same.hpp>

namespace lma
{

  // extract parameters
  template<class L1> using ListOfListOfParameters = mpl::transform<L1,Function<mpl::_1>>;

  template<class A, class B> struct MakePair
  {
    typedef bf::pair<A,B> type;
  };

  template<class Int_, class List, class Int, class Result> struct CreatePairs3_
  : mpl::push_back<
                  Result,
                  typename MakePair<typename mpl::at<List,Int_>::type,typename mpl::at<List,Int>::type>::type
                  > {};

  template<class Int> using CreatePairs3 = CreatePairs3_<Int,mpl::_1,mpl::_2,mpl::_3>;

  template<class List, class Int, class Result> struct CreatePairs2_ : For<Int::value+1,mpl::size<List>::value,List,CreatePairs3<Int>,Result> {};

  template<class L> using CreatePairs2 = CreatePairs2_<L,mpl::_2,mpl::_3>;

  template<class T> struct Unique : mpl::unique<T, boost::is_same<mpl::_1,mpl::_2> > {};// ne retire que les doublons contigus

  // L ne doit pas contenir de doublon
  template<class L> struct CreatePairs : For<0,mpl::size<L>::value,L,CreatePairs2<L>> {};

  // get cross parameters
  template<class L2> using CrossParameters = mpl::transform<typename mpl::transform<L2,Unique<mpl::_1>>::type,CreatePairs<mpl::_1>>;

  template<class T> struct Double {};
  template<class T> struct Single {};

  template<class T, class L> struct SingleOrDouble : mpl::if_c<(mpl::count_if<L,boost::is_same<mpl::_1,T>>::value>1),Double<T>,Single<T>> {};

  template<class L> struct DiagHessianFunctor : mpl::transform<typename Unique<L>::type,SingleOrDouble<mpl::_1,L>> {};

  template<class L2> using ToSingleOrDouble = mpl::transform<L2,DiagHessianFunctor<mpl::_1>>;

  template<class L3, class L4> using CatCrossAndDiag = mpl::transform<L4,L3,Cat<mpl::_1,mpl::_2>>;

  template<class L, class T> struct AddIf;

  template<class L, class A, class B> struct AddIf<L,bf::pair<A,B>>
  : mpl::if_<
              mpl::or_<
                      mpl::contains<L,bf::pair<A,B>>,
                      mpl::contains<L,bf::pair<B,A>>
                      >,
              L,
              typename mpl::push_back<L,bf::pair<A,B>>::type
            > {};

  template<class L, class T> struct AddIf<L,Single<T>>
  : mpl::if_<
              mpl::or_<
                        mpl::contains<L,Single<T>>,
                        mpl::contains<L,Double<T>>
                      >,
              L,
              typename mpl::push_front<L,Single<T>>::type
            > {};


  template<class L, class T> struct AddIf<L,Double<T>>
  :  mpl::if_<
              boost::mpl::contains<L,Single<T>>,
              typename mpl::replace<L,Single<T>,Double<T>>::type,
              typename mpl::push_front<L,Double<T>>::type
            > {};

  template<class List, class Int, class Result> struct ForEachParameters : AddIf<Result,typename mpl::at<List,Int>::type> {};

  template<class List, class Int, class Result> struct ForEachFunctor :
  For<0,mpl::size<typename mpl::at<List,Int>::type>::value,typename mpl::at<List,Int>::type,ForEachParameters<mpl::_1,mpl::_2,mpl::_3>,Result> {};

  template<class L> using TypesContainers = For<0,mpl::size<L>::value,L,ForEachFunctor<mpl::_1,mpl::_2,mpl::_3>>;
  
  template<class T, class Flt> struct ToTable;
  
  template<class A, class Flt> struct ToTable<Single<A>,Flt>// : MakeTupleTable<A,A,Flt> {
  {
    typedef bf::pair<bf::pair<A,A>,Table<A,A,Flt,Diagonal>> type;
  };
  
  template<class A, class Flt> struct ToTable<Double<A>,Flt>
  {
    typedef bf::pair<bf::pair<A,A>,Table<A,A,Flt,Symetric>> type;
  };
  
  template<class A, class B, class Flt> struct ToTable<bf::pair<A,B>,Flt> : MakeTupleTable<A,B,Flt> {};
  
  template<class L, class Flt> using Containers = mpl::transform<L,ToTable<mpl::_1,Flt>>;
  
  
  template<class Result, class A, class B> struct add_ { typedef Result type; };
  template<class Result, class A> struct add_<Result,A,Single<A>> : mpl::push_back<Result,Single<A>> {};
  template<class Result, class A> struct add_<Result,A,Double<A>> : mpl::push_back<Result,Double<A>> {};
  template<class Result, class A, class B> struct add_<Result,A,bf::pair<A,B>> : mpl::push_back<Result,bf::pair<A,B>> {};
  template<class Elt, class L, class Int, class Result> struct Ordering2 : add_<Result,Elt,typename mpl::at<L,Int>::type> {};
  
  template<class L, class Ordre, class Int, class Result> struct Ordering : For<0,mpl::size<L>::value,L,Ordering2<typename mpl::at<Ordre,Int>::type,mpl::_1,mpl::_2,mpl::_3>,Result> {};
  template<class L, class Ordre> struct Order : For<0,mpl::size<Ordre>::value,Ordre,Ordering<L,mpl::_1,mpl::_2,mpl::_3>> {};
  
  template<class Keys, class T> struct IsKeyUs : mpl::false_ { };
  template<class Keys, class Key, class P, class Q, class T, class Tag> struct IsKeyUs<Keys, bf::pair< Key, Table<P,Q,T,Tag> >>
    : mpl::and_< mpl::contains<Keys,P>, mpl::contains<Keys,Q> > { };
  
  template<class T> struct SymetricToDiagonal { typedef T type; };
  template<class Key, class P, class T> struct SymetricToDiagonal<bf::pair< Key, Table<P,P,T,Diagonal> >>  { typedef bf::pair< Key, Table<P,P,T,Symetric> > type; };
  
  template<class H, class KeyUs> struct ListS
  {
    typedef typename mpl::copy_if<H, IsKeyUs<KeyUs,mpl::_1>>::type type0;
    typedef typename mpl::transform<type0, SymetricToDiagonal<mpl::_1>>::type type;
  };
  
  template<class Bundle, class flt> struct ListH
  {
    typedef typename Bundle::ListFunction ListFunction;
    typedef typename Bundle::ListeParam ListeParam;
    typedef typename Bundle::ParamFonctor ParamFonctor;
    typedef ListFunction L1;

//       typedef typename CreateListParam<ListFunction,ParamFonctor>::type parameters;
      
      typedef typename ListOfListOfParameters<L1>::type L2_;
      typedef typename mpl::transform<L2_,ParamFonctor>::type L2;
      
      typedef typename CrossParameters<L2>::type L3;
      
      typedef typename ToSingleOrDouble<L2>::type L4;

      typedef typename CatCrossAndDiag<L3,L4>::type L5;

      typedef typename TypesContainers<L5>::type L6;
      
      typedef typename Order<L6,ListeParam>::type L7;
      
      typedef typename Containers<L7,flt>::type type;
      
      
      
      static void disp()
      {
        std::cout << " Functors : " << ttt::name<L1>() << std::endl;
        std::cout << " ListOfListOfParameters : " << ttt::name<L2>() << std::endl;
        std::cout << " ToSingleOrDouble : " << ttt::name<L4>() << std::endl;
        std::cout << std::endl;
        std::cout << " CrossAndDiag : " << ttt::name<L5>() << std::endl;
        std::cout << std::endl;
        std::cout << " Types : " << ttt::name<L6>() << std::endl;
        std::cout << " Order  : " << ttt::name<L7>() << std::endl;
        std::cout << " Containers  : " << ttt::name<type>() << std::endl;
//         std::cout << " Parmaeters " << ttt::name<ListeParam>() << std::endl;
      }
  };
  
  
  
}

namespace ttt
{
  template<class T> struct Name<lma::Single<T>> { static std::string name() { return std::string("Single<") + ttt::name<T>() + ">"; } };
  template<class T> struct Name<lma::Double<T>> { static std::string name() { return std::string("Double<") + ttt::name<T>() + ">"; } };
}
  
#endif