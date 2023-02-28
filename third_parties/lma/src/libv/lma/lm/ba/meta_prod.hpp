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

#ifndef __LMA_OPTIMISATION_BA_METAPROD_HPP__
#define __LMA_OPTIMISATION_BA_METAPROD_HPP__

#include <libv/lma/ttt/mpl/for.hpp>
#include <libv/lma/ttt/mpl/naming.hpp>
#include <libv/lma/lm/trait/size.hpp>
#include <boost/fusion/mpl.hpp>
#include <boost/mpl/remove_if.hpp>
#include <boost/mpl/transform.hpp>
#include <libv/lma/ttt/mpl/cat.hpp>
#include <boost/fusion/include/pair.hpp>

namespace lma
{
  template<class> struct Transpose;
}

namespace boost
{
  namespace mpl
  {
    template<class T> struct size<lma::Transpose<T>> { static const std::size_t value = boost::mpl::size<T>::value; };
    template<class L, class Int> struct at<lma::Transpose<L>,Int> { typedef lma::Transpose<typename boost::mpl::at<L,Int>::type> type; };
  }
}

namespace lma
{
  template<class, class> struct Trans {};
  
  template<class T> struct TransToPair
  {
    typedef T type;
  };
  
  template<class A, class B> struct TransToPair<Trans<A,B>>
  { 
    typedef bf::pair<A,B> type;
  };
  
  template<class> struct Transpose {};
  
  template<class A, class B> struct Transpose<bf::pair<A,B>>
  {
    typedef Trans<A,B> type;
  };
  
  template<class T> struct Size<Transpose<T>> 
  {
    static const std::size_t value = Size<T>::value;
  };

  template<class T> struct ToKey;

  template<class A, class B, template<class,class> class Pair> struct ToKey<Pair<A,B>>
  {
    typedef A type;
  };

  template<class A, class B> struct ToKey<Transpose<bf::pair<A,B>>>
  {
    typedef typename Transpose<A>::type type;
  };
  
  template<class A, class B> struct Product : mpl::false_
  {
    typedef bf::pair<A,B> type;
  };
  
  template<class A, class B, class C> struct Product<bf::pair<A,B>,bf::pair<B,C>> : mpl::true_
  {
    typedef bf::pair<A,C> type;
  };

  template<class A, class B, class C> struct Product<bf::pair<A,C>,Trans<B,C>> : mpl::true_
  {
    typedef bf::pair<A,B> type;
  };

  template<class A, class B> struct Product<bf::pair<A,B>,B> : mpl::true_
  {
    typedef A type;
  };

  template<class A, class B> struct Product<Trans<A,B>,A> : mpl::true_
  {
    typedef B type;
  };


  template<class Prod, class Result> struct ProdFonctor2Elt 
  : mpl::if_< 
              Prod,
              typename mpl::push_back< Result, Prod >::type ,
              Result
            > {};
  
  
  template<class Key1, class List, class Int, class Result, class> struct ProdFonctor2_ 
  : ProdFonctor2Elt<
                     Product< Key1, typename ToKey<typename mpl::at<List,Int>::type>::type >,
                     Result
                   > {};


  struct _meta_prog_debug_ {};
  
  template<class Key1, class List, class Int, class Result> struct ProdFonctor2_<Key1,List,Int,Result,_meta_prog_debug_>
    : mpl::push_back< Result, Product< Key1, typename ToKey<typename mpl::at<List,Int>::type>::type > > {};

  
  template<class L2, class List, class Int, class Result, class _ = void> struct ProdFonctor_ : 
      ttt::For<
                0,
                mpl::size<L2>::value,
                L2,
                ProdFonctor2_<
                              typename ToKey<typename mpl::at<List,Int>::type>::type,
                              mpl::_1,mpl::_2,mpl::_3,_
                             >,
                Result
              > {};
      
  template<class L1, class L2> struct MetaProd : ttt::For<0,mpl::size<L1>::value,L1,ProdFonctor_<L2,mpl::_1,mpl::_2,mpl::_3>> {};
  
  template<class L1, class L2> struct MetaProdTest : ttt::For<0,mpl::size<L1>::value,L1,ProdFonctor_<L2,mpl::_1,mpl::_2,mpl::_3,_meta_prog_debug_>> {};

  template<class> struct Unary{};
  template<class,class> struct Binary{};

  template<class L1> struct MetaBinary : mpl::transform<L1,Binary<mpl::_1,mpl::_1>> {};
  template<class L1> struct MetaUnary : mpl::transform<L1,Unary<mpl::_1>> {};
  
  namespace add_transpose
  {
    template<class> struct IsPairAA : mpl::true_ {};
    template<class A, class _> struct IsPairAA<bf::pair<bf::pair<A,A>,_>> : mpl::false_ {};
    template<class T> struct ReversePair {typedef Transpose<T> type;};
    template<class List, class Int, class Result> struct AddTranspose 
      : mpl::if_< 
                  IsPairAA<typename mpl::at<List,Int>::type>,
                  typename mpl::push_back< Result, typename ReversePair<typename mpl::at<List,Int>::type>::type >::type ,
                  Result
                > {};
  }
  template<class L> struct AddTranspose : For<0,mpl::size<L>::value,L,add_transpose::AddTranspose<mpl::_1,mpl::_2,mpl::_3>,L> {};
}

namespace ttt
{
  template<class T> struct Name<lma::Transpose<T>> { static const std::string name() { return std::string("Transpose<") + ttt::name<T>() + ">"; } };
  template<class A, class B> struct Name<lma::Trans<A,B>> { static const std::string name() { return std::string("Trans<") + ttt::name<A>() + "," + ttt::name<B>() + ">"; } };
  template<class A, class B> struct Name<lma::Product<A,B>> { static const std::string name() 
  { return color.yellow("\n[ ") + ttt::name<typename lma::Product<A,B>::type>() + color.yellow(" = ") +  ttt::name<A>() + color.yellow(" * ") + ttt::name<B>() + color.yellow("]");} };
}

#endif
