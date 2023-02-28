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

#ifndef __OPTIMISATION2_BUNDLE_MAKE_SPARSE_INDICE_HPP__
#define __OPTIMISATION2_BUNDLE_MAKE_SPARSE_INDICE_HPP__

#include <libv/lma/global.hpp>
#include <libv/lma/ttt/traits/typed_indice.hpp>
#include "indice_container.hpp"
#include <boost/fusion/include/as_map.hpp>
#include <boost/mpl/erase.hpp>
#include <boost/mpl/transform.hpp>
#include <boost/mpl/find.hpp>

/*
 * Generative Algorithm : MakeSparseIndice
 * Input       : boost::mpl::vector<A,B,C>
 * Output      :
                map<
                    pair<
                            A,
                            map<
                                pair<B,Indice<B>>,
                                pair<C,Indice<C>>
                              >
                          >,
                    pair<
                            B, 
                            map<
                                pair<A,Indice<A>>,
                                pair<C,Indice<C>>
                              >
                          >,
                    pair<
                            C,
                            map<
                                pair<A,Indice<A>>,
                                pair<B,Indice<B>>
                              >
                        >
                  >
!*/

namespace lma
{

  template<class Vector, class T> struct GenPairKeyIndice :
    br::as_map<
                typename mpl::transform<
                                          Vector,
//                                           typename mpl::erase<
//                                                               Vector,
//                                                               typename mpl::find<
//                                                                                   Vector,
//                                                                                   T
//                                                                                 >::type
//                                                               >::type,
                                          bf::pair<
                                                    mpl::_1,
                                                    ttt::Indice<
                                                                bf::pair<
                                                                        T,
                                                                        mpl::_1
                                                                      >
                                                                >
                                                  >
                                        >::type
              > {};

  template<class Vector> struct MakeSparseIndice :
    br::as_map<
                typename mpl::transform<
                                        Vector,
                                        bf::pair<
                                                  mpl::_1,
                                                  GenPairKeyIndice<Vector,mpl::_1>
                                                >
                                        >::type
              > {};


  template<class Vector, class T> struct GenPairKeyContainer1 :
    boost::fusion::result_of::as_map<
                                      typename boost::mpl::transform<
                                                                      Vector,
//                                                                       typename boost::mpl::erase<
//                                                                                                   Vector,
//                                                                                                   typename boost::mpl::find<Vector,T>::type
//                                                                                                 >::type,
                                                                      boost::fusion::pair<
                                                                                          boost::mpl::_1,
                                                                                          SIC<boost::mpl::_1,T>
                                                                                          >
                                                                    >::type
                                    > {};

  template<class Vector> struct MakeSparseIndiceContainer1 :
    boost::fusion::result_of::as_map<
                                      typename boost::mpl::transform<
                                                                      Vector,
                                                                      boost::fusion::pair<
                                                                                            boost::mpl::_1,
                                                                                            GenPairKeyContainer1<Vector,boost::mpl::_1>
                                                                                         >
                                                                    >::type
                                    > {};

  //! seule différence : SIC -> SIC2
  template<class Vector, class T> struct GenPairKeyContainer2 :
    boost::fusion::result_of::as_map<typename boost::mpl::transform<typename
    boost::mpl::erase<Vector,typename boost::mpl::find<Vector,T>::type>::type,
    boost::fusion::pair<boost::mpl::_1,SIC2<boost::mpl::_1,T>>>::type> {};

  template<class Vector> struct MakeSparseIndiceContainer2 :
    boost::fusion::result_of::as_map<typename boost::mpl::transform<Vector,
    boost::fusion::pair<boost::mpl::_1,GenPairKeyContainer2<Vector,boost::mpl::_1>
    >>::type> {};
}

#endif
