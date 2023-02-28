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

#ifndef __OPTIMISATION2_BA_INITIALIZE_HPP__
#define __OPTIMISATION2_BA_INITIALIZE_HPP__

#include "bas.hpp"
#include <boost/fusion/include/for_each.hpp>
#include <boost/fusion/include/pair.hpp>

namespace lma
{
  namespace detail
  {
    template<class Bundle> struct ResizeInter
    {
      const Bundle& bundle;
      ResizeInter(const Bundle& bundle_):bundle(bundle_){}

      template<class Key1, class Key2, class Cont, template<class,class> class Pair> void operator()(Pair<Pair<Key1,Key2>,Cont>& obj) const
      {
        obj.second.resize(bundle.template indices<Key1,Key2>());
      }
    };
    
    template<class Bundle> struct ResizeInterInit
    {
      const Bundle& bundle;
      ResizeInterInit(const Bundle& bundle_):bundle(bundle_){}

      template<class Key1, class Key2, class Cont, template<class,class> class Pair> void operator()(Pair<Pair<Key1,Key2>,Cont>& obj) const
      {
        auto& indice = bundle.template indices<Key1,Key2>();
        auto& table = obj.second;

        table.indice.set_max(bundle.template at_opt<Key1>().size(),bundle.template at_opt<Key2>().size());

        for(auto i = indice.first() ; i < indice.size() ; ++i)
          for(auto j = indice.first(i) ; j < indice.size(i) ; ++j)
            table.indice.add(i,indice(i,j));
      }
    };
    
    template<class Bundle> struct ResizeDelta
    {
      const Bundle& bundle;
      ResizeDelta(const Bundle& bundle_):bundle(bundle_){}

      template<class Key, class Cont> void operator()(bf::pair<Key,Cont>& obj) const
      {
        obj.second.resize(bundle.template at_opt<Key>().size());
      }
    };

    struct SetZero
    {
      template<class Container> void operator()(Container& obj) const
      {
        obj.second.set_zero();
      }
    };
  }// eon

  template<class BDL, class BA> void initialize(const BDL& bundle, BA& ba)
  {
    using namespace detail;
    bf::for_each(ba.h,ResizeInter<BDL>(bundle));
    bf::for_each(ba.delta,ResizeDelta<BDL>(bundle));
    bf::for_each(ba.jte,ResizeDelta<BDL>(bundle));
  }

  template<class BA> void set_zero_(BA& ba)
  {
    using namespace detail;
    bf::for_each(ba.delta,SetZero());
  }


}// eon
#endif

