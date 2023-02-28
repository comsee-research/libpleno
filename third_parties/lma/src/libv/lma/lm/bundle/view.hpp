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

#ifndef __OPTIMISATION_BUNDLE_VIEW_HPP__
#define __OPTIMISATION_BUNDLE_VIEW_HPP__

#include "bundle.hpp"
#include <boost/type_traits/add_pointer.hpp>
#include <boost/fusion/include/transform.hpp>

namespace lma
{

  struct AddMCAPointerToValue
  {
    template<class A, class B> void operator()(A& pair1, const B& pair2) const
    {
      for(const auto& x : pair2.second)
        pair1.second.add(*x);
    }
  };

  struct RestoreMCAValueToPointer
  {
    template<class A, class B> void operator()(A& pair1, const B& pair2) const
    {
      auto& a = pair1.second;
      auto& b = pair2.second;
      auto j = a.first();
      assert(a.size()() == b.size()());
      for(auto i = b.first() ; i < b.size() ; ++i, ++j)
        *a(j) = b(i);
    }
  };

  template< class V > struct AddPointerToListParam : mpl::transform<V,boost::add_pointer<mpl::_1>> {};


/**
 * \struct View
 * \brief A View is a bundle where each parameter T is replaced by T*
 * LM solver need to modify bundle's data and eventualy restore datas
 * So the parameters must be copyable and restored by value. This is
 * done by the functions restore_container and clone_opt_container
 */


  template<class V> struct View : Bundle< V , AddPointerToListParam<mpl::_1> >
  {
    typedef Bundle< V , AddPointerToListParam<mpl::_1> > parent;
    typedef typename Bundle<V>::MCA ValuedMCA;

    void restore_container(const ValuedMCA& vmca)
    {
      ttt::copy_f(this->opt_container.map(),vmca.map(),RestoreMCAValueToPointer());
    }

    ValuedMCA clone_opt_container()
    {
      ValuedMCA vmca;
      ttt::copy_f(vmca.map(),this->opt_container.map(),AddMCAPointerToValue());
      return vmca;
    }
  };
}

#endif

