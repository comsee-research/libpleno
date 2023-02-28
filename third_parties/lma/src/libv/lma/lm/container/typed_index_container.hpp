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

#ifndef __OPTIMISATION2_CONTAINER_TYPED_INDEX_CONTAINER_HPP__
#define __OPTIMISATION2_CONTAINER_TYPED_INDEX_CONTAINER_HPP__

#include <libv/lma/ttt/traits/naming.hpp>
#include <libv/lma/ttt/traits/typed_indice.hpp>
#include <libv/lma/ttt/traits/to_ref.hpp>
#include <libv/lma/color/console.hpp>
#include "container.hpp"


//! Typed index container

namespace lma
{
  template<class T> using AlignVector = std::vector<T,Eigen::aligned_allocator<T>>;
  
  template<class Key, class Value = Key> class TIC
  {
    public:
      typedef Value Type;
      typedef ttt::Indice<Key> Indice;

    private:
      typedef AlignVector<Type> Container;
      Container v_;

    public:

      typedef typename Container::iterator iterator;
      typedef typename Container::const_iterator const_iterator;

      static std::string name() { return " TIC<" + ttt::name<Key>() + "," +ttt::name<Value>() + ">" ; }

      void resize(size_t n, const Type& value = Type()) { v_.resize(n,value); }
      
      void clear() { v_.clear(); }
      
      Indice first() const { return Indice(0); }
      Indice size() const { return Indice(v_.size()); }

      iterator begin() { return v_.begin(); }
      iterator end() { return v_.end(); }
      const_iterator begin() const { return v_.cbegin(); }
      const_iterator end() const { return v_.cend(); }

      Type& operator()(const Indice& indice) { assert(indice<v_.size());return v_[indice()]; }
      const Type& operator()(const Indice& indice) const { assert(indice<v_.size());return v_[indice()]; }

      //! dereference l'objet s'il est de type pointeur
      typename ttt::ToRef<Type>::ref reference(const Indice& indice)
        {assert(indice<v_.size()); return ttt::to_ref(v_[indice()]);}

      typename ttt::ToRef<Type>::const_ref reference(const Indice& indice) const
        {assert(indice<v_.size()); return ttt::to_ref(v_[indice()]);}

      Indice add(const Type& a)
      {
        v_.push_back(a);
        return v_.size()-1;
      }

      std::ostream& disp(std::ostream& o) const
      {
        o << color.green() << std::endl << "TIC<" << ttt::name<Key>() << "," << ttt::name<Value>() << ">"  << color.reset() << color.italic() << " [Indice,Value] :\n" << color.reset();
        for(auto i = first(); i < size() ; ++i)
          o << " [" << i << ";" << reference(i) << "];";
        return o;
      }

  };

  struct TicCounter
  {
    typedef size_t result_type;

    template<template<class,class> class Pair, class Key, class Value> result_type operator()(const result_type& sum, const Pair<Key,Value>& pair) const
    {
      return this->operator()(sum,pair.second);
    }

    template<class Key, class Value> result_type operator()(const result_type& sum, const TIC<Key,Value>& tic) const
    {
      return sum + tic.size()();
    }
  };
}// eon

template<class Key, class Value> std::ostream& operator<<(std::ostream& o, const lma::TIC<Key,Value>& tic)
{
  return tic.disp(o);
}

namespace ttt
{
  template<class Key, class Value> struct Name<lma::TIC<Key,Value>> { static std::string name(){ return std::string("TIC<") + ttt::name<Key>() << "," + ttt::name<Value>() << ">";} };
}

#endif
