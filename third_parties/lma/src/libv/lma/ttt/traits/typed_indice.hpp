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

#ifndef __TTT_TRAITS_TYPED_INDICE_HPP__
#define __TTT_TRAITS_TYPED_INDICE_HPP__

#include <cstddef>
#include <iostream>
#include "naming.hpp"
#include <boost/assert.hpp>

namespace ttt
{
  template<class ID> class Indice
  {
    public:
      typedef int Type;
      Type indice;
      typedef ID IdType;
      typedef Indice<ID> type;
      Indice():indice(-1) {}
      Indice(Type val):indice(val) {}
      inline Type operator++() { return ++indice; }
      inline Type operator--() { return --indice; }
      inline Type operator--(int) { return --indice; }
      inline Type operator-=(int v) { return indice-=v; }
      inline Type operator+=(int v) { return indice+=v; }
      inline Type operator++(int) { return indice++; }
      inline const Type& operator()() const { return indice; }
      inline Type& operator()() { return indice; }
      inline Type operator*(int v){ return indice*v;}
      template<class Q> Indice& operator=(const Q& val) { indice = val; return *this;}
      template<class Q> Indice<Q> to() const { return Indice<Q>(indice); }
      static std::string name() { return ttt::name<Indice<ID>>(); };
  };

  template<class ID, class Scalar> Indice<ID> to_indice(const Scalar& value){ return Indice<ID>(value); }
  template<class ID, class ID2> Indice<ID> to_indice(const Indice<ID2>& indice){ return Indice<ID>(indice()); }

  template<class ID> inline bool operator<(const Indice<ID>& indice, const typename Indice<ID>::Type& val)
    { return indice() < val; }

  template<class ID> inline bool operator<=(const Indice<ID>& indice1, const Indice<ID>& indice2)
    { return indice1() <= indice2(); }
    
  template<class ID> inline bool operator>=(const Indice<ID>& indice1, const Indice<ID>& indice2)
    { return indice1() >= indice2(); }

  template<class ID> inline bool operator>=(const Indice<ID>& indice1, const typename Indice<ID>::Type& indice2)
    { return indice1() >= indice2; }

  template<class ID> inline bool operator>(const Indice<ID>& indice, const typename Indice<ID>::Type& val)
    { return indice() > val; }

  template<class I> inline bool operator<(const Indice<I>& indice1, const Indice<I>& indice2)
    { return indice1() < indice2(); }

  template<class I> inline bool operator>(const Indice<I>& indice1, const Indice<I>& indice2)
    { return indice1() > indice2(); }

  template<class I> inline bool operator !=(const Indice<I>& indice1, const Indice<I>& indice2)
    { return indice1() != indice2(); }

  template<class I> inline bool operator ==(const Indice<I>& indice1, const Indice<I>& indice2)
    { return indice1() == indice2(); }

  template<class ID> inline Indice<ID> operator+(const Indice<ID>& indice, const typename Indice<ID>::Type& val)
    { return Indice<ID>(indice()+val);}

  template<class ID> inline Indice<ID> operator+(const Indice<ID>& indice1, const Indice<ID>& indice2)
    { return Indice<ID>(indice1()+indice2());}
    
  template<class ID> inline Indice<ID> operator/(const Indice<ID>& indice, const typename Indice<ID>::Type& val)
    { return Indice<ID>(indice() / val); }

  template<class ID> inline Indice<ID> operator-(const Indice<ID>& indice, const typename Indice<ID>::Type& val)
    { BOOST_ASSERT_MSG((indice() >= val),"Indice must be positive.");return Indice<ID>(indice() - val); }

  template<class ID> inline typename Indice<ID>::Type operator-(const Indice<ID>& indice, const Indice<ID>& indice2)
    { return indice() - indice2(); }//--> si T==size_t, on passera par le assert d'operator-(Ind,size_t)

  template<class ID> inline Indice<ID>& operator+=(Indice<ID>& indice, const typename Indice<ID>::Type& val)
    { indice() += val; return indice; }

//   template<class ID> inline Indice<ID> operator-(const Indice<ID>& indice, const typename Indice<ID>::Type& val)
//     {  /*BOOST_ASSERT_MSG((indice() >= val),"Indice must be positive.");*/return Indice<ID>(indice()-val);}

  template<class ID> inline bool operator !=(const Indice<ID>& indice, const typename Indice<ID>::Type& val)
    { return indice() != val; }

  template<class ID> inline bool operator ==(const Indice<ID>& indice, const typename Indice<ID>::Type& val)
    { return val == indice(); }

  template<class ID> std::ostream& operator<<(std::ostream& o, const Indice<ID>& indice)
    { return o << indice();}


  template<class ID> struct Name<Indice<ID>>
  {
    static std::string name(){ return "Indice<" + ttt::name<ID>() + ">";}
  };
}

#endif
