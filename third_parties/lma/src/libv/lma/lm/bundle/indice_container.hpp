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

#ifndef __OPTIMISATION2_BUNDLE_INDICE_CONTAINER_HPP__
#define __OPTIMISATION2_BUNDLE_INDICE_CONTAINER_HPP__

#include <cassert>
#include <vector>
#include <map>
#include <set>
#include <algorithm>

#include <libv/lma/ttt/traits/naming.hpp>
#include <libv/lma/ttt/traits/typed_indice.hpp>
#include <libv/lma/color/console.hpp>
#include <utility>
#include <boost/fusion/include/pair.hpp>

//! SparseIndiceContainer

/**
 * Le container d'indice SIC<A,B> lie les paramètres A avec les paramètres B :
 * SIC<A,B> contient pour chaque instance de A, la liste des B associés.
 * Le container std::map est utilisé uniquement pour accélérer le remplissage avec add()
 */

template<class A, class B> class SIC
{
  public:
    typedef A first_type;
    typedef B second_type;
    typedef ttt::Indice<A> IndiceA;
    typedef ttt::Indice<B> IndiceB;
    typedef ttt::Indice<boost::fusion::pair<B,A> > SparseAB;

    static std::string name() { return " SIC<" + ttt::name<A>() + "," + ttt::name<B>() + ">" ; }

    SIC(){/*std::cout << name() << std::endl;*/}
    IndiceA first() const { return IndiceA(0); }
    IndiceA size() const { return IndiceA(vaz.size()); }

    SparseAB first(const IndiceA&) const { return SparseAB(0); }
    SparseAB size(const IndiceA& ind) const { assert(ind<vaz.size());return SparseAB(vaz[ind()].size()); }

    const IndiceB& operator()(const IndiceA& inda, const SparseAB& indab) const
    {
      assert(inda<vaz.size());
      assert(indab<vaz[inda()].size());
      return vaz[inda()][indab()];
    }

    void set_max(const IndiceA& max_a, const IndiceB& max_b)
    {
      vazset.resize(max_a());
      maxb = max_b;
    }

    void add(const IndiceA& inda, const IndiceB& indb)
    {
      assert(inda() < int(vazset.size()));
      vazset[inda()].insert(indb);
    }
    
    SparseAB get(const IndiceA& a, const IndiceB& b)
    {
      assert( size_t(a()) < map.size() );
      assert( map[a()].find(b) != map[a()].end() );
      return map[a()][b];
    }

    void update()
    {
      vaz.resize(vazset.size());
      map.resize(vazset.size());
      
      for(size_t i = 0 ; i < vaz.size() ; ++i)
        for(auto& s : vazset[i])
        {
          map[i][s] = vaz[i].size();
          vaz[i].push_back(s);
        }
      std::vector<std::set<IndiceB>>().swap(vazset);  
    }

    std::ostream& disp(std::ostream& o) const
    {
      o << color.blue() << std::endl << "SIC<" << ttt::name<A>() << "," << ttt::name<B>() << ">" << color.reset() << " :\n";
      if (size()==0) std::cout << " empty " << std::endl;
      for(auto i = first(); i != size() ; ++i)
        for(auto j = first(i); j != size(i) ; ++j)
          o << i << "," << j << "->" << this->operator()(i,j) << ";";
      return o;
    }

    IndiceB maxb;

  private:
    
    std::vector<std::vector<IndiceB>> vaz;
    std::vector<std::set<IndiceB>> vazset;
    std::vector<std::map<IndiceB,int>> map;
};

/**
 * Le container SIC2 est utilisé uniquement pour l'algo de ExplicitSchur
 * pour accéder aux indices éparses de SIC<A,B>
 */

template<class A, class B> class SIC2
{
  public:
    typedef A first_type;
    typedef B second_type;
    typedef ttt::Indice<A> IndiceA;
    typedef ttt::Indice<B> IndiceB;
    typedef ttt::Indice<boost::fusion::pair<B,A> > SparseAB;
    typedef ttt::Indice<boost::fusion::pair<A,B> > ReverseBA;

    static std::string name() { return " SIC2<" + ttt::name<A>() + "," + ttt::name<B>() + ">" ; }

    SIC2(){/*std::cout << name() << std::endl;*/}

    SparseAB add(const IndiceA& inda, const IndiceB&, const ReverseBA& rba)
    {
      size_t i(inda());
      if (i >= vaz.size())
      {
        vaz.resize(i+1);
        vaz.back().push_back(rba);
        return 0;
      }

      vaz[i].push_back(rba);
      return vaz[i].size() - 1;
    }

    ReverseBA reverse(const IndiceA& inda, const SparseAB& indab) const
    {
      assert(inda<vaz.size());assert(indab<vaz[inda()].size());
      return vaz[inda()][indab()];
    }

  private:
    std::vector<std::vector<ReverseBA>> vaz;
};

namespace ttt
{
  template<class A, class Z> struct Name<SIC<A,Z>> { static std::string name() { return "SIC<" + ttt::name<A>() + "," + ttt::name<Z>() + ">"; } };
}

template<class A, class Z> std::ostream& operator<<(std::ostream& o, const SIC<A,Z>& va)
{
  return va.disp(o);
  o << ttt::name<SIC<A,Z>>() << std::endl; //"SIC<" << ttt::name<A>() << "," << ttt::name<Z>() << ">\n";
  if (va.size()==0) std::cout << " empty " << std::endl;
  for(auto i = va.first() ; i < va.size() ; ++i)
  {
//     if (va.size(i)!=0)
      o << " [ ";
    for(auto j = va.first(i) ; j < va.size(i) ; ++j)
      o << va(i,j) << " ";
//     if (va.size(i)!=0)
      o << "] ";
    o << std::endl;
  }
  return o;
}


#endif
