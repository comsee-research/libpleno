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

#ifndef __OPTIMISATION2_BA_LDLT_HPP__
#define __OPTIMISATION2_BA_LDLT_HPP__

#include "../container/container.hpp"
#include "tuple_to_mat.hpp"
#include "../omp/omp.hpp"
#include "isdiagonal1f.hpp"
#include "mat.hpp"
#include "make_type.hpp"
#include <boost/mpl/for_each.hpp>
#include <boost/fusion/include/front.hpp>
#include <libv/lma/time/tictoc.hpp>

namespace lma
{  
  template<class Delta, class X> struct AssignDelta
  {
    Delta& delta;
    const X& x;
    int& cpt;

    AssignDelta(Delta& delta_, const X& x_, int& cpt_):delta(delta_),x(x_),cpt(cpt_){}

    template<class Key> void operator()(ttt::wrap<Key>)
    {
      auto& refa = boost::fusion::at_key<Key>(delta);
//       #pragma omp parallel for if(use_omp())
      for(auto i = refa.first() ; i < refa.size() ; ++i)
      {
        for(size_t k = 0 ; k < refa.I ; ++k)
          refa(i)[k] = x[cpt++];
      }
    }
  };

  template<class Delta, class X> AssignDelta<Delta,X> assign_delta(Delta& delta, const X& x, int& cpt)
  {
    return AssignDelta<Delta,X>(delta,x,cpt);
  }

  namespace internal
  {
    template<bool IsDiagonal1F> struct LDLT
    {
      template<class Tag, class Container, class Delta>
      static void compute(const Container& container, Delta& delta)
      {
        auto a = to_mat<Tag,typename Container::OptimizeKeys,typename Container::Hessian>(container.A(),size_tuple<mpl::size<typename Container::OptimizeKeys>::value>(delta));
//         auto a = to_sparse<typename Container::OptimizeKeys,typename Container::Hessian>(container.A(),size_tuple<mpl::size<typename Container::OptimizeKeys>::value>(delta));
        auto b = to_matv<Tag>(container.B());

        typename ContainerOption<Tag,0,0>::MatrixD1 x(b.size());

        ldlt_solve(x,a,b);

#ifndef NDEBUG
        if (is_invalid(x)) throw NAN_ERROR("Ldlt : delta contains nan");
#endif
        int cpt = 0;
        mpl::for_each<typename Container::OptimizeKeys,ttt::wrap<mpl::_1>>(assign_delta(delta,x,cpt));
      }
    };
    
    template<> struct LDLT<true>
    {
      template<class Float, class Container, class Delta>
      static void compute(const Container& container, Delta& delta)
      {
        auto& h = bf::front(container.A()).second;
        auto& jte = bf::front(container.B()).second;
        auto& d = bf::front(delta).second;
        
        if (h.size() == h.v.size()) // matrice diagonale par block
      	{
      	  for(auto i = h.first() ; i < h.size() ; ++i)
      	    d(i) = h(i).llt().solve(jte(i));
      	}
      	else
      	{
      	  LDLT<false>::compute(container,delta);
      	}
      }
    };
  }
  
  struct LDLT
  {
    template<class Config>
    LDLT(Config){}

    template<class MatrixTag, class Container, class Delta, size_t TotalFParams = 1>
    void operator()(const Container& container, Delta& delta, const MatrixTag&) const
    {
      internal::LDLT< IsDiagonal<Container>::value >::template compute<MatrixTag>(container,delta);
    }
  };

}

namespace ttt
{
  template<> struct Name<lma::LDLT> { static std::string name(){ return "LDLT"; } };
}
#endif
