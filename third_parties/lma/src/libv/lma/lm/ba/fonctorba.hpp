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

#ifndef __VISION_STRUCTURE_STATIC_STUFF_FONCTORBA_HPP__
#define __VISION_STRUCTURE_STATIC_STUFF_FONCTORBA_HPP__

#include "computing.hpp"

#include <boost/mpl/for_each.hpp>
#include <boost/mpl/push_back.hpp>
#include <boost/mpl/vector.hpp>
#include <boost/mpl/transform.hpp>
#include <boost/fusion/include/at_key.hpp>

#include <libv/lma/global.hpp>
#include <libv/lma/ttt/traits/wrap.hpp>
#include <libv/lma/ttt/traits/naming.hpp>
#include <libv/lma/ttt/traits/unroll1.hpp>
#include <libv/lma/ttt/traits/rm_all.hpp>
#include <libv/lma/lm/trait/accessor.hpp>

#include <boost/fusion/include/find.hpp>
#include <boost/fusion/include/is_sequence.hpp>
#include <tuple>

#include <libv/lma/lm/ba/meta_prod.hpp>
#include <libv/lma/lm/container/container.hpp>

namespace lma
{
  template<class A, class B, class Flt, class _, class I, class J> typename Table<A,B,Flt,_>::Matrix& to_block(Table<A,B,Flt,_>& table, I i, J j)
  {
    return table(i,j);
  }
  
  template<class A, class B, class Flt, class I, class J> typename Table<A,B,Flt,Diagonal>::Matrix& to_block(Table<A,B,Flt,Diagonal>& table, I i, J)
  {
    return table(i);
  }

  // Helping accessors
  template<class Key, class Map>
  typename br::value_at_key<Map,bf::pair<Key,Key>>::type::Matrix&
   at_h(Map& h, const ttt::Indice<Key>& indice)
  {
    return to_block(bf::at_key<bf::pair<Key,Key>>(h),indice,0);
  }
  
  template<class Key1, class Key2, class Map>
  typename br::value_at_key<Map,bf::pair<Key1,Key2>>::type::Matrix&
   at_h(Map& h, const ttt::Indice<Key1>& indice, const ttt::Indice<bf::pair<Key2,Key1>>& isparse)
  {
    return to_block(bf::at_key<bf::pair<Key1,Key2>>(h),indice,isparse);
  }
  
  template<class Key, class Map>
  typename br::value_at_key<Map,Key>::type::Matrix& at_jte(Map& jte, const ttt::Indice<Key>& indice)
  {
    return bf::at_key<Key>(jte)(indice);
  }
  
  // compute JtJ at block level
  template<class Result, class JA, class JB> inline void jtj(Result& result, const JA& Ja, const JB& Jb)
  {
    result += transpose(Ja) * Jb;
  }
  
  // compute Jte at block level
  template<class Result, class J, class E, class Tag> inline void jte(Result& result, const J& j, const E& erreur, ttt::wrap<Tag> const&)
  {
    result -= transpose(j) * make_view(erreur,ttt::wrap<Tag>());
  }

  template<size_t K, class Tie> struct JaTJb
  {
    Tie& tie;
    JaTJb(Tie& tie_):tie(tie_){}
    
    template<size_t I> void operator()(Int<I> const&)
    {
      auto& map_indice = bf::at_c<0>(tie);
      auto& jacob = bf::at_c<1>(tie);
      auto& bundle = bf::at_c<3>(tie);
      auto& iobs = bf::at_c<4>(tie);
      auto& hessien= bf::at_c<5>(tie);
      
      const auto& indice1 = bf::at_c<K>(map_indice);
      const auto& Ja = bf::at_c<K>(jacob).second;
      const auto& Jb = bf::at_c<I>(jacob).second;
      const auto& isparse = bf::at_c< I - (K + 1) >(bundle.spi2.template sparse_indices<K>(iobs));
      
      // sparse_indices structure with 4 paramters A B C D :
      //  ---------------
      //  |  AB  AC  AD
      //  |  BC  BD   
      //  |  CD      
      
      // sparse_indices access :
      // K\  L =   0   1   2   with L = I - (K + 1)
      //       ---------------
      // K = 0 |   AB  AC  AD
      // K = 1 |   BC  BD   
      // K = 2 |   CD

      jtj(at_h(hessien,indice1,isparse),Ja,Jb);
    }
  };

  template<size_t S, class Tie, class MatrixTag> struct JtJ
  {
    Tie& tie;
    JtJ(Tie& tie_):tie(tie_){}
    template<size_t I> void operator()(Int<I> const &)
    {
      auto& map_indice   = bf::at_c<0>(tie);
      auto& jacob        = bf::at_c<1>(tie);
      auto& residu       = bf::at_c<2>(tie);
      auto& hessien      = bf::at_c<5>(tie);
      auto& residus      = bf::at_c<6>(tie);
      
      const auto& J      = bf::at_c<I>(jacob).second;
      const auto& indice = bf::at_c<I>(map_indice);
      
      jtj(at_h(hessien,indice),J,J);
      jte(at_jte(residus,indice),J,residu,ttt::wrap<MatrixTag>());
      //JaTJb (only trig sup) :
      ttt::unroll<I+1,S>(JaTJb<I,Tie>(tie));
    }
  };



  
  template<class H> struct AssignSame
  {
    const H& h_;
    AssignSame(const H& h__):h_(h__){}
    
    template<class _> void operator()(const _&) const {}
    
    template<class Key, class Cont, template<class,class> class Pair> void operator()(Pair<Key,Cont>& obj) const
      { 
        auto& h = bf::at_key<Key>(h_);
        auto& s = obj.second;
        
        _assign_(s,h);
      }
    
    template<class S_, class H_>
    void _assign_(S_& s,const H_& h) const
    {
//       std::cout << " assign table table : " << s.name() << std::endl;
      if (s.size()==0)
      {
        s = h;
      }
      else
      {
        for(auto i = h.first() ; i < h.size() ; ++i)
          for(auto j = h.first(i) ; j < h.size(i) ; ++j)
          {
            s(i,j) = h(i,j);
          }
      }
    }

    template<class A, class Flt>
    void _assign_(Table<A,A,Flt,Symetric>& s,const Table<A,A,Flt,Diagonal>& h) const
    {
//       std::cout << " assign symetric diagonal : " << s.name() << std::endl;
      for(auto i = h.first() ; i < h.size() ; ++i)
        s(i,0) = h(i);
    }
    
    template<class A, class Flt>
    void _assign_(Table<A,A,Flt,Symetric>& s,const Table<A,A,Flt,Symetric>& h) const
    {
//       std::cout << " assign symetric symetric : " << s.name() << std::endl;
      for(auto i = h.first() ; i < h.size() ; ++i)
        for(auto j = h.first(i) ; j < h.size(i) ; ++j)
//           if (i <= s.indice(i,j))
        {
          s(i,s.indice.get(i,h.indice(i,j))) = h(i,j);
        }
    }
    
    template<class A, class Flt>
    void _assign_(Table<A,A,Flt,Diagonal>& s,const Table<A,A,Flt,Diagonal>& h) const
    {
//       std::cout << " assign diagonal diagonal : " << s.name() << std::endl;
      s.copy_mat(h);
//       s = h;
    }
  };
  
  
  struct OpAssignSame
  {
    template<class A, class B> void operator()(A& a, const B& b) const
    {
      a.copy_mat(b);
    }
  };
 
  struct Inv
  {
    template<class T> void operator()(T& t) const
    {
      t.inv();
    }
  };

  struct Correct
  {
    template<class Bdl, class Delta> void operator()(Bdl& ref, const Delta& refdelta) const
    {
      assert(ref.size()==refdelta.size());
      for(auto i = ref.first() ; i < ref.size() ; ++i)
      {
        detail::internal_apply_increment(ref.reference(i),get_ptr(refdelta(i)));
      }
    }
  };
  
  struct LambdaDiag
  {
    double lambda;
    LambdaDiag(double lambda_):lambda(lambda_) {}
    template<class T> void operator()(T& table) const
    {
      table.augment_diag(lambda);
    }
  };
  
  template<class Tie, class Op> struct TieOp
  {
    Tie tie;
    Op op;
    TieOp(Tie tie_, Op op_):tie(tie_),op(op_) {}

    template<class T> void operator()(ttt::wrap<T>) {}
    
    template<class Key1, class Key2> void operator()
    (
      ttt::wrap<Product<Key1,Key2>>,
      typename boost::disable_if<
                                  mpl::not_<
                                            br::has_key<
                                                          typename ttt::rm_all<
                                                                                typename std::tuple_element<0,Tie>::type
                                                                              >::type,
                                                          typename Product<Key1,Key2>::type
                                                        >
                                          >
                                >::type* =0// trig sup éparse de S non géré dans S = Ws * WsT
    )
    {
      op(
          bf::at_key<typename Product<Key1,Key2>::type>(std::get<0>(tie)),
          bf::at_key<typename TransToPair<Key1>::type>(std::get<1>(tie)),
          bf::at_key<typename TransToPair<Key2>::type>(std::get<2>(tie))
        );
    }

    template<class Key1> void operator()(ttt::wrap<Unary<Key1>>)
    {
      op(bf::at_key<Key1>(std::get<0>(tie)));
    }
  
    template<class Key1, class Key2> void operator()(ttt::wrap<Binary<Key1,Key2>>)
    {
      op(
            bf::at_key<Key1>(std::get<0>(tie)),
            bf::at_key<Key2>(std::get<1>(tie))
        );
    }
  };
  
  template<class Keys, class Op, class Tie> void for_each(Tie tie, Op op)
  {
    mpl::for_each<typename Keys::type,wrap_>(TieOp<Tie,Op>(tie,op));
  }

}// eon

#endif
