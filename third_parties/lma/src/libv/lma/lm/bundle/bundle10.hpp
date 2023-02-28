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

#ifndef __OPTIMISATION2_BUNDLE_BUNDLE10_HPP__
#define __OPTIMISATION2_BUNDLE_BUNDLE10_HPP__

#include <libv/lma/ttt/traits/functor_trait.hpp>
#include <libv/lma/ttt/traits/rm_all.hpp>
#include <libv/lma/ttt/traits/unroll1.hpp>
#include "../function/function.hpp"
#include "bundle_utils.hpp"
#include <boost/fusion/include/fold.hpp>
#include <boost/fusion/include/value_at_key.hpp>
#include <boost/fusion/include/make_vector.hpp>
#include <boost/mpl/for_each.hpp>
#include <boost/mpl/pop_front.hpp>
#include <set>

namespace lma
{
  template<class T> struct SizeInf3 : mpl::if_c< (mpl::size<T>::value < 3), mpl::true_, mpl::false_> {};
  
  template<class L, class, class Result = mpl::vector<>> struct MakeSP :
    MakeSP<
            typename mpl::pop_front<L>::type,
            typename SizeInf3<L>::type,
            typename mpl::push_back<
                                    Result,
                                    typename br::as_vector<
                                                          typename mpl::transform<
                                                                                  typename mpl::pop_front<L>::type,
                                                                                  ttt::Indice<bf::pair<mpl::_1,typename mpl::front<L>::type>>
                                                                                >::type
                                                        >::type
                                    >::type
          > {};
  
  template<class L, class R> struct MakeSP< L, mpl::true_ , R >  { typedef R type;};

  template<class T> struct MakeSP2
  {
    template<class L> struct TTT : mpl::transform<L, br::as_vector<mpl::_1>> {};
    
    typedef typename MakeSP<T,SizeInf3<T>>::type list;
    typedef typename mpl::transform<
                                      list,
                                      br::as_vector<
                                                     mpl::_1
                                                   >
                                   >::type list2;
    typedef typename br::as_vector<list2>::type type;
  };
  
  
  template<class ListFunction, class Fonctor> struct MAPZZ2
  {
    typedef typename mpl::transform<ListFunction,CreateListArg<mpl::_1>>::type l0;
    typedef typename mpl::transform<l0, Fonctor>::type l1;
    
    template<class T> struct ToIndicesVector : br::as_vector<typename mpl::transform<T,ttt::Indice<mpl::_1>>::type> {};
    template<class T> struct ToParamsVector : br::as_vector<typename mpl::transform<T,mpl::_1>::type> {};

    typedef typename mpl::transform<
                                      l1,
                                      ListFunction,
                                      bf::pair<
                                                mpl::_2,
                                                std::vector<
                                                            std::pair<
                                                                        ToIndicesVector<mpl::_1>,
                                                                        MakeSP2<mpl::_1>
                                                                      >
                                                           >
                                              >
                                   >::type MapSparseIndices;

    typedef typename mpl::transform<
                                      l1,
                                      ListFunction,
                                      bf::pair<
                                                mpl::_2,
                                                std::vector<
                                                              ToParamsVector<mpl::_1>
                                                           >
                                              >
                                   >::type MapSparseParameters;
                                   
    typedef typename br::as_map<MapSparseIndices>::type Container;
    
    typedef typename br::as_map<MapSparseParameters>::type ContainerParameters;
    
    template<class F> struct Second
    {
      typedef typename br::value_at_key<Container,F>::type Vector;
      typedef typename Vector::value_type::second_type type;
    };
    
    template<class F> struct First
    {
      typedef typename br::value_at_key<Container,F>::type Vector;
      typedef typename Vector::value_type::first_type type;
    };
    
    Container container;
    
    ContainerParameters parameters;
    /*
     * container = 
     * br::map
     *        <
     *         Key = Function,
     *         Value = std::vector<
     *                             std::pair<
     *                                        first = br::vector< Indice<Function_Parametre1>,...,Indice<Function_ParametreN> >
     *                                        second = br::vector< 
     *                                                             br::vector< ttt::Indice<Sparse<Function_Parametre1,Function_Parametre2>>, ttt::Indice<Sparse<Function_Parametre1,Function_Parametre3>>, ..., ttt::Indice<Sparse<Function_Parametre1,Function_ParametreN>> >
     *                                                             br::vector< ttt::Indice<Sparse<Function_Parametre2,Function_Parametre3>>, ttt::Indice<Sparse<Function_Parametre2,Function_Parametre4>>, ..., ttt::Indice<Sparse<Function_Parametre1,Function_ParametreN>> >
     *                                                             ...
     *                                                           >
     *                                      >
     *                            > 
     *        >
     */

    template<class Obs> struct ToVector
    {
      typedef typename br::at_key<Container,Obs>::type type;
      typedef typename std::decay<type>::type naked_type;
      typedef typename boost::add_const<naked_type>::type const_type;
      typedef typename boost::add_reference<const_type>::type const_ref_type;
    };

    template<class Obs> struct ToTuple
    {
      typedef typename ToVector<Obs>::type vector_ref;
      typedef typename std::decay<vector_ref>::type vector;
      typedef typename vector::value_type value_type;

      typedef typename boost::add_const<value_type>::type const_type;
      typedef typename boost::add_reference<const_type>::type const_ref_type;

      typedef typename boost::add_reference<value_type>::type type_ref;

    };

    template<class Obs> struct ToTupleElement : ToTuple<Obs>
    {
      typedef typename ToTuple<Obs>::value_type value_type;
      typedef typename value_type::first_type first_type;
      typedef typename value_type::second_type second_type;

      typedef typename boost::add_const<first_type>::type const_first_type;
      typedef typename boost::add_reference<const_first_type>::type first_const_ref_type;

      typedef typename boost::add_reference<first_type>::type type_first_ref;
      typedef typename boost::add_reference<second_type>::type type_second_ref;

    };


    template<class T> struct ForceAddConst
    {
      typedef typename std::decay<T>::type naked_type;
      typedef typename boost::add_const<naked_type>::type const_type;
      typedef typename boost::add_reference<const_type>::type const_ref_type;
    };

    template<class Key> 
    typename ToVector<Key>::type at()
    { 
      BOOST_MPL_ASSERT((boost::is_same<decltype(bf::at_key<Key>(container)),typename ToVector<Key>::type>));
      return bf::at_key<Key>(container);
    }
    
    template<class Key>
    typename ToTuple<Key>::type_ref pair(ttt::Indice<Key> indice)
    { 
      BOOST_MPL_ASSERT((boost::is_same<decltype(this->at<Key>().at(indice())),typename ToTuple<Key>::type_ref>));
      return this->at<Key>().at(indice());
    }

    template<class Key>
    typename ToTupleElement<Key>::type_first_ref indices(ttt::Indice<Key> indice)
    {
      BOOST_MPL_ASSERT((boost::is_same<decltype(pair(indice).first),typename ToTupleElement<Key>::first_type>));
      return pair(indice).first;
    }
    
    template<size_t I, class Key>
    typename br::at_c<typename ToTupleElement<Key>::first_type,I>::type indice(ttt::Indice<Key> indice)
    {
      BOOST_MPL_ASSERT((boost::is_same<decltype(bf::at_c<I>(pair(indice).first)),typename br::at_c<typename ToTupleElement<Key>::first_type,I>::type>));
      return bf::at_c<I>(pair(indice).first);
    }

    // const
    template<class Key> 
    typename ToVector<Key>::const_ref_type
     xat() const
    { return bf::at_key<Key>(container); }
    
    template<class Key>
    typename ToTuple<Key>::const_ref_type xpair(const ttt::Indice<Key>& indice)  const
    { 
      BOOST_MPL_ASSERT((boost::is_same<typename ToTuple<Key>::const_ref_type,decltype(this->xat<Key>().at(indice()))>));
      return this->xat<Key>().at(indice());
    }
    
    template<class Key>
    typename ToTupleElement<Key>::first_const_ref_type 
    indices(const ttt::Indice<Key>& indice)  const
    { return xpair(indice).first; }
    
    template<size_t I, class Key>
    typename ForceAddConst<typename br::at_c<typename ToTupleElement<Key>::second_type,I>::type>::const_ref_type
    sparse_indices(const ttt::Indice<Key>& indice)  const
    { 
      BOOST_MPL_ASSERT((boost::is_same<typename ForceAddConst<typename br::at_c<typename ToTupleElement<Key>::second_type,I>::type>::const_ref_type,decltype(bf::at_c<I>(xpair(indice).second))>));
      return bf::at_c<I>(xpair(indice).second);
    }
    
    static void disp()
    {
      std::cout << " =======================\n";
      std::cout << "L0 : " << ttt::name<l0>() << std::endl;
      std::cout << "L1 : " << ttt::name<l1>() << std::endl;
      std::cout << "MapSparseIndices : " << ttt::name<MapSparseIndices>() << std::endl;
      std::cout << "MapSparseParameters : " << ttt::name<MapSparseParameters>() << std::endl;
      
      std::cout << " =======================\n";
    }
  };
  
  
  
  template<class Set, class Tuple> struct AddSet2
  {
    Set& set;
    Tuple& tuple;
    AddSet2(Set& set_,Tuple& tuple_):set(set_),tuple(tuple_){}
    
    template<size_t I> void operator()(Int<I> const &)
    {
      // ajoute un parametre à la map correspondante
      //map[KeyParam].add(&param)
      bf::at_key<typename br::value_at_c<Tuple,I>::type>(set)[bf::at_c<I>(tuple)];
    }
  };
  
  template<class Set, class Parameters> struct AddSet
  {
    Set& set;
    Parameters& parameters;
    AddSet(Set& set_,Parameters& parameters_):set(set_),parameters(parameters_){}
    
    template<class Obs> void operator()(ttt::wrap<Obs> const &)
    {
      auto& vector = bf::at_key<Obs>(parameters);
      typedef typename ttt::rm_all<decltype(vector)>::type V;
      typedef typename V::value_type Tuple;
      
      for(auto& tuple : vector)
        // on ajoute chaque paramètre du tuple à la map
        // on utilise unroll plutôt que for_each<Types..> car un tuple peut contenir plusieurs paramètres de mêmes types.
        ttt::unroll<0,mpl::size<typename ttt::rm_all<decltype(tuple)>::type>::value>(AddSet2<Set,Tuple>(set,tuple));
    }
  };
  
  template<class Set, class Bundle> struct AddParam
  {
    Set& set;
    Bundle& bundle;
    
    AddParam(Set& set_, Bundle& bundle_):set(set_),bundle(bundle_){}
    
    template<class Param> void operator()(ttt::wrap<Param> const &)
    {
      // ajout des paramètres au bundle et mise à jour de l'indice
      for(auto& elt : bf::at_key<Param>(set))
        elt.second = bundle.opt_container.add(elt.first);
    }
  };
  
  template<class Set> struct AdressToIndice
  {
    const Set& set;
    AdressToIndice(const Set& set_):set(set_){}
    template<class A, class B> inline void operator()(A& a, const B& b) const { a = bf::at_key<B>(set).at(b); }
  };
  
  template<class Set, class Bundle> struct UpdateVab
  {
    const Set& set;
    Bundle& bundle;
    UpdateVab(const Set& set_, Bundle& bundle_):set(set_),bundle(bundle_){}
    
    template<class Obs> void operator()(ttt::wrap<Obs> const &)
    {
      typename Bundle::MapZZ2::template ToTupleElement<Obs>::first_type tuple;
      auto& vector = bf::at_key<Obs>(bundle.spi2.parameters);
      for(size_t i = 0 ; i < vector.size() ; ++i)
      {
        // tuple d'adresses -> tuple d'indices
        copy_f(tuple,vector[i],AdressToIndice<Set>(set));
        detail::AddTuple::add<Obs>(bundle,tuple);
      }
    }
  };
  
  template<class Set, class Bundle> struct UpdateVab2
  {
    const Set& set;
    Bundle& bundle;
    UpdateVab2(const Set& set_, Bundle& bundle_):set(set_),bundle(bundle_){}
    
    template<class Obs> void operator()(ttt::wrap<Obs> const &)
    {
      typename Bundle::MapZZ2::template ToTupleElement<Obs>::first_type tuple;
      auto& vector = bf::at_key<Obs>(bundle.spi2.parameters);
      for(size_t i = 0 ; i < vector.size() ; ++i)
      {
        copy_f(tuple,vector[i],AdressToIndice<Set>(set));
        bundle.spi2.template at<Obs>().emplace_back(tuple,typename Bundle::MapZZ2::template Second<Obs>::type());
        detail::AddTuple::add2<Obs>(bundle,tuple);
      }
    }
  };
  
  template<class Map> struct ForEachSetMax2
  {
    const Map& map;
    ForEachSetMax2(const Map& map_):map(map_){}
    template<class Pair> void operator()(Pair& pair) const
    {
      auto& sic = pair.second;
      typedef typename ttt::rm_all<decltype(sic)>::type Sic;
      typedef typename Sic::first_type A;
      typedef typename Sic::second_type B;
      sic.set_max(bf::at_key<A>(map).size(),bf::at_key<B>(map).size());
    }
  };
  
  template<class Map> struct ForEachSetMax
  {
    const Map& map;
    ForEachSetMax(const Map& map_):map(map_){}
    
    template<class Pair> void operator()(Pair& pair) const
    {
      bf::for_each(pair.second,ForEachSetMax2<Map>(map));
    }
  };
  
  template<class Map> struct ForEachUpdate2
  {
    const Map& map;
    ForEachUpdate2(const Map& map_):map(map_){}
    template<class Pair> void operator()(Pair& pair) const
    {
      auto& sic = pair.second;
      sic.update();
    }
  };
  
  template<class Map> struct ForEachUpdate
  {
    const Map& map;
    ForEachUpdate(const Map& map_):map(map_){}
    
    template<class Pair> void operator()(Pair& pair) const
    {
      bf::for_each(pair.second,ForEachUpdate2<Map>(map));
    }
  };

          
  template<class FL, class ParamFonctor_ = DoNothing<mpl::_1> > struct Bundle
  {
    typedef ParamFonctor_ ParamFonctor;
    typedef Bundle<FL,ParamFonctor> self;

    typedef typename FindOrder<FL,ParamFonctor>::type::list_function ListFunction;
    typedef typename FindOrder<FL,ParamFonctor>::type::list_parameter ListeParam;

    static const std::size_t NbClass = mpl::size<ListeParam>::value;

    typedef MultiContainer<ListeParam,detail::FonctorPairTic<mpl::_1>> MCA;
    typedef MultiContainer<ListFunction,detail::FonctorPairTic<mpl::_1>> MCFonction;
    typedef typename MakeSparseIndiceContainer1<ListeParam>::type VABMap;

    typedef typename mpl::transform<ListFunction,CreateListArg<mpl::_1>>::type ListListArg_;
    typedef typename mpl::transform<ListListArg_,ParamFonctor>::type ListListArg;
    typedef typename mpl::transform<ListListArg,TransformToPair<mpl::_1>>::type VectorVectorParametre;
    
    typedef typename MapMapMaker<ListFunction,VectorVectorParametre,FunctorPairKeyMap>::type MapMapParametre;// use in get_map2
    
    typedef MAPZZ2<ListFunction,ParamFonctor> MapZZ2;
    MapZZ2 spi2;

    public:
    // Container des paramètres (MapOfTIC : bf::map< bf::pair< A,TIC< A > >, .., bp::pair< X, TIC< Z > > >)
    MCA opt_container;
    
    // Container des foncteurs
    MCFonction fonction_container;

    // Conteneur d'indices
    VABMap vab_map;

    Bundle(){
//       MapZZ2::disp();
//       std::cout << " VAB : " << ttt::name<VABMap>() << std::endl;
//       abort();
    }

    template<class MContainer, class Key> struct GetValueType
    {
      typedef typename MContainer::template AtKey<Key>::type ContainerRef;
      typedef typename std::decay<ContainerRef>::type Container;
      typedef typename Container::Type type_;
      typedef typename boost::add_reference<typename boost::add_const<type_>::type>::type type;
    };

    template<class MContainer, class Key> struct GetValueType2
    {
      typedef typename br::at_key<MContainer,Key>::type ContainerRef;
      typedef typename std::decay<ContainerRef>::type Vector;
      typedef typename Vector::value_type type_;
      typedef typename boost::add_reference<typename boost::add_const<type_>::type>::type type;
    };

    //! at_key< Key>() -> TIC< Key >
    template<class Key> typename MCA::template AtKey<Key>::type& at_opt()       { return opt_container.template at_key<Key>(); }
    template<class Key> typename MCA::template AtKey<Key>::const_ref_type& at_opt() const { return opt_container.template at_key<Key>(); }
    template<class Key> typename MCFonction::template AtKey<Key>::type& at_obs() { return fonction_container.template at_key<Key>(); }
    template<class Key> typename MCFonction::template AtKey<Key>::const_ref_type at_obs() const {return fonction_container.template at_key<Key>(); }
    template<class Key> typename GetValueType<MCA,Key>::type opt(const ttt::Indice<Key>& indice) const { return at_opt<Key>()(indice); }
    template<class Key> typename GetValueType<MCFonction,Key>::type obs(const ttt::Indice<Key>& indice) const { return at_obs<Key>()(indice); }




    //! operator< Key1, Key2 >() -> SIC< Key1, Key2 >
    template<class Key1, class Key2> typename BinaryAt<VABMap,Key1,Key2>::type_ref indices()
    { 
      BOOST_MPL_ASSERT((boost::is_reference<typename BinaryAt<VABMap,Key1,Key2>::type_ref>));
      return bf::at_key<Key1>(bf::at_key<Key2>(vab_map));
    }

    template<class Key1, class Key2> typename BinaryAt<VABMap,Key1,Key2>::const_ref_type indices() const
    { 
      return bf::at_key<Key1>(bf::at_key<Key2>(vab_map));
    }

    //! Map of parameters from an observation indice
    //template<class Obs> typename br::value_at_key<MapMapParametre,Obs>::type map(const ttt::Indice<Obs>& indice) const

    template<class Obs> typename GetValueType2<typename MapZZ2::ContainerParameters,Obs>::type map(const ttt::Indice<Obs>& indice) const
    {
      assert(size_t(indice()) < bf::at_key<Obs>(spi2.parameters).size());
      return bf::at_key<Obs>(spi2.parameters)[indice()];
    }

    template<class Obs, class ... Params> void add(const Obs& f, const Params& ... params)
    {
      // stocke le tuple d'adresse dans spi2.container
      bf::at_key<Obs>(spi2.parameters).emplace_back(params...);
      fonction_container.add(f);
    }
    
    bool is_updated = false;
    void update()
    {
      if (is_updated) return ;
      is_updated = true;
      typedef typename br::as_map<
                                  typename mpl::transform<
                                                          ListeParam,
                                                          bf::pair<
                                                                    mpl::_1,
                                                                    std::map<mpl::_1,ttt::Indice<mpl::_1>>
                                                                  >
                                                        >::type
                                >::type MapAdressParameters;
      MapAdressParameters map;
      // liste et ordonne les parameters dans map
      mpl::for_each<ListFunction,ttt::wrap<mpl::_1>>(AddSet<MapAdressParameters,typename MapZZ2::ContainerParameters>(map,spi2.parameters));
      // ajoute les parametres dans le bundle et met à jour map avec les indices correspondants
      mpl::for_each<ListeParam,  ttt::wrap<mpl::_1>>(AddParam<MapAdressParameters,self>(map,*this));
      // update indices relations and sparse_indices
      
      bf::for_each(vab_map,ForEachSetMax<MapAdressParameters>(map));
      /*
      bf::for_each(vab_map,[&map](auto& m)
      {
        bf::for_each(m.second,[&map](auto& pair)
        {
          auto& sic = pair.second;
          typedef typename ttt::rm_all<decltype(sic)>::type Sic;
          typedef typename Sic::first_type A;
          typedef typename Sic::second_type B;
          sic.set_max(bf::at_key<A>(map).size(),bf::at_key<B>(map).size());
        });
      });
      */
      
      mpl::for_each<ListFunction,ttt::wrap<mpl::_1>>(UpdateVab<MapAdressParameters,self>(map,*this));

      bf::for_each(vab_map,ForEachUpdate<MapAdressParameters>(map));
      /*bf::for_each(vab_map,[&map](auto& m)
      {
        bf::for_each(m.second,[&map](auto& pair)
        {
          auto& sic = pair.second;
          sic.update();
        });
      });*/
      
      mpl::for_each<ListFunction,ttt::wrap<mpl::_1>>(UpdateVab2<MapAdressParameters,self>(map,*this));
      
    }
    size_t nb_obs() const { return bf::fold(fonction_container.map(),0,TicCounter()); }

    std::ostream& disp(std::ostream& o) const
    {
      detail::DispName<false> dn(o);
      bf::for_each(opt_container.map(), dn);
      detail::DispName<true> dn2(o);
      bf::for_each(fonction_container.map(), dn2);
      return o;
    }

    const typename self::MCA& clone_opt_container() const{return this->opt_container;}
    void restore_container(const typename self::MCA& mca){this->opt_container = mca;}
  };

}// eon lma

template<class X, class _>
std::ostream& operator<<(std::ostream& o, const lma::Bundle<X,_>& bundle)
{
  bundle.disp(o);
  lma::detail::Print6 p1(o);
  boost::fusion::for_each(bundle.opt_container().map(), p1);
  return o;
}

#endif
