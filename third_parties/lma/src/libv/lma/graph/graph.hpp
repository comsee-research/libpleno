#pragma once

#include <boost/fusion/include/vector.hpp>
#include <boost/fusion/include/find.hpp>
#include <boost/mpl/vector.hpp>
#include <libv/lma/ttt/mpl/cat.hpp>
#include <libv/lma/lma.hpp>
#include "nodes.hpp"

namespace lma
{

  struct Constraints {};
  struct Parameters {};

  template<typename, typename> struct Graph;

  template<typename ... C, typename ... P>
  struct Graph<Constraints(C...),Parameters(P...)> 
  {
    struct DataGraph : boost::fusion::vector<C...,P...>
    {
      template<class T> operator T& () { return *boost::fusion::find<T>(*this); }
      template<class T> operator const T& () { return *boost::fusion::find<T>(*this); }
      template<class T> operator const T& () const { return *boost::fusion::find<T>(*this); }
    };

    DataGraph graph;
    DataGraph& node() { return graph;}

    struct ExtractFunctors
    {
      template<typename T> struct GetFunctors : T::functors {};
      using folded = boost::mpl::fold<boost::mpl::vector<C...>,boost::mpl::vector<>,ttt::Cat<boost::mpl::_1,GetFunctors<boost::mpl::_2>>>;
      using type = typename boost::fusion::result_of::as_vector<typename folded::type>::type;
    };

    struct ExtractParameters
    {
      template<typename V, typename T> struct GetParameter : boost::mpl::push_back<V,typename T::type> {};
      using type = typename boost::mpl::fold<boost::mpl::vector<P...>,boost::mpl::vector<>,GetParameter<boost::mpl::_1,boost::mpl::_2>>::type;
    };

    template<typename ...> struct CreateSolver;
    template<template <typename ...> typename V, typename ... T, typename ... Optional> struct CreateSolver<V<T...>,Optional...> 
      { using type = lma::Solver<Optional...,T...>; };
    using Solver = typename CreateSolver<typename ExtractFunctors::type, typename ExtractParameters::type>::type;

    void cover(auto& node)
    {
      //node.cover(*this);
      _cover(*this,node);
    }

    void solve(auto ... options)
    {
      boost::fusion::for_each(graph,[this](auto& node){node.covered=false;});
      Solver solver;
      boost::fusion::for_each(graph,[this](auto& node){_cover(*this,node);});
      boost::fusion::for_each(graph,[this,&solver](auto& node){fill_solver(node,solver);});
      solver.solve(options...);
      boost::fusion::for_each(graph,[this](auto& node){post_solve(*this,node);});
	  }
  };

}
	