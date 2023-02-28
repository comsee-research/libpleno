#ifdef USE_BRIGAND

#include <brigand/brigand.hpp>
//#include <brigand/types/integer.hpp>
#include <iostream>

#include <libv/lma/ttt/traits/naming.hpp>

struct A{};
struct B{};
struct C{};
struct D{};
struct E{};
struct F{};
struct G{};

namespace ttt
{
  template<class T> struct Name<brigand::type_<T>>
  {
    static std::string name()
    {
      return std::string("brigand::type_<") + color.bold() + color.red() + ttt::name<T>() + color.reset() + ">";
    }
  };

  template<class A, class B> struct Name<brigand::pair<A,B>>
  {
    static std::string name()
    {
      return std::string("brigand::pair<") + ttt::name<A>() + "," + ttt::name<B>() + ">";
    }
  };

  template<template<class ...> class List, class ... T> struct Name<List<T...>>
  {
    static std::string name(){
      std::string str;
      brigand::for_each<brigand::list<T...>>([&str]
        (auto t) { str += ttt::name<decltype(t)>() + ","; });
     return str ; }
  };

  template<class T, T value> struct Name<std::integral_constant<T,value>>
  {
    static std::string name()
    {
      return std::string("std::IntegralConstant<")+ttt::name<T>()+","+std::to_string(value)+">";
    }
  };

  template<> struct Name<A>{ static std::string name(){ return "A";}};
  template<> struct Name<B>{ static std::string name(){ return "B";}};
  template<> struct Name<C>{ static std::string name(){ return "C";}};
  template<> struct Name<D>{ static std::string name(){ return "D";}};
  template<> struct Name<E>{ static std::string name(){ return "E";}};
  template<> struct Name<F>{ static std::string name(){ return "F";}};
  template<> struct Name<G>{ static std::string name(){ return "G";}};

}

template<class A, class B> struct Key {};
template<class A, class B> struct Pair { B second; };


template<class X, class State, class Element> struct F1
{

  template<class Result, class Element2, class Elt>
  struct F2 { using type = brigand::push_back<Result,brigand::pair<brigand::pair<Element2,Elt>,void>>; };

  using new_list = 
    brigand::fold<
                  X,brigand::list<>,
                  F2<brigand::_state,Element,brigand::_element>
                 >;
  using type = brigand::append<State,new_list>;
};


template<class ... X> struct as_map;
template<class ... X> struct as_map<brigand::list<X...>>
{
  using type = brigand::map<X...>;
};

int main()
{
  using All = brigand::list<A,B,C>;

  using Result  = brigand::fold<All,brigand::list<>,F1<All,brigand::_state,brigand::_element>>;

  using Map = typename as_map<Result>::type;

  std::cout << " List : " << ttt::name<All>() << std::endl;
  std::cout << std::endl;
  std::cout << " Keys : " << ttt::name<Result>() << std::endl;
  std::cout << std::endl;
  std::cout << " Map  : " << ttt::name<Map>() << std::endl;
  //clement(Map{});

}

#else
int main() {}
#endif

