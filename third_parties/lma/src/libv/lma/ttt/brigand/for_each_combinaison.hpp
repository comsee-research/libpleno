/* runtime version of for_each_combinaison

#include <vector>
#include <iostream>
typedef std::vector<int> Vector;
typedef std::vector<std::vector<int>> Vectors;

template<class F>
void repeat(F functor, int i, const Vectors& vectors, Vector n_uplet)
{
  if (i!=vectors.size())
    for(size_t j = 0 ; j < vectors[i].size() ; ++j)
    {
      n_uplet[i] = vectors[i][j];
      repeat(functor,i+1,vectors,n_uplet);
    }
  else
    functor(n_uplet);
}

template<class F>
void for_each_combinaison(F functor, const Vectors& vectors)
{
  repeat(functor,0,vectors,Vector(vectors.size(),0));
}

struct Disp {
  void operator()(const Vector& v) const {
    for(size_t i = 0 ; i < v.size() ; ++i)
      std::cout << v[i] << ",";
    std::cout << std::endl;
  }};

int main()
{
  Vectors w {{1,2},{3,4,5},{6,7,8,9}};
  for_each_combinaison(Disp(),w);
}
*/

// compile time version of for_each_combinaison

#ifndef __TTT_BRIGAND_FOR_EACH_COMBINAISON_HPP__
#define __TTT_BRIGAND_FOR_EACH_COMBINAISON_HPP__

#ifdef USE_BRIGAND

#include <brigand/brigand.hpp>
#include <brigand/types/integer.hpp>
#include <brigand/types/type.hpp>
#include <iostream>

namespace brigand
{
  using stop_ = std::false_type;
  using continue_ = std::true_type;

  brigand::size_t<0> zero {};
  brigand::size_t<1> one {};

  template<class ... T> transform<list<T...>,type_<_1>> remove_type(T...) { return {}; }
  template<class V, V v1, V v2> std::integral_constant<V,v1+v2> plus_(std::integral_constant<V,v1>, std::integral_constant<V,v2>) { return {}; }

  template<class V, V v> std::true_type equal_(std::integral_constant<V,v>, std::integral_constant<V,v>) { return {}; }
  template<class V, V v1, V v2> std::false_type equal_(std::integral_constant<V,v1>, std::integral_constant<V,v2>) { return {}; }

  template<class V, V v> std::false_type not_equal_(std::integral_constant<V,v>, std::integral_constant<V,v>) { return {}; }
  template<class V, V v1, V v2> std::true_type not_equal_(std::integral_constant<V,v1>, std::integral_constant<V,v2>) { return {}; }

  template<class L, class I> brigand::at<L,I> at_(L, I) { return {}; }
  void for_each(auto L, auto f) { brigand::for_each<decltype(L)>(f); }

  template<class ... L> brigand::size_t<brigand::size<brigand::list<L...>>::value> size_(brigand::list<L...>) { return {};}
  template<class ... L> brigand::list<L...> list_(L...) { return {};}
  template<class ... T> void expand(auto f, list<T...>) { f(T{}...); }
}

namespace ttt
{
  void repeat(auto f, auto, auto, std::false_type, auto ... t)
  {
    brigand::expand(f,brigand::remove_type(t...));
  }

  void repeat(auto f, auto lists, auto i, std::true_type, auto ... n_uplets)
  {
    brigand::for_each(at_(lists,i),[&](auto type)
    {
      repeat(f,lists,brigand::plus_(i,brigand::one),brigand::not_equal_(brigand::plus_(i,brigand::one),brigand::size_(lists)),n_uplets...,type);
    });
  }

  void for_each_combinaison(auto lists, auto f)
  {
    repeat(f,lists,brigand::zero,brigand::not_equal_(brigand::one,brigand::size_(lists)));
  };
}

#endif // #ifdef USE_BRIGAND

#endif
