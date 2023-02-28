#include <iostream>
#include <libv/lma/lm/ba/meta_prod.hpp>
#include <libv/lma/ttt/fusion/pair.hpp>

struct A{};
struct B{};
struct C{};
struct D{};

// template<class T> struct Vector {};
// template<class A, class B> struct Table {};

// typedef bf::map<
//                     bf::pair<A,Vector<A>>,
//                     bf::pair<B,Vector<B>>,
//                     bf::pair<C,Vector<C>>,
//                     bf::pair<D,Vector<D>>
//                    > Map;
// 
// typedef bf::map<
//                  bf::pair<bf::pair<A,A>,Table<A,A>>, bf::pair<bf::pair<A,B>,Table<A,B>> , bf::pair<bf::pair<A,C>,Table<A,C>>, bf::pair<bf::pair<A,D>,Table<A,D>>,
//                                                      bf::pair<bf::pair<B,B>,Table<B,B>> , bf::pair<bf::pair<B,C>,Table<B,C>>, bf::pair<bf::pair<B,D>,Table<B,D>>,
//                                                                                           bf::pair<bf::pair<C,C>,Table<C,C>>,
//                                                                                                                               bf::pair<bf::pair<D,D>,Table<D,D>>
//                > Hessian;





namespace bf = boost::fusion;
namespace mpl = boost::mpl;

typedef mpl::vector<bf::pair<A,C>,bf::pair<A,D>,bf::pair<B,C>,bf::pair<B,D>> TypeWs;
typedef mpl::vector<bf::pair<C,C>,bf::pair<D,D>> TypeVs;
typedef mpl::vector<C,D> TypeEb;
typedef mpl::vector<A,B> TypeDa;

namespace ttt
{
  template<> struct Name<A> { static const std::string name() { return std::string("A"); } };
  template<> struct Name<B> { static const std::string name() { return std::string("B"); } };
  template<> struct Name<C> { static const std::string name() { return std::string("C"); } };
  template<> struct Name<D> { static const std::string name() { return std::string("D"); } };
}

int main()
{

  // W * V
//   typedef lma::MetaProd<TypeWs,TypeVs>::type List0;
//   std::cout << ttt::name<List0>() << std::endl;
// 
//   // Y * Eb
//   typedef lma::MetaProd<TypeWs,TypeEb>::type List1;
//   std::cout << ttt::name<List1>() << std::endl;
//   
//   // Wt * Da
// //   typedef lma::MetaProd<lma::Transpose<TypeWs>,TypeDb>::type List2;
// //   std::cout << ttt::name<List2>() << std::endl;
//   
//   // V * eb
//   typedef lma::MetaProd<TypeVs,TypeEb>::type List3;
//   std::cout << ttt::name<List3>() << std::endl;
//   
//   // Y * Wt
//   typedef lma::MetaProd<TypeWs,lma::Transpose<TypeWs>>::type List4;
//   std::cout << ttt::name<List4>() << std::endl;
  
  return 0;
}