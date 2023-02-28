#include <libv/lma/lm/ba/create_hessian.hpp>
#include <iostream>

using namespace lma;


struct A
{
  bool operator()(int,int,double,char, int&) const
  {
    return true;
  }
};
struct B
{
  bool operator()(int,double,double,char,char,float, int&) const
  {
    return true;
  }
};

namespace ttt{ template<> struct Name<A>{ static std::string name(){ return "A"; } }; }
namespace ttt{ template<> struct Name<B>{ static std::string name(){ return "B"; } }; }



int main()
{
//   std::cout << ttt::name<ListeParam>() << std::endl;
  typedef mpl::vector<A,B> L1;
  std::cout << " Functors : " << ttt::name<L1>() << std::endl;
  typedef ListOfListOfParameters<L1>::type L2;
  std::cout << " ListOfListOfParameters : " << ttt::name<L2>() << std::endl;
  typedef CrossParameters<L2>::type L3;
  std::cout << " CrossParameters : " << ttt::name<L3>() << std::endl;
  typedef ToSingleOrDouble<L2>::type L4;
  std::cout << " ToSingleOrDouble : " << ttt::name<L4>() << std::endl;
  std::cout << std::endl;
  typedef CatCrossAndDiag<L3,L4>::type L5;
  std::cout << " CrossAndDiag : " << ttt::name<L5>() << std::endl;
  std::cout << std::endl;
//   typedef Containers<L5,>::type L6;
//   std::cout << " Containers : " << ttt::name<L6>() << std::endl;
  
  
}