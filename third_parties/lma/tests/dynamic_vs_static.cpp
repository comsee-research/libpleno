#include <Eigen/Core>
#include <tuple>
#include <iostream>
#include <libv/core/time.hpp>
#include <libv/lma/ttt/traits/unroll1.hpp>
#include <libv/lma/ttt/traits/unroll2.hpp>

template<class A, class B, class C> void compute(A& a, const B& b, const C& c)
{
  a = b * c;
}

template<class ... Args> void compute(std::tuple<Args...> tuple)
{
  compute(std::get<0>(tuple),std::get<1>(tuple),std::get<2>(tuple));
}

template<int I, int J> auto statique()
{
  typedef Eigen::Matrix<double,I,I> MatII;
  typedef Eigen::Matrix<double,I,J> MatIJ;
  typedef Eigen::Matrix<double,J,I> MatJI;
  return std::tuple<MatII,MatIJ,MatJI>(MatII::Zero(),MatIJ::Random(),MatJI::Random());
}

template<int I, int J> auto dynamique()
{
  typedef Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> MatII;
  typedef Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> MatIJ;
  typedef Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> MatJI;
  return std::tuple<MatII,MatIJ,MatJI>(MatII::Zero(I,I),MatIJ::Random(I,J),MatJI::Random(J,I));
}

struct Test
{
  template<int I, int J> void operator()() const
  {
    size_t n = 100000;

    double t = v::now();
    for(size_t i = 0 ; i < n ; ++i)
      compute(statique<I+1,J+1>());
    t = v::now() - t;
    std::cout << " Statique <" << I+1 << "," << J+1 << ">  : " << t << std::endl;


    t = v::now();
    for(size_t i = 0 ; i < n ; ++i)
      compute(dynamique<I+1,J+1>());
    t = v::now() - t;
    std::cout << " Dynamique<" << I+1 << "," << J+1 << ">  : " << t << std::endl;
    std::cout << std::endl;
  }

  template<size_t I> void operator()(ttt::Int<I>) const
  {
    static const int J = I;
    size_t n = 100000;

    double t = v::now();
    for(size_t i = 0 ; i < n ; ++i)
      compute(statique<I,J>());
    t = v::now() - t;
    std::cout << " Statique <" << I << "," << J << ">  : " << t << std::endl;


    t = v::now();
    for(size_t i = 0 ; i < n ; ++i)
      compute(dynamique<I,J>());
    t = v::now() - t;
    std::cout << " Dynamique<" << I << "," << J << ">  : " << t << std::endl;
    std::cout << std::endl;
  }

};

using namespace ttt;

int main()
{
  //double_unroll(Int<16>{},Int<16>{},Test{});
  unroll(Test{},Int<1>{},Int<32>{});
  return 0;
}
