#include <cstddef>
#include <iostream>
#include <libv/lma/ttt/traits/unroll1.hpp>
#include <libv/lma/ttt/traits/unroll2.hpp>

struct Foncteur1D
{
  template<std::size_t I> void operator()(ttt::Int<I>)
  {
    std::cout << " 1D : I = " << I << std::endl;
  }
};

struct Foncteur2D
{
  template<std::size_t I, std::size_t J> void operator()()
  {
    std::cout << " 2D : I,J = " << I << "," << J << std::endl;
  }
};

int main()
{
  static const std::size_t M = 10;
  static const std::size_t N = 5;

  Foncteur1D fonc1d;
  ttt::unroll<0,M>(fonc1d);

  Foncteur2D fonc2d;
  ttt::double_unroll(ttt::Int<M>(),ttt::Int<N>(),fonc2d);

  return EXIT_SUCCESS;
}

