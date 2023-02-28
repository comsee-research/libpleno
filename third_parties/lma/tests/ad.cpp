#ifdef USE_TOON

#include <libv/lma/numeric/ad/rt/ad.hpp>

using namespace AdRt;

int main()
{

  typedef Ad<double,2> Type;

  Type x(2,0);
  Type y(3,1);
  Type c = (1.0 - x) * (1.0 - x) + 100.0 * (y - x*x) * (y - x*x);

  std::cout << " Rosenbrock " << std::endl;
  std::cout << " x = " << x << std::endl;
  std::cout << " y = " << y << std::endl;
  std::cout << " c = " << c << std::endl;
  std::cout << " rosenbrock(" << x.value << "," << y.value << ")= " << c.value << std::endl;
  std::cout << " dx = " << c.infinite[0] << std::endl;
  std::cout << " dy = " << c.infinite[1] << std::endl;

  return 0;
}

#else

int main()
{
}

#endif

