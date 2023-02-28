#include <boost/fusion/include/make_vector.hpp>
#include <libv/lma/lm/function/function.hpp>
#include <libv/lma/global.hpp>

struct A{};
struct B{};
struct C{};

struct F
{
  bool operator()(A , const B& , const C, double & res) const
  {
    res = 3.159;
    return true;
  }
};

using namespace lma;

int main()
{

  double d;
  F f;
  Function<F> fun(f);
  A a;
  B b;
  C c;
  return (fun(bf::make_vector(&a,&b,&c),d) && d == 3.159 ? EXIT_SUCCESS : EXIT_FAILURE);
}

