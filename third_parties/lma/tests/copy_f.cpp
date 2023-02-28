#include <libv/lma/ttt/fusion/copy_f.hpp>
#include <libv/lma/ttt/traits/to_ref.hpp>
#undef NDEBUG // make assert() work
#include <cassert>

struct AssignPointerOrValueToValue
{
  template<class A, class B> inline void operator()(A& a, const B& b) const
  {
    a = ttt::to_ref(b);
  }
};

using namespace boost;

int main()
{
  int a = 1;
  float b = 2;
  double c = 3;

  //! tuple de pointeur -> tuple de valeur
  auto tuple = fusion::make_tuple(0,0.0f,0.0);
  ttt::copy_f(tuple,fusion::make_tuple(&a,&b,&c),AssignPointerOrValueToValue());
  assert((tuple == fusion::make_tuple(1,2.0f,3.0)));

  //! tuple de valeur -> tuple de valeur
  tuple = fusion::make_tuple(0,0.0f,0.0);
  ttt::copy_f(tuple,fusion::make_tuple(4,5.0f,6.0),AssignPointerOrValueToValue());
  assert((tuple == fusion::make_tuple(4,5.0f,6.0)));

  return EXIT_SUCCESS;
}
