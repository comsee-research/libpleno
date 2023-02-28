#include <libv/lma/ttt/traits/to_ref.hpp>
#include <libv/lma/ttt/traits/clement.hpp>
#include <cstdlib>
#include <type_traits>
#undef NDEBUG // make assert() work
#include <cassert>

using namespace ttt;

int main()
{

  int a = 5;
  const int b = 5;
  int *p = &a;
  const int *q = &a;

  //static_assert(std::is_same<int,decltype(to_ref(1))>::value, "");

  static_assert(std::is_same<int&,decltype(to_ref(a))>::value, "");
  static_assert(std::is_same<const int&,decltype(to_ref(b))>::value, "");

  static_assert(std::is_same<int&,decltype(to_ref(p))>::value, "");
  static_assert(std::is_same<const int&,decltype(to_ref(q))>::value, "");

  assert( &b == &to_ref(b));
  assert( &a == &to_ref(p));

  int& c = to_ref(p);
  assert( &c == &a );
  return EXIT_SUCCESS;
}
