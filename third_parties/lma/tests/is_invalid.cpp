#include <libv/lma/lm/container/container.hpp>
#include <libv/core/test.hpp>

using namespace lma;

int main()
{

  V_TEST_EQUAL(is_invalid(1.0),false);
  V_TEST_EQUAL(is_invalid(-1.1456),false);
  V_TEST_EQUAL(is_invalid(0.0),false);
  V_TEST_EQUAL(is_invalid(1.0/0.),true);
  V_TEST_EQUAL(is_invalid(NAN),true);
  V_TEST_EQUAL(is_invalid(INFINITY),true);


  return EXIT_SUCCESS;
}
