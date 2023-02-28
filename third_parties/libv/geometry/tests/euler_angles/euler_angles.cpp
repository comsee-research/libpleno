#include <cstdlib>
#include <libv/geometry/euler_angles/euler_angles.hpp>

using namespace v;

int main()
{

  EulerAngles v1(1.0,2.0,3.0);

  // Accesor and constructor test
  if(v1.a() != 1.0 || v1.b() != 2.0 || v1.g() != 3.0) return EXIT_FAILURE;

  // Assignement test
  v1.a(5.0);
  v1.b(70.0);
  v1.g(14.0);
  if(v1.a() != 5.0 || v1.b() != 70.0 || v1.g() != 14.0) return EXIT_FAILURE;

  return EXIT_SUCCESS;

}
