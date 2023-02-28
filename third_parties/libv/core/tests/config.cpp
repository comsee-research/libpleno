/// \example
/// Basic use of the \ref config module.
/// \author Alexis Wilhelm (2012)
/// \privatesection

#include <iostream>

#include <libv/core/test.hpp>
#include <libv/core/serialization/associative_containers.hpp>
#include <libv/core/serialization/contiguous_containers.hpp>
#include <libv/core/serialization/eigen.hpp>
#include <libv/core/serialization/serializable_properties.hpp>

using namespace v;

V_DEFINE_PROPERTIES(SubConfig)
(
  V_DEFINE_PROPERTY(z, 3, 0)
)

V_DEFINE_CONFIG
(
  V_DEFINE_PROPERTY(a, 3, 0)
  V_DEFINE_PROPERTY(b, 4.2, 0)
  V_DEFINE_PROPERTY(c, SubConfig(), 0)
  V_DEFINE_PROPERTY(e, (std::map<std::string, int>()), 0)
  V_DEFINE_PROPERTY(f, (std::map<std::string, SubConfig>()), 0)
  V_DEFINE_PROPERTY(g, std::vector<int>(), 0)
  V_DEFINE_PROPERTY(h, std::vector<SubConfig>(), 0)
  V_DEFINE_PROPERTY(i, Eigen::Matrix3d(), 0)
  V_DEFINE_PROPERTY(j, Eigen::Array22f(), 0)
  V_DEFINE_PROPERTY(k, std::string("a"), 0)
  V_DEFINE_PROPERTY(l, std::vector<std::vector<int> >(), 0)
)

int main(int, char **argv)
{
  const std::string path = argv[1];

  Config config = Config().a(5).b(-2.3);
  V_TEST_EQUAL(config.a(), 5);
  V_TEST_EQUAL(config.b(), -2.3);

  load(path + "/config.ini", config);
  pretty_print(std::cerr, config);
  V_TEST_EQUAL(config.a(), 6);
  V_TEST_EQUAL(config.b(), -1.1);
  V_TEST_EQUAL(config.c().z(), 1);
  V_TEST_EQUAL(config.e().find("z")->second, -1);
  V_TEST_EQUAL(config.f().find("z")->second.z(), -3);
  V_TEST_EQUAL(config.g().back(), 8);
  V_TEST_EQUAL(config.h().front().z(), 7);
  V_TEST_EQUAL(config.i()(1, 2), 1.6);
  V_TEST_EQUAL(config.j()(1, 0), 1.5);
  V_TEST_EQUAL(config.k(), "lorem ipsum");
  V_TEST_EQUAL(config.l()[1][0], 4);
}
