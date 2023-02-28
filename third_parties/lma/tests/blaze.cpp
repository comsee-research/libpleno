#if defined(USE_TOON) && defined(USE_BLAZE)

#include <libv/lma/lm/container/blaze.hpp>

#include <libv/lma/lma.hpp>



template<class Tag> void test1(double x)
{
  struct F{ bool operator()(const double& x, double& r) const { r = x - 10; return true; } };
  lma::Solver<F>().add(F{},&x).solve(lma::LdltTag<Tag,double>{});
  std::cout << ttt::name<Tag>() << "\t: " << x << std::endl;
}

template<class T> void test1()
{
  test1<lma::Eig>(0);
  test1<lma::Toon>(0);
  test1<lma::Blaze>(0);
}




template<class Tag> void test2(Eigen::Vector3d x)
{
  struct F{ bool operator()(const Eigen::Vector3d& x, double& r) const { r = x.squaredNorm() - 10; return true; } };
  lma::Solver<F>().add(F{},&x).solve(lma::LdltTag<Tag,double>{});
  std::cout << ttt::name<Tag>() << "\t: " << x.transpose() << std::endl;
}


int main()
{
  test1<void>();
  // test2<lma::Eig>(Eigen::Vector3d::Zero());
  // test2<lma::Toon>(Eigen::Vector3d::Zero());
  //test2<lma::Blaze>(Eigen::Vector3d::Ones());
}

#else
int main() {}
#endif
