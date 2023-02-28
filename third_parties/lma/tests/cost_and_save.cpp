#include <libv/lma/numeric/divers.hpp>
#include <libv/lma/lma.hpp>

using namespace lma;

struct F1
{
  bool operator()(const double& b, const double& c, double& r) const
  {
    r = b * c;
    return true;
  }
};

struct F2
{
  bool operator()(const double& a, const double& c, double& r) const
  {
    r = a * c;
    return true;
  }
};
  
namespace lma
{
  struct DispErrors : enable_verbose_output
  {
    template<class S, class Algo>
    void at_end_bundle_adjustment_iteration(const S& solver, const Algo& algo) const
    {
      auto& vf1 = algo.map_erreur.template at_key<F1>();
      auto& vf2 = algo.map_erreur.template at_key<F2>();
      double e1(0),e2(0);
      for(auto& x1 : vf1)
        e1 += squared_norm(x1.first);
      for(auto& x2 : vf2)
        e2 += squared_norm(x2.first);
      std::cout << " error : " << (e1 + e2)/2.0 << std::endl;
      enable_verbose_output::at_end_bundle_adjustment_iteration(solver,algo);
    }
  };
}

template<class Solver, class AlgoTag, class V0, class V1, class V2> void solve(V0 v0, V1 v1, V2 v2)
{
  Solver solver(1,5);

  for(size_t i = 0 ; i < v1.size() ; ++i)
    for(size_t j = 0 ; j < v2.size() ; ++j)
      solver.add(F1(),&v1[i],&v2[j]);

  for(size_t i = 0 ; i < v0.size() ; ++i)
    for(size_t k = 0 ; k < v2.size() ; ++k)
      solver.add(F2(),&v0[i],&v2[k]);

  solver.solve(AlgoTag(),lma::DispErrors());
}


int main()
{
  size_t size0 = 30;
  size_t size1 = 10;
  size_t size2 = 100;

  std::vector<double,Eigen::aligned_allocator<double>> v0(size0);
  std::vector<double,Eigen::aligned_allocator<double>> v1(size1);
  std::vector<double,Eigen::aligned_allocator<double>> v2(size2);

  for(auto& x : v0)
    x = random(1.0);

  for(auto& x : v1)
    x = random(1.0);
  
  for(auto& x : v2)
    x = random(1.0);

  typedef Solver<F1,F2> Solver;
  solve<Solver,DENSE_>(v0,v1,v2);

  return 0;
}
