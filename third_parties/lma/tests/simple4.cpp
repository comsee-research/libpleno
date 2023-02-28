#define BOOST_MPL_LIMIT_VECTOR_SIZE 50
#include <libv/lma/numeric/divers.hpp>
#include <libv/lma/lma.hpp>

using namespace lma;

typedef double type;
struct A : Eigen::Matrix<type,1,1> {};
struct B : Eigen::Matrix<type,1,1> {};
struct C : Eigen::Matrix<type,1,1> {};
struct D : Eigen::Matrix<type,1,1> {};


namespace ttt
{
  template<> struct Name<A> { static std::string name(){ return "A"; } };
  template<> struct Name<B> { static std::string name(){ return "B"; } };
  template<> struct Name<C> { static std::string name(){ return "C"; } };
  template<> struct Name<D> { static std::string name(){ return "D"; } };
}
  
  struct F1
  {
    bool operator()(const A& a0, const A& a, const B& b, const C& c, type& r) const
    {
      r = a0.x() - a.x() * b.x() * c.x();
      return true;
    }
  };
  
  struct F2
  {
    bool operator()(const A& a, const B& b, const D& d, type& r) const
    {
      r = a.x() * b.x() * d.x();
      return true;
    }
  };

namespace lma
{
  template<> struct Size<A> { static const size_t value = 1; };
  template<> struct Size<B> { static const size_t value = 1; };
  template<> struct Size<C> { static const size_t value = 1; };
  template<> struct Size<D> { static const size_t value = 1; };
};



template<class Solver, class Tag, class V0, class V1, class V2, class V3> void solve(Tag tag, V0 v0, V1 v1, V2 v2, V3 v3)
{
  Solver bundle(1,1);

  for(size_t i = 0 ; i < v0.size()-1 ; ++i)
    for(size_t j = 0 ; j < v1.size() ; ++j)
      for(size_t k = 0 ; k < v2.size() ; ++k)
	bundle.add(F1(),&v0[i],&v0[(i+1)],&v1[j],&v2[k]);
  
  for(size_t i = 0 ; i < v0.size() ; ++i)
    for(size_t j = 0 ; j < v1.size() ; ++j)
      for(size_t k = 0 ; k < v3.size() ; ++k)
	bundle.add(F2(),&v0[i],&v1[j],&v3[k]);

  bundle.solve(tag,lma::enable_verbose_output());//lma::minimal_verbose());
}


int main()
{
  size_t size0 = 10;
  size_t size1 = 10;
  size_t size2 = 10;
  size_t size3 = 10;

  std::vector<A,Eigen::aligned_allocator<A>> v0(size0);
  std::vector<B,Eigen::aligned_allocator<B>> v1(size1);
  std::vector<C,Eigen::aligned_allocator<C>> v2(size2);
  std::vector<D,Eigen::aligned_allocator<D>> v3(size3);

  for(auto& x : v0)
    x << random(1.0);

  for(auto& x : v1)
    x << random(1.0);
  
  for(auto& x : v2)
    x << random(1.0);

  for(auto& x : v3)
    x << random(1.0);

  typedef Solver<F1,F2> Solver;
  solve<Solver>(DENSE,v0,v1,v2,v3);
  solve<Solver>(IMPLICIT_SCHUR2,v0,v1,v2,v3);
  solve<Solver>(SPARSE_SCHUR2,v0,v1,v2,v3);
  return 0;
}
