#define FUSION_MAX_VECTOR_SIZE 20
#include <libv/lma/numeric/divers.hpp>
#include <libv/lma/lma.hpp>

  typedef double type;

  struct Scalar
  {
    double x_ = 0;
    double& x() { return x_; }
    const double& x() const { return x_; }
    void operator<<(double value) { x_ = value; }
  };

  struct A : Scalar {};
  struct B : Scalar {};
  struct C : Scalar {};
  struct D : Scalar {};
  struct E : Scalar {};

  struct F1
  {
    bool operator()(const A& a, const C& c, const E& e, type& r) const
    {
      r = a.x() * c.x() * e.x();
      return true;
    }
  };
  
  struct F2
  {
    bool operator()(const B& b, const C& c, const D& d, type& r) const
    {
      r = b.x() * c.x() * d.x();
      return true;
    }
  };

  struct F3
  {
    bool operator()(const C& c, const E& e, type& r) const
    {
      r = c.x() * e.x();
      return true;
    }
  };
  
namespace lma
{
  template<> struct Size<A> { static const size_t value = 1; };
  template<> struct Size<B> { static const size_t value = 1; };
  template<> struct Size<C> { static const size_t value = 1; };
  template<> struct Size<D> { static const size_t value = 1; };
  template<> struct Size<E> { static const size_t value = 1; };

  void apply_increment(Scalar& d, const double h[1], const Adl&) { d.x() += h[0]; }
  void apply_small_increment(Scalar& d, double h, v::numeric_tag<0>, const Adl&) { d.x() += h; }
}

namespace ttt
{
  template<> struct Name<A> { static std::string name(){ return "A"; } };
  template<> struct Name<B> { static std::string name(){ return "B"; } };
  template<> struct Name<C> { static std::string name(){ return "C"; } };
  template<> struct Name<D> { static std::string name(){ return "D"; } };
  template<> struct Name<E> { static std::string name(){ return "E"; } };
}

template<class Solver, class AlgoTag, class VA, class VB, class VC, class VD, class VE> void solve(VA a, VB b, VC c, VD d, VE e)
{
  {
    Solver bundle(1,1);
    
    for(size_t i = 0 ; i < a.size() ; ++i)
      for(size_t j = 0 ; j < c.size() ; ++j)
        for(size_t k = 0 ; k < e.size() ; ++k)
          bundle.add(F1(),&a[i],&c[j],&e[k]);
    
    for(size_t i = 0 ; i < b.size() ; ++i)
      for(size_t j = 0 ; j < c.size() ; ++j)
        for(size_t k = 0 ; k < d.size() ; ++k)
          bundle.add(F2(),&b[i],&c[j],&d[k]);

//     for(size_t i = 0 ; i < c.size() ; ++i)
//       for(size_t j = 0 ; j < e.size() ; ++j)
//           bundle.add(F3(),&c[j],&e[j]);
        
    bundle.solve(AlgoTag(),lma::minimal_verbose());
//         bundle.solve(AlgoTag(),lma::enable_verbose_output());
  }
}

using namespace lma;

int main()
{
  size_t size0 = 5;
  size_t size1 = 10;
  size_t size2 = 15;
  size_t size3 = 20;
  size_t size4 = 25;

  
  std::vector<A> a(size0);
  std::vector<B> b(size1);
  std::vector<C> c(size2);
  std::vector<D> d(size3);
  std::vector<E> e(size4);

  for(auto& x : a)
    x << random(1.0);

  for(auto& x : b)
    x << random(1.0);
  
  for(auto& x : c)
    x << random(1.0);

  for(auto& x : d)
    x << random(1.0);

  for(auto& x : e)
    x << random(1.0);
  
  typedef Solver<mpl::vector<A,B,C,D,E>,F1,F2,F3> Solver;
    
  solve<Solver,DENSE_>(a,b,c,d,e);
  solve<Solver,SPARSE_>(a,b,c,d,e);
  
  solve<Solver,DENSE_SCHUR_>(a,b,c,d,e);
  solve<Solver,DENSE_SCHUR2_>(a,b,c,d,e);

  solve<Solver,SPARSE_SCHUR_>(a,b,c,d,e);
  solve<Solver,SPARSE_SCHUR2_>(a,b,c,d,e);

  solve<Solver,IMPLICIT_SCHUR_>(a,b,c,d,e);
  solve<Solver,IMPLICIT_SCHUR2_>(a,b,c,d,e);

  return 0;
}
