
#include <libv/lma/lma.hpp>


using namespace lma;

struct A { double value; };
struct B { double value; };
struct C { double value; };


namespace ttt
{
  template<> struct Name<A> { static std::string name(){ return "A"; } };
  template<> struct Name<B> { static std::string name(){ return "B"; } };
  template<> struct Name<C> { static std::string name(){ return "C"; } };
}

  struct F1
  {
    bool operator()(const A& a, const B& b, const C& c, double& r) const
    {
      r = a.value * b.value * c.value;
      return true;
    }
  };
  
namespace lma
{
  template<> struct Size<A> { static const size_t value = 3; };
  template<> struct Size<B> { static const size_t value = 4; };
  template<> struct Size<C> { static const size_t value = 1; };
  
  void apply_small_increment(A& obj, double inc, v::numeric_tag<0>, const Adl&) { obj.value += inc ; }
  void apply_small_increment(A& obj, double inc, v::numeric_tag<1>, const Adl&) { obj.value += inc * 2.0 ; }
  void apply_small_increment(A& obj, double inc, v::numeric_tag<2>, const Adl&) { obj.value -= inc; }
  template<int I> 
  void apply_small_increment(B& obj, double inc, v::numeric_tag<I>, const Adl&) { obj.value += inc * I; }
  template<int I> 
  void apply_small_increment(C& obj, double inc, v::numeric_tag<I>, const Adl&) { obj.value += inc * I; }

  void apply_increment(A& obj, const double delta[3], const Adl&) { obj.value += delta[0]+delta[1]+delta[2]; }
  void apply_increment(B& obj, const double delta[4], const Adl&) { obj.value += delta[0]+delta[1]+delta[2]+delta[3]; }
  void apply_increment(C& obj, const double delta[1], const Adl&) { obj.value += delta[0]; }
}



int main()
{
  lma::Solver<F1> solver;
  
  A a = {1.0};
  B b = {5.0};
  C c = {5.0};
  
  solver.add(F1(),&a,&b,&c);
  
  solver.solve(lma::DENSE,lma::enable_verbose_output());
  
  return 0;
}
