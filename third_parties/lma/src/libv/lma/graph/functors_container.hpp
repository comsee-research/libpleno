#pragma once

#include <boost/type_traits/add_pointer.hpp>
#include <boost/mpl/transform.hpp>
#include <boost/mpl/for_each.hpp>
#include <boost/fusion/include/as_vector.hpp>
#include <boost/fusion/include/mpl.hpp>
#include <boost/fusion/include/for_each.hpp>
#include <libv/lma/ttt/traits/functor_trait.hpp>
#include <libv/lma/ttt/traits/wrap.hpp>

#include <vector>
#include <Eigen/Core>


template<class T> using AlignedVector = std::vector<T,Eigen::aligned_allocator<T>>;


namespace lma
{

  template<class Solver, class Functors> struct SolverAdd
  {
    Solver& solver;
    Functors& functors;
    SolverAdd(Solver& solver_, Functors& functors_):solver(solver_),functors(functors_){}
    template<class Key> void operator()(const ttt::wrap<Key>&)
    {
      auto& v = *boost::fusion::find<Key>(functors);
      for(auto& pair : v) solver.add(pair.first,pair.second);
    }
  };

  template<class L> class FunctorsContainer
  {
    private:
      template<class F> struct F2type
      {
        typedef typename ttt::Extract<decltype(&F::operator())>::Args args_;
        typedef typename boost::mpl::transform<args_,boost::add_pointer<boost::mpl::_1>>::type args;
        typedef typename boost::fusion::result_of::as_vector<args>::type v_args;
        typedef AlignedVector<std::pair<F,v_args>> type;
      };

      typedef typename boost::mpl::transform<L, F2type<boost::mpl::_1> >::type Types;
      typedef typename boost::fusion::result_of::as_vector<Types>::type Functors;

      Functors functors;

    public:
      template<class F, class ... P> FunctorsContainer& add(const F& f, const P& ... p)
      {
        typedef typename F2type<F>::type Container;
        typedef typename F2type<F>::v_args v_args;
        (*boost::fusion::find<Container>(functors)).emplace_back(f,v_args(p...));
        return *this;
      }
      
      void clear()
      {
        boost::fusion::for_each(functors,[](auto& v){v.clear();});
      }

      template<class Solver> void fill(Solver& solver)
      {
        boost::mpl::for_each<Types,ttt::wrap_>(SolverAdd<Solver,Functors>(solver,functors));
      }
  };

}