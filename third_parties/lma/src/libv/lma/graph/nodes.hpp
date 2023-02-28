#pragma once

#include "functors_container.hpp"

namespace lma
{
  struct Node
  {
    bool covered=false  ;
    void init(const auto& ...) {}
    void cover(const auto& ...) {}
    void post(const auto& ...) {}
    void stop(const auto& ...) {}
  };

  struct NodeP : Node {};
  struct NodeC : Node {};

  template<class T> struct Parameter : NodeP
  {
    using type = T;
  };

  template<class ... Functors> struct Constraint : NodeC, FunctorsContainer<boost::fusion::vector<Functors ...>>
  {
    using functors = boost::mpl::vector<Functors ...>;
  };


  template<class Const> void _cover(auto& algo, Const& constraint, typename boost::enable_if<boost::is_convertible<Const*,NodeC*>>::type* = 0)
  {
    if (constraint.covered) return;
    constraint.clear();
    constraint.cover(algo);
    constraint.covered = true;
  }

  template<class Params> void _cover(auto& algo, Params& parameter, typename boost::enable_if<boost::is_convertible<Params*,NodeP*>>::type* = 0)
  {
    if (parameter.covered) return;
    parameter.cover(algo);
    parameter.covered = true;
  }


  template<class Const> void fill_solver(Const& constraint, auto& solver, typename boost::enable_if<boost::is_convertible<Const*,NodeC*>>::type* = 0)
  {
    constraint.fill(solver);
  }

  template<class Params> void fill_solver(Params&, auto&, typename boost::enable_if<boost::is_convertible<Params*,NodeP*>>::type* = 0) {}

  template<class Algo, class Contrainte> void post_solve(Algo& algo, Contrainte& contrainte)
  {
    contrainte.post(algo);
  }
}