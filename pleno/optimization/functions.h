#pragma once

#include <libv/lma/lma.hpp>

#include "processing/tools/functions.h"

namespace ttt
{
    template<>
    struct Name<LinearFunction>{ static std::string name(){ return "LinearFunction"; } };
    template<>
    struct Name<QuadraticFunction>{ static std::string name(){ return "QuadraticFunction"; } };
    template<>
    struct Name<CubicFunction>{ static std::string name(){ return "CubicFunction"; } };
    template<>
    struct Name<QuarticFunction>{ static std::string name(){ return "QuarticFunction"; } };
} // namespace ttt

namespace lma
{
////////////////////////////////////////////////////////////////////////////////////////////////////
    template<>
    struct Size<LinearFunction>{ enum{ value = 2 }; };

    void apply_increment(LinearFunction& f, const double delta[Size<LinearFunction>::value], const Adl&);
    void apply_small_increment(LinearFunction& f, const double h, const v::core::numeric_tag<0>&, const Adl&);
    void apply_small_increment(LinearFunction& f, const double h, const v::core::numeric_tag<1>&, const Adl&);
////////////////////////////////////////////////////////////////////////////////////////////////////
    template<>
    struct Size<QuadraticFunction>{ enum{ value = 3 }; };

    void apply_increment(QuadraticFunction& f, const double delta[Size<QuadraticFunction>::value], const Adl&);
    void apply_small_increment(QuadraticFunction& f, const double h, const v::core::numeric_tag<0>&, const Adl&);
    void apply_small_increment(QuadraticFunction& f, const double h, const v::core::numeric_tag<1>&, const Adl&);
    void apply_small_increment(QuadraticFunction& f, const double h, const v::core::numeric_tag<2>&, const Adl&);
////////////////////////////////////////////////////////////////////////////////////////////////////
    template<>
    struct Size<CubicFunction>{ enum{ value = 4 }; };

    void apply_increment(CubicFunction& f, const double delta[Size<CubicFunction>::value], const Adl&);
    void apply_small_increment(CubicFunction& f, const double h, const v::core::numeric_tag<0>&, const Adl&);
    void apply_small_increment(CubicFunction& f, const double h, const v::core::numeric_tag<1>&, const Adl&);
    void apply_small_increment(CubicFunction& f, const double h, const v::core::numeric_tag<2>&, const Adl&);
    void apply_small_increment(CubicFunction& f, const double h, const v::core::numeric_tag<3>&, const Adl&);
////////////////////////////////////////////////////////////////////////////////////////////////////
    template<>
    struct Size<QuarticFunction>{ enum{ value = 5 }; };

    void apply_increment(QuarticFunction& f, const double delta[Size<QuarticFunction>::value], const Adl&);
    void apply_small_increment(QuarticFunction& f, const double h, const v::core::numeric_tag<0>&, const Adl&);
    void apply_small_increment(QuarticFunction& f, const double h, const v::core::numeric_tag<1>&, const Adl&);
    void apply_small_increment(QuarticFunction& f, const double h, const v::core::numeric_tag<2>&, const Adl&);
    void apply_small_increment(QuarticFunction& f, const double h, const v::core::numeric_tag<3>&, const Adl&);
    void apply_small_increment(QuarticFunction& f, const double h, const v::core::numeric_tag<4>&, const Adl&);
} // namespace lma
