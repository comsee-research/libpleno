#pragma once

#include <libv/lma/lma.hpp>

#include "geometry/mesh.h"

#include "types.h"

namespace ttt
{
    template<>
    struct Name<GridMesh3D>{ static std::string name(){ return "GridMesh3D"; } };
} // namespace ttt

namespace lma
{
////////////////////////////////////////////////////////////////////////////////////////////////////
// On GridMesh3D
////////////////////////////////////////////////////////////////////////////////////////////////////
	template<>
    struct Size<GridMesh3D>{ enum{ value = 1 }; };

    void apply_increment(GridMesh3D& g, const double delta[Size<GridMesh3D>::value], const Adl&);

    void apply_small_increment(GridMesh3D& g, double h, const v::core::numeric_tag<0>&, const Adl&);
} // namespace lma
