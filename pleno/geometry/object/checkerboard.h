#pragma once

#include "geometry/mesh.h"

#include "io/cfg/scene.h"

struct CheckerBoard : public GridMesh3D {
public:
	CheckerBoard(const CheckerboardConfig& config) {
		this->geometry() = Geometry::Orthogonal;
		this->width() = config.x_grid();
		this->height() = config.y_grid();
		this->pitch() = P2D{config.lph(), config.lph()};
		this->pose() = config.pose();
	}
};

using CheckerBoards = AlignedVector<CheckerBoard>;
