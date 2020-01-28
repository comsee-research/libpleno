#pragma once

#include "geometry/mesh.h"

//io
#include "cfg/mia.h"

struct MicroImage {
	std::size_t k, l;
	P2D center;
	double radius;
	int type;	
};

struct MicroImagesArray : public GridMesh2D {
public:
	MicroImagesArray(const MIAConfig& config = {}) {
		this->geometry() = Geometry(config.mesh().geometry());
		this->width() = config.mesh().width();
		this->height() = config.mesh().height();
		this->edge_length() = config.mesh().pitch();
		this->pose() = config.mesh().pose();
	}
};

using MI					= MicroImage;
using MIA 					= MicroImagesArray;
