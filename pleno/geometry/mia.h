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
		this->orientation() = Orientation(config.mesh().orientation());
		this->width() = config.mesh().width();
		this->height() = config.mesh().height();
		this->edge_length() = config.mesh().pitch();
		this->pose() = config.mesh().pose();
		this->angle() = config.mesh().angle();
	}

	MicroImage at(std::size_t k, std::size_t l) const {
		return MicroImage{k,l, this->nodeInWorld(k,l), -1., static_cast<int>(std::fmod(std::fmod(l,2)+k, 3))};
	}
};

using MI					= MicroImage;
using MIA 					= MicroImagesArray;
