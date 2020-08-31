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
	
	double radius() const { return (edge_length()[0] + edge_length()[1]) / 4.; }
	double diameter() const { return (edge_length()[0] + edge_length()[1]) / 2.; }
};

using MI					= MicroImage;
using MIA 					= MicroImagesArray;
