#pragma once

#include "geometry/mesh.h"

#include "cfg/mla.h"

#include "types.h"

struct MicroLensesArray : public GridMesh3D {
private:
	FocalLength _focals[3];

public:
	MicroLensesArray(const MLAConfig& config = {}) {
		this->geometry() = Geometry(config.mesh().geometry());
		this->orientation() = Orientation(config.mesh().orientation());
		this->width() = config.mesh().width();
		this->height() = config.mesh().height();
		this->edge_length() = config.mesh().pitch();
		this->pose() = config.mesh().pose();
		this->angle() = config.mesh().angle();
		
		assert(config.focal_lengths().size() == 3u);
		this->_focals[0].f = config.focal_lengths()[0]; 
		this->_focals[1].f = config.focal_lengths()[1]; 
		this->_focals[2].f = config.focal_lengths()[2]; 
	}
	
	double f(std::size_t i) const { 
		assert(i<3); 
		return _focals[i].f; 
	}
	double& f(std::size_t i) { 
		assert(i<3); 
		return _focals[i].f; 
	}
	
	FocalLength f(std::size_t k, std::size_t l) const {
		assert(k<width() and l<height());
		const int t = static_cast<int>(std::fmod(std::fmod(l,2)+k, 3));
		return _focals[t];
	}
	
	FocalLength& f(std::size_t k, std::size_t l) {
		assert(k<width() and l<height());
		const int t = static_cast<int>(std::fmod(std::fmod(l,2)+k, 3));
		return _focals[t];
	}
		
};

using MLA 					= MicroLensesArray;
