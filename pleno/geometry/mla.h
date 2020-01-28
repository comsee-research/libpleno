#pragma once

#include "geometry/mesh.h"

#include "cfg/mla.h"

#include "types.h"
#include "processing/tools/lens.h"


struct MicroLensesArray : public GridMesh3D {
private:
	std::size_t _I = 3u; //number of micro-lens type
	FocalLengths _focals;

public:
	MicroLensesArray(const MLAConfig& config = {}) {
		this->geometry() = Geometry(config.mesh().geometry());
		this->width() = config.mesh().width();
		this->height() = config.mesh().height();
		this->edge_length() = config.mesh().pitch();
		this->pose() = config.mesh().pose();
		
		init(config.focal_lengths().size())
		
		for(std::size_t i=0; i<I(); ++i) 
			this->_focals[i].f = config.focal_lengths()[i];
	}
	
	void init(std::size_t I_) { this->_I = I_; this->_focals.resize(I_); }
	
	std::size_t I() const { return _I; }
	
	double f(std::size_t i) const { 
		assert(i<I()); 
		return _focals[i].f; 
	}
	double& f(std::size_t i) { 
		assert(i<I()); 
		return _focals[i].f; 
	}
	
	FocalLength f(std::size_t k, std::size_t l) const {
		assert(I()!=0u and k<width() and l<height());
		const int t = lens_type(I(), k, l);
		return _focals[t];
	}
	
	FocalLength& f(std::size_t k, std::size_t l) {
		assert(I()!=0u and k<width() and l<height());
		const int t = lens_type(I(), k, l);
		return _focals[t];
	}
		
};

using MLA 					= MicroLensesArray;
