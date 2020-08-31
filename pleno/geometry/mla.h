#pragma once

#include "geometry/mesh.h"

#include "cfg/mla.h"

#include "types.h"
#include "processing/tools/lens.h"


struct MicroLensesArray : public GridMesh3D {
private:
	std::size_t I_ = 3u; //number of micro-lens type
	FocalLengths focals_;

public:
	MicroLensesArray(const MLAConfig& config = {}) {
		this->geometry() = Geometry(config.mesh().geometry());
		this->width() = config.mesh().width();
		this->height() = config.mesh().height();
		this->edge_length() = config.mesh().pitch();
		this->pose() = config.mesh().pose();
		
		init(config.focal_lengths().size());
		
		for(std::size_t i=0; i<I(); ++i) 
			this->focals_[i].f = config.focal_lengths()[i];
	}
	
	void init(std::size_t I) { this->I_ = I; this->focals_.resize(I); }
	
	std::size_t I() const { return I_; }
	
	double f(std::size_t i) const { 
		assert(i<I()); 
		return focals_[i].f; 
	}
	double& f(std::size_t i) { 
		assert(i<I()); 
		return focals_[i].f; 
	}
	
	FocalLength f(std::size_t k, std::size_t l) const {
		assert(I()!=0u and k<width() and l<height());
		const int t = lens_type(I(), k, l);
		return focals_[t];
	}
	
	FocalLength& f(std::size_t k, std::size_t l) {
		assert(I()!=0u and k<width() and l<height());
		const int t = lens_type(I(), k, l);
		return focals_[t];
	}
	
	double radius() const { return (edge_length()[0] + edge_length()[1]) / 4.; }
	double diameter() const { return (edge_length()[0] + edge_length()[1]) / 2.; }
};

using MLA 					= MicroLensesArray;
