#pragma once

#include "types.h"
#include "geometry/mesh.h"

#include "cfg/mla.h"

struct MicroLensesArray : public GridMesh3D {
private:
	std::size_t I_ = 3u; //number of micro-lens type
	FocalLengths focals_;

public:
	MicroLensesArray(const MLAConfig& config = {});
	
	void init(std::size_t I);
	std::size_t I() const;
	
	double f(std::size_t i) const;
	double& f(std::size_t i);
	
	double f(std::size_t k, std::size_t l) const;
	double& f(std::size_t k, std::size_t l);
	
	FocalLength focal_length(std::size_t k, std::size_t l) const;
	FocalLength& focal_length(std::size_t k, std::size_t l);
	
	double radius() const;
	double diameter() const;
	
	int type(std::size_t k, std::size_t l) const;
};

using MLA 					= MicroLensesArray;
