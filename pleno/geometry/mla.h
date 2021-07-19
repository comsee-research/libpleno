#pragma once

#include "types.h"

#include "geometry/mesh.h"
#include "geometry/plane.h"

#include "cfg/mla.h"

struct MicroLensesArray : public GridMesh3D {
private:
	std::size_t I_ = 3ul; //number of micro-lens type
	FocalLengths focals_;

public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW    

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
	
	//k,l are expressed in MLA space
	int type(std::size_t k, std::size_t l) const;
	
	// the plane equation coefficients
    PlaneCoefficients plane() const;
    // the plane equation coefficients in WORLD coordinate system
    PlaneCoefficients planeInWorld() const;
};

using MLA 					= MicroLensesArray;
