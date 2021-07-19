#pragma once

#include "geometry/mesh.h"

//io
#include "cfg/mia.h"

//******************************************************************************
//******************************************************************************
struct MicroImage {
EIGEN_MAKE_ALIGNED_OPERATOR_NEW    

	std::size_t k, l;
	
	P2D center;
	
	double radius;
	int type;
	
	Image mi;
	
	MicroImage(std::size_t k_ = 0ul, std::size_t l_ = 0ul, const P2D& c = {-1., -1.}, double r = -1., int t = -1);
	MicroImage(std::size_t k_, std::size_t l_, const P2D& c, double r, int t, const Image& i);
	
	MicroImage(const MicroImage& o);
	MicroImage(MicroImage&& o);
	
	double contrast() const;
};

//******************************************************************************
//******************************************************************************
struct MicroImagesArray : public GridMesh2D {
	static constexpr double MI_BORDER = 1.5; //1.5 pixel
public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW    

	MicroImagesArray(const MIAConfig& config = {});
	
	static double border();
	
	double radius() const;
	double diameter() const;
	
	//k,l are expressed in MIA space
	int type(std::size_t I, std::size_t k, std::size_t l) const;
	
	MicroImage mi(const Image& scene, std::size_t k, std::size_t l, std::size_t I = 3ul) const;
	
	MicroImage mi(std::size_t k, std::size_t l, std::size_t I = 3ul) const;
	void extract(MicroImage& mi, const Image& from) const;
	
	std::pair<std::size_t, std::size_t> uv2kl(double u, double v) const;
};

//******************************************************************************
//******************************************************************************
using MI					= MicroImage;
using MicroImageObservation = MicroImage;
using MIObservation 		= MicroImageObservation;
using MIObservations		= AlignedVector<MIObservation>;

using MIA 					= MicroImagesArray;
//******************************************************************************
