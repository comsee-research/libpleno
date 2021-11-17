#pragma once

#include "types.h"
#include "../../types.h"

#include "geometry/camera/plenoptic.h"
#include "geometry/observation.h"

#include "geometry/depth/depthmap.h"

#include "strategy.h"

struct DepthHypothesis {
//******************************************************************************
	VirtualDepth vd = {0.};
	
	std::size_t k = 0ul, l = 0ul;
	double u = -1., v = -1.;
	
	double cost = 0., sigma = 1.;
	double min = 2.01, max = 15., precision = 0.1;
		
//******************************************************************************
	double 	depth() const 	{ return vd.v; }
	double& depth() 		{ return vd.v; }
	
	double invdepth() const { return 1. / depth(); }
	
	double confidence() const { return precision / sigma; }
	
	void at(double pu, double pv) 	{ u = pu; v = pv; }
	P2D  at() const 				{ return P2D{u, v}; }
	
	bool is_valid() const 	{ return (cost > 0.); }
	
//******************************************************************************	
	DepthHypothesis& operator +=(const DepthHypothesis& o) {
		double z = invdepth();
		const double oz = o.invdepth();
		
		z = (sigma * oz + o.sigma * z) / (sigma + o.sigma);
		
		sigma = (sigma * o.sigma) / (sigma + o.sigma);
		depth() = 1. / z;
				
		return *this;
	}
};

//******************************************************************************
//******************************************************************************
//******************************************************************************
struct DepthEstimationStrategy;
struct DepthMap;

void estimate_depth(
	DepthMap& dm,
	const PlenopticCamera& mfpc,
	const Image& img,
	const DepthEstimationStrategy& strategies,
	const Image& color, 
	bool gui = true
);

void estimate_depth_from_obs(
	DepthMap& dm,
	const PlenopticCamera& mfpc,
	const Image& img,
	const BAPObservations& observations,/*  (u,v,rho) */
	bool gui = true
);

