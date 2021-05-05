#pragma once

#include <iostream>

#include "types.h"

#include "io/archive.h"

struct MicroImageCenterObservation;
struct CheckerBoardObservation;
struct BlurAwarePlenopticObservation;

//******************************************************************************
//******************************************************************************
//******************************************************************************
struct MicroImageCenterObservation{
EIGEN_MAKE_ALIGNED_OPERATOR_NEW    

    int k = -1, l = -1; //col,row
    
    double u = -1.0, v = -1.0;
    
    bool isValid = true;
    
    double& operator[](std::size_t i) { assert(i < 2); return (i == 0) ? u : v; }
	double operator[](std::size_t i) const { assert(i < 2); return (i == 0) ? u : v; }
	
	double norm() const { return std::sqrt(u*u + v*v); }
	P2D operator-(const MicroImageCenterObservation& o) const { return P2D{u - o.u, v - o.v}; }
};

std::ostream& operator<<(std::ostream& os, const MicroImageCenterObservation& ob);
void save(v::OutputArchive& archive, const MicroImageCenterObservation& m);
void load(v::InputArchive& archive, MicroImageCenterObservation& m);

//******************************************************************************
//******************************************************************************
//******************************************************************************
struct CheckerBoardObservation {
EIGEN_MAKE_ALIGNED_OPERATOR_NEW    

	int k = -1, l = -1; //col,row
	
	double u = -1.0, v = -1.0;
	
	int cluster = -1; //-1 = not assigne to a cluster
	int frame = -1; //default = -1
	
	bool isValid = true;
	
    double& operator[](std::size_t i) { assert(i < 2); return (i == 0) ? u : v; }
	double operator[](std::size_t i) const { assert(i < 2); return (i == 0) ? u : v; }
	
	double norm() const { return std::sqrt(u*u + v*v); }
	P2D operator-(const CheckerBoardObservation& o) const { return P2D{u - o.u, v - o.v}; }
	
	operator BlurAwarePlenopticObservation() const;
};

void save(v::OutputArchive& archive, const CheckerBoardObservation& o);
void load(v::InputArchive& archive, CheckerBoardObservation& o);
std::ostream& operator<<(std::ostream& os, const CheckerBoardObservation& ob);

//******************************************************************************
//******************************************************************************
//******************************************************************************
struct BlurAwarePlenopticObservation {
EIGEN_MAKE_ALIGNED_OPERATOR_NEW    

	int k = -1, l = -1; //col,row
	
	double u = -1.0, v = -1.0, rho = -1.0;
	
	int cluster = -1;
	int frame = -1;
	
	bool isValid = true;
	
	double& operator[](std::size_t i) { assert(i < 3); return (i == 0) ? u : (i == 1) ? v : rho; }
	double operator[](std::size_t i) const { assert(i < 3); return (i == 0) ? u : (i == 1) ? v : rho; }
	
	double norm() const { return std::sqrt(u*u + v*v + rho*rho); }
	P3D operator-(const BlurAwarePlenopticObservation& o) const { return P3D{u - o.u, v - o.v, rho - o.rho}; }
	
	operator CheckerBoardObservation() const;
};

void save(v::OutputArchive& archive, const BlurAwarePlenopticObservation& o);
void load(v::InputArchive& archive, BlurAwarePlenopticObservation& o);
std::ostream& operator<<(std::ostream& os, const BlurAwarePlenopticObservation& ob);

//******************************************************************************
//******************************************************************************
//******************************************************************************
template<typename ObservationsIn, typename ObservationsOut>
void convert(const ObservationsIn& from, ObservationsOut& to)
{
	using Observation = typename ObservationsOut::value_type;
	
	to.clear();
	to.reserve(from.size());
	
	std::transform(
		from.begin(), from.end(), std::back_inserter(to),
		[](const auto& ob) -> Observation { return static_cast<Observation>(ob); }
	);
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
using CBObservation 		= CheckerBoardObservation;
using CBObservations		= AlignedVector<CBObservation>;

using MICObservation 		= MicroImageCenterObservation;
using MICObservations 		= AlignedVector<MICObservation>;

using BAPObservation		= BlurAwarePlenopticObservation;
using BAPObservations		= AlignedVector<BAPObservation>;

