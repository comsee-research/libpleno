#pragma once

#include <iostream>
#include <libv/core/serialization/archives/base.hpp> //OutputArchive, InputArchive

#include "geometry/depth/depthmap.h"

enum InitStrategy : std::uint16_t { RANDOM = 0, REGULAR_GRID = 1, FROM_LEFT_BORDER = 2};
enum BeliefPropagationStrategy : std::uint16_t { NONE = 0, FIRST_RING = 1, ALL_NEIGHS = 2};
enum ObservationsPairingStrategy : std::uint16_t { CENTRALIZED = 0, ALL_PAIRS = 1};
enum SearchStrategy : std::uint16_t {NONLIN_OPTIM = 0, BRUTE_FORCE = 1, GOLDEN_SECTION = 2};

struct DepthEstimationStrategy {
	bool multithread					= true;
	int nbthread						= -1;
	
	DepthMap::DepthType	dtype		= DepthMap::DepthType::VIRTUAL;
	DepthMap::MapType	mtype		= DepthMap::MapType::COARSE;
	
	double vmin							= 2.;
	double vmax							= 20.;
	
	InitStrategy init 					= InitStrategy::REGULAR_GRID;
	bool randomize						= false;
	
	ObservationsPairingStrategy pairing = ObservationsPairingStrategy::CENTRALIZED;
	bool filter							= true;
	
	BeliefPropagationStrategy belief	= BeliefPropagationStrategy::NONE;
	
	SearchStrategy search				= SearchStrategy::BRUTE_FORCE;
	bool metric							= true;
	bool probabilistic					= false;
	double precision					= 0.01; //in mm if metric, in virtual space otherwise
};


void save(v::OutputArchive& archive, const DepthEstimationStrategy& strategies);
void load(v::InputArchive& archive, DepthEstimationStrategy& strategies);

//******************************************************************************
//******************************************************************************
//******************************************************************************
std::ostream& operator<<(std::ostream& os, const InitStrategy& mode);
std::string to_string(const InitStrategy& mode);
void from_string(const std::string & mode, InitStrategy& strat);
//******************************************************************************

std::ostream& operator<<(std::ostream& os, const BeliefPropagationStrategy& mode);
std::string to_string(const BeliefPropagationStrategy& mode);
void from_string(const std::string & mode, BeliefPropagationStrategy& strat);
//******************************************************************************

std::ostream& operator<<(std::ostream& os, const ObservationsPairingStrategy& mode);
std::string to_string(const ObservationsPairingStrategy& mode);
void from_string(const std::string & mode, ObservationsPairingStrategy& strat);
//******************************************************************************

std::ostream& operator<<(std::ostream& os, const SearchStrategy& mode);
std::string to_string(const SearchStrategy& mode);
void from_string(const std::string & mode, SearchStrategy& strat);
//******************************************************************************

std::ostream& operator<<(std::ostream& os, const DepthEstimationStrategy& mode);


