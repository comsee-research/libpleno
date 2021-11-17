#include "strategy.h"

void save(v::OutputArchive& archive, const DepthEstimationStrategy& strategies)
{
	std::string init 		= to_string(strategies.init);
	std::string pairing 	= to_string(strategies.pairing);
	std::string belief	= to_string(strategies.belief);
	std::string search	= to_string(strategies.search);
	
	bool dtype	= static_cast<bool>(strategies.dtype);
	bool mtype	= static_cast<bool>(strategies.mtype);
	
	archive
		("multithread", strategies.multithread)
		("nbthread", strategies.nbthread)
		("dtype", dtype)
		("mtype", mtype)
		("init", init)
		("randomize", strategies.randomize)
		("pairing", pairing)
		("filter", strategies.filter)
		("belief", belief)
		("search", search)
		("metricspace", strategies.metric)
		("probabilistic", strategies.probabilistic)
		("precision", strategies.precision)
		("vmin", strategies.vmin)
		("vmax", strategies.vmax);
}

void load(v::InputArchive& archive, DepthEstimationStrategy& strategies)
{
	std::string init, pairing, belief, search;
	
	bool dtype = static_cast<bool>(strategies.dtype);
	bool mtype = static_cast<bool>(strategies.mtype);
	
	archive
		("multithread", strategies.multithread)
		("nbthread", strategies.nbthread)
		("virtual", dtype)
		("coarse", mtype)
		("init", init)
		("randomize", strategies.randomize)
		("pairing", pairing)
		("filter", strategies.filter)
		("belief", belief)
		("search", search)
		("metricspace", strategies.metric)
		("probabilistic", strategies.probabilistic)
		("precision", strategies.precision)
		("vmin", strategies.vmin)
		("vmax", strategies.vmax);
		
	from_string(init, strategies.init);
	from_string(pairing, strategies.pairing);
	from_string(belief, strategies.belief);
	from_string(search, strategies.search);
	
	strategies.dtype = DepthMap::DepthType(dtype);
	strategies.mtype = DepthMap::MapType(mtype);
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
std::ostream& operator<<(std::ostream& os, const InitStrategy& mode)
{
	os << "Initilisation strategy = ";
	switch(mode)
	{
		case InitStrategy::RANDOM: os << "Random"; break;
		case InitStrategy::REGULAR_GRID: os << "Regular grid pattern"; break;
		case InitStrategy::FROM_LEFT_BORDER: os << "Regular pattern shifted from left"; break;
	}
	return os;
}
std::string to_string(const InitStrategy& mode)
{
	if (mode == InitStrategy::RANDOM) return "RANDOM";
	else if (mode == InitStrategy::REGULAR_GRID) return "REGULAR_GRID";
	else if (mode == InitStrategy::FROM_LEFT_BORDER) return "FROM_LEFT_BORDER";
	else return "RANDOM";
}
void from_string(const std::string& mode, InitStrategy& strat)
{
	if (mode == to_string(InitStrategy::RANDOM)) strat = InitStrategy::RANDOM;
	else if (mode == to_string(InitStrategy::REGULAR_GRID)) strat = InitStrategy::REGULAR_GRID;
	else if (mode == to_string(InitStrategy::FROM_LEFT_BORDER)) strat = InitStrategy::FROM_LEFT_BORDER;
	else strat = InitStrategy::RANDOM;
}
//******************************************************************************
std::ostream& operator<<(std::ostream& os, const BeliefPropagationStrategy& mode)
{
	os << "Belief propagation strategy = ";
	switch(mode)
	{
		case BeliefPropagationStrategy::NONE: os << "No propagation"; break;
		case BeliefPropagationStrategy::FIRST_RING: os << "Propagate to inner ring"; break;
		case BeliefPropagationStrategy::ALL_NEIGHS: os << "Propagate to all neighbors"; break;
	}
	return os;
}
std::string to_string(const BeliefPropagationStrategy& mode)
{
	if (mode == BeliefPropagationStrategy::NONE) return "NONE";
	else if (mode == BeliefPropagationStrategy::FIRST_RING) return "FIRST_RING";
	else if (mode == BeliefPropagationStrategy::ALL_NEIGHS) return "ALL_NEIGHS";
	else return "NONE";
}
void from_string(const std::string& mode, BeliefPropagationStrategy& strat)
{
	if (mode == to_string(BeliefPropagationStrategy::NONE)) strat = BeliefPropagationStrategy::NONE;
	else if (mode == to_string(BeliefPropagationStrategy::FIRST_RING)) strat = BeliefPropagationStrategy::FIRST_RING;
	else if (mode == to_string(BeliefPropagationStrategy::ALL_NEIGHS)) strat = BeliefPropagationStrategy::ALL_NEIGHS;
	else strat = BeliefPropagationStrategy::NONE;
}

//******************************************************************************
std::ostream& operator<<(std::ostream& os, const ObservationsPairingStrategy& mode)
{
	os << "Observations pairing strategy = ";
	switch(mode)
	{
		case ObservationsPairingStrategy::CENTRALIZED: os << "Centralized from ref"; break;
		case ObservationsPairingStrategy::ALL_PAIRS: os << "All possible combinations"; break;
	}
	return os;
}
std::string to_string(const ObservationsPairingStrategy& mode)
{
	if (mode == ObservationsPairingStrategy::CENTRALIZED) return "CENTRALIZED";
	else if (mode == ObservationsPairingStrategy::ALL_PAIRS) return "ALL_PAIRS";
	else return "CENTRALIZED";
}
void from_string(const std::string& mode, ObservationsPairingStrategy& strat)
{
	if (mode == to_string(ObservationsPairingStrategy::CENTRALIZED)) strat = ObservationsPairingStrategy::CENTRALIZED;
	else if (mode == to_string(ObservationsPairingStrategy::ALL_PAIRS)) strat = ObservationsPairingStrategy::ALL_PAIRS;
	else strat = ObservationsPairingStrategy::CENTRALIZED;
}

//******************************************************************************
std::ostream& operator<<(std::ostream& os, const SearchStrategy& mode)
{
	os << "Depth search strategy = ";
	if (mode == SearchStrategy::NONLIN_OPTIM) os << "Non-linear optimization (LM)";
	if (mode == SearchStrategy::BRUTE_FORCE) os << "Brute-Force search";
	if (mode == SearchStrategy::GOLDEN_SECTION) os << "Golden-Section search (GSS)";
	return os;
}
std::string to_string(const SearchStrategy& mode)
{
	if (mode == SearchStrategy::NONLIN_OPTIM) return "NONLIN_OPTIM";
	else if (mode == SearchStrategy::BRUTE_FORCE) return "BRUTE_FORCE";
	else if (mode == SearchStrategy::GOLDEN_SECTION) return "GOLDEN_SECTION";
	else return "GOLDEN_SECTION";
}
void from_string(const std::string& mode, SearchStrategy& strat)
{
	
	if (mode == to_string(SearchStrategy::NONLIN_OPTIM)) strat = SearchStrategy::NONLIN_OPTIM;
	else if (mode == to_string(SearchStrategy::BRUTE_FORCE)) strat = SearchStrategy::BRUTE_FORCE;
	else if (mode == to_string(SearchStrategy::GOLDEN_SECTION)) strat = SearchStrategy::GOLDEN_SECTION;
	else strat = SearchStrategy::GOLDEN_SECTION;
}

//******************************************************************************
std::ostream& operator<<(std::ostream& os, const DepthEstimationStrategy& mode)
{
	os  << "Depth estimation strategies: " << std::endl
		<< "\t" << mode.init << std::endl
		<< "\t" << mode.pairing << std::endl
		<< "\t" << mode.belief << std::endl
		<< "\t" << mode.search << std::endl
		<< "\tvmin = " << mode.vmin << ", vmax = " << mode.vmax << std::endl
		<< std::boolalpha 
		<< "\tmetricspace = " << mode.metric << std::endl
		<< "\tprobabilistic = " << mode.probabilistic << std::endl
		<< "\tfilter = " << mode.filter << std::endl
		<< "\trandomize = " << mode.randomize
		<< std::noboolalpha;

	return os;
}


