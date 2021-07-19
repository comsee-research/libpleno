#pragma once

#include "types.h"
#include "io/printer.h"

#include "geometry/mia.h"
#include "geometry/internals.h"
#include "geometry/observation.h"

#include "processing/tools/lens.h"

template<typename Observation_t>
double virtualdepth(const Observation_t& lhs, const Observation_t& rhs, const MIA& mia, double dC, double pixel2metric)
{
	const P2D lc = mia.nodeInWorld(lhs.k, lhs.l);								
	const P2D rc = mia.nodeInWorld(rhs.k, rhs.l);								
	const double dc = mia.diameter();		
	const double n = (lc - rc).norm() / dc;										
	const double ndC = n * dC;					
	
	const P2D plhs{lhs[0], lhs[1]};
	const P2D prhs{rhs[0], rhs[1]};
									
	const double di = (plhs - prhs).norm() * pixel2metric; 					
		
	return ndC / (ndC - di) ; // Eq.(15)

}

template<typename Observations_t>
double compute_virtualdepth(const Observations_t&obs, const MIA& mia, const InternalParameters& params)
{
	std::vector<double> vdepths;
	
	DEBUG_ASSERT((obs.size() >= 2), "Need at least 2 observations to compute virtual depth !");
	
	for (std::size_t i = 0; i < obs.size(); ++i)
	{
		auto current = obs.begin()+i;
		
		std::transform(current+1, obs.end(), std::back_inserter(vdepths),
			[lhs = *current, pixel2metric =  params.scale, dC = params.dC, &mia]
				(const auto& rhs) -> double {
					return 	virtualdepth(lhs, rhs, mia, dC, pixel2metric);		
			}
		);		
	}
	
	DEBUG_ASSERT(vdepths.size() >= 1u, "No virtual depth has been computed...");
#if 0	//Mean estimator
		const double v = std::accumulate(vdepths.begin(), vdepths.end(), 0.) / (vdepths.size() + 1e-9);
		DEBUG_VAR(v);
#else	//Median estimator
		std::nth_element(vdepths.begin(), vdepths.begin() + vdepths.size() / 2, vdepths.end());
    	const double v = vdepths[vdepths.size() / 2];
		DEBUG_VAR(v);
#endif	
	return v;
}

template<typename InputObservations_t>
void update_observations(const InputObservations_t& inobs, BAPObservations& outobs, double v, const MIA& mia, const InternalParameters& params)
{	
	std::transform(inobs.begin(), inobs.end(), std::back_inserter(outobs),
		[&v, &params, &mia](const auto& io) -> BAPObservation {
			const double r = (params.dC / 2. ) * (1. / v) 
							+ params.q_prime[mia.type(params.I, io.k, io.l)] 
							- (params.dC / 2. ); // Eq.(14)
				
			return BAPObservation{
				io.k, io.l,  //k,l
				io[0], io[1], r / params.scale, //u,v,rho
				io.cluster, io.frame //cluster, frame
			};		
		}
	);
}
template<typename InputObservations_t>
BAPObservations compute_bapfeatures(const InputObservations_t& cbos, const MIA& mia, const InternalParameters& params)
{	
	BAPObservations bapobs;
	bapobs.reserve(cbos.size());
	
	if (params.I == 0ul) 
	{
		std::transform(
			cbos.begin(), cbos.end(), std::back_inserter(bapobs),
			[&](const auto& o) -> BAPObservation { return static_cast<BAPObservation>(o); }
		);
	}
	else
	{
		std::unordered_map<Index /* cluster */, InputObservations_t> observations;
		
		PRINT_INFO("=== Splitting observations according to cluster");
		for (const auto& ob : cbos)
			observations[ob.cluster].push_back(ob);			
				
	//For each cluster, compute virtual depth	
		PRINT_INFO("=== Computing virtual depth for each cluster");
		for (auto & [c, obs] : observations)
		{
			if (obs.size() < 2) 
			{
				PRINT_ERR("No enought observations to compute virtual depth of cluster ("<<c<<")");
				continue;
			}
			PRINT_DEBUG("Estimate virtualdepth of cluster ("<<c<<")");	
			double v = compute_virtualdepth(obs, mia, params);
				
			PRINT_DEBUG("Update observations");
			update_observations(obs, bapobs, v, mia, params);
		}
	}
	bapobs.shrink_to_fit();
	return bapobs;
}
