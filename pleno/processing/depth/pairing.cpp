#include "pairing.h"

#include "unused.h"


template <typename Functors>
void make_functors(
	Functors& functors, const NeighborsIndexes& neighs,
	std::size_t ck, std::size_t cl,
	const PlenopticCamera& mfpc, const Image& scene, 
	ObservationsPairingStrategy mode,
	double cu, double cv
)
{
	functors.clear();
	const std::size_t I = mfpc.I();
	const P2D at = P2D{cu, cv};
	
	//compute ref observation	 	
 	MicroImage ref = mfpc.mia().mi(ck, cl, I); 
 	//ref.center -= P2D{0.5, 0.5}; //FIXME: move half pixel? 
 	mfpc.mia().extract(ref, scene);
 
 	if (mode == ObservationsPairingStrategy::CENTRALIZED)
 	{	
 		functors.reserve(neighs.size());
 		
	 	//for each neighbor, create observation
	 	for (auto [nk, nl] : neighs)
	 	{
	 		MicroImage target = mfpc.mia().mi(nk, nl, I); 
	 		//target.center -= P2D{0.5, 0.5}; //FIXME: move half pixel? 
 			mfpc.mia().extract(target, scene);
			
			functors.emplace_back(
				ref, target,
				mfpc, at
			);		
	 	}
	}
	else if (mode == ObservationsPairingStrategy::ALL_PAIRS)
	{
		const std::size_t n = neighs.size() + 1;
		std::vector<MicroImage> vmi; vmi.reserve(n);
		
		vmi.emplace_back(ref);
		
		for (auto [nk, nl] : neighs)
	 	{
	 		MicroImage target = mfpc.mia().mi(nk, nl, I);
 			mfpc.mia().extract(target, scene);
			
			vmi.emplace_back(std::move(target));
		}
		
		functors.reserve(neighs.size() * (neighs.size() - 1) / 2);
		
		//for each observation
		for (std::size_t i = 0; i < vmi.size(); ++i)
		{		
			for (std::size_t j = i+1; j < vmi.size(); ++j)
			{
				//add in solver
				functors.emplace_back(
					vmi[i], vmi[j],
					mfpc, at
				);
			}
		}
		functors.shrink_to_fit();	
	}
	else
	{
		DEBUG_ASSERT(false, "Can't pair observations, no other strategy implemented yet");
	}
}


template <typename Functors>
void make_functors_from_obs(
	Functors& functors,
	const PlenopticCamera& mfpc, const Image& scene, 
	const BAPObservations& observations
)
{
	functors.clear();

	///split observations according to cluster index
	std::unordered_map<Index /* cluster index */, BAPObservations> clusters;
	for (const auto& ob : observations)
		clusters[ob.cluster].push_back(ob);	
	
	///for each cluster
	for (auto & [cluster, obs_] : clusters)
	{
		UNUSED(cluster);
		
		NeighborsIndexes neighs; neighs.reserve(obs_.size());
		
		//assume that observations idnexes are in mi space
		for (const auto& ob : obs_) neighs.emplace_back(ob.k, ob.l);
		
		const auto [ck, cl] = neighs[0];
		
		Functors fnctrs;
		make_functors(fnctrs, neighs, ck, cl, mfpc, scene, ObservationsPairingStrategy::ALL_PAIRS);	
		
		for (auto &&f: fnctrs) functors.emplace_back(f);
	}
	
	functors.shrink_to_fit();
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
#include "optimization/errors/disparity.h" //DisparityCostError, BlurAwareDisparityCostError

template void make_functors(
	std::vector<BlurAwareDisparityCostError>& functors, const NeighborsIndexes& neighs,
	std::size_t ck, std::size_t cl,
	const PlenopticCamera& mfpc, const Image& scene, 
	ObservationsPairingStrategy mode,
	double cu, double cv
);

template void make_functors(
	std::vector<DisparityCostError>& functors, const NeighborsIndexes& neighs,
	std::size_t ck, std::size_t cl,
	const PlenopticCamera& mfpc, const Image& scene, 
	ObservationsPairingStrategy mode,
	double cu, double cv
);

template void make_functors_from_obs(
	std::vector<BlurAwareDisparityCostError>& functors,
	const PlenopticCamera& mfpc, const Image& scene, 
	const BAPObservations& observations
);

template void make_functors_from_obs(
	std::vector<DisparityCostError>& functors,
	const PlenopticCamera& mfpc, const Image& scene, 
	const BAPObservations& observations
);
