#include "compute.h"

#include <algorithm>

#include "unused.h"

#include "io/printer.h"
#include "io/choice.h"

#include "graphic/display.h"

#include "processing/imgproc/trim.h"

#include "geometry/depth/depthmap.h"

#include "depth.h"
#include "strategy.h"
#include "neighbors.h"
#include "initialization.h"
#include "search.h"

//******************************************************************************
//******************************************************************************
//******************************************************************************
//FIXME: move to mia ?
bool is_contrasted_enough(
	const MIA& mia, const Image& scene,
	std::size_t ck, std::size_t cl
) 
{
	constexpr double threshold_contrast = 5.;
	
	const int W 		= int(std::ceil(mia.diameter()));
	const auto center 	= mia.nodeInWorld(ck, cl); 
	const double radius = mia.radius() - mia.border() - 1.5;
	
	Image mi;
	cv::getRectSubPix(
		scene, cv::Size{W,W}, 
		cv::Point2d{center[0], center[1]}, mi
	);				
	Image mask = mi.clone();
	trim_binarize(mask, radius);
	
	cv::Scalar mean, std;
	cv::meanStdDev(mi, mean, std, mask); 
	
	const double contrast = std[0];
#if 0
	DEBUG_VAR(ck);
	DEBUG_VAR(cl);
	DEBUG_VAR(contrast);
#endif
	return (contrast >= threshold_contrast);
}

bool is_pixel_contrasted_enough(
	const MIA& /* mia */, const Image& scene,
	double u, double v
) 
{
	constexpr double threshold_contrast = 5.;
	constexpr int W = 3;
	
	Image r;
	cv::getRectSubPix(
		scene, cv::Size{W,W}, 
		cv::Point2d{u, v}, r
	);				
	
	cv::Scalar mean, std;
	cv::meanStdDev(r, mean, std); 
	
	const double contrast = std[0];

	return (contrast >= threshold_contrast);
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
void compute_depthmap(
	DepthMap& dm, 
	const PlenopticCamera& mfpc, const Image& scene, 
	std::size_t kinit, std::size_t linit,
	const DepthEstimationStrategy& strategies
)
{	
	constexpr double nbsample = 15.;
	constexpr double N = 1.96; //FIXME: increase N ?
	
	DEBUG_ASSERT((dm.is_coarse_map()), "The map type must be set to COARSE.");
		
	std::queue<IndexPair> microimages;
	microimages.emplace(kinit, linit);
	
	while (not (microimages.empty()))
	{
		auto [ck, cl] = microimages.front(); //current indexes
		microimages.pop();
				
		//Already computed, nothing to do
		if (dm.state(ck,cl) == DepthInfo::State::COMPUTED) continue;
		
		//Compute first hypothesis using inner ring
		if (dm.state(ck,cl) == DepthInfo::State::UNINITIALIZED)
		{
			//get neighbors
			std::map<double, NeighborsIndexes> ordered_neighs = neighbors_by_rings(mfpc.mia(), ck, cl, 3.5, 2.);
			
			DepthHypothesis hypothesis;
				hypothesis.k = ck;
				hypothesis.l = cl;
				hypothesis.min = dm.min_depth();
				hypothesis.max = dm.max_depth();
				hypothesis.precision = nbsample;
							 			
			//add to queue
			for (auto &n: ordered_neighs[1.]) 
	 		{
	 			if (dm.state(n.k, n.l) == DepthInfo::State::UNINITIALIZED)  
	 			{
	 				microimages.push(n);
	 			}
	 		}
			
			//filter texture		 	
		 	if (not (strategies.filter) or is_contrasted_enough(mfpc.mia(), scene, ck, cl))
		 	{			 	
			 	DepthEstimationStrategy strat = strategies;
			 	strat.metric = false; //force init in virtual space
			 	
			 	initialize_depth(
			 		hypothesis, ordered_neighs[1.7],  //micro-images of the same type
			 		mfpc, scene, strat
			 	);
		 	}
		 	
		 	if (not (hypothesis.is_valid() and dm.is_valid_depth(hypothesis.depth())))
		 	{		 		
		 		dm.depth(ck, cl) 		= DepthInfo::NO_DEPTH;
		 		dm.confidence(ck, cl) 	= DepthInfo::NO_CONFIDENCE;
		 		dm.state(ck, cl)		= DepthInfo::State::COMPUTED;	 	
		 	}
		 	else
		 	{		 	
		 		dm.depth(ck, cl) 		= hypothesis.depth();
		 		dm.confidence(ck, cl) 	= hypothesis.confidence();
		 		dm.state(ck, cl) 		= DepthInfo::State::INITIALIZED;				
		 	}
		}
		
		//Compute depth hypothesis
		if(dm.state(ck,cl) == DepthInfo::State::INITIALIZED)
		{
			//compute neighbors
			const double maxabsv = std::ceil(std::fabs(dm.depth(ck, cl)));
			NeighborsIndexes neighs = neighbors(mfpc.mia(), ck, cl, maxabsv, 2., 12.);
			
			//filter neighbors not having enougth contrast
			neighs.erase(
				std::remove_if(neighs.begin(), neighs.end(),
					[&mfpc, &scene](const IndexPair& n) -> bool {
						return not(is_contrasted_enough(mfpc.mia(), scene, n.k, n.l));
					}
				), 
				neighs.end()
			);
			neighs.shrink_to_fit();
			
		#if 0 //SAME TYPE ONLY	
			neighs.erase(
				std::remove_if(neighs.begin(), neighs.end(),
					[t = mfpc.mia().type(3u, ck, cl), &mfpc](const IndexPair& n) -> bool {
						return (mfpc.mia().type(3u, n.k, n.l) != t);
					}
				), 
				neighs.end()
			);
			neighs.shrink_to_fit();
		#endif	
		
			DepthHypothesis hypothesis;
				hypothesis.depth() = dm.depth(ck, cl);
				hypothesis.k = ck;
				hypothesis.l = cl;		
			
			//compute hypothesis
			if (strategies.search == SearchStrategy::NONLIN_OPTIM)
			{	
				optimize_depth(hypothesis, neighs, mfpc, scene, strategies);
			}
			else if (strategies.search == SearchStrategy::BRUTE_FORCE)
			{
				const double stepv = (dm.max_depth() - dm.min_depth()) / nbsample;
				
				hypothesis.min = hypothesis.depth() - stepv;
				hypothesis.max = hypothesis.depth() + stepv;
				hypothesis.precision = nbsample;
				
				bruteforce_depth(
					hypothesis, neighs, mfpc, scene, strategies
				);
		 	}
		 	else if (strategies.search == SearchStrategy::GOLDEN_SECTION)
			{				
				hypothesis.min = std::max(hypothesis.depth() - N, dm.min_depth());
				hypothesis.max = hypothesis.depth() + N;
				hypothesis.precision = std::sqrt(strategies.precision);
			
				gss_depth(
					hypothesis, neighs, mfpc, scene, strategies
				);
			}
			
			bool isValidDepth = dm.is_valid_depth(hypothesis.depth());
		 	
			if (hypothesis.is_valid() and isValidDepth) 
			{
				dm.depth(ck, cl) 		= hypothesis.depth();
				dm.confidence(ck, cl) 	= hypothesis.confidence();
		 		dm.state(ck, cl) 		= DepthInfo::State::COMPUTED;
			}
			else 
			{
				dm.depth(ck, cl) 		= DepthInfo::NO_DEPTH;
				dm.confidence(ck, cl) 	= DepthInfo::NO_CONFIDENCE;
		 		dm.state(ck, cl) 		= DepthInfo::State::COMPUTED;
				
				continue;
			}		
	#if 0		
			//depth is valid
			for (auto &n: neighs) 
			{
				//if already computed, do nothing
				if (dm.state(n.k, n.l) == DepthInfo::State::COMPUTED) 
				{
					/* Add strategy to check if estimation is coherent */
					continue;
				}
				
				//if already initialized, average hypotheses
				if (dm.state(n.k, n.l) == DepthInfo::State::INITIALIZED)
				{					
					/* Add strategy to update hypothesis */					
					continue;
				}
				
				//else uninitialized then initalize hypothesis if valid
				if (strategies.belief == BeliefPropagationStrategy::ALL_NEIGHS)
				{		
					dm.depth(n.k, n.l) = hypothesis.depth();
			 		dm.state(n.k, n.l) = DepthInfo::State::INITIALIZED;			
				}
			}
			
			if(strategies.belief == BeliefPropagationStrategy::FIRST_RING)
			{		
				NeighborsIndexes innerneighs = inner_ring(mfpc.mia(), ck, cl);
				
				for(auto& n: innerneighs)
				{
					if(dm.state(n.k, n.l) == DepthInfo::State::UNINITIALIZED)
					{
						dm.depth(n.k, n.l) = hypothesis.depth();
			 			dm.state(n.k, n.l) = DepthInfo::State::INITIALIZED;	
			 		}
		 		}		
			}
	#endif			
		}		
	}
	PRINT_DEBUG("Local depth estimation finished.");
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
void compute_refined_depthmap(
	DepthMap& dm, 
	const PlenopticCamera& mfpc, const Image& scene, 
	std::size_t kinit, std::size_t linit,
	const DepthEstimationStrategy& strategies
)
{		
	constexpr double nbsample = 15.;
	constexpr double N = 1.96; //FIXME: increase N ?
	
	DEBUG_ASSERT((dm.is_refined_map()), "The map type must be set to REFINED.");
	
	std::queue<IndexPair> microimages;
	microimages.emplace(kinit, linit);
	
	while (not (microimages.empty()))
	{
		auto [ck, cl] = microimages.front(); //current indexes
		microimages.pop();
				
		NeighborsIndexes pixels = pixels_neighbors(mfpc.mia(), mfpc.sensor().width(), mfpc.sensor().height(), ck, cl);	
		
		auto is_current_mi_state = [&pixels, &dm](DepthInfo::State state) -> bool {
			for (const auto& [u, v] : pixels)
				if (dm.state(u, v) == state) 
					return true; 
			return false;		
		};
		
		auto min_max_mi_depth_hypothesis = [&pixels, &dm]() -> std::pair<double, double> {
			double min = 1e9;
			double max = -1e9;
			for (const auto& [u, v] : pixels)
			{
				if (dm.state(u,v) != DepthInfo::State::UNINITIALIZED)
				{
					const double d = dm.depth(u, v);
					if (d != DepthInfo::NO_DEPTH) 
					{
						if (d > max) max = d;
						if (d < min) min = d;
					}
				}
			}			
			return {min, max};
		};
			
		//Already computed, nothing to do
		if (is_current_mi_state(DepthInfo::State::COMPUTED)) continue;
		
		//Compute first hypothesis using inner ring
		if (is_current_mi_state(DepthInfo::State::UNINITIALIZED))
		{
			//get neighbors
			std::map<double, NeighborsIndexes> ordered_neighs = neighbors_by_rings(mfpc.mia(), ck, cl, 3.5, 2.);
			
			//add to queue
			for (auto &n: ordered_neighs[1.]) microimages.push(n);

			//filter at micro-image level	
			const bool mi_rejected = (strategies.filter) and not(is_contrasted_enough(mfpc.mia(), scene, ck, cl));
			
			for (const auto& [u, v] : pixels)
			{
				DepthHypothesis hypothesis;
					//set microimage indexes
					hypothesis.k = ck;
					hypothesis.l = cl;
					//set min/max depth
					hypothesis.min = dm.min_depth();
					hypothesis.max = dm.max_depth();
					//set nb sample to draw
					hypothesis.precision = nbsample;
					//set pixel coordinates
					hypothesis.u = static_cast<double>(u);
					hypothesis.v = static_cast<double>(v);	
				
				const bool pixel_rejected = (strategies.filter) and not(is_pixel_contrasted_enough(mfpc.mia(), scene, u, v));
						 	
			 	if (not mi_rejected and not pixel_rejected) //filter texture
			 	{
				 	DepthEstimationStrategy strat = strategies;
				 	strat.metric = false; //Force init in virtual space
				 	
				 	initialize_depth(
				 		hypothesis,	ordered_neighs[1.7], //micro-images of the same type
				 		mfpc, scene, strat
				 	);
			 	}
			 	
			 	if (not (hypothesis.is_valid() and dm.is_valid_depth(hypothesis.depth())))
			 	{			 		
			 		//PRINT_DEBUG("Invalid depth("<<u<<", "<<v<<") = " << hypothesis.depth());
			 		dm.depth(u, v) 		= DepthInfo::NO_DEPTH;
					dm.confidence(u, v)	= DepthInfo::NO_CONFIDENCE;
			 		dm.state(u, v) 		= DepthInfo::State::COMPUTED;	 	
			 	}
			 	else 
			 	{
				 	//PRINT_DEBUG("depth("<<u<<", "<<v<<") = " << hypothesis.depth());
				 	dm.depth(u, v) 		= hypothesis.depth();
				 	dm.confidence(u, v)	= hypothesis.confidence();
				 	dm.state(u, v) 		= DepthInfo::State::INITIALIZED;
			 	}
			}//end for each pixel				
		}
	
		//Compute depth hypothesis
		if (is_current_mi_state(DepthInfo::State::INITIALIZED))
		{
			const auto [min, max] = min_max_mi_depth_hypothesis();
			const double maxabsv = std::ceil(std::max(std::fabs(min), std::fabs(max)));
			
			//compute neighbors
			NeighborsIndexes neighs = neighbors(mfpc.mia(), ck, cl, maxabsv, 2., 12.);
			
			//filter neighbors not having enougth contrast
			neighs.erase(
				std::remove_if(neighs.begin(), neighs.end(),
					[&mfpc, &scene](const IndexPair& n) -> bool {
						return not(is_contrasted_enough(mfpc.mia(), scene, n.k, n.l));
					}
				), 
				neighs.end()
			);
			neighs.shrink_to_fit();
			
			//for each pixel
			for (const auto& [u, v] : pixels)
			{
				//if already computed pixel
				if (dm.state(u, v) == DepthInfo::COMPUTED) continue;
				
				DepthHypothesis hypothesis;
					hypothesis.depth() = dm.depth(u, v);
					hypothesis.k = ck;
					hypothesis.l = cl;	
					hypothesis.u = static_cast<double>(u);
					hypothesis.v = static_cast<double>(v);	
				
				//compute hypothesis
				if (strategies.search == SearchStrategy::NONLIN_OPTIM)
				{	
					optimize_depth(hypothesis, neighs, mfpc, scene, strategies);
				}
				else if (strategies.search == SearchStrategy::BRUTE_FORCE)
				{
					const double stepv = (dm.max_depth() - dm.min_depth()) / nbsample;
					
					hypothesis.min = min - stepv;
					hypothesis.max = max + stepv;
					hypothesis.precision = nbsample;
					
					bruteforce_depth(
						hypothesis, neighs, mfpc, scene, strategies
					);
			 	}
			 	else if (strategies.search == SearchStrategy::GOLDEN_SECTION)
				{				
					hypothesis.min = std::max(min - N, dm.min_depth());
					hypothesis.max = max + N;
					hypothesis.precision = std::sqrt(strategies.precision);
				
					gss_depth(
						hypothesis, neighs, mfpc, scene, strategies
					);
				}
							 	
				if (hypothesis.is_valid() and dm.is_valid_depth(hypothesis.depth())) 
				{
					dm.depth(u, v) 		= hypothesis.depth();
					dm.confidence(u, v) = hypothesis.confidence();
			 		dm.state(u, v) 		= DepthInfo::State::COMPUTED;
				}
				else 
				{
					dm.depth(u, v) 		= DepthInfo::NO_DEPTH;
					dm.confidence(u, v) = DepthInfo::NO_CONFIDENCE;
			 		dm.state(u, v) 		= DepthInfo::State::COMPUTED;
				}	
			}//end for each pixel			
		}		
	}// end while there is still some mi in queue
	PRINT_DEBUG("Local depth estimation finished.");
}


//******************************************************************************
//******************************************************************************
//******************************************************************************
//FIXME: experimental yet
void compute_probabilistic_depthmap(
	DepthMap& dm,
	const PlenopticCamera& mfpc, const Image& scene, 
	std::size_t kinit, std::size_t linit,
	const DepthEstimationStrategy& strategies
)
{	
	constexpr double nbsample = 15.;
	constexpr double N = 1.96;
	constexpr double PHI = (1. + std::sqrt(5.)) / 2.;
	constexpr double PHI2 = PHI * PHI;
	
	DEBUG_ASSERT((dm.is_coarse_map()), "The map type must be set to COARSE.");
	
	std::queue<IndexPair> microimages;
	microimages.emplace(kinit, linit);
	
	while (not (microimages.empty()))
	{
		auto [ck, cl] = microimages.front(); //current indexes
		microimages.pop();
	
		//Already computed, nothing to do
		if (dm.state(ck, cl) == DepthInfo::State::COMPUTED) continue;
	
		//Compute hypothesis
		if (dm.state(ck, cl) == DepthInfo::State::UNINITIALIZED)
		{
			DepthHypothesis hypothesis;
				hypothesis.k = ck;
				hypothesis.l = cl;
				hypothesis.min = dm.min_depth();
				hypothesis.max = dm.max_depth();
				hypothesis.precision =  nbsample;	
							 	
			//get neighbors		 	
		 	std::map<double, NeighborsIndexes> ordered_neighs = neighbors_by_rings(mfpc.mia(), ck, cl, 3.5, 2., 12.);
		 	
		 	for (auto &n: ordered_neighs[1.]) 
	 		{
	 			if (dm.state(n.k, n.l) == DepthInfo::State::UNINITIALIZED)  
	 			{
	 				microimages.push(n);
	 			}
	 		}
		 	
		 	//filter texture		 	
		 	if (not (strategies.filter) or is_contrasted_enough(mfpc.mia(), scene, ck, cl))
		 	{
			 	initialize_depth(
			 		hypothesis,
			 		ordered_neighs[1.7], mfpc, scene, 
			 		strategies
			 	);
		 	}
		 	
		 	if (not (hypothesis.is_valid()) or not (dm.is_valid_depth(hypothesis.depth())))
		 	{
		 		dm.depth(ck, cl) 		= DepthInfo::NO_DEPTH;
		 		dm.state(ck, cl) 		= DepthInfo::State::COMPUTED;	
		 		dm.confidence(ck, cl) 	= DepthInfo::NO_CONFIDENCE;
		 		
		 		continue;		 	
		 	}
		 	
		 	dm.depth(ck, cl) = hypothesis.depth();
		 	dm.state(ck, cl) = DepthInfo::State::INITIALIZED;
		 		 	
		 	//update inverse depth observation
	 		{	
		 		const double dmin = dm.min_depth();
				const double dmax = dm.max_depth(); 
		 		double stepv = 0.;
		 		
		 		if (strategies.search == SearchStrategy::BRUTE_FORCE) stepv = (dmax - dmin) / nbsample;
		 		else if (strategies.search == SearchStrategy::GOLDEN_SECTION) stepv = hypothesis.sigma;
		 		
		 		const double sigmav = (stepv * stepv);
		 		double muz = hypothesis.invdepth();
	 		
	 			hypothesis.sigma = muz * muz * muz * muz * sigmav;
	 		}
	 		
	 		dm.confidence(ck, cl) = hypothesis.sigma; //at init contains sigma
		}
		
		//Compute depth hypothesis
		if(dm.state(ck,cl) == DepthInfo::State::INITIALIZED)
		{
			DepthHypothesis hypothesis;
				hypothesis.depth() 	= dm.depth(ck, cl);
				hypothesis.sigma	= dm.confidence(ck, cl);
				hypothesis.k 		= ck;
				hypothesis.l 		= cl;
							
			const double maxdepth = hypothesis.depth() + PHI2;
			
			std::map<double, NeighborsIndexes> ordered_neighs = 
				neighbors_by_rings(mfpc.mia(), ck, cl, maxdepth, 2., 12.);
						
			for(auto& [baseline, neighs]: ordered_neighs)
			{
		 		UNUSED(baseline);
		 		
		 		const auto& [nk, nl] = neighs[0];
		 		
		 		const double B = (mfpc.mla().nodeInWorld(ck, cl) - mfpc.mla().nodeInWorld(nk, nl)).head<2>().norm() / mfpc.sensor().scale();
			 				 	
			 	DepthHypothesis nhypothesis;
					nhypothesis.depth() = hypothesis.depth();
					nhypothesis.sigma	= 1. / (B * B);
					nhypothesis.k 		= ck;
					nhypothesis.l 		= cl;
			 	
			 				 	
			 	if (strategies.search == SearchStrategy::NONLIN_OPTIM)
				{	
					optimize_depth(nhypothesis, neighs, mfpc, scene, strategies);
				}
				else if (strategies.search == SearchStrategy::BRUTE_FORCE)
				{					
					const double dmin = 1. / (hypothesis.invdepth() + N * std::sqrt(hypothesis.sigma));
					const double dmax = 1. / (hypothesis.invdepth() - N * std::sqrt(hypothesis.sigma)); 
										
					nhypothesis.min = std::max(std::min(dmin, dmax), dm.min_depth());
					nhypothesis.max = std::max(dmax, dmin);
					nhypothesis.precision = nbsample;
					
					//compute depth hypothesis
					bruteforce_depth(
						nhypothesis,
						neighs, mfpc, scene, 
						strategies
					);
															
					//compute invz sigma 
					{
						const double stepv = (nhypothesis.max - nhypothesis.min) / nbsample; 
		 				const double sigmav = (stepv * stepv);
						const double z = nhypothesis.invdepth();
						
		 				nhypothesis.sigma = z * z * z * z * sigmav;
					}
				}
				else if (strategies.search == SearchStrategy::GOLDEN_SECTION)
				{
					const double dmin = hypothesis.depth() - N;
					const double dmax = hypothesis.depth() + N;
										
					nhypothesis.min = std::max(std::min(dmin, dmax), dm.min_depth());
					nhypothesis.max = std::max(dmax, dmin);
					nhypothesis.precision = std::sqrt(strategies.precision);
					
					gss_depth(
						nhypothesis,
						neighs, mfpc, scene, 
						strategies
					);
												
					{
						const double sigmav = (nhypothesis.sigma * nhypothesis.sigma);
						const double z = nhypothesis.invdepth();
		 				nhypothesis.sigma = z * z * z * z * sigmav;
					}	
				}
							
			 	//if no observation has been taken into account or estimation is not valid, continue
				if (not (nhypothesis.is_valid()) or not (dm.is_valid_depth(nhypothesis.depth()))) 
				{
					continue;
				}
				
				//update hypothesis
				hypothesis += nhypothesis;		 				 			
			}
						
			//set depth
	 		dm.state(ck, cl) = DepthInfo::State::COMPUTED;	
	 		dm.depth(ck, cl) = hypothesis.depth();	
	 		
	 		dm.confidence(ck, cl) = hypothesis.confidence(); 	
		}
	}
	PRINT_DEBUG("Local probabilistic depth estimation finished.");
}


//******************************************************************************
//******************************************************************************
//******************************************************************************
void compute_depthmap_from_obs(
	DepthMap& dm, 
	const PlenopticCamera& mfpc, const Image& scene, 
	const BAPObservations& observations
)
{	
	DEBUG_ASSERT((dm.is_coarse_map()), "The map type must be set to COARSE.");
	
//1) Initialize hypothesis
	const std::size_t I = (mfpc.I() == 0) ? 1 : mfpc.I();
	
	double mdfp = 0.; //mean distance focal plane
	for(std::size_t i =0; i < I; ++i) mdfp +=  mfpc.focal_plane(i);
	mdfp /= double(I);
	const double v = mfpc.obj2v(mdfp*0.8);
	
	DepthHypothesis hypothesis;
		hypothesis.depth() = v;

//2) Optimize depth
	optimize_depth_from_obs(
		hypothesis, observations, mfpc, scene
	);
	
//3) Build depth map
	for (const auto& ob : observations)	
	{
		dm.depth(ob.k, ob.l) = hypothesis.depth();
		dm.state(ob.k, ob.l) = DepthInfo::COMPUTED;
	}
}
