#include "filter.h"

#include "processing/tools/stats.h"

#include "neighbors.h"
//******************************************************************************
//******************************************************************************
DepthMap median_filter_depth(const DepthMap& dm, const PlenopticCamera& mfpc, double size, bool permicroimage)
{
	const auto& mia = mfpc.mia();
	
	DepthMap filtereddm{dm};
	
	constexpr std::size_t margin = 2;
	
	if (dm.is_coarse_map())
	{
		const std::size_t kmax = dm.width()-margin; 
		const std::size_t kmin = 0+margin;
		const std::size_t lmax = dm.height()-margin; 
		const std::size_t lmin = 0+margin;
		
		for(std::size_t k = kmin; k < kmax; ++k)
		{
			for(std::size_t l = lmin; l < lmax; ++l)
			{
				double sz = size;
				if (sz == AUTOMATIC_FILTER_SIZE) 
				{
					sz = dm.depth(k,l);
					if (sz != DepthInfo::NO_DEPTH and dm.is_metric_depth()) 
					{
						const P2D idx =  mfpc.mi2ml(k,l);
						sz = mfpc.obj2v(dm.depth(k,l), idx(0), idx(1));
					}
				}
				//get neighbors
			 	NeighborsIndexes neighs = neighbors(mia, k, l, sz, sz); 
				
				std::vector<double> depths; depths.reserve(neighs.size()+1);
				depths.emplace_back(dm.depth(k,l));
				
				for(auto&n : neighs) depths.emplace_back(dm.depth(n.k, n.l));
			
				filtereddm.depth(k,l) = median(depths);
			}
		}
	}
	else if (dm.is_refined_map())
	{
		if (permicroimage)
		{	
			const std::size_t kmax = dm.width()-margin; 
			const std::size_t kmin = 0+margin;
			const std::size_t lmax = dm.height()-margin; 
			const std::size_t lmin = 0+margin;
			
			for(std::size_t k = kmin; k < kmax; ++k)
			{
				for(std::size_t l = lmin; l < lmax; ++l)
				{					
					const std::size_t sz = static_cast<std::size_t>(size == AUTOMATIC_FILTER_SIZE ? 2. : size);
					
					//get neighbors				
					std::vector<double> depths; depths.reserve((sz * 2 + 1) * (sz * 2 + 1));
					
					for (std::size_t u = std::max(kmin, k - sz); u < std::min(kmax, k + sz); ++u)
					{
						for (std::size_t v = std::max(lmin, l - sz); v < std::min(lmax, l + sz); ++v)
						{
							if (double depth = dm.depth(u,v); depth != DepthInfo::NO_DEPTH)
								depths.emplace_back(dm.depth(u,v));
						}
					}
								
					filtereddm.depth(k,l) = median(depths);
				}
			}
		}
		else
		{
			constexpr std::size_t margin = 0;
		
			const std::size_t kmax = mia.width()-margin; 
			const std::size_t kmin = 0+margin;
			const std::size_t lmax = mia.height()-margin; 
			const std::size_t lmin = 0+margin;
			
			for(std::size_t k = kmin; k < kmax; ++k)
			{
				for(std::size_t l = lmin; l < lmax; ++l)
				{
					//get pixels
				 	const NeighborsIndexes pixels = pixels_neighbors(mfpc.mia(), dm.width(), dm.height(), k, l); 
					
					for (const auto& pixel : pixels)
					{		
						double d = dm.depth(pixel.k, pixel.l);
						if (d == DepthInfo::NO_DEPTH) continue;	
						
						if (dm.is_metric_depth()) 
						{
							const P2D idx =  mfpc.mi2ml(k, l);
							d = mfpc.obj2v(dm.depth(pixel.k, pixel.l), idx(0), idx(1));
						}
						
						//get neighbors
					 	NeighborsIndexes neighs = neighbors(mfpc.mia(), k, l, d, d); 	
					 	std::vector<double> depths; depths.reserve(neighs.size() + 1);
					 	depths.emplace_back(d);
					 	
						for(auto&n : neighs) 
						{
							const P2D disparity = mfpc.disparity(k, l, n.k, n.l, d);
							const std::size_t nk = static_cast<std::size_t>(pixel.k - disparity[0]); 
							const std::size_t nl = static_cast<std::size_t>(pixel.l - disparity[1]); 
							
							const double nd = dm.depth(nk, nl); 
							if (nd == DepthInfo::NO_DEPTH) continue;
							depths.emplace_back(nd);						
						}
						
						filtereddm.depth(pixel.k,pixel.l) = (depths.size() > 3 ? median(depths) : DepthInfo::NO_DEPTH);
					}
				}
			}
		}
	}
	
	return filtereddm;	
}

void inplace_median_filter_depth(DepthMap& dm, const PlenopticCamera& mfpc, double size, bool permicroimage)
{	
	const DepthMap temp = median_filter_depth(dm, mfpc, size, permicroimage);
	temp.copy_to(dm);
} 

//******************************************************************************
DepthMap mean_filter_depth(const DepthMap& dm, const PlenopticCamera& mfpc, double size)
{
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");
	
	const auto& mia = mfpc.mia();
	
	DepthMap filtereddm{dm};
	
	constexpr std::size_t margin = 2;
	
	const std::size_t kmax = dm.width()-margin; 
	const std::size_t kmin = 0+margin;
	const std::size_t lmax = dm.height()-margin; 
	const std::size_t lmin = 0+margin;
	
	for(std::size_t k = kmin; k < kmax; ++k)
	{
		for(std::size_t l = lmin; l < lmax; ++l)
		{
			double sz = size;
			if (sz == AUTOMATIC_FILTER_SIZE) 
			{
				sz = dm.depth(k,l);
				if (sz != DepthInfo::NO_DEPTH and dm.is_metric_depth()) 
				{
					const P2D idx =  mfpc.mi2ml(k,l);
					sz = mfpc.obj2v(dm.depth(k,l), idx(0), idx(1));
				}
			}
			
			//get neighbors
		 	NeighborsIndexes neighs = neighbors(mia, k, l, sz, sz); 
			
			std::vector<double> depths; depths.reserve(neighs.size()+1);
			depths.emplace_back(dm.depth(k,l));
			
			for(auto&n : neighs) 
				if (double nd = dm.depth(n.k, n.l); nd != DepthInfo::NO_DEPTH) 
					depths.emplace_back(nd);
		
			filtereddm.depth(k,l) = mean(depths);
		}
	}
	
	return filtereddm;	
}
void inplace_mean_filter_depth(DepthMap& dm, const PlenopticCamera& mfpc, double size)
{
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");
	
	const DepthMap temp = mean_filter_depth(dm, mfpc, size);
	temp.copy_to(dm);
} 

//******************************************************************************
DepthMap minmax_filter_depth(const DepthMap& dm, double min, double max)
{	
	DepthMap temp{dm};
	inplace_minmax_filter_depth(temp, min, max);
	
	return temp;
}

void inplace_minmax_filter_depth(DepthMap& dm, double min, double max)
{		
	constexpr std::size_t margin = 2;
	
	const std::size_t kmax = dm.width()-margin; 
	const std::size_t kmin = 0+margin;
	const std::size_t lmax = dm.height()-margin; 
	const std::size_t lmin = 0+margin;
	
	for(std::size_t k = kmin; k < kmax; ++k)
	{
		for(std::size_t l = lmin; l < lmax; ++l)
		{
			const double d = dm.depth(k,l);
			if(d < min or d > max) dm.depth(k,l) = DepthInfo::NO_DEPTH;
		}
	}
}

//******************************************************************************
//******************************************************************************
DepthMap erosion_filter_depth(const DepthMap& dm, const PlenopticCamera& mfpc, double size)
{
	const auto& mia = mfpc.mia();
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");
	
	DepthMap filtereddm{dm}; //depths are copied
	
	constexpr std::size_t margin = 2;
	
	const std::size_t kmax = dm.width()-margin; 
	const std::size_t kmin = 0+margin;
	const std::size_t lmax = dm.height()-margin; 
	const std::size_t lmin = 0+margin;
	
	for(std::size_t k = kmin; k < kmax; ++k)
	{
		for(std::size_t l = lmin; l < lmax; ++l)
		{
			if(dm.depth(k,l) == DepthInfo::NO_DEPTH)
			{
				//get neighbors
		 		NeighborsIndexes neighs;
		 		
		 		if (size == AUTOMATIC_FILTER_SIZE) neighs = inner_ring(mia, k, l);
		 		else neighs = neighbors(mia, k, l, size, size); 
		 		
		 		for(auto& n: neighs) filtereddm.depth(n.k, n.l) = DepthInfo::NO_DEPTH;
			}
		}
	}
	
	return filtereddm;	
}
void inplace_erosion_filter_depth(DepthMap& dm, const PlenopticCamera& mfpc, double size)
{
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");
	
	const DepthMap temp = erosion_filter_depth(dm, mfpc, size);
	temp.copy_to(dm);
} 

//******************************************************************************
//******************************************************************************
DepthMap bilateral_filter_depth(
	const DepthMap& dm, const PlenopticCamera& mfpc, double sigmar, double sigmad, bool permicroimage
)
{
	const auto& mia = mfpc.mia();
	DepthMap filtereddm{dm};
	
	double sr = sigmar;
	if (sr == AUTOMATIC_FILTER_SIZE)
		sr = dm.is_virtual_depth() ? 1. : 20. /* mm */; 
	
	if (dm.is_coarse_map())
	{
		constexpr std::size_t margin = 2;
	
		const std::size_t kmax = dm.width()-margin; 
		const std::size_t kmin = 0+margin;
		const std::size_t lmax = dm.height()-margin; 
		const std::size_t lmin = 0+margin;
		
		auto kernel = [&](std::size_t k, std::size_t l, std::size_t nk, std::size_t nl, double sd, double sr) -> double {
			return std::exp(
				- (mia.nodeInWorld(k,l) - mia.nodeInWorld(nk, nl)).squaredNorm() / (2. * sd * sd)
				- (dm.depth(k, l) - dm.depth(nk, nl)) * (dm.depth(k, l) - dm.depth(nk, nl))	 / (2. * sr * sr)
			);
		};
	
		for(std::size_t k = kmin; k < kmax; ++k)
		{
			for(std::size_t l = lmin; l < lmax; ++l)
			{
				double sd = sigmad;
				if (sd == AUTOMATIC_FILTER_SIZE) 
				{
					sd = dm.depth(k,l);
					if (sd != DepthInfo::NO_DEPTH and dm.is_metric_depth()) 
					{
						const P2D idx =  mfpc.mi2ml(k,l);
						sd = mfpc.obj2v(dm.depth(k,l), idx(0), idx(1));
					}
				}				
				//if (double d = dm.depth(k, l); d == DepthInfo::NO_DEPTH) continue;
				
				//get neighbors
			 	NeighborsIndexes neighs = neighbors(mia, k, l, sd, sd);  
				
				double sum = 0., weight = 0.;	
				for(auto&n : neighs) 
				{
					//const double d = dm.depth(n.k, n.l);
					//if (d == DepthInfo::NO_DEPTH) continue;
					
					const double w = kernel(k, l, n.k, n.l, sd * mia.radius(), sr);
					sum += dm.depth(n.k, n.l) * w;
					weight += w;			
				}
			
				filtereddm.depth(k,l) = sum / (weight + 1e-12);
			}
		}
	}
	else if (dm.is_refined_map())
	{
		auto kernel = [&](std::size_t k, std::size_t l, std::size_t nk, std::size_t nl, double sd, double sr) -> double {
			const double d = dm.depth(k, l);
			const double nd = dm.depth(nk, nl);
			
			return std::exp(
				- (P2D{k,l} - P2D{nk, nl}).squaredNorm() / (2. * sd * sd)
				- (d != DepthInfo::NO_DEPTH) * (d - nd) * (d - nd) / (2. * sr * sr)
			);
		};
		
		constexpr std::size_t margin = 0;
	
		const std::size_t kmax = mia.width()-margin; 
		const std::size_t kmin = 0+margin;
		const std::size_t lmax = mia.height()-margin; 
		const std::size_t lmin = 0+margin;
		
		for(std::size_t k = kmin; k < kmax; ++k)
		{
			for(std::size_t l = lmin; l < lmax; ++l)
			{				
				if (permicroimage)
				{
					double sd = sigmad;
					if (sd == AUTOMATIC_FILTER_SIZE) sd = 0.3;
					
					//get neighbors
				 	NeighborsIndexes neighs = pixels_neighbors(mia, dm.width(), dm.height(), k, l); 
					
					//for each pixel
					for (const auto&c : neighs)
					{
						if (double d = dm.depth(c.k, c.l); d == DepthInfo::NO_DEPTH) continue;
						
						double sum = 0., weight = 0.;	
						for(const auto&n : neighs) 
						{
							const double d = dm.depth(n.k, n.l);
							if (d == DepthInfo::NO_DEPTH) continue;
							
							const double w = kernel(c.k, c.l, n.k, n.l, sd * mia.radius(), sr);
							sum += dm.depth(n.k, n.l) * w;
							weight += w;			
						}
					
						filtereddm.depth(c.k, c.l) = sum / (weight + 1e-12);
					}
				}
				else
				{
					//get pixels
				 	const NeighborsIndexes pixels = pixels_neighbors(mfpc.mia(), dm.width(), dm.height(), k, l); 
					
					for (const auto& pixel : pixels)
					{		
						double d = dm.depth(pixel.k, pixel.l);
						//if (d == DepthInfo::NO_DEPTH) continue;	
						
						if (dm.is_metric_depth()) 
						{
							const P2D idx =  mfpc.mi2ml(k, l);
							d = mfpc.obj2v(dm.depth(pixel.k, pixel.l), idx(0), idx(1));
						}
						
						double sd = sigmad;
						if (sd == AUTOMATIC_FILTER_SIZE) sd = d;
						
						//get neighbors
					 	NeighborsIndexes neighs = neighbors(mfpc.mia(), k, l, d, d); 	
						
						double sum = 0., weight = 0.;
						for(auto&n : neighs) 
						{
							const P2D disparity = mfpc.disparity(k, l, n.k, n.l, d);
							const std::size_t nk = static_cast<std::size_t>(pixel.k - disparity[0]); 
							const std::size_t nl = static_cast<std::size_t>(pixel.l - disparity[1]); 
							
							const double nd = dm.depth(nk, nl); 
							const double w = kernel(pixel.k, pixel.l, nk, nl, sd * mia.radius(), sr);
							
							sum += nd * w;
							weight += w;			
						}
						
						filtereddm.depth(pixel.k, pixel.l) = sum / (weight + 1e-12);
					} //rof pixels	
				} //fi permicroimage
			} //rof l
		} //rof k
	} //fi coarse
	
	return filtereddm;	
}

void inplace_bilateral_filter_depth(
	DepthMap& dm, const PlenopticCamera& mfpc, double sigmar, double sigmad, bool permicroimage
)
{
	const DepthMap temp = bilateral_filter_depth(dm, mfpc, sigmar, sigmad, permicroimage);
	temp.copy_to(dm);
} 


//******************************************************************************
//******************************************************************************
DepthMap consistency_filter_depth(const DepthMap& dm, const PlenopticCamera& mfpc, double threshold)
{	
	DepthMap filtereddm{dm};
	
	constexpr std::size_t margin = 2;
	
	if (dm.is_coarse_map())
	{
		const std::size_t kmax = dm.width()-margin; 
		const std::size_t kmin = 0+margin;
		const std::size_t lmax = dm.height()-margin; 
		const std::size_t lmin = 0+margin;
		
		for(std::size_t k = kmin; k < kmax; ++k)
		{
			for(std::size_t l = lmin; l < lmax; ++l)
			{
				double sz = dm.depth(k,l);
				if (sz != DepthInfo::NO_DEPTH and dm.is_metric_depth()) 
				{
					const P2D idx =  mfpc.mi2ml(k,l);
					sz = mfpc.obj2v(dm.depth(k,l), idx(0), idx(1));
				}
				
				//get neighbors
			 	NeighborsIndexes neighs = neighbors(mfpc.mia(), k, l, sz, sz); 
				
				std::vector<double> depths; depths.reserve(neighs.size()+1);
				depths.emplace_back(dm.depth(k,l));
				
				for(auto&n : neighs) 
					if(double nd = dm.depth(n.k, n.l); nd != DepthInfo::NO_DEPTH) 
						depths.emplace_back(nd);
			
				const double medd = median(depths);
				
				if (std::fabs(medd - dm.depth(k,l)) > threshold or depths.size() <= 3)
					filtereddm.depth(k,l) = DepthInfo::NO_DEPTH;						
			}
		}
	}
	else if (dm.is_refined_map())
	{
		const std::size_t kmax = mfpc.mia().width()-margin; 
		const std::size_t kmin = 0+margin;
		const std::size_t lmax = mfpc.mia().height()-margin; 
		const std::size_t lmin = 0+margin;
		
		for(std::size_t k = kmin; k < kmax; ++k)
		{
			for(std::size_t l = lmin; l < lmax; ++l)
			{
				//get pixels
			 	const NeighborsIndexes pixels = pixels_neighbors(mfpc.mia(), dm.width(), dm.height(), k, l); 
				
				for (const auto& pixel : pixels)
				{		
					double sz = dm.depth(pixel.k, pixel.l);
					if (sz == DepthInfo::NO_DEPTH) continue;	
					
					if (dm.is_metric_depth()) 
					{
						const P2D idx = mfpc.mi2ml(k, l);
						sz = mfpc.obj2v(dm.depth(pixel.k, pixel.l), idx(0), idx(1));
					}
					
					//get neighbors
				 	NeighborsIndexes neighs = neighbors(mfpc.mia(), k, l, sz, sz); 	
					
					std::vector<double> depths; depths.reserve(neighs.size()+1);
					depths.emplace_back(sz);
					
					for(auto&n : neighs) 
					{
						const P2D disparity = mfpc.disparity(k, l, n.k, n.l, sz);
						const std::size_t nk = static_cast<std::size_t>(pixel.k - disparity[0]); 
						const std::size_t nl = static_cast<std::size_t>(pixel.l - disparity[1]); 
						
						if (double nd = dm.depth(nk, nl); nd != DepthInfo::NO_DEPTH) depths.emplace_back(nd);
					}
				
					const double medd = median(depths);
					
					if (depths.size() < 4 or std::fabs(medd - sz) > threshold) filtereddm.depth(pixel.k, pixel.l) = DepthInfo::NO_DEPTH;	
				}					
			}
		}
	}	
	
	return filtereddm;
}

void inplace_consistency_filter_depth(DepthMap& dm, const PlenopticCamera& mfpc, double threshold)
{	
	const DepthMap temp = consistency_filter_depth(dm, mfpc, threshold);
	temp.copy_to(dm);
}

//******************************************************************************
//******************************************************************************
/* E(I,Z)= min I(Z) */ 
DepthMap morph_erosion_filter_depth(const DepthMap& dm, const PlenopticCamera& mfpc, double size)
{
	const auto& mia = mfpc.mia();
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");
	
	DepthMap filtereddm{dm}; //depths are copied
	
	constexpr std::size_t margin = 2;
	
	const std::size_t kmax = dm.width()-margin; 
	const std::size_t kmin = 0+margin;
	const std::size_t lmax = dm.height()-margin; 
	const std::size_t lmin = 0+margin;
	
	for (std::size_t k = kmin; k < kmax; ++k)
	{
		for (std::size_t l = lmin; l < lmax; ++l)
		{
			//get neighbors
	 		NeighborsIndexes neighs;
	 		
	 		if (size == AUTOMATIC_FILTER_SIZE) neighs = inner_ring(mia, k, l);
	 		else neighs = neighbors(mia, k, l, size, size); 
	 		
	 		for (auto& n: neighs)
	 		{
	 			if (filtereddm.depth(k,l) > dm.depth(n.k,n.l) and dm.depth(n.k,n.l) != DepthInfo::NO_DEPTH) //FIXME
	 			{
	 				filtereddm.depth(k,l) = dm.depth(n.k,n.l);
	 			}
	 		}
		}
	}
	
	return filtereddm;	
}
void inplace_morph_erosion_filter_depth(DepthMap& dm, const PlenopticCamera& mfpc, double size)
{
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");
	
	const DepthMap temp = morph_erosion_filter_depth(dm, mfpc, size);
	temp.copy_to(dm);
} 

//******************************************************************************
/* D(I,Z)= max I(Z) */ 
DepthMap morph_dilation_filter_depth(const DepthMap& dm, const PlenopticCamera& mfpc, double size)
{
	const auto& mia = mfpc.mia();
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");
	
	DepthMap filtereddm{dm}; //depths are copied
	
	constexpr std::size_t margin = 2;
	
	const std::size_t kmax = dm.width()-margin; 
	const std::size_t kmin = 0+margin;
	const std::size_t lmax = dm.height()-margin; 
	const std::size_t lmin = 0+margin;
	
	for (std::size_t k = kmin; k < kmax; ++k)
	{
		for (std::size_t l = lmin; l < lmax; ++l)
		{
			//get neighbors
	 		NeighborsIndexes neighs;
	 		
	 		if (size == AUTOMATIC_FILTER_SIZE) neighs = inner_ring(mia, k, l);
	 		else neighs = neighbors(mia, k, l, size, size); 
	 		
	 		for (auto& n: neighs)
	 		{
	 			if (filtereddm.depth(k,l) < dm.depth(n.k,n.l))
	 			{
	 				filtereddm.depth(k,l) = dm.depth(n.k,n.l);
	 			}
	 		}
		}
	}
	
	return filtereddm;	
}
void inplace_morph_dilation_filter_depth(DepthMap& dm, const PlenopticCamera& mfpc, double size)
{
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");
	
	const DepthMap temp = morph_dilation_filter_depth(dm, mfpc, size);
	temp.copy_to(dm);
} 

//******************************************************************************
/* O(I,Z)= D(E(I,Z),Z) */ 
DepthMap morph_opening_filter_depth(const DepthMap& dm, const PlenopticCamera& mfpc, double size)
{	
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");
	
	DepthMap filtereddm{dm};
	inplace_morph_opening_filter_depth(filtereddm, mfpc, size);
	
	return filtereddm;
} 
void inplace_morph_opening_filter_depth(DepthMap& dm, const PlenopticCamera& mfpc, double size)
{
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");
	
	inplace_morph_erosion_filter_depth(dm, mfpc, size);
	inplace_morph_dilation_filter_depth(dm, mfpc, size);
} 

//******************************************************************************
/* C(I,Z)= E(D(I,Z),Z) */ 
DepthMap morph_closing_filter_depth(const DepthMap& dm, const PlenopticCamera& mfpc, double size)
{
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");
	
	DepthMap filtereddm{dm};
	inplace_morph_closing_filter_depth(filtereddm, mfpc, size);
	
	return filtereddm;
} 
void inplace_morph_closing_filter_depth(DepthMap& dm, const PlenopticCamera& mfpc, double size)
{	
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");
	
	inplace_morph_dilation_filter_depth(dm, mfpc, size);
	inplace_morph_erosion_filter_depth(dm, mfpc, size);
} 

//******************************************************************************
/* S(I,Z)= C(O(I,Z),Z) */ 
DepthMap morph_smoothing_filter_depth(const DepthMap& dm, const PlenopticCamera& mfpc, double size)
{
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");

	DepthMap filtereddm{dm};
	inplace_morph_smoothing_filter_depth(filtereddm, mfpc, size);
	
	return filtereddm;
} 
void inplace_morph_smoothing_filter_depth(DepthMap& dm, const PlenopticCamera& mfpc, double size)
{	
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");
	
	inplace_morph_opening_filter_depth(dm, mfpc, size);
	inplace_morph_closing_filter_depth(dm, mfpc, size);
} 

//******************************************************************************
/* DYT(I,Z) = 0.5 * (E(I,Z) + D(I,Z)) */
DepthMap morph_dyt_filter_depth(const DepthMap& dm, const PlenopticCamera& mfpc, double size)
{	
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");
	
	DepthMap filtereddm{dm};
	inplace_morph_dyt_filter_depth(filtereddm, mfpc, size);
	
	return filtereddm;
} 
void inplace_morph_dyt_filter_depth(DepthMap& dm, const PlenopticCamera& mfpc, double size)
{
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");
	
	const DepthMap edm = morph_erosion_filter_depth(dm, mfpc, size);
	const DepthMap ddm = morph_dilation_filter_depth(dm, mfpc, size);
	
	constexpr std::size_t margin = 2;
	
	const std::size_t kmax = dm.width()-margin; 
	const std::size_t kmin = 0+margin;
	const std::size_t lmax = dm.height()-margin; 
	const std::size_t lmin = 0+margin;
	
	for (std::size_t k = kmin; k < kmax; ++k)
	{
		for (std::size_t l = lmin; l < lmax; ++l)
		{
			dm.depth(k,l) = 0.5 * (edm.depth(k,l) + ddm.depth(k,l));
		}
	}
} 

//******************************************************************************
/* TET(I,Z) = 0.5 * (O(I,Z) + C(I,Z)) */
DepthMap morph_tet_filter_depth(const DepthMap& dm, const PlenopticCamera& mfpc, double size)
{
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");
	
	DepthMap filtereddm{dm};
	inplace_morph_tet_filter_depth(filtereddm, mfpc, size);
	
	return filtereddm;	
} 
void inplace_morph_tet_filter_depth(DepthMap& dm, const PlenopticCamera& mfpc, double size)
{
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");
	
	const DepthMap odm = morph_opening_filter_depth(dm, mfpc, size);
	const DepthMap cdm = morph_closing_filter_depth(dm, mfpc, size);
	
	constexpr std::size_t margin = 2;
	
	const std::size_t kmax = dm.width()-margin; 
	const std::size_t kmin = 0+margin;
	const std::size_t lmax = dm.height()-margin; 
	const std::size_t lmin = 0+margin;
	
	for (std::size_t k = kmin; k < kmax; ++k)
	{
		for (std::size_t l = lmin; l < lmax; ++l)
		{
			dm.depth(k,l) = 0.5 * (odm.depth(k,l) + cdm.depth(k,l));
		}
	}
} 

//******************************************************************************
/* OCCO(I,Z) = 0.5 * (O(C(I,Z),Z) + C(O(I,Z),Z)) */
DepthMap morph_occo_filter_depth(const DepthMap& dm, const PlenopticCamera& mfpc, double size)
{
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");
	
	DepthMap filtereddm{dm}; //depths are copied
	inplace_morph_occo_filter_depth(filtereddm, mfpc, size);
	
	return filtereddm;	
} 
void inplace_morph_occo_filter_depth(DepthMap& dm, const PlenopticCamera& mfpc, double size)
{
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");
	
	DepthMap ocdm = morph_closing_filter_depth(dm, mfpc, size);
	inplace_morph_opening_filter_depth(ocdm, mfpc, size);
	
	DepthMap codm = morph_opening_filter_depth(dm, mfpc, size);
	inplace_morph_closing_filter_depth(codm, mfpc, size);
	
	constexpr std::size_t margin = 2;
	
	const std::size_t kmax = dm.width()-margin; 
	const std::size_t kmin = 0+margin;
	const std::size_t lmax = dm.height()-margin; 
	const std::size_t lmin = 0+margin;
	
	for (std::size_t k = kmin; k < kmax; ++k)
	{
		for (std::size_t l = lmin; l < lmax; ++l)
		{
			dm.depth(k,l) = 0.5 * (ocdm.depth(k,l) + codm.depth(k,l));
		}
	}
} 
//******************************************************************************
//******************************************************************************
//******************************************************************************
//******************************************************************************
//******************************************************************************
//******************************************************************************
//******************************************************************************
//******************************************************************************
PointCloud minmax_filter_depth(const PointCloud& pc, double min, double max, Axis ax)
{	
	PointCloud temp{pc};
	inplace_minmax_filter_depth(temp, min, max, ax);
	
	return temp;
}

void inplace_minmax_filter_depth(PointCloud& pc, double min, double max, Axis ax)
{		
	for (std::size_t i = 0; i < pc.nbPoints(); ++i)
	{
		const double x = pc.feature(i).x(); 
		const double y = pc.feature(i).y(); 
		const double z = pc.feature(i).z(); 
		
		//check x
		if ((x < min or x > max) and (ax == Axis::X or ax == Axis::XY or ax == Axis::XZ or ax == Axis::XYZ))
		{
			pc.remove(i);
		}
		//check y
		else if ((y < min or y > max) and (ax == Axis::Y or ax == Axis::XY or ax == Axis::YZ or ax == Axis::XYZ))
		{
			pc.remove(i);
		}
		//check z
		else if ((z < min or z > max) and (ax == Axis::Z or ax == Axis::YZ or ax == Axis::XZ or ax == Axis::XYZ))
		{
			pc.remove(i);
		}
	}
}

//******************************************************************************
PointCloud maxcount_filter_depth(const PointCloud& pc, std::size_t n)
{	
	PointCloud temp{pc};
	inplace_maxcount_filter_depth(temp, n);
	
	return temp;
}

void inplace_maxcount_filter_depth(PointCloud& pc, std::size_t n)
{
#if SAME_SEED 
    static thread_local std::mt19937 mt;
#else
    static thread_local std::random_device rd;
    static thread_local std::mt19937 mt(rd());
#endif
	
	if (pc.size() < n) return;

	std::size_t left = pc.size() - n;
    while (left--) 
    {
    	std::uniform_int_distribution<> dis(0, left);
    	pc.remove(dis(mt));    	
    }
    
    pc.shrink();
}
