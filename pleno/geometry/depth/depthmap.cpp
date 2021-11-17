#include "depthmap.h"

//******************************************************************************
// constructors
//******************************************************************************
DepthMap::DepthMap(
	std::size_t width, std::size_t height,
	double mind, double maxd,
	DepthType dtype, MapType mtype
) : //map
	map{height/* rows */, width /* cols */},
	//distances
	min_depth_{mind}, max_depth_{maxd},
	//depth map configuration
	depth_type{dtype}, map_type{mtype}
{
}

DepthMap::DepthMap(const DepthMap& o)
:	//map
	map{o.height(), o.width()},
	//distances
	min_depth_{o.min_depth()}, max_depth_{o.max_depth()},
	//depth map configuration
	depth_type{o.depth_type}, map_type{o.map_type}
{
	copy_from(o); //copy depth map data
}

DepthMap::DepthMap(DepthMap&& o)
:	//map
	map{std::move(o.map)},
	//distances
	min_depth_{o.min_depth()}, max_depth_{o.max_depth()},
	//depth map configuration
	depth_type{o.depth_type}, map_type{o.map_type}
{
}


DepthMap::DepthMap(const PointCloud& pc, const PlenopticCamera& mfpc)
:	//map
	map{mfpc.sensor().height(), mfpc.sensor().width()},
	//distances
	min_depth_{pc.min()}, max_depth_{pc.max()},
	//depth map configuration
	depth_type{METRIC}, map_type{REFINED}
{
	using ZBufferContainer = Eigen::Matrix<std::vector<double>, Eigen::Dynamic /* row */, Eigen::Dynamic /* col */>;
	
	ZBufferContainer mzbuffer{height(), width()};
	auto zbuffer = [&](int k, int l) -> std::vector<double>& { return mzbuffer(l, k); }; 
	
	#pragma omp parallel for
	for(int l = 0; l < int(height()); ++l)
		for(int k = 0; k < int(width()); ++k)
			zbuffer(k, l).reserve(20);

	const double R = std::ceil(mfpc.mia().radius());
	
	#pragma omp parallel for 
	//for each point in the point cloud
	for (int j = 0; j < int(pc.size()); ++j)
	{
		const P3D& p = pc.feature(j);
		//project point
		BAPObservations obs;
		mfpc.project(p, obs);
		
		//for each reprojected point
		for (std::size_t i = 0; i < obs.size(); ++i)
		{
			const auto& o 	= obs[i];
			const int u_ 	= static_cast<int>(std::trunc(o.u));
			const int v_ 	= static_cast<int>(std::trunc(o.v));
			const int rho_ 	= static_cast<int>(std::ceil(std::fabs(o.rho))); 
			
			//get micro-image
			const P2D idx =  mfpc.ml2mi(o.k, o.l);
			const P2D c = mfpc.mia().nodeInWorld(idx(0), idx(1));
			
			const int umin = std::max(static_cast<int>(u_ - rho_), 0);
			const int umax = std::min(static_cast<int>(width()), static_cast<int>(u_ + rho_ + 1));
			const int vmin = std::max(static_cast<int>(v_ - rho_), 0);
			const int vmax = std::min(static_cast<int>(height()), static_cast<int>(v_ + rho_ + 1));
			
			//assign depth for all pixels within blur radius
			for (int v = vmin; v < vmax; ++v)
			{	
				for (int u = umin; u < umax; ++u)
				{
					if (((P2D{u_, v_} - P2D{u, v}).norm() < rho_) and ((c - P2D{u, v}).norm() < R))
					{
						#pragma omp critical
						zbuffer(u, v).emplace_back(p.z());
					}
				}
			}		
		}		
	}	
	
	#pragma omp parallel for
	//assign median of z-buffer if enough observations
	for(int l = 0; l < int(height()); ++l)
	{
		for(int k = 0; k < int(width()); ++k)
		{
			state(k,l) = DepthInfo::State::COMPUTED;			
			if (std::size_t sz = zbuffer(k, l).size(); sz > 1) 
			{
				std::nth_element(std::begin(zbuffer(k, l)), std::begin(zbuffer(k, l)) + sz / 2, std::end(zbuffer(k, l)));
				const double d = zbuffer(k, l)[sz / 2];
				const std::size_t threshold = static_cast<std::size_t>(std::floor(mfpc.obj2v(d))) / 2;
				
				if (sz > threshold) depth(k,l) = d;
			}
		}
	}
}

//******************************************************************************
// accessors
//******************************************************************************
void DepthMap::copy_to(DepthMap& o) const { o.copy_from(*this); }
void DepthMap::copy_from(const DepthMap& o)
{
	DEBUG_ASSERT((o.map_type == map_type), "Can't copy data from different depth map type.");
	
	#pragma omp parallel for
	for(std::size_t k = 0; k < width(); ++k)
	{
		for(std::size_t l = 0; l < height(); ++l)
		{
			depth(k,l) 		= o.depth(k,l);
			confidence(k,l) = o.confidence(k,l);
			state(k,l) 		= o.state(k,l);
		}
	}
}

std::size_t DepthMap::width() const { return map.cols(); }
std::size_t DepthMap::height() const { return map.rows(); }

double DepthMap::depth(std::size_t k, std::size_t l) const { return map(l,k).depth; }
double& DepthMap::depth(std::size_t k, std::size_t l) { return map(l,k).depth; }

double DepthMap::confidence(std::size_t k, std::size_t l) const { return map(l,k).confidence; }
double& DepthMap::confidence(std::size_t k, std::size_t l) { return map(l,k).confidence; }

DepthInfo::State DepthMap::state(std::size_t k, std::size_t l) const { return map(l,k).state; }
DepthInfo::State& DepthMap::state(std::size_t k, std::size_t l) { return map(l,k).state; }

DepthInfo DepthMap::depth_with_info(std::size_t k, std::size_t l) const { return map(l,k); }
DepthInfo& DepthMap::depth_with_info(std::size_t k, std::size_t l) { return map(l,k); }

double DepthMap::min_depth() const { return min_depth_; }
void DepthMap::min_depth(double mind) { min_depth_ = mind;}
double DepthMap::max_depth() const { return max_depth_; }
void DepthMap::max_depth(double maxd) { max_depth_ = maxd; }

//******************************************************************************
// export functions
//******************************************************************************
DepthMap DepthMap::to_metric(const PlenopticCamera& pcm) const 
{
	if(is_metric_depth()) return DepthMap(*this); //already metric map
	
	//else, convert to metric
	const double mind = pcm.v2obj(max_depth());
	const double maxd = std::min(std::fabs(pcm.v2obj(min_depth())), 100. * pcm.focal()); //max 100 * F
	
	DepthMap mdm{width(), height(), mind, maxd, METRIC, map_type};

	#pragma omp parallel for
	//copy and convert depth map data
	for(std::size_t k = 0; k < width(); ++k)
	{
		for(std::size_t l = 0; l < height(); ++l)
		{						
			if(depth(k,l) != DepthInfo::NO_DEPTH)
			{
				P2D idx;
				if (is_refined_map()) { const auto [k_, l_] =  pcm.mia().uv2kl(k, l); idx = pcm.mi2ml(k_, l_); }
				else { idx = pcm.mi2ml(k, l); }
				
				mdm.depth(k,l) = pcm.v2obj(depth(k,l), idx(0), idx(1));
			}
			mdm.state(k,l) = state(k,l);
			mdm.confidence(k,l) = confidence(k,l);
		}
	}
	
	return mdm;	
}

DepthMap DepthMap::to_virtual(const PlenopticCamera& pcm) const 
{
	if(is_virtual_depth()) return DepthMap(*this); //already virtual map
	
	//else, convert to virtual
	const double minv = pcm.obj2v(max_depth());
	const double maxv = std::max(std::fabs(pcm.obj2v(min_depth())), pcm.obj2v(4. * std::ceil(pcm.focal())));
		
	DepthMap vdm{width(), height(), minv, maxv, VIRTUAL, map_type};
	
	#pragma omp parallel for
	//copy and convert depth map data
	for(std::size_t k = 0; k < width(); ++k)
	{
		for(std::size_t l = 0; l < height(); ++l)
		{
			if(depth(k,l) != DepthInfo::NO_DEPTH)
			{
				P2D idx;
				if (is_refined_map()) { const auto [k_, l_] =  pcm.mia().uv2kl(k, l); idx = pcm.mi2ml(k_, l_); }
				else { idx = pcm.mi2ml(k, l); }
				
				vdm.depth(k,l) = pcm.obj2v(depth(k,l), idx(0), idx(1));
			}
			vdm.state(k,l) = state(k,l);
			vdm.confidence(k,l) = confidence(k,l);
		}
	}
	
	return vdm;	
}


//******************************************************************************
// Helper functions
//******************************************************************************
bool DepthMap::is_virtual_depth() const { return (depth_type == VIRTUAL); }
bool DepthMap::is_metric_depth() const { return not is_virtual_depth(); }

bool DepthMap::is_coarse_map() const { return (map_type == COARSE); }
bool DepthMap::is_refined_map() const { return not is_coarse_map(); }

bool DepthMap::is_valid_depth(double d) const { return not(is_depth_out_of_bounds(d)); }

bool DepthMap::is_depth_out_of_bounds(double d) const 
{
	return (d >= max_depth() or d < min_depth());
}

//******************************************************************************
// save/load functions
//******************************************************************************
void save(v::OutputArchive& archive, const DepthInfo& di)
{
	std::uint16_t istate = di.state;
	archive
		("depth", di.depth)
		("confidence", di.confidence)
		("state", istate);
}
void load(v::InputArchive& archive, DepthInfo& di)
{
	std::uint16_t istate;
	archive
		("depth", di.depth)
		("confidence", di.confidence)
		("state", istate);
		
	di.state = DepthInfo::State(istate);
}

void save(v::OutputArchive& archive, const DepthMap& dm)
{
	int cols = dm.map.cols();
	int rows =  dm.map.rows();
	int n = cols * rows;
	
	AlignedVector<DepthInfo> depths(n);
	DepthMap::DepthMapContainer::Map(depths.data(),1,n) = dm.map;
	
	bool metric = dm.is_metric_depth();
	bool coarse = dm.is_coarse_map();
	
	double min = dm.min_depth();
	double max = dm.max_depth();
	
	archive
		("map", depths)
		("cols", cols)
		("rows", rows)
		("min", min)
		("max", max)
		("metric", metric)
		("coarse", coarse);
}
void load(v::InputArchive& archive, DepthMap& dm)
{
	AlignedVector<DepthInfo> depths;
	int cols, rows;
	bool metric, coarse;
	double min, max;
	
	archive
		("map", depths)
		("cols", cols)
		("rows", rows)
		("min", min)
		("max", max)
		("metric", metric)
		("coarse", coarse);
		
	dm.map = Eigen::Map<DepthMap::DepthMapContainer>(&depths[0], rows, cols);

	dm.min_depth(min);
	dm.max_depth(max);
	
	dm.map_type = coarse ? DepthMap::MapType::COARSE : DepthMap::MapType::REFINED;
	dm.depth_type = metric ? DepthMap::DepthType::METRIC : DepthMap::DepthType::VIRTUAL;
}
