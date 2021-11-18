#include "scaling.h"

#include "geometry/plane.h"
#include "geometry/ray.h"

#include "processing/tools/error.h"

template <typename FunctionType>
ScalingCostError<FunctionType>::ScalingCostError(
	const PlenopticCamera& mfpc_,
	const CheckerBoard& scene_, 
	const DepthMap& dm_,
	const BAPObservations& observations
) : mfpc{mfpc_}, scene{scene_}, dm{dm_}
{
	//split observations according to cluster index
	for(const auto& ob : observations) clusters[ob.cluster].push_back(ob);	
}

template <typename FunctionType>
bool ScalingCostError<FunctionType>::operator()( 
	const FunctionType& f,
	ErrorType& error
) const
{
	error.setZero();
	
	MBE mbe{0., 0u}; //MAE mae{0., 0u}; //RMSE rmse{0., 0u};	
		
	std::map<Index /* cluster index */, P3D> centroids; 
	
	//for each cluster
	for(auto & [cluster, obs] : clusters)
	{
		//compute reprojected point 
		P3D centroid = P3D::Zero(); 
		double n = 0.;
		
		//for each observation
		for (const auto& ob : obs)
		{
			//get u,v
			const double u = std::floor(ob.u);
			const double v = std::floor(ob.v);
			
			//get depth
			double depth = dm.is_refined_map() ? dm.depth(u,v) : dm.depth(ob.k, ob.l);
			if (depth == DepthInfo::NO_DEPTH) continue;
			if (dm.is_virtual_depth()) depth = mfpc.v2obj(depth, ob.k, ob.l);				
			
			//apply scaling transformation
			depth = f(depth);
			
			//get pixel
			const P2D pixel = P2D{ob.u, ob.v};
			
			//get ml indexes
			const P2D kl = mfpc.mi2ml(ob.k, ob.l); 
				
			//raytrace
			Ray3D ray; //in CAMERA frame
			if (mfpc.raytrace(pixel, kl[0], kl[1], ray))
			{
				//get depth plane
				PlaneCoefficients plane; plane << 0., 0., 1., -depth;
				
				//get position
				const P3D point = line_plane_intersection(plane, ray);
			
				//accumulate
				centroid += point; ++n;
			}			
		}
		
		if (n != 0.) 
		{
			centroid /= n;
			centroids[cluster] = std::move(centroid);	
		}				
	}

	//compute distances and errors	
	for (auto it = centroids.cbegin(); it != centroids.cend(); ++it)
	{		
		for (auto nit = std::next(it, 1); nit != centroids.cend(); ++nit)	
		{
			const double ref = (scene.node(it->first) - scene.node(nit->first)).norm();
			const double dist = (it->second - nit->second).norm();
			
			//mae.add(100. * (ref-dist) / ref);
			//rmse.add(100. * (ref-dist) / ref);	
			mbe.add(100. * (ref-dist) / ref);
		}
	}	
	
	error << mbe.get(); //mae.sum(); //rmse.get(); //
	
	return true;
}

template struct ScalingCostError<LinearFunction>;
template struct ScalingCostError<QuadraticFunction>;
