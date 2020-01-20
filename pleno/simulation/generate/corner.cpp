#include "generate.h"

#include "geometry/reprojection.h"

void generate_corner(
	CBObservations& observations, 
	const CameraModel_t& model,
	const Pose& pose,
	const CheckerBoard& grid
)
{
	const std::size_t K = model.mla().width();
	const std::size_t L = model.mla().height();
	const int nodeNbr 	= grid.nodeNbr();
	
	//For each checkboard node
	for(int cluster = 0 ; cluster < nodeNbr ; ++cluster)
	{
		for(std::size_t k = 0 ; k < K ; ++k)
		{
			for(std::size_t l = 0 ; l < L ; ++l)
			{
				const P3D p3d = grid.nodeInWorld(cluster); // WORLD
				const P3D p3d_cam = to_coordinate_system_of(pose, p3d); // CAMERA

				P2D prediction; //IMAGE UV
				bool is_projected = model.project(p3d_cam, k, l, prediction);		
				
				if( not is_projected ) continue;
				
				P2D kl{k,l}; model.ml2mi(kl); //ML -> MI indexes
				
				observations.emplace_back(
					CBObservation{
						static_cast<int>(kl[0]), static_cast<int>(kl[1]),
						prediction[0], prediction[1], //u,v
						cluster
					}
				);	
			}
		}
	}
}


