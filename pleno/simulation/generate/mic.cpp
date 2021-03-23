#include "generate.h"

#include "geometry/reprojection.h"

void generate_mic(
	MICObservations& observations, 
	const CameraModel_t& model
)
{
	const std::size_t nbnode = model.mla().nodeNbr();
	observations.clear();
	observations.reserve(nbnode);
	
	//in ML space
	for(int k = 0 ; k < int(model.mla().width()) ; ++k)
	{
		for(int l = 0 ; l < int(model.mla().height()) ; ++l)
		{
			MICObservation obs;
			obs.k = k; obs.l = l;
			const P2D c = reproject_miccenter(model, obs);
			obs[0] = c[0], obs[1] = c[1];
			
			observations.emplace_back(obs);		
		}	
	}
	observations.shrink_to_fit();
	
	//to MI space
	model.ml2mi(observations);
}


