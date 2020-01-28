#include "mfpc.h"

#include "processing/tools/rotation.h"
#include "io/printer.h"

//******************************************************************************
//******************************************************************************
//******************************************************************************
void MultiFocusPlenopticCamera::init(
	const Sensor& sensor_, 
	const MicroImagesArray& mia_, 
	const InternalParameters& params_, 
	double F, double aperture, double h,
	PlenopticCamera::Mode mode
) override
{
	PlenopticCamera::init(sensor_, mia_, params_, F, aperture, h, mode);
	
	const double d = mla.pose().translation()[2] - sensor().pose().translation()[2];
	
	//set focal lengths
	mla().f(0) = (1. / params().c_prime[0] ) * params().kappa_approx * (d / 2.); 
	mla().f(1) = (1. / params().c_prime[1] ) * params().kappa_approx * (d / 2.); 
	mla().f(2) = (1. / params().c_prime[2] ) * params().kappa_approx * (d / 2.); 	 	
}


//******************************************************************************
//******************************************************************************
//******************************************************************************
bool MultiFocusPlenopticCamera::project_radius_through_micro_lens(
	const P3D& p, std::size_t k, std::size_t l, double& radius
) const
{
    // computing radius
    P3D Ckl_mla = mla().node(k,l);
    P3D p_mla = to_coordinate_system_of(mla().pose(), p); // MLA
    P3D p_kl_mla = (p_mla - Ckl_mla); // NODE(K,L)
    
    const double a = (p_kl_mla[2]);
	const double d = std::fabs(mla().pose().translation()[2] - sensor().pose().translation()[2]);
	const double D = std::fabs(mla().pose().translation()[2]);

	P2D mi_index{k,l}; ml2mi(mi_index);
	const double f = mla().f(mi_index[0], mi_index[1]).f;
 
    const double r = params().kappa * (D / (D + d)) * (d / 2.) * ((1. / f) - (1. / a) - (1. / d));
    
    radius = sensor().metric2pxl(r);
    
    return true;
}
	
//******************************************************************************
//******************************************************************************
//******************************************************************************
bool MultiFocusPlenopticCamera::project(
	const P3D& p3d_cam,
    std::size_t k, std::size_t l,
    P3D& bap
) const
{
	P3D p;
    bool is_projected_through_main_lens = project_through_main_lens(p3d_cam, p);
    
    double radius;
    bool is_radius_projected = project_radius_through_micro_lens(p, k, l, radius);

    P2D pixel;
	bool is_projected_through_micro_lens = project_through_micro_lens(p, k, l, pixel);	

	bap.head(2) = pixel;
    bap[2]	= radius;

    return is_projected_through_main_lens and is_radius_projected and is_projected_through_micro_lens;
}

bool MultiFocusPlenopticCamera::project(
	const P3D& p3d_cam,
    std::size_t k, std::size_t l,
    double& rho
) const
{
	P3D p;
    bool is_projected_through_main_lens = project_through_main_lens(p3d_cam, p);
    bool is_radius_projected = project_radius_through_micro_lens(p, k, l, rho);
	
	return is_projected_through_main_lens and is_radius_projected;
}

bool MultiFocusPlenopticCamera::project(
	const P3D& p3d_cam,
    BAPObservations& observations
) const
{
	observations.clear();
	observations.reserve(mla().nodeNbr());

#pragma omp parallel for	
	for(std::size_t k = 0; k < mla().width() ; ++k) //iterate through columns //x-axis
    {
    	for(std::size_t l = 0 ; l < mla().height() ; ++l) //iterate through lines //y-axis
		{
			P3D bap;
			if(project(p3d_cam, k, l, bap))
			{
				observations.emplace_back(
					BAPObservation{
						int(k), int(l), 
						bap[0], bap[1], bap[2], /* u, v, rho */
					}
				);			
			}
		}
	}
	
	observations.shrink_to_fit();
	return observations.size() > 0u;
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
std::ostream& operator<<(std::ostream& os, const MultiFocusPlenopticCamera& mfpc)
{	
	os << *(static_cast<PlenopticCamera*>(&mfpc));		
	os << "\tf = {" << mfpc.mla().f(0) <<", " << mfpc.mla().f(1) << ", " << mfpc.mla().f(2) << "}" << std::endl;
		
	return os;
}
