#include "calibration.h"

//optimization
#include "optimization/optimization.h"
#include "optimization/errors/blur.h" //RelativeBlurCostError

#include "processing/improcess.h"
#include "processing/tools/lens.h"
#include "io/printer.h"

#include "unused.h"

//******************************************************************************
//******************************************************************************
//******************************************************************************
void optimize(
	//OUT
	BlurProportionalityCoefficient& kappa, /* extrinsics */
	//IN
	const InternalParameters& internals,
	const BAPObservations& observations, /*  (u,v,rho) */
	const std::vector<Image>& images
)
{	
	constexpr int W = 9u;
	
	using Solver_t = lma::Solver<RelativeBlurCostError>;
	
	Solver_t solver{1e3, 25, 1.0 - 1e-12};
	
	//split observations according to frame index
	std::unordered_map<Index /* frame index */, BAPObservations> obs;
	for(const auto& ob : observations)
		obs[ob.frame].push_back(ob);	

	//for each frame
	for(auto & [frame, baps]: obs)
	{ 
		//split observations according to cluster index
		std::unordered_map<Index /* cluster index */, BAPObservations> clusters;
		for(const auto& ob : baps)
			clusters[ob.cluster].push_back(ob);	
		
		//for each cluster
		for(auto & [cluster, obs_] : clusters)
		{
			UNUSED(cluster);
			
			//for each pair of observations
			for(std::size_t i = 0; i < obs_.size() ; ++i)
			{
				auto current = obs_.begin()+i;

				std::for_each( current+1, obs_.end(), 
					[lhs=*current, I=internals.I, &W, img=images[frame], &kappa, &solver](const auto &rhs) -> void {
						//check type
						if (lens_type(I, lhs.k, lhs.l) != lens_type(I, rhs.k, rhs.l))
						{
							//compute rho_r
							const double rho_r = std::sqrt(std::abs(lhs.rho * lhs.rho - rhs.rho * rhs.rho));
							
							float refu = 0.f, refv = 0.f, ediu = 0.f, ediv = 0.f;
							
							//get more in-focus image
							if(std::abs(lhs.rho) > std::abs(rhs.rho)) //lhs -> ref, rhs -> edi
							{
								refu = lhs.u; refv = lhs.v;
								ediu = rhs.u; ediv = rhs.v;
							}
							else //rhs -> ref, lhs -> edi
							{
								refu = rhs.u; refv = rhs.v;
								ediu = lhs.u; ediv = lhs.v;
							}
							
							Image ref, edi;
							cv::getRectSubPix(img, cv::Size{W,W}, cv::Point2f{refu, refv}, ref);
							cv::getRectSubPix(img, cv::Size{W,W}, cv::Point2f{ediu, ediv}, edi);
							
							solver.add(
								RelativeBlurCostError{ref, edi, rho_r},
								&kappa
							);
						}					
					}
				);
			}
		}
	}
	
	solver.solve(lma::DENSE, lma::enable_verbose_output());
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
void calibration_relativeBlur(
	InternalParameters& internals,     
	const BAPObservations& observations, /*  (u,v,rho) */
	const std::vector<Image>& images
)
{
//1) Init Parameters
	PRINT_INFO("=== Init Parameter");	
	BlurProportionalityCoefficient kappa{internals.kappa};
		 
//3) Run optimization
	PRINT_INFO("=== Run optimization");	
	optimize(kappa, internals, observations, images);
	
	PRINT_INFO("=== Optimization finished! Results:");
	DEBUG_VAR(kappa.kappa);
	
	internals.kappa = kappa.kappa;

	std::getchar();	
}
