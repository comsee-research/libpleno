#include "calibration.h"

#include <type_traits> // std::remove_reference_t

//optimization
#include "optimization/optimization.h"
#include "optimization/errors/mic.h" //MICReprojectionError
#include "optimization/errors/corner.h" //CornerReprojectionError
#include "optimization/errors/radius.h" //RadiusReprojectionError
#include "optimization/errors/extrinsics/extrinsics_corner.h" //ExtrinsicsCornerReprojectionError
#include "optimization/errors/extrinsics/extrinsics_bap.h" //ExtrinsicsBlurAwarePlenopticReprojectionError

//io
#include "io/cfg/observations.h" // save and load
#include "io/cfg/poses.h"

#include "io/printer.h"

//graphic
#include "graphic/gui.h"
#include "graphic/display.h"

//calibration
#include "init.h"
#include "link.h"
#include "evaluate.h"

//******************************************************************************
//******************************************************************************
//******************************************************************************
template<bool useCornerOnly>
void optimize(
	//OUT
	CalibrationPoses& poses, /* extrinsics */
	PlenopticCamera& model, /* intrinsics */
	//IN
	const CheckerBoard & checkboard,
	const BAPObservations& observations, /*  (u,v,rho?) */
	const MICObservations& centers /* c_{k,l} */
)
{	
	constexpr bool useRadius = not(useCornerOnly); 
	
	using Solver_t = typename
		std::conditional<useRadius,
			lma::Solver<CornerReprojectionError, RadiusReprojectionError, MicroImageCenterReprojectionError>,
			lma::Solver<CornerReprojectionError, MicroImageCenterReprojectionError>
		>::type;	
		
	//using Solver_t = lma::Solver<CornerReprojectionError, MicroImageCenterReprojectionError>;

	Solver_t solver{1e-4, 25, 1.0 - 1e-6};//std::numeric_limits<double>::epsilon()};
	
	auto extract_f = [&model](std::size_t k, size_t l) -> FocalLength& {
		P2D pkl{k,l}; //ML
		model.ml2mi(pkl); //MI
		
		return model.mla().f(pkl[0], pkl[1]);
	}; 
	
	//split observations according to frame index
	std::unordered_map<Index /* frame index */, BAPObservations> obs;
	for(const auto& ob : observations)
		obs[ob.frame].push_back(ob);		

	for (auto & [p, frame] : poses) //for each frame with its pose
	{
		for (const auto& o : obs[frame]) //for each observation of this frame
		{
			//ADD CORNER OBSERVATIONS
			solver.add(
				CornerReprojectionError{
					model, checkboard, o
				},
				&p,
				&model.mla().pose(),
				&model.mla(),
				&model.sensor(),
				&model.main_lens()//, 
				, &model.main_lens_distortions()
			);
			if constexpr (useRadius)
			{		
				//ADD RADIUS OBSERVATIONS
				solver.add(
					RadiusReprojectionError{
						model, checkboard, o
					},
					&p,
					&model.mla().pose(),
					&model.mla(),
					&(extract_f(o.k, o.l)),
					&model.sensor(),
					&model.main_lens()//, 
					, &model.main_lens_distortions()
				);
			}
		}
	}
	
	for (const auto& c : centers)
    {
    	//ADD CENTER OBSERVATIONS
		solver.add(
			MicroImageCenterReprojectionError{model, c},
			&model.mla().pose(),
			&model.mla(),
			&model.sensor()
		);
    }
    solver.solve(lma::DENSE, lma::enable_verbose_output());
}

template<bool useCornerOnly, typename Observations>
void calibration(                 
	CalibrationPoses& poses, /* out */                                   
	PlenopticCamera& model, /* out */
	const CheckerBoard & grid,
	const Observations& observations, /*  (u,v,rho) */
	const MICObservations& micenters, /* c_{k,l} */
	const std::vector<Image>& pictures /* for GUI only */
)
{
	Observations features;
	features.reserve(observations.size());
	
	MICObservations centers{micenters.begin(), micenters.end()};
	
//1) Init Extrinsics
	PRINT_INFO("=== Init Extrinsics Parameters");	
	init_extrinsics(
		features, poses,
		model, grid, observations,
		pictures
	);
//2) Sanitize Observations
	PRINT_INFO("=== Sanitize Observations");
	
	PRINT_DEBUG("Link center to node index");
	link_center_to_node_index(centers, model.mia());	
		
	PRINT_DEBUG("Remove not affected centers");
	centers.erase(
		std::remove_if(centers.begin(), centers.end(), 
			[](const auto& c){return (not c.isValid);}
		),
		centers.end()
	);
	centers.shrink_to_fit();
	
	PRINT_DEBUG("Remove not affected features");
	features.erase(
		std::remove_if(features.begin(), features.end(), 
			[max_id = grid.nodeNbr()](const auto& f){
				return (not f.isValid) or f.cluster == -1 or f.frame == -1 or f.cluster >= int(max_id);
			}
		),
		features.end()
	);
	features.shrink_to_fit();

	PRINT_DEBUG("Change indexes' space from MI to ML");
	model.mi2ml(features);
	model.mi2ml(centers);
	
	display(grid); display(poses);
	 
//3) Run optimization
	PRINT_INFO("=== Run optimization");
	auto initial_model = model;
	auto initial_poses = poses;
	BAPObservations obs; convert(features, obs);
	
	optimize<useCornerOnly>(poses, model, grid, obs, centers);
	
	PRINT_INFO("=== Optimization finished! Results:");
#if 0
	if constexpr (useCornerOnly)
	{
		auto update_focal_lengths = [&model]() -> void { 
			const double d = std::fabs(model.mla().pose().translation()[2] - model.sensor().pose().translation()[2]);
			//const double D = std::fabs(model.mla().pose().translation()[2]);
			PRINT_WARN("\tf = {" << model.mla().f(0) <<", " << model.mla().f(1) << ", " << model.mla().f(2) << "}");
			
			model.mla().f(0) = (1. / model.params().c_prime[0]) * model.mla().edge_length()[0] * (d / 2.);
			model.mla().f(1) = (1. / model.params().c_prime[1]) * model.mla().edge_length()[0] * (d / 2.);
			model.mla().f(2) = (1. / model.params().c_prime[2]) * model.mla().edge_length()[0] * (d / 2.);	
		};	
		update_focal_lengths();
	}
#endif	
	PRINT_DEBUG("Optimized model:");
	{
		DEBUG_VAR(model);
		
		save("model-intrinsics-"+std::to_string(getpid())+".js", model);
		v::save(
			"model-params-"+std::to_string(getpid())+".js", 
			v::make_serializable(&model.params())
		);
	}
	PRINT_DEBUG("Optimized poses:");
	{
		CalibrationPosesConfig cfg_poses;
		cfg_poses.poses().resize(poses.size());
		
		int i=0;
		for(const auto& [p, f] : poses) {
			DEBUG_VAR(f); DEBUG_VAR(p); 
			cfg_poses.poses()[i].pose() = p;
			cfg_poses.poses()[i].frame() = f;
			++i;
		}
		
		v::save(
			"poses-extrinsics-"+std::to_string(getpid())+".js", 
			cfg_poses
		);
	}
	
Viewer::enable(true);
//4) Checking the parameters
	PRINT_INFO("=== Computing individual RMSE");
	evaluate_rmse(model, poses, grid, features, centers);
	PRINT_INFO("=== Graphically checking the parameters");
	display(model, poses, grid, features, centers, pictures);
	
	std::getchar();		
}


//******************************************************************************
//******************************************************************************
//******************************************************************************
template<typename Observations>
void calibration_PlenopticCamera(                   
	CalibrationPoses& poses, /* out */                                 
	PlenopticCamera& model, /* out */
	const CheckerBoard & grid,
	const Observations& observations, /*  (u,v,rho?) */
	const MICObservations& centers, /* c_{k,l} */
	const std::vector<Image>& pictures /* for GUI only */
)
{
	constexpr bool cornerOnly = true;
	calibration<cornerOnly>(poses, model, grid, observations, centers, pictures);
}

template void calibration_PlenopticCamera<BAPObservations>(
	CalibrationPoses&,PlenopticCamera&,const CheckerBoard&,const BAPObservations&,const MICObservations&,const std::vector<Image>&);
template void calibration_PlenopticCamera<CBObservations>(
	CalibrationPoses&,PlenopticCamera&,const CheckerBoard&,const CBObservations&,const MICObservations&,const std::vector<Image>&);


void calibration_MultiFocusPlenopticCamera(                   
	CalibrationPoses& poses, /* out */                                 
	PlenopticCamera& model, /* out */
	const CheckerBoard & grid,
	const BAPObservations& observations, /*  (u,v,rho) */
	const MICObservations& centers, /* c_{k,l} */
	const std::vector<Image>& pictures /* for GUI only */
)
{
	constexpr bool cornerOnly = false;
	calibration<cornerOnly>(poses, model, grid, observations, centers, pictures);
}



//******************************************************************************
//******************************************************************************
//******************************************************************************
//******************************************************************************
//******************************************************************************
//******************************************************************************
template<bool useCornerOnly, typename Observations>
void optimize(
	//OUT
	CalibrationPoses& poses, /* extrinsics */
	//IN
	const PlenopticCamera& model, /* intrinsics */
	const CheckerBoard & checkboard,
	const Observations& observations /*  (u,v,rho) */
)
{
	constexpr bool useRadius = not(std::is_same_v<Observations, CBObservations>) and not(useCornerOnly); 
	using Error_t = typename
		std::conditional<useRadius,
			ExtrinsicsCornerReprojectionError,
			ExtrinsicsBlurAwarePlenopticReprojectionError
		>::type;
		
	using Solver_t = typename lma::Solver<Error_t>;	

	Solver_t solver{1e-4, 1000, 1.0 - 1e-8};//std::numeric_limits<double>::epsilon()};

	//split observations according to frame index
	std::unordered_map<Index /* frame index */, Observations> obs;
	for(const auto& ob : observations)
		obs[ob.frame].push_back(ob);		

	for (auto & [p, frame] : poses) //for each frame with its pose
	{
		for (const auto& o : obs[frame]) //for each observation of this frame
		{
			solver.add(
				Error_t{
					model, checkboard, o
				},
				&p
			);
		}
	}

    solver.solve(lma::DENSE, lma::enable_verbose_output());
}

template<bool useCornerOnly, typename Observations>
void calibration(                        
	CalibrationPoses& poses, /* out */                   
	const PlenopticCamera& model, /* in */   
	const CheckerBoard & grid,
	const Observations& observations, /*  (u,v,rho?) */
	const std::vector<Image>& pictures /* for GUI only */
)
{
	Observations features;
	features.reserve(observations.size());
	
//1) Init Extrinsics
	PRINT_INFO("=== Init Extrinsics Parameters");
	init_extrinsics(
		features, poses,
		model, grid, observations,
		pictures
	);

//2) Sanitize Observations
	PRINT_INFO("=== Sanitize Observations");	
	PRINT_DEBUG("Remove not affected features");
	features.erase(
		std::remove_if(
			features.begin(), features.end(), 
			[max_id = grid.nodeNbr()](const auto& f){
				return (not f.isValid) or f.cluster == -1 or f.frame == -1 or f.cluster >= int(max_id);
			}
		),
		features.end()
	);
	features.shrink_to_fit();
	
	PRINT_DEBUG("Change indexes' space from MI to ML");
	model.mi2ml(features);
	
//3) Run optimization
	PRINT_INFO("=== Run optimization");
	display(grid); display(poses);
	
	optimize<useCornerOnly>(poses, model, grid, features);

	PRINT_DEBUG("Optimized poses:");
	{
		CalibrationPosesConfig cfg_poses;
		cfg_poses.poses().resize(poses.size());
		
		int i=0;
		for(const auto& [p, f] : poses) {
			DEBUG_VAR(f); DEBUG_VAR(p); 
			cfg_poses.poses()[i].pose() = p;
			cfg_poses.poses()[i].frame() = f;
			++i;
		}
		
		v::save(
			"poses-"+std::to_string(getpid())+".js", 
			cfg_poses
		);
	}		

Viewer::enable(true);
//4) Checking the parameters
	PRINT_INFO("=== Computing individual RMSE");
	evaluate_rmse(model, poses, grid, features, {});
	PRINT_INFO("=== Graphically checking the parameters");
	display(model, poses, grid, features, {}, pictures);
	
	std::getchar();	
}

template<typename Observations>
void calibration_ExtrinsicsPlenopticCamera(                        
	CalibrationPoses& poses, /* out */                   
	const PlenopticCamera& model, /* in */   
	const CheckerBoard & grid,
	const Observations& observations, /*  (u,v,rho?) */
	const std::vector<Image>& pictures /* for GUI only */
)
{
	constexpr bool cornerOnly = true;
	calibration<cornerOnly>(poses, model, grid, observations, pictures);	
}

template void calibration_ExtrinsicsPlenopticCamera<BAPObservations>(CalibrationPoses&,const PlenopticCamera&,const CheckerBoard&,const BAPObservations&,const std::vector<Image>&);
template void calibration_ExtrinsicsPlenopticCamera<CBObservations>(CalibrationPoses&,const PlenopticCamera&,const CheckerBoard&,const CBObservations&,const std::vector<Image>&);

void calibration_ExtrinsicsMultiFocusPlenopticCamera(                        
	CalibrationPoses& poses, /* out */                   
	const PlenopticCamera& model, /* in */   
	const CheckerBoard & grid,
	const BAPObservations& observations, /*  (u,v,rho?) */
	const std::vector<Image>& pictures /* for GUI only */
)
{
	constexpr bool cornerOnly = false;
	calibration<cornerOnly>(poses, model, grid, observations, pictures);	
}
