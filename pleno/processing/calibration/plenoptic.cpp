#include "calibration.h"

#include <type_traits> // std::remove_reference_t
#include <variant> //std::variant

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
	const bool useRadius = (model.I() > 0u); 
	
	using SolverBAP = lma::Solver<CornerReprojectionError, RadiusReprojectionError, MicroImageCenterReprojectionError>;
	using SolverCorner = lma::Solver<CornerReprojectionError, MicroImageCenterReprojectionError>;
	using Solver_t = std::variant<std::monostate, SolverBAP, SolverCorner>;
	
	Solver_t vsolver;
		if(useRadius) vsolver.emplace<SolverBAP>(1e-4, 25, 1.0 - 1e-6);
		else vsolver.emplace<SolverCorner>(1e-4, 25, 1.0 - 1e-6);  
	
	//split observations according to frame index
	std::unordered_map<Index /* frame index */, BAPObservations> obs;
	for(const auto& ob : observations)
		obs[ob.frame].push_back(ob);	

	auto extract_f = [&model](std::size_t k, size_t l) -> FocalLength& {
		P2D pkl{k,l}; //ML 
		model.ml2mi(pkl); //MI
		return model.mla().f(pkl[0], pkl[1]);
	};

	std::visit(
		[&](auto&& s) { 
			using T = std::decay_t<decltype(s)>;
			if constexpr (not std::is_same_v<T, std::monostate>) {
				for (auto & [p, frame] : poses) { //for each frame with its pose
					for (const auto& o : obs[frame]) { //for each observation of this frame
						//ADD CORNER OBSERVATIONS	
						s.add(
							CornerReprojectionError{
								model, checkboard, o
							},
							&p,
							&model.mla().pose(),
							&model.mla(),
							&model.sensor(),
							&model.main_lens(),
							&model.main_lens_distortions()
						);
						//ADD RADII OBSERVATIONS
						if constexpr (std::is_same_v<T, SolverBAP>) {
							s.add(
								RadiusReprojectionError{
									model, checkboard, o
								},
								&p,
								&model.mla().pose(),
								&model.mla(),
								&(extract_f(o.k, o.l)),
								&model.sensor(),
								&model.main_lens(),
								&model.main_lens_distortions()
							);
						}
					}
				}
				for (const auto& c : centers) {								
					//ADD CENTER OBSERVATIONS
					s.add(
						MicroImageCenterReprojectionError{model, c},
						&model.mla().pose(),
						&model.mla(),
						&model.sensor()
					);
				}
				
				s.solve(lma::DENSE, lma::enable_verbose_output());
			}
		}, vsolver
	);	
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
void calibration_PlenopticCamera(                 
	CalibrationPoses& poses, /* out */                                   
	PlenopticCamera& model, /* out */
	const CheckerBoard & grid,
	const BAPObservations& observations, /*  (u,v,rho) */
	const MICObservations& micenters, /* c_{k,l} */
	const std::vector<Image>& pictures /* for GUI only */
)
{
	BAPObservations features;
	features.reserve(observations.size());
	
	MICObservations centers{micenters.begin(), micenters.end()};

FORCE_GUI(true);	
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
FORCE_GUI(false);
	
	evaluate_rmse(model, poses, grid, features, centers);
	display(model, poses, grid, features, centers, pictures);
		
	std::getchar();	
	
	 
//3) Run optimization
	PRINT_INFO("=== Run optimization");
	auto initial_model = model;
	auto initial_poses = poses;
	
	optimize(poses, model, grid, features, centers);
	
	PRINT_INFO("=== Optimization finished! Results:");
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
FORCE_GUI(true);
//4) Checking the parameters
	PRINT_INFO("=== Computing individual RMSE");
	evaluate_rmse(model, poses, grid, features, centers);
	PRINT_INFO("=== Graphically checking the parameters");
	display(model, poses, grid, features, centers, pictures);
		
	std::getchar();	
	
FORCE_GUI(false);	
}



//******************************************************************************
//******************************************************************************
//******************************************************************************
//******************************************************************************
//******************************************************************************
//******************************************************************************
void optimize(
	//OUT
	CalibrationPoses& poses, /* extrinsics */
	//IN
	const PlenopticCamera& model, /* intrinsics */
	const CheckerBoard & checkboard,
	const BAPObservations& observations /*  (u,v,rho) */
)
{
	const bool useRadius = (model.I() > 0u); 
	
	using SolverBAP = lma::Solver<ExtrinsicsBlurAwarePlenopticReprojectionError>;
	using SolverCorner = lma::Solver<ExtrinsicsCornerReprojectionError>;
	using Solver_t = std::variant<std::monostate, SolverBAP, SolverCorner>;
	
	Solver_t vsolver;
		if(useRadius) vsolver.emplace<SolverBAP>(1e-4, 1000, 1.0 - 1e-8);
		else vsolver.emplace<SolverCorner>(1e-4, 1000, 1.0 - 1e-8);  

	//split observations according to frame index
	std::unordered_map<Index /* frame index */, BAPObservations> obs;
	for(const auto& ob : observations)
		obs[ob.frame].push_back(ob);		
	
	std::visit(
		[&](auto&& s) { 
			using T = std::decay_t<decltype(s)>;
			if constexpr (not std::is_same_v<T, std::monostate>) 
			{
				using Error_t = typename std::conditional<std::is_same_v<T, SolverBAP>, 
					ExtrinsicsBlurAwarePlenopticReprojectionError, 
					ExtrinsicsCornerReprojectionError
				>::type;
				
				for (auto & [p, frame] : poses) //for each frame with its pose
					for (const auto& o : obs[frame]) //for each observation of this frame
						s.add(Error_t{model, checkboard, o}, &p);
						
				s.solve(lma::DENSE, lma::enable_verbose_output());
			}
		}, vsolver
	);
}

//******************************************************************************
//******************************************************************************
//******************************************************************************s
void calibration_ExtrinsicsPlenopticCamera(                        
	CalibrationPoses& poses, /* out */                   
	const PlenopticCamera& model, /* in */   
	const CheckerBoard & grid,
	const BAPObservations& observations, /*  (u,v,rho?) */
	const std::vector<Image>& pictures /* for GUI only */
)
{
	BAPObservations features;
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
	
	optimize(poses, model, grid, features);

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

FORCE_GUI(true);
//4) Checking the parameters
	PRINT_INFO("=== Computing individual RMSE");
	evaluate_rmse(model, poses, grid, features, {});
	PRINT_INFO("=== Graphically checking the parameters");
	display(model, poses, grid, features, {}, pictures);
	
	std::getchar();	
	
FORCE_GUI(false);
}
