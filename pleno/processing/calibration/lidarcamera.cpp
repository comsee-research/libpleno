#include "calibration.h"

#include <type_traits> // std::remove_reference_t
#include <variant> //std::variant

//optimization
#include "optimization/optimization.h"
#include "optimization/errors/lidarcamera.h" //LidarCameraCornerReprojectionError, LidarCameraBlurAwarePlenopticReprojectionError

//io
#include "io/cfg/observations.h" // save and load
#include "io/cfg/poses.h"

#include "io/printer.h"
#include "io/choice.h"

//graphic
#include "graphic/gui.h"
#include "graphic/display.h"

//calibration
#include "link.h"
#include "init.h"

//******************************************************************************
//******************************************************************************
//******************************************************************************
//******************************************************************************
//******************************************************************************
//******************************************************************************
void optimize(
	//OUT
	CalibrationPose& transformation, /* extrinsics */
	//IN
	const PlenopticCamera& model, /* intrinsics */
	const PointsConstellation& constellation,
	const BAPObservations& observations /*  (u,v,rho) */
)
{
	const bool useRadius = (model.focused()); 
	
	using SolverBAP = lma::Solver<LidarCameraBlurAwarePlenopticReprojectionError>;
	using SolverCorner = lma::Solver<LidarCameraCornerReprojectionError>;
	using Solver_t = std::variant<std::monostate, SolverBAP, SolverCorner>;
	
	Solver_t vsolver;
		if(useRadius) vsolver.emplace<SolverBAP>(-1., 250, 1.0 - 1e-18);
		else vsolver.emplace<SolverCorner>(-1., 250, 1.0 - 1e-18);  

	Pose p = transformation.pose;	
	
	std::visit(
		[&](auto&& s) { 
			using T = std::decay_t<decltype(s)>;
			if constexpr (not std::is_same_v<T, std::monostate>) 
			{
				using Error_t = typename std::conditional<std::is_same_v<T, SolverBAP>, 
					LidarCameraBlurAwarePlenopticReprojectionError, 
					LidarCameraCornerReprojectionError
				>::type;
				
				for (const auto& o : observations) //for each observation
						s.add(Error_t{model, constellation, o}, &p);
						
				s.solve(lma::DENSE, lma::enable_verbose_output());
			}
		}, vsolver
	);
	
	transformation.pose = p;
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
void calibration_LidarPlenopticCamera(                        
	CalibrationPose& pose, /* in: initial pose, out: optimized pose */                   
	const PlenopticCamera& model, /* in */   
	const PointsConstellation& constellation,
	const BAPObservations& observations, /* (u,v,rho?) */
	const Image& scene
)
{	
	DEBUG_VAR(observations.size());
	
	BAPObservations features{observations.begin(), observations.end()};	
	DEBUG_VAR(features.size());

//1) Link Observations
	PRINT_INFO("=== Link Observations");	
	link_cluster_to_point_constellation_index(features, model, constellation, scene);
	DEBUG_VAR(features.size());
	
	PRINT_DEBUG("Remove not affected features");
	features.erase(
		std::remove_if(
			features.begin(), features.end(), 
			[max_id = constellation.size()](const auto& f){
				return (not f.isValid) or f.cluster == -1 or f.cluster >= int(max_id);
			}
		),
		features.end()
	);
	features.shrink_to_fit();	
	DEBUG_VAR(features.size());
	
	ObservationsConfig cfg_obs;
	cfg_obs.features() = features;
		
	v::save("linked-observations-"+std::to_string(getpid())+".bin.gz", cfg_obs);
	
	PRINT_DEBUG("Change indexes' space from MI to ML");
	model.mi2ml(features);
	
//1) Link Observations
	PRINT_INFO("=== Init extrinsic");
	CalibrationPose extrinsic = pose;		
	init_extrinsic(extrinsic, model, constellation, features, scene);
	DEBUG_VAR(extrinsic.pose);
	pose.pose = extrinsic.pose;
	
//2) Run optimization
	PRINT_INFO("=== Run optimization");
	const CalibrationPose porigin{Pose{}, -1}; 
	pose.frame = 0;
	display(porigin); display(constellation); display(pose);
	
	PointsConstellation constellation2;
	for	(const P3D& pc : constellation)
	{
		const P3D p = to_coordinate_system_of(pose.pose, pc);
		constellation2.add(p);
	}
	display(constellation2); //constellation transformed, coord in (0,0,0), i.e. camera frame
	for (const auto& p : constellation2) DEBUG_VAR(p.transpose());
	
	wait();
	optimize(pose, model, constellation, features);

	PRINT_DEBUG("Optimized pose:" << pose.pose);
	{
		CalibrationPoseConfig cfg_pose;
		cfg_pose.pose() = pose.pose;
		cfg_pose.frame() = pose.frame;
		
		v::save(
			"lidar-camera-pose-"+std::to_string(getpid())+".js", 
			cfg_pose
		);
	}
			
	PointsConstellation constellation3;
	for	(const P3D& pc : constellation) 
	{
		const P3D p = to_coordinate_system_of(pose.pose, pc);
		constellation3.add(p);
	}
	for (const auto& p : constellation3) DEBUG_VAR(p);
	pose.frame = 1;
	display(constellation3); display(pose);
	
	display(model, pose, constellation, features, scene);
	
	wait();
	clear();
}
