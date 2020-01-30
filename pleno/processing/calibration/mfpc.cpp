#include "calibration.h"

#include <omp.h>

#include <type_traits> // std::remove_reference_t

//optimization
#include "optimization/optimization.h"
#include "optimization/errors/mic.h" //MICReprojectionError
#include "optimization/errors/corner.h" //CornerReprojectionError
#include "optimization/errors/radius.h" //RadiusReprojectionError
#include "optimization/errors/extrinsics/extrinsics_bap.h" //ExtrinsicsBAPReprojectionError

#include "geometry/reprojection.h" //bap reprojection + mic reprojection

//processing
#include "processing/tools/rmse.h"		
#include "processing/tools/vector.h" // random_n_unique
#include "processing/tools/matrix.h" // index_to_colRow

#include "processing/algorithms/neighbour_search.h"
#include "processing/algorithms/p3p.h"

//io
#include "io/cfg/observations.h" // save and load
#include "io/cfg/poses.h"

#include "io/printer.h"

//graphic
#include "graphic/gui.h"
#include "graphic/display.h"

//calibration
#include "link.h"
#include "init.h"

template<bool useCornerOnly>
void optimize(
	//OUT
	CalibrationPoses& poses, /* extrinsics */
	PlenopticCamera& model, /* intrinsics */
	//IN
	const CheckerBoard & checkboard,
	const BAPObservations& observations, /*  (u,v,rho) */
	const MICObservations& centers /* c_{k,l} */
)
{	
	using Solver_t = typename
		std::conditional<useCornerOnly,
			lma::Solver<CornerReprojectionError, MicroImageCenterReprojectionError>,
			lma::Solver<CornerReprojectionError, RadiusReprojectionError, MicroImageCenterReprojectionError>
		>::type;

	Solver_t solver{1e-4, 20, 1.0 - 1e-6};//std::numeric_limits<double>::epsilon()};

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
			if constexpr (not useCornerOnly)
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

template void optimize<true>(
	CalibrationPoses&, PlenopticCamera&, const CheckerBoard&,const BAPObservations&, const MICObservations&);
template void optimize<false>(
	CalibrationPoses&, PlenopticCamera&, const CheckerBoard&,const BAPObservations&, const MICObservations&);
	
	
void optimize(
	//OUT
	CalibrationPoses& poses, /* extrinsics */
	//IN
	const PlenopticCamera& model, /* intrinsics */
	const CheckerBoard & checkboard,
	const BAPObservations& observations /*  (u,v,rho) */
)
{
	using ExtrinsicsBAPReprojectionError = ExtrinsicsBlurAwarePlenopticReprojectionError;
	using Solver_t = lma::Solver<ExtrinsicsBAPReprojectionError>;

	Solver_t solver{1e-4, 1000, 1.0 - 1e-8};//std::numeric_limits<double>::epsilon()};

	//split observations according to frame index
	std::unordered_map<Index /* frame index */, BAPObservations> obs;
	for(const auto& ob : observations)
		obs[ob.frame].push_back(ob);		

	for (auto & [p, frame] : poses) //for each frame with its pose
	{
		for (const auto& o : obs[frame]) //for each observation of this frame
		{
			solver.add(
				ExtrinsicsBAPReprojectionError{
					model, checkboard, o
				},
				&p
			);
		}
	}

    solver.solve(lma::DENSE, lma::enable_verbose_output());
}

#if 0
//******************************************************************************
//******************************************************************************
//******************************************************************************
void visualize(
	const PlenopticCamera& mfpc,
	const CalibrationPoses& poses,
	const CheckerBoard & grid,
	const BAPObservations& observations,
	const MICObservations& centers,
	const std::vector<Image>& pictures
)
{
	//Split observations according to frame
	std::unordered_map<int /* frame index */, BAPObservations> obs;
	for(const auto& ob : observations)
		obs[ob.frame].push_back(ob);
		
	auto model = mfpc;
	
GUI(
	display(grid); display(poses);
	
	v::Palette<int> palette;

	const Viewer::Layer center_layer = 100;
	for(const auto& c : centers)
	{	
		RENDER_DEBUG_2D(
  			Viewer::context().layer(center_layer)
  				.name("Center")
  				.pen_color(v::purple).pen_width(1),
  			P2D{c[0], c[1]}
		);
		const auto p = reproject_miccenter(model, c);
		RENDER_DEBUG_2D(
  			Viewer::context().layer(center_layer+1)
  				.name("Reprojected Center")
  				.pen_color(v::cyan).pen_width(1),
  			p
		);		
	}
	Viewer::update();
	Viewer::update();

//For each frame 
	for(auto & [f, ob] : obs)
	{
		PRINT_DEBUG("Display information of frame f = " << f);
		
		for(const auto& [p,f] : poses) if(f == ob[0].frame) model.pose() = p;
		display(model);
#if 1		
		RENDER_DEBUG_2D(
			Viewer::context().layer(Viewer::layer()++).name("Frame f = "+std::to_string(f)), 
			pictures[f]
		);
#endif	
		for(const auto& o : ob)
		{	
			RENDER_DEBUG_2D(
	  			Viewer::context().layer(Viewer::layer()++)
	  				.name("BAP ("+std::to_string(f)+")")
	  				.point_style(v::Pixel)
	  				.pen_color(palette[o.cluster]).pen_width(2),
	  			Disk{P2D{o[0], o[1]}, o[2]}
			);
			const auto rbap = reproject_bapfeature(model, model.pose(), grid, o);
			RENDER_DEBUG_2D(
	  			Viewer::context().layer(Viewer::layer()--)
	  				.name("Reprojected BAP ("+std::to_string(f)+")")
					.point_style(v::Cross)
	  				.pen_color(palette[o.cluster+1]).pen_width(2),
	  			Disk{P2D{rbap[0], rbap[1]}, rbap[2]}
			);
		}

		Viewer::context().point_style(v::Pixel); //restore point style
		Viewer::update();
		Viewer::update();
		std::getchar();	
	}
); //END GUI
}
//******************************************************************************
//******************************************************************************
//******************************************************************************
void evaluate_rmse(
	const PlenopticCamera& mfpc,
	const CalibrationPoses& poses,
	const CheckerBoard & grid,
	const BAPObservations& observations,
	const MICObservations& centers
)
{
	//Split observations according to frame
	std::unordered_map<int /* frame index */, BAPObservations> obs;
	for(const auto& ob : observations)
		obs[ob.frame].push_back(ob);
		
	auto model = mfpc;

	PRINT_DEBUG("\t\tERROR\t\tRMSE");
	PRINT_DEBUG("---------------------------------------------");
	RMSE rmse_total{0., 0}, rmse_center{0., 0};
	for(const auto& c : centers)
	{	
		const auto p = reproject_miccenter(model, c);
		
		rmse_center.add(P2D{P2D{c[0], c[1]} - p});
		rmse_total.add(P2D{P2D{c[0], c[1]} - p});	
	}
	
	std::ofstream ofs("rmse-"+std::to_string(getpid())+".csv");
	if (!ofs.good())
		throw std::runtime_error(std::string("Cannot open file errors.csv"));
	
	ofs << "f,bap,uv,rho,center\n";
	
//For each frame 
	RMSE rmse_bap_all_frame{0., 0}, rmse_uv_all_frame{0., 0}, rmse_rho_all_frame{0., 0};
	for(auto & [f, ob] : obs)
	{
		for(const auto& [p,f] : poses) if(f == ob[0].frame) model.pose() = p;
		
		RMSE rmse_bap{0., 0}, rmse_uv{0., 0}, rmse_rho{0., 0};
		for(const auto& o : ob)
		{	
			const auto rbap = reproject_bapfeature(model, model.pose(), grid, o);
			//BAP
			rmse_bap.add(			P3D{P3D{o[0], o[1], o[2]} - rbap});
			rmse_bap_all_frame.add(	P3D{P3D{o[0], o[1], o[2]} - rbap});	
			//RHO
			rmse_rho.add(			o[2] - rbap[2]);	
			rmse_rho_all_frame.add(	o[2] - rbap[2]);
			rmse_total.add(			o[2] - rbap[2]);
			//UV
			rmse_uv.add(			P2D{P2D{o[0], o[1]} - rbap.head(2)});
			rmse_uv_all_frame.add(	P2D{P2D{o[0], o[1]} - rbap.head(2)});	
			rmse_total.add(			P2D{P2D{o[0], o[1]} - rbap.head(2)});	
		}
		
		PRINT_DEBUG("Frame = " << f);
		PRINT_DEBUG("bap\t"<< (rmse_bap.sum()) << "\t\t" << rmse_bap.get());
		PRINT_DEBUG("uv\t"<< (rmse_uv.sum()) << "\t\t" << rmse_uv.get());
		PRINT_DEBUG("rho\t"<< (rmse_rho.sum()) << "\t\t" << rmse_rho.get());
		PRINT_DEBUG("---------------------------------------------");
		
		std::ostringstream oss;	
		oss << f << ","
			<< rmse_bap.get()	<< ","
			<< rmse_uv.get() 	<< ","
			<< rmse_rho.get() 	<< ","
			<< 0. 	<< "\n"; 
		
		ofs << oss.str();
	}
	
	PRINT_DEBUG("MIC\t"<< (rmse_center.sum()) << "\t\t" << rmse_center.get());
	PRINT_DEBUG("BAP\t"<< (rmse_bap_all_frame.sum()) << "\t\t" << rmse_bap_all_frame.get());
	PRINT_DEBUG("UV\t"<< (rmse_uv_all_frame.sum()) << "\t\t" << rmse_uv_all_frame.get());
	PRINT_DEBUG("RHO\t"<< (rmse_rho_all_frame.sum()) << "\t\t" << rmse_rho_all_frame.get());
	PRINT_DEBUG("---------------------------------------------");
	PRINT_DEBUG("TOT.\t"<< (rmse_total.sum()) << "\t\t" << rmse_total.get());
	
	std::ostringstream oss;	
	oss << -1 << ","
		<< rmse_bap_all_frame.get()	<< ","
		<< rmse_uv_all_frame.get() 	<< ","
		<< rmse_rho_all_frame.get() << ","
		<< rmse_center.get() 		<< "\n"; 
	
	ofs << oss.str();
}
#endif
//******************************************************************************
//******************************************************************************
//******************************************************************************


//******************************************************************************
//******************************************************************************
//******************************************************************************
template<bool useCornerOnly>
void calibration_MFPC(                 
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
	optimize<useCornerOnly>(poses, model, grid, features, centers);
	
	PRINT_INFO("=== Optimization finished! Results:");
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
	visualize(model, poses, grid, features, centers, pictures);
	
	std::getchar();		
}

template void calibration_MFPC<true>(                 
	CalibrationPoses&, PlenopticCamera&, const CheckerBoard&, 
	const BAPObservations&, const MICObservations&, const std::vector<Image>&);
template void calibration_MFPC<false>(                 
	CalibrationPoses&, PlenopticCamera&, const CheckerBoard&, 
	const BAPObservations&, const MICObservations&, const std::vector<Image>&);

//******************************************************************************
//******************************************************************************
//******************************************************************************
void calibration_ExtrinsicsMFPC(                        
	CalibrationPoses& poses, /* out */                   
	const PlenopticCamera& model, /* in */   
	const CheckerBoard & grid,
	const BAPObservations& observations, /*  (u,v,rho) */
	const std::vector<Image>& pictures, /* for GUI only */
	bool useCornerOnly
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

Viewer::enable(true);
//4) Checking the parameters
	PRINT_INFO("=== Computing individual RMSE");
	evaluate_rmse(model, poses, grid, features, {});
	PRINT_INFO("=== Graphically checking the parameters");
	visualize(model, poses, grid, features, {}, pictures);
	
	std::getchar();	
}
