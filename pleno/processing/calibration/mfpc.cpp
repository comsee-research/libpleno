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

#include "link.h"


template<bool useCornerOnly>
void optimize(
	//OUT
	CalibrationPoses& poses, /* extrinsics */
	MultiFocusPlenopticCamera& model, /* intrinsics */
	//IN
	const CheckerBoard & checkboard,
	const BAPObservations& observations, /*  (u,v,rho) */
	const MICObservations& centers /* c_{k,l} */
)
{	
	using CornerReprojectionError_t = CornerReprojectionError<MultiFocusPlenopticCamera>;
	using RadiusReprojectionError_t = RadiusReprojectionError<MultiFocusPlenopticCamera>;
	using MICReprojectionError_t 	= MicroImageCenterReprojectionError<MultiFocusPlenopticCamera>;

	using Solver_t = typename
		std::conditional<useCornerOnly,
			lma::Solver<CornerReprojectionError_t, MICReprojectionError_t>,
			lma::Solver<CornerReprojectionError_t, RadiusReprojectionError_t, MICReprojectionError_t>
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
				CornerReprojectionError_t{
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
					RadiusReprojectionError_t{
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
			MICReprojectionError_t{model, c},
			&model.mla().pose(),
			&model.mla(),
			&model.sensor()
		);
    }
    solver.solve(lma::DENSE, lma::enable_verbose_output());
}

template void optimize<true>(
	CalibrationPoses&, MultiFocusPlenopticCamera&, const CheckerBoard&,const BAPObservations&, const MICObservations&);
template void optimize<false>(
	CalibrationPoses&, MultiFocusPlenopticCamera&, const CheckerBoard&,const BAPObservations&, const MICObservations&);
	
	
void optimize(
	//OUT
	CalibrationPoses& poses, /* extrinsics */
	//IN
	const MultiFocusPlenopticCamera& model, /* intrinsics */
	const CheckerBoard & checkboard,
	const BAPObservations& observations, /*  (u,v,rho) */
	bool useCornerOnly = false
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
					, useCornerOnly
				},
				&p
			);
		}
	}

    solver.solve(lma::DENSE, lma::enable_verbose_output());
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
#if 0
//TOOLS
BAPObservations compute_barycenters(const BAPObservations& observations) 
{
	//Split observations according to cluster
	std::unordered_map<int /* cluster index */, BAPObservations> obs;
	for(const auto& ob : observations)
		obs[ob.cluster].push_back(ob);
		
	BAPObservations barycenters;
	barycenters.reserve(obs.size());
	
	struct Accumulator {
		double u, v;
		double n = 0;
	};
		
	for(const auto & [c, ob] : obs)
	{		
		Accumulator acc = std::accumulate(
			ob.begin(),
			ob.end(), 
			Accumulator{0.,0.},
			[](Accumulator acc, const BAPObservation& current) {
				return Accumulator{acc.u + current.u, acc.v + current.v, acc.n+1};
			}
		);
		
		barycenters.emplace_back(
			BAPObservation{
				-1, -1, /* k,l */
				acc.u / acc.n, acc.v / acc.n, -1., /* u,v,rho */
				c, ob[0].frame
			}
		);
	}
	
	return barycenters;
}
//******************************************************************************
//******************************************************************************
//******************************************************************************
template<typename CameraModel_t>
void link_cluster_to_node_index(
    BAPObservations& observations, /* in/out */
    const BAPObservations& barycenters,
	const CameraModel_t& monocular, 
	const CheckerBoard& grid,
    const Pose& pose,
    bool enable_gui = false
)
{
	std::unordered_map<int /* old id */, int /* new id */> id_mapping;
		
	//Find mean dist inter-reprojected node
	P2D p00, p01, p10;
	monocular.project(to_coordinate_system_of(pose, grid.nodeInWorld(0,0)), p00);
	monocular.project(to_coordinate_system_of(pose, grid.nodeInWorld(grid.width()-1, 0)), p01);
	monocular.project(to_coordinate_system_of(pose, grid.nodeInWorld(0, grid.height()-1)), p10);
	const double interdist = ( 
								std::hypot(p00[0] - p01[0], p00[1] - p01[1]) / static_cast<double>(grid.width()-1)
							 + 	std::hypot(p00[0] - p10[0], p00[1] - p10[1])  / static_cast<double>(grid.height()-1)
							 ) / 2. ;
	constexpr double tolerance = 0.6;
	
	//For each checkerboard node
	for(std::size_t k = 0; k < grid.width(); ++k) //iterate through columns //x-axis
    {
    	for(std::size_t l = 0; l < grid.height(); ++l) //iterate through lines //y-axis
		{
			const int id = l * grid.width() + k; //node id
			
			//Project node in image
			const P3D p3d_cam = to_coordinate_system_of(pose, grid.nodeInWorld(k,l));		
			P2D projection; // a projected checkerboard node in IMAGE UV
        	bool projected = monocular.project(p3d_cam, projection);
        	       	
        	if(not projected) 
        	{ 
        		PRINT_ERR("CheckerBoard Node ("<<k<<", "<<l<<") not reprojected in image"); 
        		continue; 
        	}	
        	
        	//Find nearest cluster
        	int cluster = -1;
        	double dist = 1e20;
        	for(const auto &ob : barycenters)
        	{
        		const double new_dist = std::hypot(ob.u - projection[0], ob.v - projection[1]);
        		if( new_dist < dist and new_dist < interdist * tolerance) 
        		{
        			dist = new_dist;
        			cluster = ob.cluster;
        		}
        	}
			
			if(enable_gui)		
			{
				GUI(
					RENDER_DEBUG_2D(Viewer::context().layer(Viewer::layer())
						.point_style(v::Cross).pen_color(v::cyan).pen_width(5)
						.add_text(projection[0], projection[1] + 5, "("+std::to_string(k)+", "+std::to_string(l)+")")
						.name("Projected nodes"),
						projection
					);
				);
			}
			
			if(cluster != -1 and id_mapping.count(cluster) == 0) 
				id_mapping[cluster] = id;
		}
	}
	
	if(enable_gui)	
	{
		GUI(
			Viewer::context().point_style(v::Pixel); //restore point style
			Viewer::update();
		);
	}
	//Assign new cluster id
	for (auto& o : observations)
	{
		if(id_mapping.count(o.cluster) > 0) 
			o.cluster = id_mapping[o.cluster];
		else
		{
			o.cluster = -1;
			o.isValid = false;		
		}
	}
}
#endif

//******************************************************************************
//******************************************************************************
//******************************************************************************
template<typename CameraModel_t, typename Observations_t>
Pose select_best_pose(
	const CameraModel_t& camera, 
	const CheckerBoard& grid,
    const Observations_t& observations, 
    const Poses& poses
)
{
    struct PoseWithError { Pose pose; RMSE rmse; };
    using PosesWithError = AlignedVector<PoseWithError>;
       
    PosesWithError rmse_poses;
    rmse_poses.reserve(poses.size());

    // for each camera pose
    for (const auto& p : poses)
    {
    	PRINT_DEBUG("For pose p = " << p);
        RMSE rmse{0., 0};
        
        Observations_t obs{observations.begin(), observations.end()};
        link_cluster_to_node_index(obs, obs, camera, grid, p);        
        
    	PRINT_DEBUG("For each observation, compute rmse");
        for (const auto& o : obs)
        {
            if(o.cluster == -1) { rmse.add(1e10); continue; }
            
            const P3D p3d_cam = to_coordinate_system_of(p, grid.nodeInWorld(o.cluster));
            
            P2D prediction; // a projected chessboard node projected in IMAGE UV
            if (camera.project(p3d_cam, prediction))
            {
                rmse.add( P2D{P2D{o[0], o[1]} - prediction} );
            }
            else //projected outside the checkboard
            {
            	//PRINT_ERR("Node projected outside of sensor");
                rmse.add(1e10); // penalty
                break;
            }
        }
        
        if(p.translation().z() > 0.) rmse.add(1e20); // penalty if pose is on the otherside

        rmse_poses.emplace_back(PoseWithError{p, rmse});
    }
	
	PRINT_DEBUG("Find the best pose with the lowest rmse");
    // then sort the tuple according to rms
    std::sort(
    	rmse_poses.begin(), rmse_poses.end(),
		[](const auto& a, const auto& b) { 
              return a.rmse.get() < b.rmse.get(); 
        }
    );
	
	PRINT_DEBUG("Best pose is p = " << rmse_poses[0].pose);
   	return rmse_poses[0].pose;
}

template<typename CameraModel_t, typename Observations_t>
Pose estimate_pose(
	const CameraModel_t& model, 
	const CheckerBoard& grid, 
	const Observations_t& barycenters
)
{
	//Configure monocular camera 
	PRINT_DEBUG("Configure monocular camera ");
	ThinLensCamera monocular(model.main_lens(), model.sensor());
	
	//Compute pose candidates using p3p
	PRINT_DEBUG("Compute pose candidates using p3p");
    Observations_t nodes; /* tl - tr - br */
    get_3_corners(barycenters, nodes); 
    for(auto& n : nodes) n.frame = barycenters[0].frame; //set the frame from observations
    
    GUI(
    	for(const auto& n : nodes)
			RENDER_DEBUG_2D(Viewer::context().layer(Viewer::layer())
				.point_style(v::Cross).pen_color(v::pink).pen_width(5)
				.name("Nodes for p3p"),
				P2D{n[0], n[1]}
			);
		Viewer::context().point_style(v::Pixel); //restore point style
		Viewer::update();
	);
    
    std::array<Ray3D, 3> rays; // computing rays corresponding to each nodes
    {
		for (size_t i = 0; i < rays.size(); ++i)
		{
		    rays.at(i).origin() = {0., 0., 0.};
		    P2D pixel = P2D{nodes[i][0], nodes[i][1]}; //in UV space
		    		    
		    monocular.raytrace(pixel, rays.at(i).direction());
		}
    }

    // computing p3p
    Poses candidates(4);
	/** 
	 * IN UV SPACE :
	 *  - the top-left 		(tl) corner is the (0,0) 	node
 	 *	- the top-right 	(tr) corner is the (K, 0) 	node
	 *	- the bottom-right 	(br) corner is the (K,L) 	node
	 **/ 
    bool p3p_ok = solve_p3p(
    	grid.nodeInWorld(0), //tl
    	grid.nodeInWorld(grid.width() - 1), //tr
		grid.nodeInWorld(grid.nodeNbr() -1), //br
		rays.at(0).direction(), 
		rays.at(1).direction(),
		rays.at(2).direction(),
		candidates
	);

	DEBUG_VAR(p3p_ok);
	
	//Select best using RANSAC
	PRINT_DEBUG("Select best using RANSAC");
	Pose extrinsics = select_best_pose(monocular, grid, barycenters, candidates);
			
	return extrinsics;
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
void visualize(
	const MultiFocusPlenopticCamera& mfpc,
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
	const MultiFocusPlenopticCamera& mfpc,
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

//******************************************************************************
//******************************************************************************
//******************************************************************************
void init_extrinsics(
	//OUT
	BAPObservations& features,
	CalibrationPoses& poses,
	//IN
	const MultiFocusPlenopticCamera& model,
	const CheckerBoard & grid,
	const BAPObservations& observations,
	//GUI
	const std::vector<Image>& pictures /* for GUI only */
)
{
	//Split observations according to frame
	std::unordered_map<int /* frame index */, BAPObservations> obs;
	for(const auto& ob : observations)
		obs[ob.frame].push_back(ob);
	
	//Configure monocular camera
	const ThinLensCamera monocular(model.main_lens(), model.sensor());
	
	//CalibrationPoses poses;
	poses.clear();
	poses.reserve(pictures.size());	
	
	//For each frame 
	for(auto & [f, ob] : obs)
	{
		Viewer::stash();
		//Estimate barycenters
		PRINT_DEBUG("Estimate barycenters of frame f = " << f);
		BAPObservations barycenters = compute_barycenters(ob); //IMAGE UV
		
		GUI(
			PRINT_DEBUG("[GUI] Display information of frame f = " << f);
		#if 0		
			RENDER_DEBUG_2D(
				Viewer::context().layer(Viewer::layer()++).name("Frame f = "+std::to_string(f)), 
				pictures[f]
			);	
		#endif	
			display(f, ob);
			display(f, barycenters, tag::Barycenters{});	
		);
				
		//Estimate Pose
		PRINT_DEBUG("Estimate Pose of frame f = " << f);
		Pose pose = estimate_pose(model, grid, barycenters);
		
		PRINT_DEBUG("Sanity check of pose frame f = " << f);
		if( pose.translation()[2] > 0. 
			or not(((pose.translation().array() == pose.translation().array())).all()) //check is_nan
			or not(((pose.rotation().array() == pose.rotation().array())).all()) //check is_nan
		) 
		{
			PRINT_ERR("Wrong hypothesis. Can't fix it. Remove pose and observations of frame f = " << f <<".");
			DEBUG_VAR(pose);
			Viewer::pop();
			continue;
		}
		
		DEBUG_VAR(pose);

		//Link cluster to node index
		PRINT_DEBUG("Link cluster to node index of frame f = " << f);
		link_cluster_to_node_index(ob, barycenters, monocular, grid, pose);
			
		GUI(	
			BAPObservations ubarycenters = compute_barycenters(ob);			
			display(f, ob);
			display(f, ubarycenters, tag::Barycenters{});
				
			Viewer::pop();
			std::getchar();	
		);
		//Viewer::enable(false);
		
		poses.emplace_back( CalibrationPose{ pose, f } );
		features.insert(features.end(), ob.begin(), ob.end());	
	}
}


//******************************************************************************
//******************************************************************************
//******************************************************************************
template<bool useCornerOnly>
void calibration_MFPC(                 
	CalibrationPoses& poses, /* out */                                   
	MultiFocusPlenopticCamera& model, /* out */
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
	CalibrationPoses&, MultiFocusPlenopticCamera&, const CheckerBoard&, 
	const BAPObservations&, const MICObservations&, const std::vector<Image>&);
template void calibration_MFPC<false>(                 
	CalibrationPoses&, MultiFocusPlenopticCamera&, const CheckerBoard&, 
	const BAPObservations&, const MICObservations&, const std::vector<Image>&);

//******************************************************************************
//******************************************************************************
//******************************************************************************
void calibration_ExtrinsicsMFPC(                        
	CalibrationPoses& poses, /* out */                   
	const MultiFocusPlenopticCamera& model, /* in */   
	const CheckerBoard & grid,
	const BAPObservations& observations, /*  (u,v,rho) */
	const std::vector<Image>& pictures, /* for GUI only */
	bool useCornerOnly = false
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
	
	optimize(poses, model, grid, features, useCornerOnly);

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
