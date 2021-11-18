#pragma once

#include "types.h"

#include "geometry/camera/models.h" //ThinLensCamera
#include "geometry/object/checkerboard.h" //CheckerBoard
#include "geometry/object/constellation.h" //PointsConstellation

#include "geometry/pose.h"

#include "io/printer.h"
#include "io/choice.h"

#include "graphic/gui.h"
#include "graphic/display.h"

#include "processing/tools/error.h"
#include "processing/algorithms/p3p.h"

#include "link.h"

//******************************************************************************
//******************************************************************************
//******************************************************************************
template<typename CameraModel, typename Observations>
PoseWithError select_best_pose(
	const CameraModel& camera, 
	const CheckerBoard& grid,
    const Observations& observations, 
    const Poses& poses
);

template<typename CameraModel, typename Observations>
PoseWithError estimate_pose(
	const CameraModel& model, 
	const CheckerBoard& grid, 
	const Observations& barycenters
);

template<class CameraModel, class Observations>
void init_extrinsics(
	//OUT
	Observations& features,
	CalibrationPoses& poses,
	//IN
	const CameraModel& model,
	const CheckerBoard & grid,
	const Observations& observations,
	//GUI
	const IndexedImages& pictures = IndexedImages{}/* for GUI only */
);

//******************************************************************************
//******************************************************************************
//******************************************************************************
template<typename CameraModel, typename Observations>
PoseWithError select_best_pose(
	const CameraModel& camera, 
	const PointsConstellation& scene,
    const Observations& observations, 
    const Poses& poses
);

template<typename CameraModel, typename Observations>
PoseWithError estimate_pose(
	const CameraModel& model, 
	const PointsConstellation& scene, 
	const Observations& barycenters
);

template<class CameraModel, class Observations>
void init_extrinsic(
	//OUT
	CalibrationPose& pose,
	//IN
	const CameraModel& model,
	const PointsConstellation& scene,
	const Observations& observations,
	//GUI
	const Image& picture = {} /* for GUI only */
);

#define INIT_AT_FOCAL_LENGTH 0

//******************************************************************************
//******************************************************************************
//******************************************************************************
template<typename CameraModel, typename Observations>
PoseWithError select_best_pose(
	const CameraModel& camera, 
	const CheckerBoard& grid,
    const Observations& observations, 
    const Poses& poses
)
{       
    PosesWithError rmse_poses;
    rmse_poses.reserve(poses.size());

    // for each camera pose
    for (const auto& p : poses)
    {
    	PRINT_DEBUG("For pose p = " << p);
        RMSE rmse{0., 0};
        
        Observations obs{observations.begin(), observations.end()};
        link_cluster_to_node_index(obs, obs, camera, grid, p, false);        
        
    	PRINT_DEBUG("For each observation, compute rmse");
        for (const auto& o : obs)
        {
            if(not o.isValid) { rmse.add(1e10); continue; }
            
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
    for(const auto&p : rmse_poses) PRINT_DEBUG("RMSE = " << p.rmse.get());
	
	PRINT_DEBUG("Best pose is p = " << rmse_poses[0].pose << "with rmse = " << rmse_poses[0].rmse.get());
   	return rmse_poses[0];
}

template<typename CameraModel, typename Observations>
PoseWithError estimate_pose(
	const CameraModel& model, 
	const CheckerBoard& grid, 
	const Observations& barycenters
)
{
	//Configure monocular camera 
	PRINT_DEBUG("Configure monocular camera ");
	Sensor film = Sensor{model.sensor().width(), model.sensor().height(), model.sensor().scale()};
	film.pose() = model.sensor().pose();
#if defined(INIT_AT_FOCAL_LENGTH) && INIT_AT_FOCAL_LENGTH
	film.pose().translation().z() = - model.focal();
#endif
	const PinholeCamera monocular{model.focal(), film};
	
	//Compute pose candidates using p3p
	PRINT_DEBUG("Compute pose candidates using p3p");
    Observations nodes; /* tl - tr - br - bl */
    get_4_corners(barycenters, nodes); 
    {
		//setting indexes and frame
		const int frame = barycenters[0].frame;
		const int K = grid.width() - 1;
		const int L = grid.height() - 1;
		nodes[Corner::TL].k = 0; nodes[Corner::TL].l = 0; nodes[Corner::TL].frame = frame; //tl
		nodes[Corner::TR].k = K; nodes[Corner::TR].l = 0; nodes[Corner::TR].frame = frame; //tr
		nodes[Corner::BR].k = K; nodes[Corner::BR].l = L; nodes[Corner::BR].frame = frame; //br
		nodes[Corner::BL].k = 0; nodes[Corner::BL].l = L; nodes[Corner::BL].frame = frame; //bl
		    
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
	}

    
    std::array<Ray3D, 3> rays; // computing rays corresponding to each nodes
    {//TL
		P2D pixel = P2D{nodes[Corner::TL][0], nodes[Corner::TL][1]}; //in UV space	    
		monocular.raytrace(pixel, rays.at(0));
	}
	{//BL
		P2D pixel = P2D{nodes[Corner::BL][0], nodes[Corner::BL][1]}; //in UV space	    
		monocular.raytrace(pixel, rays.at(1));
	}	
	{//BR
		P2D pixel = P2D{nodes[Corner::BR][0], nodes[Corner::BR][1]}; //in UV space	    
		monocular.raytrace(pixel, rays.at(2));
	}	
    // computing p3p
    Poses candidates(4);
	/** 
	 * IN UV SPACE :
	 *  - the top-left 		(tl) corner is the (0,0) 	node
	 *	- the bottom-left 	(bl) corner is the (0, L) 	node
	 *	- the bottom-right 	(br) corner is the (K,L) 	node
	 **/ 
    bool p3p_ok = solve_p3p(
    	grid.nodeInWorld(nodes[Corner::TL].k, nodes[Corner::TL].l), //tl
    	grid.nodeInWorld(nodes[Corner::BL].k, nodes[Corner::BL].l), //bl
		grid.nodeInWorld(nodes[Corner::BR].k, nodes[Corner::BR].l), //br
		rays.at(0).direction(), 
		rays.at(1).direction(),
		rays.at(2).direction(),
		candidates
	);

	DEBUG_VAR(p3p_ok);
	
	for (const auto &pose : candidates)
	{
		if ( not(((pose.translation().array() == pose.translation().array())).all()) //check is_nan
			or not(((pose.rotation().array() == pose.rotation().array())).all()) //check is_nan
		)
		{
			PRINT_ERR("Pose contains NaN");
			DEBUG_VAR(pose);
			DEBUG_VAR(nodes[Corner::TL]);
			//DEBUG_VAR(nodes[Corner::TR]);
			DEBUG_VAR(nodes[Corner::BR]);
			DEBUG_VAR(nodes[Corner::BL]);
			DEBUG_VAR(rays.at(0));
			DEBUG_VAR(rays.at(1));
			DEBUG_VAR(rays.at(2));
		}
	}
	
	//Select best using RANSAC
	PRINT_DEBUG("Select best using RANSAC");
	PoseWithError extrinsics = select_best_pose(monocular, grid, barycenters, candidates);
			
	return extrinsics;
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
template<class CameraModel, class Observations>
void init_extrinsics(
	//OUT
	Observations& features,
	CalibrationPoses& poses,
	//IN
	const CameraModel& model,
	const CheckerBoard & grid,
	const Observations& observations,
	//GUI
	const IndexedImages& pictures/* for GUI only */
)
{
	const bool usePictures = (pictures.size() > 0u);
	
	//Split observations according to frame
	std::unordered_map<int /* frame index */, Observations> obs;
	for(const auto& ob : observations)
		obs[ob.frame].push_back(ob);
	
	//Configure monocular camera
	Sensor film = Sensor{model.sensor().width(), model.sensor().height(), model.sensor().scale()};
	film.pose() = model.sensor().pose();
#if defined(INIT_AT_FOCAL_LENGTH) && INIT_AT_FOCAL_LENGTH
	film.pose().translation().z() = - model.focal();
#endif
	
	const PinholeCamera monocular{model.focal(), film};
	
	//CalibrationPoses poses;
	poses.clear();
	poses.reserve(pictures.size());	
	
	Viewer::stash();
	//For each frame 
	for(auto & [f, ob] : obs)
	{		
		Viewer::pop();
		Viewer::stash();
		//Estimate barycenters
		PRINT_DEBUG("Estimate barycenters of frame f = " << f);
		Observations barycenters = compute_barycenters(ob); //IMAGE UV
		
		GUI(
			if (usePictures)
			{	
				try { pictures.at(f); }
				catch (std::out_of_range&) { break; }
		
				RENDER_DEBUG_2D(
					Viewer::context().layer(Viewer::layer()).name("Frame f = "+std::to_string(f)), 
					pictures.at(f)
				);	
				Viewer::update();
			}
			PRINT_DEBUG("[GUI] Display information of frame f = " << f);
			display(f, ob);
			display(f, barycenters, tag::Barycenters{});	
		);
				
		//Estimate Pose
		PRINT_DEBUG("Estimate Pose of frame f = " << f);
		const auto [pose, rmse] = estimate_pose(model, grid, barycenters);
		
		PRINT_DEBUG("Sanity check of pose frame f = " << f);
		if( pose.translation()[2] > 0. 
			or rmse.get() > 1e9
			or not(((pose.translation().array() == pose.translation().array())).all()) //check is_nan
			or not(((pose.rotation().array() == pose.rotation().array())).all()) //check is_nan
		) 
		{
			PRINT_ERR("Wrong hypothesis. Can't fix it. Remove pose and observations of frame f = " << f <<".");
			DEBUG_VAR(pose);
			DEBUG_VAR(rmse.get());
			
			wait();
			continue;
		}
		
		DEBUG_VAR(pose);

		//Link cluster to node index
		PRINT_DEBUG("Link cluster to node index of frame f = " << f);
		link_cluster_to_node_index(ob, barycenters, monocular, grid, pose, true);
			
		GUI(	
			Observations ubarycenters = compute_barycenters(ob);			
			display(f, ob);
			display(f, ubarycenters, tag::Barycenters{});
			wait();	
		);
		
		poses.emplace_back( CalibrationPose{ pose, f } );
		features.insert(features.end(), ob.begin(), ob.end());	
	}
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
template<typename CameraModel, typename Observations>
PoseWithError select_best_pose(
	const CameraModel& camera, 
	const PointsConstellation& scene,
    const Observations& observations, 
    const Poses& poses
)
{       
    PosesWithError rmse_poses;
    rmse_poses.reserve(poses.size());

    // for each camera pose
    for (const auto& p : poses)
    {
    	PRINT_DEBUG("For pose p = " << p);
        RMSE rmse{0., 0};
             
    	PRINT_DEBUG("For each observation, compute rmse");
        for (const auto& o : observations)
        {
            if(not o.isValid) { rmse.add(1e10); continue; }
            
            const P3D p3d_cam = to_coordinate_system_of(p, scene.get(o.cluster));
            
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
    for(const auto&p : rmse_poses) PRINT_DEBUG("RMSE = " << p.rmse.get());
	
	PRINT_DEBUG("Best pose is p = " << rmse_poses[0].pose << "with rmse = " << rmse_poses[0].rmse.get());
   	return rmse_poses[0];
}

template<typename CameraModel, typename Observations>
PoseWithError estimate_pose(
	const CameraModel& model, 
	const PointsConstellation& scene, 
	const Observations& barycenters
)
{
	//Configure monocular camera 
	PRINT_DEBUG("Configure monocular camera ");
	Sensor film = Sensor{model.sensor().width(), model.sensor().height(), model.sensor().scale()};
	film.pose() = model.sensor().pose();
#if defined(INIT_AT_FOCAL_LENGTH) && INIT_AT_FOCAL_LENGTH
	film.pose().translation().z() = - model.focal();
#endif
	const PinholeCamera monocular{model.focal(), film};
	
	//Compute pose candidates using p3p
	PRINT_DEBUG("Compute pose candidates using p3p"); 
    std::array<Ray3D, 3> rays; // computing rays corresponding to each nodes
    {//0
		P2D pixel = P2D{barycenters[0].u, barycenters[0].v}; //in UV space	    
		monocular.raytrace(pixel, rays.at(0));
	}
	{//1
		P2D pixel = P2D{barycenters[1].u, barycenters[1].v}; //in UV space	    
		monocular.raytrace(pixel, rays.at(1));
	}	
	{//2
		P2D pixel = P2D{barycenters[2].u, barycenters[2].v}; //in UV space	     
		monocular.raytrace(pixel, rays.at(2));
	}	
    // computing p3p
    Poses candidates(4); 
    bool p3p_ok = solve_p3p(
    	scene.get(barycenters[0].cluster),
    	scene.get(barycenters[1].cluster),
    	scene.get(barycenters[2].cluster),
		rays.at(0).direction(), 
		rays.at(1).direction(),
		rays.at(2).direction(),
		candidates
	);

	DEBUG_VAR(p3p_ok);
	
	for (const auto &pose : candidates)
	{
		if ( not(((pose.translation().array() == pose.translation().array())).all()) //check is_nan
			or not(((pose.rotation().array() == pose.rotation().array())).all()) //check is_nan
		)
		{
			PRINT_ERR("Pose contains NaN");
			DEBUG_VAR(pose);
			DEBUG_VAR(barycenters[0]);
			DEBUG_VAR(barycenters[1]);
			DEBUG_VAR(barycenters[2]);
			DEBUG_VAR(rays.at(0));
			DEBUG_VAR(rays.at(1));
			DEBUG_VAR(rays.at(2));
		}
	}
	
	//Select best using RANSAC
	PRINT_DEBUG("Select best using RANSAC");
	PoseWithError extrinsics = select_best_pose(monocular, scene, barycenters, candidates);
			
	return extrinsics;
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
template<class CameraModel, class Observations>
void init_extrinsic(
	//OUT
	CalibrationPose& extrinsic,
	//IN
	const CameraModel& model,
	const PointsConstellation & scene,
	const Observations& observations,
	//GUI
	const Image& picture/* for GUI only */
)
{
	PRINT_WARN("Ensure that features are linked to corresponding cluster.");
	
	constexpr int f = -1;
	const bool usePicture = (picture.cols > 0);
	
	//Configure monocular camera
	Sensor film = Sensor{model.sensor().width(), model.sensor().height(), model.sensor().scale()};
	film.pose() = model.sensor().pose();
#if defined(INIT_AT_FOCAL_LENGTH) && INIT_AT_FOCAL_LENGTH
	film.pose().translation().z() = - model.focal();
#endif
	
	const PinholeCamera monocular{model.focal(), film};
	
	//Estimate barycenters
	PRINT_DEBUG("Estimate barycenters of frame");
	Observations barycenters = compute_barycenters(observations); //IMAGE UV
		
	GUI(
		if (usePicture)
		{	
			RENDER_DEBUG_2D(
				Viewer::context().layer(Viewer::layer()).name("Frame"), 
				picture
			);	
			Viewer::update();
		}
		PRINT_DEBUG("[GUI] Display information of frame");
		display(f, observations);
		display(f, barycenters, tag::Barycenters{});	
	);
				
	//Estimate Pose
	PRINT_DEBUG("Estimate Pose of frame");
	const auto [pose, rmse] = estimate_pose(model, scene, barycenters);
		
	PRINT_DEBUG("Sanity check of pose frame f = " << f);
	if( pose.translation()[2] > 0. 
		or rmse.get() > 1e12
		or not(((pose.translation().array() == pose.translation().array())).all()) //check is_nan
		or not(((pose.rotation().array() == pose.rotation().array())).all()) //check is_nan
	) 
	{
		PRINT_ERR("Wrong hypothesis. Can't fix it. Remove pose and observations of frame f = " << f <<".");
		DEBUG_VAR(pose);
		DEBUG_VAR(rmse.get());
		
		wait();
		return;
	}
	
	DEBUG_VAR(pose);
	
	GUI(	
		Observations ubarycenters = compute_barycenters(observations);			
		display(f, observations);
		display(f, ubarycenters, tag::Barycenters{});
		wait();	
	);
		
	extrinsic = CalibrationPose{pose, -1};
}
