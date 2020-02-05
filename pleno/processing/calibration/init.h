#pragma once

#include "types.h"

#include "geometry/camera/models.h" //ThinLensCamera
#include "geometry/object/checkerboard.h" //CheckerBoard

#include "geometry/pose.h"

#include "io/printer.h"

#include "graphic/gui.h"
#include "graphic/display.h"

#include "processing/tools/rmse.h"


#include "link.h"

template<typename CameraModel, typename Observations>
Pose select_best_pose(
	const CameraModel& camera, 
	const CheckerBoard& grid,
    const Observations& observations, 
    const Poses& poses
);

template<typename CameraModel, typename Observations>
Pose estimate_pose(
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
	const std::vector<Image>& pictures /* for GUI only */
);

//******************************************************************************
//******************************************************************************
//******************************************************************************
template<typename CameraModel, typename Observations>
Pose select_best_pose(
	const CameraModel& camera, 
	const CheckerBoard& grid,
    const Observations& observations, 
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
	
	PRINT_DEBUG("Best pose is p = " << rmse_poses[0].pose);
   	return rmse_poses[0].pose;
}

template<typename CameraModel, typename Observations>
Pose estimate_pose(
	const CameraModel& model, 
	const CheckerBoard& grid, 
	const Observations& barycenters
)
{
	//Configure monocular camera 
	PRINT_DEBUG("Configure monocular camera ");
	const PinholeCamera monocular{model.focal(), model.sensor()};
	
	//Compute pose candidates using p3p
	PRINT_DEBUG("Compute pose candidates using p3p");
    Observations nodes; /* tl - tr - br */
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
		    //rays.at(i).origin() = {0., 0., 0.};
		    P2D pixel = P2D{nodes[i][0], nodes[i][1]}; //in UV space
		    		    
		    monocular.raytrace(pixel, rays.at(i));
		}
    }

    // computing p3p
    Poses candidates(4);
	/** tl - tr - br 
	 * IN UV SPACE :
	 *  - the top-left 		(tl) corner is the (0,0) 	node
	 *	- the top-right 	(tr) corner is the (K, 0) 	node
	 *	- the bottom-right 	(br) corner is the (K,L) 	node
	 **/ 
    bool p3p_ok = solve_p3p(
    	grid.nodeInWorld(0), //tl
    	grid.nodeInWorld(grid.width()-1), //bl
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
	const std::vector<Image>& pictures /* for GUI only */
)
{
	//Split observations according to frame
	std::unordered_map<int /* frame index */, Observations> obs;
	for(const auto& ob : observations)
		obs[ob.frame].push_back(ob);
	
	//Configure monocular camera
	const PinholeCamera monocular(model.focal(), model.sensor());
	
	//CalibrationPoses poses;
	poses.clear();
	poses.reserve(pictures.size());	
	
	//For each frame 
	for(auto & [f, ob] : obs)
	{
		Viewer::stash();
		//Estimate barycenters
		PRINT_DEBUG("Estimate barycenters of frame f = " << f);
		Observations barycenters = compute_barycenters(ob); //IMAGE UV
		
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
		link_cluster_to_node_index(ob, barycenters, monocular, grid, pose, true);
			
		GUI(	
			Observations ubarycenters = compute_barycenters(ob);			
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
