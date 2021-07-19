#include "generate.h"

#include <random>

#include "geometry/sampler.h"

#include "processing/tools/rotation.h"

#include "io/printer.h"

#include "graphic/gui.h"
#include "graphic/display.h"

bool is_pose_valid(
	const Pose& pose,
	const CameraModel_t& model,     
	const CheckerBoard& grid
)
{
    bool is_validated = true;
	CBObservations obs;

    //All the nodes is at least projected once in a microimage
    for(const P3D& node : grid)
    {
        const P3D pcam = to_coordinate_system_of(pose, node); //CAMERA
        
        if( pcam[2] < 0. or not model.project(pcam, obs) or obs.size() < 2)
        {
        	is_validated = false;
        	break;
        }
    }
    return is_validated;
}

void generate_pose(Pose& pose, double min, double max)
{
	std::random_device rd;
    std::mt19937 gen(rd());
    
    std::uniform_real_distribution<double> dist_tz(min, max);    
    P3D p = uniform_sample_hemisphere(); 
        
    pose.translation() = - p * dist_tz(gen);
    
    std::normal_distribution<double> dist_R(0.,1.);
    apply_rotation(
    	pose.rotation(), 
    	Eigen::Vector3d{0.1 * dist_R(gen), 0.1 * dist_R(gen), 0.1 * dist_R(gen)}
    );
}

void generate_poses(
	CalibrationPoses& poses, /* out */
	const CameraModel_t& model, 
	const CheckerBoard& grid, 
	std::size_t n, double min, double max
)
{
	PRINT_INFO("=== Generating n = " << n << " poses");
	
	poses.clear();
	poses.resize(n);
	
	for(std::size_t f = 0; f < n ; ++f)
	{
		PRINT_DEBUG("Generate pose f = " << f);
		Pose p;
		do
		{
			generate_pose(p, min, max);
		}
		while((not is_pose_valid(p, model, grid)) or std::fabs(p.translation().z()) < min);
		poses[f] = CalibrationPose{p, static_cast<int>(f)};
		DEBUG_VAR(f); DEBUG_VAR(p);
	}
}

