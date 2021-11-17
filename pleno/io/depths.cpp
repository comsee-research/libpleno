#include "depths.h"

#include <iostream>

#include "io/printer.h"

#include "processing/depth/initialization.h"

//******************************************************************************
//******************************************************************************
//******************************************************************************
PointCloud read_xyz(std::string path)
{
	std::ifstream ifs(path);
	PRINT_DEBUG("Open file = " << path);
	
    std::size_t count;
    ifs >> count;
    ifs.ignore(1, '\n');
    ifs.ignore(1, '\n');
    DEBUG_VAR(count);
    
    PointCloud pts{count};
    
    for(std::size_t i = 0; i < count; i++)
    {
        std::string line;
        if(!std::getline(ifs, line)) {PRINT_ERR("("<<i << ") no line"); break; }
        
        //DEBUG_VAR(line);
        std::istringstream iss(line);
        
		double x = 0., y = 0., z = 0.;

        iss >> x >> y >> z;
        
        //DEBUG_VAR(p.x);DEBUG_VAR(p.y);DEBUG_VAR(p.z);
        pts.add(P3D{x, y, z}, P2D{-1, -1}, RGBA{255., 255., 255., 255.});  
        iss.clear(); 
    }	
    
    ifs.close();
	return pts;
}
//******************************************************************************
Pose read_mat(std::string path)
{
		
	std::ifstream ifs(path);
	PRINT_DEBUG("Open file = " << path);

    std::string line;
    
    for (std::size_t i = 0; i < 5; ++i)
    {
    	std::getline(ifs, line, '\n');//ignore header
    }
 
 	double r11, r12, r13, t1;
 	double r21, r22, r23, t2;
 	double r31, r32, r33, t3;
 	
    //first line
    {
     	std::getline(ifs, line, '\n');
     	std::istringstream iss(line);
		
        iss >> r11 >> r12 >> r13 >> t1;    
    }
    //second line
    {
     	std::getline(ifs, line, '\n');
     	std::istringstream iss(line);
		
        iss >> r21 >> r22 >> r23 >> t2;    
    }
    //third line
    {
     	std::getline(ifs, line, '\n');
     	std::istringstream iss(line);
		
        iss >> r31 >> r32 >> r33 >> t3;    
    }
	ifs.close();

	Pose pose;
	pose.translation() << t1, t2, t3;
	pose.rotation() << 	r11, 	r12, 	r13,
						r21,	r22,	r23,
						r31,	r32,	r33;	

	return pose;
}
//******************************************************************************
PointCloud read_pts(std::string path)
{
	std::ifstream ifs(path);
	PRINT_DEBUG("Open file = " << path);
	
    std::size_t count;
    ifs >> count;
    ifs.ignore(1, '\n');
    DEBUG_VAR(count);
    
    PointCloud pts{count};
    
    for(std::size_t i = 0; i < count; i++)
    {
        std::string line;
        if(!std::getline(ifs, line)) {PRINT_ERR("("<<i << ") no line"); break; }
        
        //DEBUG_VAR(line);
        std::istringstream iss(line);
		
		double x = 0., y = 0., z = 0., a = 0., r = 0., g = 0., b = 0.;

        iss >> x >> y >> z >> a >> r >> g >> b;
        
        pts.add(P3D{1000. * x, 1000. * y, 1000. * z}, P2D{-1, -1}, RGBA{r, g, b, 255.});    
        iss.clear(); 
    }	
    
    ifs.close();
	return pts;
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
std::map<Index, PointCloud> load(const XYZsConfig& config)
{
	std::map<Index, PointCloud> maps;
	for (auto & xyz_cfg : config)
	{
		maps.emplace(xyz_cfg.frame(), read_xyz(xyz_cfg.path()));	
	}
	
	return maps;
}

//******************************************************************************
std::map<Index, PointCloud> load(const PTSsConfig& config)
{
	std::map<Index, PointCloud> maps;
	for (auto &pts_cfg : config)
	{
		maps.emplace(pts_cfg.frame(), read_pts(pts_cfg.path()));	
	}
	
	return maps;
}

//******************************************************************************
std::map<Index, PointCloud> load(const PointCloudsConfig& config)
{
	std::map<Index, PointCloud> maps;
	
	for (auto & pc_cfg : config)
	{	
		PointCloud pc;
		v::load(pc_cfg.path(), v::make_serializable(&pc));
		
		maps.emplace(pc_cfg.frame(), std::move(pc));
	}
	
	return maps;
}

//******************************************************************************
std::map<Index, Plane> load(const PlanesConfig& config)
{
	std::map<Index, Plane> maps;
	
	for (const auto & plane_cfg : config)
	{	
		Plane plane;
		v::load(plane_cfg.path(), v::make_serializable(&plane));
		
		maps.emplace(plane_cfg.frame(), std::move(plane));
	}
	
	return maps;
}

//******************************************************************************
std::map<Index, DepthMap> load(const DepthMapsConfig& config)
{
	std::map<Index, DepthMap> maps;
	
	for (auto & dm_cfg : config)
	{	
		DepthMap dm;
		v::load(dm_cfg.path(), v::make_serializable(&dm));
		
		maps.emplace(dm_cfg.frame(), std::move(dm));
	}
	
	return maps;
}

//******************************************************************************
std::map<Index, DepthMap> load_from_csv(std::string path, const PlenopticCamera& mfpc)
{
	std::map<Index, DepthMap> maps;
	
	using KLD = std::tuple<Index, Index, double>;
	std::map<Index, std::vector<KLD>> depths;
	
	std::ifstream ifs(path);
	PRINT_DEBUG("Open file = " << path);
    
    std::string line;
    char comma;
    
    std::getline(ifs, line, '\n');//ignore header
    
    while (std::getline(ifs, line, '\n'))
    {
     	std::istringstream iss(line);

		Index frame, k, l;
		double d;
		
        iss >> frame >> comma >> k >> comma >> l >> comma >> d;
        
        depths[frame].emplace_back(k,l,d);    
        
        //iss.clear(); 
    }
	ifs.close();

	const auto [mind, maxd] = initialize_min_max_distance(mfpc);
	
	for (auto & [frame, klds] : depths)
	{
		bool coarse = not (klds.size() > mfpc.mia().width() * mfpc.mia().height());
		const std::size_t W = coarse? mfpc.mia().width(): mfpc.sensor().width();
		const std::size_t H = coarse? mfpc.mia().height(): mfpc.sensor().height();
		
		DepthMap dm{
			W, H, mind, maxd, 
			DepthMap::DepthType::METRIC, 
			coarse ? DepthMap::MapType::COARSE : DepthMap::MapType::REFINED
		};
		
		for (const auto& [k, l, d] : klds)
		{
			dm.depth(k,l) = d;
			dm.state(k,l) = DepthInfo::State::COMPUTED;
		}

		maps.emplace(frame, std::move(dm));
	}
	
	return maps;
}

//******************************************************************************
std::map<Index, Pose> load(const MatsConfig& config)
{
	std::map<Index, Pose> maps;
	for (auto & mat_cfg : config)
	{
		maps.emplace(mat_cfg.frame(), read_mat(mat_cfg.path()));	
	}
	
	return maps;
}

//******************************************************************************
std::map<Index, Pose> load(const CalibrationPosesConfig& config)
{
	std::map<Index, Pose> maps;
	for (auto & pose_cfg : config.poses())
	{
		maps.emplace(pose_cfg.frame(), pose_cfg.pose());	
	}
	return maps;
}

//******************************************************************************
std::map<Index, Image> load(const CSADsConfig& config)
{
	std::map<Index, Image> maps;
	for (auto & csad_cfg : config)
	{
		maps.emplace(csad_cfg.frame(), cv::imread(csad_cfg.path(), cv::IMREAD_UNCHANGED));	
	}
	return maps;
}

//******************************************************************************
std::map<Index, double> load_gt_dist(std::string path)
{
	std::ifstream ifs(path, std::ios::in);
	PRINT_DEBUG("Open file = " << path);
             				
	std::map<Index, double> maps;
	
    std::string line;
	while (std::getline(ifs, line))
	{
		double d = 0.; int frame = 0;
        std::istringstream iss(line);
        
        iss >> frame >> d;
        
        maps[frame] = d;
	}
	
	ifs.close();
	return maps;
}
