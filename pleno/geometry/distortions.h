#pragma once

#include <iostream>
#include "types.h"

class Distortions
{
public:
	enum DepthDistortionModel : std::uint16_t { 
		NO_DDM = 0, 
		HEINZE_DDM = 1, ZELLER_DDM = 2, //radially dependent
		OFFSET_DDM = 3, LINEAR_DDM = 4, AFFINE_DDM = 5, QUADRATIC_DDM = 6 //only depth dependent
	};

private:
    Eigen::Vector3d radial_ 	= Eigen::Vector3d::Zero(); //3 components
    Eigen::Vector2d tangential_	= Eigen::Vector2d::Zero(); //2 components
    Eigen::Vector3d depth_		= Eigen::Vector3d::Zero(); //3 components
    
    DepthDistortionModel ddm_ = DepthDistortionModel::NO_DDM;
    
public: 
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//Radial distortions	
	Eigen::Vector3d& radial() { return radial_; }
	const Eigen::Vector3d& radial() const { return radial_; }
	
    void apply_radial(P2D& p) const;
    void apply_radial(P2DS& ps) const;
    
//Tangential distortions
	Eigen::Vector2d& tangential() { return tangential_; }
	const Eigen::Vector2d& tangential() const { return tangential_; }
	
    void apply_tangential(P2D& p) const;
    void apply_tangential(P2DS& ps) const;

//Depth distortions
	DepthDistortionModel& model() { return ddm_; }
	DepthDistortionModel model() const { return ddm_; }

	Eigen::Vector3d& depth() { return depth_; }
	const Eigen::Vector3d& depth() const { return depth_; }
   
    void apply_depth(P3D& p) const;
    void apply_depth(P3DS& ps) const;
    
    inline double radius(const P2D& p) const;
    std::vector<double> radius(const P2DS& ps) const;
  
    void apply(P3D& p) const;
    void apply(P3DS& ps) const;
};

std::ostream& operator<<(std::ostream& o, const Distortions& dist);
