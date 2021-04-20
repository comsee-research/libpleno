#pragma once

#include <iostream>
#include "types.h"

class Distortions
{
private:
    Eigen::Vector3d radial_; //3 components
    Eigen::Vector2d tangential_; //2 components
        
#if defined(USE_DEPTH_DISTORTION) && USE_DEPTH_DISTORTION
    Eigen::Vector3d depth_; //3 components
#endif
    
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
#if defined(USE_DEPTH_DISTORTION) && USE_DEPTH_DISTORTION	
	Eigen::Vector3d& depth() { return depth_; }
	const Eigen::Vector3d& depth() const { return depth_; }
   
    void apply_depth(P3D& p) const;
    void apply_depth(P3DS& ps) const;
#endif		
    inline double radius(const P2D& p) const;
    std::vector<double> radius(const P2DS& ps) const;
  
    void apply(P3D& p) const;
    void apply(P3DS& ps) const;
};

std::ostream& operator<<(std::ostream& o, const Distortions& dist);
