#pragma once

#include <iostream>
#include "types.h"

class Distortions
{
    Eigen::Vector3d _radial; //3 components
    Eigen::Vector2d _tangential; //2 components
    
#if defined(USE_DEPTH_DISTORTION) && USE_DEPTH_DISTORTION
    Eigen::Vector3d _depth; //3 components
    
	Eigen::Vector3d& depth() { return _depth; }
	const Eigen::Vector3d& depth() const { return _depth; }
   
    void apply_depth(P3D& p) const;
    void apply_depth(P3DS& ps) const;
#endif
public: 	
	Eigen::Vector3d& radial() { return _radial; }
	const Eigen::Vector3d& radial() const { return _radial; }
	
	Eigen::Vector2d& tangential() { return _tangential; }
	const Eigen::Vector2d& tangential() const { return _tangential; }
	
    inline double radius(const P2D& p) const;
    std::vector<double> radius(const P2DS& ps) const;

    void apply_radial(P2D& p) const;
    void apply_radial(P2DS& ps) const;

    void apply_tangential(P2D& p) const;
    void apply_tangential(P2DS& ps) const;
    
    void apply(P3D& p) const;
    void apply(P3DS& ps) const;

    void unapply(P3D& p) const;
};

std::ostream& operator<<(std::ostream& o, const Distortions& dist);
