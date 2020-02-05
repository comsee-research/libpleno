#pragma once

#include "types.h"

#include "camera.h"
#include "geometry/camera/thinlens.h"

#include "geometry/mia.h"
#include "geometry/mla.h"
#include "geometry/distortions.h"

#include "geometry/internals.h"

#include "geometry/pose.h"
#include "geometry/ray.h"

#include "geometry/observation.h"

//******************************************************************************
//******************************************************************************
//******************************************************************************
class PlenopticCamera : public Camera {
public:
    enum Mode {Unfocused = 0, Keplerian = 1, Galilean = 2};
    
	using PrincipalPoint = P2D;
	
protected:
	MicroImagesArray mia_; 
	MicroLensesArray mla_;	//I=0 act as pinholes
	
	ThinLensCamera main_lens_;	
    Distortions distortions_;
    
    InternalParameters params_;
    
    double dist_focus_;
    
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
//Ctor/Dtor
	PlenopticCamera() {}
	virtual ~PlenopticCamera() override {}
	
	virtual void init(
		const Sensor& sensor, 
		const MicroImagesArray& mia, 
		const InternalParameters& params, 
		double F, double aperture, double h=1e8,
		Mode mode = Galilean
	);
	
//Accessors	
    Mode mode() const;
    
    double distance_focus() const;
    double& distance_focus();
    
    const ThinLensCamera& main_lens() const;
          ThinLensCamera& main_lens();

    const Distortions& main_lens_distortions() const;
          Distortions& main_lens_distortions();
       
 	const MicroLensesArray& mla() const;
          MicroLensesArray& mla();
          
    const MicroImagesArray& mia() const;
          MicroImagesArray& mia();
          
    const InternalParameters& params() const;
          InternalParameters& params();   
  
//Computed parameters           
	double focal() const;
    double& focal(); 
    
	double aperture() const;
    double& aperture(); 
 
    PrincipalPoint pp() const;
	PrincipalPoint mlpp(std::size_t k, std::size_t l) const;
	
	std::size_t I() const;
    	
//Project and Raytrace	
	bool project(const P3D& p3d_cam, P2D& pixel) const override final
    {
		PRINT_WARN("PlenopticCamera::project: No micro-lens' index specified.");
		return false;
    }
    
    bool raytrace(const P2D& pixel, Ray3D& ray) const override final
    {
		PRINT_WARN("PlenopticCamera::raytrace: No micro-lens' index specified.");
		return false;
    }	
    
	bool project(const P3D& p3d_cam, std::size_t k, std::size_t l, P3D& bap) const;	
	bool project(const P3D& p3d_cam, std::size_t k, std::size_t l, P2D& pixel) const;
	bool project(const P3D& p3d_cam, std::size_t k, std::size_t l, double& rho) const;
	
	bool project(const P3D& p3d_cam, CBObservations& observations) const;
	bool project(const P3D& p3d_cam, BAPObservations& observations) const;
		
//Space convertion	(Micro-Images Space / Micro-Lenses Space)
	void mi2ml(P2D& pij) const;
	void ml2mi(P2D& pkl) const;
	void mi2ml(double& index) const;
	void ml2mi(double& index) const;	
	
	template<typename Observations_t> void mi2ml(Observations_t& obs) const;
	template<typename Observations_t> void ml2mi(Observations_t& obs) const;

protected:
//Helper functions
	bool is_on_disk(const P2D& p, double disk_diameter) const {
    	return p.norm() <= disk_diameter / 2.0 ;
    }
    
    bool hit_main_lens(const Ray3D& ray) const {
		// compute the intersection point between the ray and the lens
		P3D p = line_plane_intersection(Eigen::Vector4d{0.0, 0.0, 1.0, 0.0}, ray);
		// Testing if the ray hit the lens
		return is_on_disk(p.head(2), main_lens().diameter());
	}
	
	bool project_through_main_lens(const P3D& p3d_cam, P3D& projection) const;
	bool project_through_micro_lens(const P3D& p, std::size_t k, std::size_t l, P2D& projection) const;
	bool project_radius_through_micro_lens(const P3D& p, std::size_t k, std::size_t l, double& radius) const;
};

std::ostream& operator<<(std::ostream& os, const PlenopticCamera::Mode& mode);
std::ostream& operator<<(std::ostream& os, const PlenopticCamera& pcm);

void load(std::string path, PlenopticCamera& pcm);
void save(std::string path, const PlenopticCamera& pcm);
