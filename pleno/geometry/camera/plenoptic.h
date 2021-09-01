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

#include "processing/tools/functions.h"

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
    Distortions invdistortions_;
    
    InternalParameters params_;
    
    double dist_focus_;
    
    QuadraticFunction scaling_;
    
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
//Ctor/Dtor
	PlenopticCamera() {}
	~PlenopticCamera() override {}
	
	void init(
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
    
    const QuadraticFunction& scaling() const;
    	  QuadraticFunction& scaling();
    
    const ThinLensCamera& main_lens() const;
          ThinLensCamera& main_lens();

    const Distortions& main_lens_distortions() const;
          Distortions& main_lens_distortions();
          
    const Distortions& main_lens_invdistortions() const;
          Distortions& main_lens_invdistortions();
       
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
    
    double mlaperture() const;
 
    PrincipalPoint pp() const;
	PrincipalPoint mlpp(std::size_t k, std::size_t l) const; //k,l in MLA space
	
	std::size_t I() const;

	double d(std::size_t k = 0, std::size_t l = 0) const;
	double D(std::size_t k = 0, std::size_t l = 0) const;
	
	double focal_plane(std::size_t i, std::size_t k = 0, std::size_t l = 0) const;
	
	bool unfocused() const;
	bool focused() const;
	bool multifocus() const;
    	
//Project
	bool project(const P3D& /*p3d_cam*/, P2D& /*pixel*/) const override final;
    
	bool project(const P3D& p3d_cam, std::size_t k, std::size_t l, P3D& bap) const;	//k,l in MLA space
	bool project(const P3D& p3d_cam, std::size_t k, std::size_t l, P2D& pixel) const; //k,l in MLA space
	bool project(const P3D& p3d_cam, std::size_t k, std::size_t l, double& rho) const; //k,l in MLA space
	
	bool project(const P3D& p3d_cam, CBObservations& observations) const;
	bool project(const P3D& p3d_cam, BAPObservations& observations) const;
	
//Raytrace		
    bool raytrace(const P2D& /*pixel*/, Ray3D& /*ray*/) const override final;
    
    bool raytrace(const P2D& pixel, std::size_t k, std::size_t l, Ray3D& ray) const; //k,l in MLA space
    bool raytrace(const P2D& pixel, std::size_t k, std::size_t l, std::size_t n, Rays3D& rays) const; //k,l in MLA space
		
//Space convertion	(Micro-Images Space / Micro-Lenses Space)
	void mi2ml(P2D& pij) const;
	void ml2mi(P2D& pkl) const;
	void mi2ml(double& index) const;
	void ml2mi(double& index) const;	
	P2D mi2ml(std::size_t k, std::size_t l) const;
	P2D ml2mi(std::size_t k, std::size_t l) const;	
	
	template<typename Observations> void mi2ml(Observations& obs) const;
	template<typename Observations> void ml2mi(Observations& obs) const;
	
//Space convertion (obj, mla, virtual)
	double v2mla(double x, std::size_t k = 0, std::size_t l = 0) const;
	double mla2v(double x, std::size_t k = 0, std::size_t l = 0) const;
	
	double obj2mla(double x, std::size_t k = 0, std::size_t l = 0) const;
	double mla2obj(double x, std::size_t k = 0, std::size_t l = 0) const;
	
	double v2obj(double x, std::size_t k = 0, std::size_t l = 0) const;
	double obj2v(double x, std::size_t k = 0, std::size_t l = 0) const;
	
//Disparity at virtual depth v, indexes in MIA space
	P2D disparity(std::size_t k, std::size_t l, std::size_t nk, std::size_t nl, double v) const;

protected:
//Helper functions
	bool is_on_disk(const P2D& p, double disk_diameter) const;
	bool is_inside_mi(const P2D& p, std::size_t k, std::size_t l) const;
    bool hit_main_lens(const Ray3D& ray) const;
    
//Helper projection function	
	bool project_through_main_lens(const P3D& p3d_cam, P3D& projection) const;
	bool project_through_micro_lens(const P3D& p, std::size_t k, std::size_t l, P2D& projection) const;
	bool project_radius_through_micro_lens(const P3D& p, std::size_t k, std::size_t l, double& radius) const;
};

std::ostream& operator<<(std::ostream& os, const PlenopticCamera::Mode& mode);
std::ostream& operator<<(std::ostream& os, const PlenopticCamera& pcm);

void load(std::string path, PlenopticCamera& pcm);
void save(std::string path, const PlenopticCamera& pcm);
