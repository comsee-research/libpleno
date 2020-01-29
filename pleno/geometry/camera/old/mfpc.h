#pragma once

#include "types.h"

#include "geometry/mesh.h"

#include "geometry/sensor.h"
#include "geometry/distortions.h"
#include "geometry/mia.h"
#include "geometry/mla.h"
#include "geometry/internals.h"

#include "geometry/camera/models.h"
#include "geometry/camera/thinlens.h"

#include "geometry/observation.h"

//******************************************************************************
//******************************************************************************
//******************************************************************************
class MultiFocusPlenopticCamera {
public:
    enum Mode {Unfocused = 0, Keplerian = 1, Galilean = 2};
	struct PrincipalPoint { double u; double v;};
    
    static constexpr int type(std::size_t k, std::size_t l) //MI
	{
		return static_cast<int>(std::fmod(std::fmod(l,2)+k, 3)); //k=col, l=row
	}
	
private:
	Sensor _sensor;
	MicroImagesArray _mia; 
	MicroLensesArray _mla;	
	
	ThinLensCameraModel _main_lens;
		
    Distortions _distortions;
    
	Pose _pose;
	    
    InternalParameters _params;
    
    double _dist_focus;
    
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	void init(
		const Sensor& sensor_, 
		const MicroImagesArray& mia_, 
		const InternalParameters& params_, 
		double F, double aperture, double h=1e8,
		Mode mode = Galilean
	);	
	
	bool project(
		const P3D& p3d_cam,
		std::size_t k, std::size_t l,
		P3D& bap
	) const;
	
	bool project(
		const P3D& p3d_cam,
		std::size_t k, std::size_t l,
		P2D& pixel
	) const;
	
	bool project(
		const P3D& p3d_cam,
		std::size_t k, std::size_t l,
		double& rho
	) const;
	
	bool project(
		const P3D& p3d_cam,
		BAPObservations& observations
	) const;
		
    Mode mode() const;
    
//OLD INTERFACE   
    const GridMesh3D& micro_lenses() const;
          GridMesh3D& micro_lenses();
    const GridMesh2D& raw_image() const;
          GridMesh2D& raw_image();
//NEW INTERFACE
 	const MicroLensesArray& mla() const;
          MicroLensesArray& mla();
    const MicroImagesArray& mia() const;
          MicroImagesArray& mia();
    const InternalParameters& params() const;
          InternalParameters& params();   
               
    PrincipalPoint pp() const;
    
    double distance_focus() const;
    double& distance_focus();
    
//SPACE CONVERSION 
	void uv2xy(P2D& puv) const;
	void xy2uv(P2D& pxy) const;
	void mi2ml(P2D& pij) const;
	void ml2mi(P2D& pkl) const;
	void mi2ml(double& index) const;
	void ml2mi(double& index) const;	
	
	template<typename Observations_t> void mi2ml(Observations_t& obs) const;
	template<typename Observations_t> void ml2mi(Observations_t& obs) const;
	
//COMMON INTERFACE
    const Sensor& sensor() const;
          Sensor& sensor();
    const ThinLensCameraModel& main_lens() const;
          ThinLensCameraModel& main_lens();
    const Pose& pose() const;
          Pose& pose();
    const Distortions& main_lens_distortions() const;
          Distortions& main_lens_distortions();
	
private:
	PrincipalPoint mlpp(std::size_t k, std::size_t l) const;
};

std::ostream& operator<<(std::ostream& os, const MultiFocusPlenopticCamera::Mode& mode);
std::ostream& operator<<(std::ostream& os, const MultiFocusPlenopticCamera& mfpc);
void load(std::string path, MultiFocusPlenopticCamera& pcm);
void save(std::string path, const MultiFocusPlenopticCamera& pcm);

using MFPC					= MultiFocusPlenopticCamera;

