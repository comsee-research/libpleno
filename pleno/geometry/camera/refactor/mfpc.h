#pragma once

#include "types.h"

#include "camera.h"
#include "geometry/camera/thinlens.h"

#include "geometry/observation.h"

//******************************************************************************
//******************************************************************************
//******************************************************************************
class MultiFocusPlenopticCamera final : public PlenopticCamera { //MLA act as a thinlenses array
public:	
	static constexpr std::size_t N = 3u;
    static constexpr int type(std::size_t k, std::size_t l) //MI
	{
		return static_cast<int>(std::fmod(std::fmod(l,(N-1))+k, N)); //k=col, l=row
	}
    
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
//Ctor/Dtor
	MultiFocusPlenopticCamera() {}
	virtual ~MultiFocusPlenopticCamera() override {}
	
	void init(
		const Sensor& sensor_, 
		const MicroImagesArray& mia_, 
		const InternalParameters& params_, 
		double F, double aperture, double h=1e8,
		Mode mode = Galilean
	) override;

//Project and Raytrace	
	bool project(
		const P3D& p3d_cam,
		std::size_t k, std::size_t l,
		P3D& bap
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

private:
	bool project_radius_through_micro_lens(const P3D& p, std::size_t k, std::size_t l, double& radius) const;
};

std::ostream& operator<<(std::ostream& os, const MultiFocusPlenopticCamera& mfpc);

using MFPC					= MultiFocusPlenopticCamera;
