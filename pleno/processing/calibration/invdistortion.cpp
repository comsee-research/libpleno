#include "calibration.h"

//geometry
#include "geometry/camera/plenoptic.h"
#include "geometry/object/checkerboard.h"
#include "geometry/sampler.h"

//optimization
#include "optimization/optimization.h"
#include "optimization/errors/invdistortion.h" //InverseDistortionCostError

//io
#include "io/printer.h"
#include "io/choice.h"

//******************************************************************************
//******************************************************************************
//******************************************************************************
void optimize(
	Distortions& invdistortions,
	const PlenopticCamera& mfpc,
	const CheckerBoards& scene
)
{		
	constexpr std::size_t nsample = 250'000ul;
	
	using Solver_t = lma::Solver<InverseDistortionCostError>;
	
	Solver_t solver{1e3, 150, 1.0 - 1e-27};

	//for each frame
	for(auto & cb : scene)
	{ 
		for (std::size_t i = 0; i < nsample; ++i)
		{
			//get point on checkerboard
			const P3D maxp = cb.node(cb.width()-1, cb.height()-1);
			P3D pcb; pcb << uniform_sample_rect(P2D{0.,0.}, maxp.x(), maxp.y()), 0.;	
			const P3D pcam = to_coordinate_system_of(cb.pose(), pcb);
			
			//project point through main lens
			P3D projection;
			if (mfpc.main_lens().project(pcam, projection))
			{
				//add ref point to solver
				solver.add(
					InverseDistortionCostError{mfpc.main_lens_distortions(), projection},
					&invdistortions
				);
			} 
		}
	}
	
	solver.solve(lma::DENSE, lma::enable_verbose_output());
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
void calibration_inverseDistortions(
	Distortions& invdistortions,
	const PlenopticCamera& mfpc,
	const CheckerBoards& scene
)
{
//1) Init Parameters
	PRINT_INFO("=== Init Parameter");	
	Distortions dists{mfpc.main_lens_distortions()};
	DEBUG_VAR(dists);
	
//3) Run optimization
	PRINT_INFO("=== Run optimization");	
	optimize(dists, mfpc, scene);
	
	PRINT_INFO("=== Optimization finished! Results:");
	DEBUG_VAR(dists);
	
	invdistortions.radial() = dists.radial();
	invdistortions.tangential() = dists.tangential();
	invdistortions.depth() = dists.depth();

	wait();
}
