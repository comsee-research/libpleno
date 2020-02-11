#include "optimize.h"

#include "geometry/camera/unified.h"

#include "optimization/homography.h"

#include "unused.h"

/**
 *  @Brief fit_template_on
 */
Transformation optimize(
    const Image& observation,
    const Image& model,
    const Image& mask,
    double& final_residual
)
{
    UnifiedCamera ucm(100,100,50,50,0);
    Transformation t{AFF, 2, ucm}; //SO=1, SE=3, SIM=4, AFF=6

    //running optimisation
    GenericLevenbergMarquardt<Image> glm{1e3, 1e-10, observation, model, mask};
    auto [ nbit, residual ] = glm.run(t, 1e-3, false); UNUSED(nbit);
    
    final_residual = residual;

    return t;
}

