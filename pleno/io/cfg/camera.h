#pragma once

#include "io/archive.h"

#include "cfg/mia.h"
#include "cfg/mla.h"
#include "cfg/sensor.h"
#include "cfg/thinlens.h"
#include "cfg/distortions.h"
#include "cfg/functions.h"

#include "geometry/camera/plenoptic.h"

V_DEFINE_PROPERTIES(PlenopticCameraConfig)
(   
    V_DEFINE_PROPERTY(mode, (int(2)), "Mode of the camera (0: Unfocused (F=D and f=d); 1: Keplerian (F<D and f<d)) ; 2: Galilean (F>D and f>d)")
    
    V_DEFINE_PROPERTY(mia, MIAConfig(), "Micro-Images Array configuration")
    V_DEFINE_PROPERTY(mla, MLAConfig(), "Micro-Lenses Array configuration")
    V_DEFINE_PROPERTY(sensor, SensorConfig(), "Sensor configuration")
    
    V_DEFINE_PROPERTY(main_lens, ThinLensConfig(), "Main Lens configuration")    
    V_DEFINE_PROPERTY(distortions, DistortionConfig(), "Distortions coefficients (radial and tangential)")
    V_DEFINE_PROPERTY(distortions_inverse, DistortionConfig(), "Inverse Distortions coefficients (radial and tangential)")
    
    V_DEFINE_PROPERTY(dist_focus, double(1e8), "Focus distance (in mm). Warn: inf ~ 1e8")
    V_DEFINE_PROPERTY(I, std::size_t(3ul), "Number of micro-lenses types (0: act as pinholes array)")
    V_DEFINE_PROPERTY(scaling, QuadraticFunctionConfig(), "Depth scaling correction")

//computed parameters
    V_DEFINE_PROPERTY(D, (double(-1.)), "Distance main-lens/MLA (in mm)")
    V_DEFINE_PROPERTY(d, (double(-1.)), "Distance MLA/sensor (in mm)")
    V_DEFINE_PROPERTY(pp, (PlenopticCamera::PrincipalPoint{0.,0.}), "Main lens principal point")
    V_DEFINE_PROPERTY(Rxyz, (P3D(0.,0.,0.)), "MLA rotation angles (RotX, RotY, RotZ)")
    V_DEFINE_PROPERTY(focal_planes, std::vector<double>{}, "Focal planes of the camera (in mm)")
)

using MultiFocusPlenopticCameraConfig = PlenopticCameraConfig;
using MFPCConfig = MultiFocusPlenopticCameraConfig;

