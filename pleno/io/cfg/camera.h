#pragma once

#include <libv/core/serialization/serializable_properties.hpp>

#include "cfg/mia.h"
#include "cfg/mla.h"
#include "cfg/sensor.h"
#include "cfg/thinlens.h"
#include "cfg/distortions.h"

V_DEFINE_PROPERTIES(PlenopticCameraConfig)
(
    V_DEFINE_PROPERTY(mia, MIAConfig(), "Micro-Images Array configuration")
    V_DEFINE_PROPERTY(mla, MLAConfig(), "Micro-Lenses Array configuration")
    V_DEFINE_PROPERTY(sensor, SensorConfig(), "Sensor configuration")
    
    V_DEFINE_PROPERTY(main_lens, ThinLensConfig(), "Main Lens configuration")    
    V_DEFINE_PROPERTY(distortions, DistortionConfig(), "Distortions coefficients (radial and tangential)")
    
    V_DEFINE_PROPERTY(dist_focus, double(1e8), "Focus distance (in mm)")
    V_DEFINE_PROPERTY(I, std::size_t(3u), "Number of micro-lenses types (0: act as pinholes array)")
)

using MultiFocusPlenopticCameraConfig = PlenopticCameraConfig;
using MFPCConfig = MultiFocusPlenopticCameraConfig;

