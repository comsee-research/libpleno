#pragma once

#include <libv/core/serialization/serializable_properties.hpp>
#include <libv/core/serialization/contiguous_containers.hpp> //support for std::vector

V_DEFINE_PROPERTIES(ImageWithInfoConfig)
(
    V_DEFINE_PROPERTY(path, std::string(""), "Path to the image to load")
    V_DEFINE_PROPERTY(fnumber, double(-1.0), "F-Number of the camera that took the image")
)

V_DEFINE_PROPERTIES(ImagesConfig)
(    
    V_DEFINE_PROPERTY(whites, std::vector<ImageWithInfoConfig>(0), "White images configurations")
    V_DEFINE_PROPERTY(checkerboards, std::vector<ImageWithInfoConfig>(0), "Checkerboard images configurations")
    V_DEFINE_PROPERTY(images, std::vector<ImageWithInfoConfig>(0), "Raw images configurations")
    V_DEFINE_PROPERTY(mask, ImageWithInfoConfig(), "Mask image configurations")
)
