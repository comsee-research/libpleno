#pragma once

#include "io/archive.h"

V_DEFINE_PROPERTIES(ImageWithInfoConfig)
(
    V_DEFINE_PROPERTY(path, std::string(""), "Path to the image to load")
    V_DEFINE_PROPERTY(fnumber, double(-1.0), "F-Number of the camera that took the image")
    V_DEFINE_PROPERTY(frame, int(-1), "Frame index")
)

using ImagesWithInfoConfig = std::vector<ImageWithInfoConfig>;

V_DEFINE_PROPERTIES(MetaDataImageConfig)
(
	V_DEFINE_PROPERTY(debayered, bool(true), "Is image already debayered?")
	V_DEFINE_PROPERTY(rgb, bool(true), "Is image in RGB format?")
	V_DEFINE_PROPERTY(format, std::size_t(8), "Image format (8-bits, 16-bits, 32-bits, 64-bits)")
)

V_DEFINE_PROPERTIES(ImagesConfig)
(    
    V_DEFINE_PROPERTY(meta, MetaDataImageConfig(), "Images metadata")
    V_DEFINE_PROPERTY(whites, ImagesWithInfoConfig(0), "White images configurations")
    V_DEFINE_PROPERTY(checkerboards, ImagesWithInfoConfig(0), "Checkerboard images configurations")
    V_DEFINE_PROPERTY(images, ImagesWithInfoConfig(0), "Raw images configurations")
    V_DEFINE_PROPERTY(mask, ImageWithInfoConfig(), "Mask image configurations")
)
