#pragma once

#include "types.h"

#include "io/cfg/images.h"

void load(const std::vector<ImageWithInfoConfig>& cfgs, std::vector<ImageWithInfo>& images, bool debayered = true);
void load(const ImageWithInfoConfig& cfg, ImageWithInfo& image, bool debayered = true);



