#pragma once

#include <utility> //std::pair

#include "types.h"

constexpr double auto_param_from_data = -1.;

enum ImageQualityType : std::uint8_t { MSE = 0, MAE = 1, PSNR = 2, SSIM = 3, BADPIX = 4 };

std::pair<Image, double>
quality(const Image& ref, const Image& reading, ImageQualityType qt = ImageQualityType::MAE, double param = auto_param_from_data);

std::pair<Image, double>
quality_mse(const Image& ref, const Image& reading, double maxd = auto_param_from_data);

std::pair<Image, double>
quality_mae(const Image& ref, const Image& reading, double maxd = auto_param_from_data);

std::pair<Image, double>
quality_badpix(const Image& ref, const Image& reading, double threshold = auto_param_from_data, double maxd = auto_param_from_data);

std::pair<Image, double>
quality_psnr(const Image& ref, const Image& reading, double maxv = auto_param_from_data);

std::pair<Image, double>
quality_ssim(const Image& ref, const Image& reading);
