#include "quality.h"

#include <opencv2/imgproc.hpp>

#include "io/printer.h"
#include "processing/tools/stats.h"

std::pair<Image, double>
quality_mse(const Image& ref, const Image& reading, double maxd)
{
	static int i = 0;
	
	if (maxd == auto_param_from_data) maxd = 255.;
	
	Image qualitymap = Image::zeros(ref.rows, ref.cols, CV_32FC3);
	double quality = 0.;
	
	Image rmask(ref.rows, ref.cols, CV_8UC1);
	Image rfmask;
	cv::threshold(ref, rfmask, 1., 255., cv::THRESH_BINARY);
	cv::cvtColor(rfmask, rfmask, cv::COLOR_BGR2GRAY);
	rfmask.convertTo(rmask, CV_8U);
	
	Image lmask(ref.rows, ref.cols, CV_8UC1);
	Image lfmask;
	cv::threshold(reading, lfmask, 1., 255., cv::THRESH_BINARY);
	cv::cvtColor(lfmask, lfmask, cv::COLOR_BGR2GRAY);
	lfmask.convertTo(lmask, CV_8U); 
	
	Image mask(ref.rows, ref.cols, CV_8UC1);
	cv::bitwise_and(rmask, lmask, mask);
	
	cv::imwrite("mask-"+std::to_string(i++)+".png", mask);
	
	cv::subtract(ref, reading, qualitymap, mask);
	cv::multiply(qualitymap, qualitymap, qualitymap); // slightly faster than pow2
	
	Image mqualitymap = Image::zeros(ref.rows, ref.cols, CV_32FC3);
	qualitymap.copyTo(mqualitymap, mask);

	auto mean = cv::mean(mqualitymap, mask); DEBUG_VAR(mean);
	quality = (mean[0] + mean[1] + mean[2]) / 3.;
	
	Image qmap;
	mqualitymap.convertTo(qmap, CV_8UC3, 255. / (maxd * maxd));
	
	return {qmap, std::sqrt(quality)};	
}

std::pair<Image, double>
quality_mae(const Image& ref, const Image& reading, double maxd)
{
	if (maxd == auto_param_from_data) maxd = 255.;
	
	Image qualitymap = Image::zeros(ref.rows, ref.cols, CV_32FC3);
	double quality = 0.;
	
	double iqrv, q1, med, q3;
	
	Image rmask(ref.rows, ref.cols, CV_8UC1);
	Image rfmask;
	cv::threshold(ref, rfmask, 1., 255., cv::THRESH_BINARY);
	cv::cvtColor(rfmask, rfmask, cv::COLOR_BGR2GRAY);
	rfmask.convertTo(rmask, CV_8UC1);
	
	Image lmask(ref.rows, ref.cols, CV_8UC1);
	Image lfmask;
	cv::threshold(reading, lfmask, 1., 255., cv::THRESH_BINARY);
	cv::cvtColor(lfmask, lfmask, cv::COLOR_BGR2GRAY);
	lfmask.convertTo(lmask, CV_8UC1); 
	
	Image mask(ref.rows, ref.cols, CV_8UC1);
	cv::bitwise_and(rmask, lmask, mask);
	
	cv::subtract(ref, reading, qualitymap, mask);
	qualitymap = cv::abs(qualitymap);
	
	//Stats
	Image temp; cv::cvtColor(qualitymap, temp, cv::COLOR_BGR2GRAY);	
	Image stats = temp.reshape(0, 1);
	std::vector<double> vstats;
	stats.copyTo(vstats);
	vstats.erase(
		std::remove_if(vstats.begin(), vstats.end(),
			[](double e) -> bool { return e == 0.; }
		), vstats.end()
	);
	
	iqrv = iqr(vstats, q1, med, q3);
	DEBUG_VAR(iqrv); DEBUG_VAR(q1); DEBUG_VAR(med); DEBUG_VAR(q3);	
	const auto [min, max] = std::minmax_element(
		std::begin(vstats), std::end(vstats)
	);
	DEBUG_VAR(*min); DEBUG_VAR(*max);
	
	auto mean = cv::mean(qualitymap, mask); DEBUG_VAR(mean);
	quality = (mean[0] + mean[1] + mean[2]) / 3.;
	
	Image qmap;
	qualitymap.convertTo(qmap, CV_8UC3, 255. / maxd);
	
	return {qmap, quality};	
}



std::pair<Image, double>
quality_badpix(const Image& ref, const Image& reading, double threshold, double maxd)
{
	if (threshold == auto_param_from_data) threshold = 255. * 0.07;
	if (maxd == auto_param_from_data) maxd = 255.;
	
	Image qualitymap = Image::zeros(ref.rows, ref.cols, CV_32FC3);
	double quality = 0.;
	
	Image rmask(ref.rows, ref.cols, CV_8UC1);
	Image rfmask;
	cv::threshold(ref, rfmask, 1., 255., cv::THRESH_BINARY);
	cv::cvtColor(rfmask, rfmask, cv::COLOR_BGR2GRAY);
	rfmask.convertTo(rmask, CV_8UC1);
	
	Image lmask(ref.rows, ref.cols, CV_8UC1);
	Image lfmask;
	cv::threshold(reading, lfmask, 1., 255., cv::THRESH_BINARY);
	cv::cvtColor(lfmask, lfmask, cv::COLOR_BGR2GRAY);
	lfmask.convertTo(lmask, CV_8UC1); 
	
	Image mask(ref.rows, ref.cols, CV_8UC1);
	cv::bitwise_and(rmask, lmask, mask);
	
	cv::subtract(ref, reading, qualitymap, mask);
	qualitymap = cv::abs(qualitymap);	
	
	Image tm;	
	cv::threshold(qualitymap, tm, threshold, 255., cv::THRESH_BINARY_INV);
	cv::cvtColor(tm, tm, cv::COLOR_BGR2GRAY);
	tm.convertTo(tm, CV_8U);
	
	qualitymap.setTo(0., tm);
	quality = cv::sum(qualitymap)[0] / cv::countNonZero(mask);
	
	Image qmap;
	qualitymap.convertTo(qmap, CV_8UC3, 255. / maxd);
	
	return {qmap, quality};	
}

std::pair<Image, double>
quality_psnr(const Image& ref, const Image& reading, double maxv)
{
	if (maxv == auto_param_from_data) maxv = 255.; 

	Image qualitymap = Image::zeros(ref.rows, ref.cols, CV_32FC3);
	double quality = 0.;
	
	Image rmask(ref.rows, ref.cols, CV_8UC1);
	Image rfmask;
	cv::threshold(ref, rfmask, 1., 255., cv::THRESH_BINARY);
	cv::cvtColor(rfmask, rfmask, cv::COLOR_BGR2GRAY);
	rfmask.convertTo(rmask, CV_8UC1);
	
	Image lmask(ref.rows, ref.cols, CV_8UC1);
	Image lfmask;
	cv::threshold(reading, lfmask, 1., 255., cv::THRESH_BINARY);
	cv::cvtColor(lfmask, lfmask, cv::COLOR_BGR2GRAY);
	lfmask.convertTo(lmask, CV_8UC1); 
	
	Image mask(ref.rows, ref.cols, CV_8UC1);
	cv::bitwise_and(rmask, lmask, mask);
	
	cv::subtract(ref, reading, qualitymap, mask);
	cv::multiply(qualitymap, qualitymap, qualitymap); // slightly faster than pow2
	
	Image mqualitymap = Image::zeros(ref.rows, ref.cols, CV_32FC3);
	qualitymap.copyTo(mqualitymap, mask);

	auto mean = cv::mean(mqualitymap, mask); DEBUG_VAR(mean);
	//mse_to_psnr
	for (std::size_t i = 0; i < 3; ++i)
	{
		if (mean[i] != 0.) mean[i] = 10. * std::log10((maxv * maxv) / mean[i]);
	}
	DEBUG_VAR(mean);	
	quality = (mean[0] + mean[1] + mean[2]) / 3.;
	
	Image qmap;
	mqualitymap.convertTo(qmap, CV_8UC3, 1. / maxv);
	
	return {qmap, quality};	
}

std::pair<Image, double>
quality_ssim(const Image& ref, const Image& reading)
{	
	Image qualitymap;
	double quality = 0.;
	
	constexpr double C1 = 6.5025, C2 = 58.5225;
	
	//blur function
	auto blur = [](const Image& mat) {
		Image result = {};
		cv::GaussianBlur(mat, result, cv::Size(11, 11), 1.5);
		return result;
	};
	
	Image rmask(ref.rows, ref.cols, CV_8UC1);
	Image rfmask;
	cv::threshold(ref, rfmask, 1., 255., cv::THRESH_BINARY);
	cv::cvtColor(rfmask, rfmask, cv::COLOR_BGR2GRAY);
	rfmask.convertTo(rmask, CV_8UC1);
	
	Image lmask(ref.rows, ref.cols, CV_8UC1);
	Image lfmask;
	cv::threshold(reading, lfmask, 1., 255., cv::THRESH_BINARY);
	cv::cvtColor(lfmask, lfmask, cv::COLOR_BGR2GRAY);
	lfmask.convertTo(lmask, CV_8UC1); 
	
	Image mask(ref.rows, ref.cols, CV_8UC1);
	cv::bitwise_and(rmask, lmask, mask);
	
	//intermediate images
	Image lhsI, rhsI, lhsI_2, rhsI_2, lhsmu, rhsmu, lhsmu_2, rhsmu_2, lhssigma_2, rhssigma_2;
	
	//LHS
	lhsI = ref; rhsI = reading;
	cv::multiply(lhsI, lhsI, lhsI_2);
	lhsmu = blur(lhsI);
	cv::multiply(lhsmu, lhsmu, lhsmu_2);
	lhssigma_2 = blur(lhsI_2);
	cv::subtract(lhssigma_2, lhsmu_2, lhssigma_2);
	
	//RHS
	rhsI = reading;
	cv::multiply(rhsI, rhsI, rhsI_2);
	rhsmu = blur(rhsI);
	cv::multiply(rhsmu, rhsmu, rhsmu_2);
	rhssigma_2 = blur(rhsI_2);
	cv::subtract(rhssigma_2, rhsmu_2, rhssigma_2);
	
	//intermediate images
	Image I1_I2, mu1_mu2, sigma12, t1, t2, t3;
	
	cv::multiply(lhsI, rhsI, I1_I2);
	cv::multiply(lhsmu, rhsmu, mu1_mu2);
	cv::subtract(blur(I1_I2), mu1_mu2, sigma12);

	// t3 = ((2*mu1_mu2 + C1).*(2*sigma12 + C2))
	cv::multiply(mu1_mu2, 2., t1);
	cv::add(t1, C1, t1);// t1 += C1

	cv::multiply(sigma12, 2., t2);
	cv::add(t2, C2, t2);// t2 += C2

	// t3 = t1 * t2
	cv::multiply(t1, t2, t3);

	// t1 =((mu1_2 + mu2_2 + C1).*(sigma1_2 + sigma2_2 + C2))
	cv::add(lhsmu_2, rhsmu_2, t1);
	cv::add(t1, C1, t1);

	cv::add(lhssigma_2, rhssigma_2, t2);
	cv::add(t2, C2, t2);

	// t1 *= t2
	cv::multiply(t1, t2, t1);

	// quality map: t3 /= t1
	cv::divide(t3, t1, qualitymap);
	
	Image mqualitymap = Image::zeros(ref.rows, ref.cols, CV_32FC3);
	qualitymap.copyTo(mqualitymap, mask);
	
	auto mean = cv::mean(mqualitymap, mask); DEBUG_VAR(mean);
	quality = (mean[0] + mean[1] + mean[2]) / 3.;
		
	Image qmap;
	mqualitymap.convertTo(qmap, CV_8UC3, 255.);
	
	return {qmap, 100. * quality};
}


std::pair<Image, double>
quality(const Image& ref, const Image& reading, ImageQualityType qt, double param)
{
	std::pair<Image, double> result;

	switch (qt)
	{
		case ImageQualityType::MSE:
		{
			result = quality_mse(ref, reading, param);		
			break;
		}	
		case ImageQualityType::MAE:
		{
			result = quality_mae(ref, reading, param);		
			break;
		}	
		case ImageQualityType::BADPIX:
		{
			result = quality_badpix(ref, reading, param);		
			break;
		}	
		case ImageQualityType::PSNR:
		{
			result = quality_psnr(ref, reading, param);		
			break;
		}	
		case ImageQualityType::SSIM:
		{
			result = quality_ssim(ref, reading);		
			break;
		}	
	}

	return result;
}
