#include "disparity.h"

#include "unused.h"

#include "io/printer.h"
#include "io/choice.h"

#include "processing/imgproc/improcess.h"

#define ENABLE_DEBUG_DISPLAY 0
#define ENABLE_DEBUG_SAVE_IMG 0
#define USE_CORRECTED_DISPARITY 1

//******************************************************************************
//******************************************************************************
//******************************************************************************
template <bool useBlur>
DisparityCostError_<useBlur>::DisparityCostError_(
	const MicroImage& mii_, const MicroImage& mij_, 
	const PlenopticCamera& mfpc_, const P2D& at_, BlurMethod method_
) : mii{mii_}, mij{mij_}, mfpc{mfpc_}, at{at_}, method{method_}
{ }

template <bool useBlur>
DisparityCostError_<useBlur>::DisparityCostError_(
	const DisparityCostError_& o
) : mii{o.mii}, mij{o.mij}, mfpc{o.mfpc},
	at{o.at}, method{o.method}
{ } 

template <bool useBlur>
DisparityCostError_<useBlur>::DisparityCostError_(
	DisparityCostError_&& o
) : mii{std::move(o.mii)}, mij{std::move(o.mij)}, mfpc{o.mfpc},
	at{std::move(o.at)}, method{o.method}
{ }

template <bool useBlur>
P2D DisparityCostError_<useBlur>::disparity(double v) const 
{ 
	//mi k,l indexes are in mi space, convert to mla space to access micro-lenses
	const P2D idxi = mfpc.mi2ml(mii.k, mii.l);
	
#if USE_CORRECTED_DISPARITY //Corrected disparity (see §5, Eq.(24))
	const P2D deltac = (mii.center - mij.center);
	
	const double D = mfpc.D(idxi(0), idxi(1)); 
	const double d = mfpc.d(idxi(0), idxi(1)); 
	
	const double lambda = D / (D + d);
	
	const P2D disparity = (deltac) * (
		((1. - lambda) * v + lambda) / (v)
	); 
#else //Commonly used disparity, i.e., orthogonal projection (see §5, Eq.(23)) 
	const P2D idxj = mfpc.mi2ml(mij.k, mij.l); 
	
	const P3D mli = mfpc.mla().node(idxi(0), idxi(1)); 
	const P3D mlj = mfpc.mla().node(idxj(0), idxj(1));

	const P2D deltaC = (mlj - mli).head<2>();
	
	const P2D disparity = (deltaC) / (v * mfpc.sensor().scale());
#endif
	return disparity; //in pixel
}

template <bool useBlur>
double DisparityCostError_<useBlur>::weight(double v) const 
{ 
	P2D disp = disparity(v); 
	const double d = disp.norm();
	
	const double sigma2_disp = 1.;
	const double sigma2_v = sigma2_disp * (v * v) / (d * d); 
	
	return 1. / sigma2_v; 
}

template <bool useBlur>
bool DisparityCostError_<useBlur>::compute_at_pixel() const
{
	return not (at[0] == -1. and at[1] == -1.);
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
template <bool useBlur>
bool DisparityCostError_<useBlur>::operator()(
    const VirtualDepth& depth,
	ErrorType& error
) const
{    
	constexpr int window_size = 5; //FIXME: size ? N=5 ?
    constexpr double threshold_reprojected_pixel = 9.; //double(window_size * window_size) - 1.; //FIXME: 9?
    constexpr double epsilon = 2.2204e-16;

    error.setZero();
    
	const double v = depth.v;
	const double radius = mfpc.mia().radius() - mfpc.mia().border() - 1.5;
    
//0) Check hypotheses:
	//0.1) Discard observation?
	const P2D disp = disparity(v);
	
	if (disp.norm() >= 2. * radius)
	{
		//PRINT_ERR("Discard observation for hypothesis = " << v << ", disparity = " << disparity.norm());
		return false;
	}
	//0.2) Is disparity estimation possible? Is object real?
	if (std::fabs(v) < 2. or mfpc.v2obj(v) < mfpc.focal()) 
	{
		error[0] = 255. / v;
		return true;
	}
    
//1) get ref and edi
	Image fref, ftarget;	
	mii.mi.convertTo(fref, CV_64FC1, 1./255.); //(i)-view is ref
	mij.mi.convertTo(ftarget, CV_64FC1, 1./255.); //(j)-view has to be warped
	
//2) compute mask	
	Image fmask = Image{fref.size(), CV_64FC1, cv::Scalar::all(1.)};
	trim_double(fmask, radius);

#if ENABLE_DEBUG_SAVE_IMG
	Image buff;  
	fref.convertTo(buff, CV_8UC1, 255.);
	cv::imwrite("ref-"+std::to_string(getpid())+".png", buff);
	ftarget.convertTo(buff, CV_8UC1, 255.);
	cv::imwrite("target-"+std::to_string(getpid())+".png", buff);
	fmask.convertTo(buff, CV_8UC1, 255.);
	cv::imwrite("mask-"+std::to_string(getpid())+".png", buff);
#endif
	
//3) compute blur	
	Image fedi, lpedi;		
	if constexpr (useBlur)
	{
		if (mii.type != mij.type) //not same type
		{
			const P2D idxi = mfpc.mi2ml(mii.k, mii.l);
			const P2D idxj = mfpc.mi2ml(mij.k, mij.l); 
			
			const double A = mfpc.mlaperture(); //mm
			const double d_i = mfpc.d(idxi(0), idxi(1));	//mm, orthogonal distance center-sensor
			const double d_j = mfpc.d(idxj(0), idxj(1));	//mm, orthogonal distance center-sensor
			const double a_i = mfpc.obj2mla(mfpc.focal_plane(mii.type), idxi(0), idxi(1));  //mm, negative signed orthogonal distance to mla
			const double a_j = mfpc.obj2mla(mfpc.focal_plane(mij.type), idxj(0), idxj(1));  //mm, negative signed orthogonal distance to mla

			const double m_ij = ((A * A) / 2.) * ((d_i / a_i) - (d_j / a_j)); //mm², see Eq.(19)
			const double q_ij = ((A * A) / 4.) * (((d_i * d_i) / (a_i * a_i)) - ((d_j * d_j) / (a_j * a_j))); //mm², see Eq.(20)

			const double rel_blur = m_ij * (1. / v) + q_ij; //mm², see Eq.(18)

			const double rho_sqr_r = rel_blur / (mfpc.sensor().scale() * mfpc.sensor().scale()); //pix²
			const double kappa = mfpc.params().kappa; 
			const double sigma_sqr_r = kappa * rho_sqr_r; //pix²
			const double sigma_r = std::sqrt(std::fabs(sigma_sqr_r)); //pix, see Eq.(22)

			const bool isOrdered = (rel_blur >= 0.); //(i)-view is more defocused the the (j)-view
		
			if (isOrdered) //(i)-view is more defocused the the (j)-view
			{
		#if ENABLE_DEBUG_DISPLAY
				PRINT_DEBUG("Views: (edi = target); ref ("<< mii.type+1<<"), target ("<< mij.type+1<<")");
		#endif 
				fedi = ftarget; //(j)-view has to be equally-defocused
			}
			else //(j)-view is more defocused the the (i)-view
			{
		#if ENABLE_DEBUG_DISPLAY
				PRINT_DEBUG("Views: (edi = ref); ref ("<< mii.type+1<<"), target ("<< mij.type+1<<")");
		#endif 
				fedi = fref; //(i)-view has to be equally-defocused
			}
			
			switch (method)
			{	
				case S_TRANSFORM:
				{
					cv::Laplacian(fedi, lpedi, CV_64FC1); //FIXME: Kernel size ? 3x3
					cv::add(fedi, (std::fabs(sigma_sqr_r) / 4.) * lpedi, fedi); //see Eq.(29)
					
					#if ENABLE_DEBUG_SAVE_IMG
						lpedi.convertTo(buff, CV_8UC1, 255.);
						cv::imwrite("laplacian-"+std::to_string(getpid())+".png", buff);
						fedi.convertTo(buff, CV_8UC1, 255.);
						cv::imwrite("edi-"+std::to_string(getpid())+".png", buff);
					#endif
					
					break;
				}	
				case GAUSSIAN_BLUR:
				{
					Image medi, bedi, bmask;
					
					medi = fedi.mul(fmask); //apply mask on edi
					cv::GaussianBlur(fmask, bmask, cv::Size{0,0}, sigma_r, sigma_r); //blur mask
					cv::GaussianBlur(fedi, bedi, cv::Size{0,0}, sigma_r, sigma_r); //blur masked image	
					cv::divide(bedi, bmask, fedi); //divide the blurred masked image by the blurred mask 
					break;
				}
				case APPROX_GAUSSIAN_BLUR: //blur w/o considering neighbors impact
				{
					cv::GaussianBlur(fedi, fedi, cv::Size{0,0}, sigma_r, sigma_r);
					break;
				}	
			}
		}
	}
	else
	{
		UNUSED(fedi); UNUSED(lpedi);
	}
		
//4) warp image according to depth hypothesis
	//4.1) compute transformation
	cv::Mat M = (cv::Mat_<double>(2,3) << 1., 0., disp[0], 0., 1., disp[1]); //Affine transformation, see Eq.(26)
	
	//4.2) warp mask and target	
	const auto interp = cv::INTER_LINEAR; //cv::INTER_CUBIC; //
	Image wmask, wtarget;
	cv::warpAffine(fmask, wmask, M, fmask.size(), interp + cv::WARP_INVERSE_MAP, cv::BORDER_CONSTANT, cv::Scalar::all(0.));
	cv::warpAffine(ftarget, wtarget, M, ftarget.size(), interp + cv::WARP_INVERSE_MAP, cv::BORDER_CONSTANT, cv::Scalar::all(0.));
	
	trim_double(wmask, radius); //Final mask =  micro-image mask INTER warped masked, see Eq.(28)
	
	//4.3) apply mask
	Image finalref = fref.mul(wmask);
	Image finaltarget = wtarget.mul(wmask); 	
	
	#if ENABLE_DEBUG_SAVE_IMG
		wmask.convertTo(buff, CV_8UC1, 255.);
		cv::imwrite("wmask-"+std::to_string(getpid())+".png", buff);
		wtarget.convertTo(buff, CV_8UC1, 255.);
		cv::imwrite("wtarget-"+std::to_string(getpid())+".png", buff);
		finalref.convertTo(buff, CV_8UC1, 255.);
		cv::imwrite("mref-"+std::to_string(getpid())+".png", buff);
		finaltarget.convertTo(buff, CV_8UC1, 255.);
		cv::imwrite("mwtarget-"+std::to_string(getpid())+".png", buff);
	#endif
	
	//4.4) get sub window if at specific pixel
	const int h = mii.mi.rows;
	const int w = mii.mi.cols;
	
	cv::Rect window{0,0, w, h}; //initialize at all the micro-image content
	if (compute_at_pixel())
	{
		const auto [cu, cv] = ((at - mii.center).norm() < (at-mij.center).norm()) ?
				std::pair<double, double>{mii.center[0], mii.center[1]} 
			: 	std::pair<double, double>{mij.center[0], mij.center[1]};
	
		const int s = std::min(
			w-(window_size/2)-1, 
			std::max(0,	static_cast<int>(std::round(at[0] - cu + double(w / 2.))))
		);
				
		const int t = std::min(
			h-(window_size/2)-1, 
			std::max(0, static_cast<int>(std::round(at[1] - cv + double(h / 2.))))
		); 		
		//FIXME: think about getting a rotated rectangle along EPI
		window = cv::Rect{s, t, window_size, window_size};
	}
	
//5) compute cost	
	const double summask = cv::sum(wmask(window))[0];
	if (summask < threshold_reprojected_pixel) return false;
	
	const double normalization = 1. / (summask + epsilon); //number of pixel to take into account, see Eq.(31)
	const double err = cv::norm(finalref(window), finaltarget(window), cv::NORM_L1) * normalization; //normalized SAD = MAD, see Eq.(30)

	error[0] = err;
	
#if ENABLE_DEBUG_DISPLAY
	Image costimg;
	cv::add(finalref, -1. * finaltarget, costimg);
	costimg = cv::abs(costimg);

	cv::namedWindow("ref", cv::WINDOW_NORMAL);
	cv::namedWindow("fref", cv::WINDOW_NORMAL);
	cv::namedWindow("target", cv::WINDOW_NORMAL);
	cv::namedWindow("ftarget", cv::WINDOW_NORMAL);
	cv::namedWindow("wmask", cv::WINDOW_NORMAL);
	cv::namedWindow("cost", cv::WINDOW_NORMAL);
	
	cv::resizeWindow("ref", 200u, 200u);
	cv::resizeWindow("fref", 200u, 200u);
	cv::resizeWindow("target", 200u, 200u);
	cv::resizeWindow("ftarget", 200u, 200u);
	cv::resizeWindow("wmask", 200u, 200u);
	cv::resizeWindow("cost", 200u, 200u);
	
	cv::imshow("fref", finalref);
	cv::imshow("ref", fref);
	cv::imshow("target", ftarget);
	cv::imshow("ftarget", finaltarget);
	cv::imshow("wmask", wmask);
	cv::imshow("cost", costimg);
	
	if (compute_at_pixel())
	{
		cv::namedWindow("wcost", cv::WINDOW_NORMAL);
		cv::resizeWindow("wcost", 200u, 200u);
		cv::imshow("wcost", costimg(window));
	}

	if constexpr (useBlur)
	{
		if (mii.type != mij.type) //if not the same type
		{
			cv::namedWindow("lpedi", cv::WINDOW_NORMAL);
			cv::resizeWindow("lpedi", 200u, 200u);
			cv::imshow("lpedi", lpedi);
		}	
		DEBUG_VAR(sigma_sqr_r);
	}
	
	DEBUG_VAR(v);
	DEBUG_VAR(deltaC);
	DEBUG_VAR(disparity);
	DEBUG_VAR(err);
	DEBUG_VAR(normalization);
	PRINT_DEBUG("---------------------------");
	
	wait();
#endif
	
    return true;
}

template struct DisparityCostError_<false>;
template struct DisparityCostError_<true>;

