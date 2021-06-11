#include "preprocess.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/fast_math.hpp>

#include "graphic/gui.h"
#include "graphic/viewer_2d.h"

#include "io/printer.h"
#include "io/choice.h"

#include "processing/estimation.h"
#include "processing/imgproc/improcess.h"
#include "processing/tools/stats.h"
#include "processing/tools/lens.h"

#include "unused.h"

#include "processing/precalibration/display.h"
#include "processing/precalibration/export.h"

////////////////////////////////////////////////////////////////////////////////
// Processing
////////////////////////////////////////////////////////////////////////////////
void compute_radii(
	const Image& img, const MIA& centers, 
	std::vector<MicroImage>& data, std::size_t I
)
{
	DEBUG_ASSERT((I > 0u), "Can't compute radii when I=0");
	
	static constexpr double sigma99p = 2.357022604; // 2.575829; //2.33;//  2.575829 sigma ---> 99%
	
	static const decltype(v::red) colors[] = {
		v::red, v::green, v::blue, v::purple, v::yellow, v::orange
	}; //FIXME: load palette

    data.clear();
    data.reserve(centers.size());
	
/////////////////////////////////////Grayscale conversion///////////////////////////////////////////
    PRINT_INFO("Grayscale conversion");
    
    Image preprocessed = Image::zeros(img.rows, img.cols, CV_8UC1);
    cv::cvtColor(img, preprocessed, cv::COLOR_BGR2GRAY);
    
/////////////////////////////////////Blurred operation///////////////////////////////////////////
    PRINT_INFO("Blurred operation preserving the edges");
    
    Image blurred;
    cv::bilateralFilter(preprocessed, blurred, 3, 3.*2., 3./2., cv::BORDER_DEFAULT);
    RENDER_DEBUG_2D(Viewer::context().layer(Viewer::layer()++).name("Preprocessing::bblurred"), blurred);  
    	 
/////////////////////////////////////Compute radii///////////////////////////////////////////////////
	PRINT_INFO("Computing radius of each micro-image");
	constexpr int roiborder = 1;
	const int roiw = std::floor(centers.diameter()) - roiborder;
	const int roih = roiw;
	
	const std::size_t borderk = centers.width() / 4;
	const std::size_t borderl = centers.height() / 4;

    Viewer::stash();
    
    for (std::size_t k = borderk; k < centers.width() - borderk ; ++k) //iterate through columns //x-axis
    {
    	for (std::size_t l = borderl; l < centers.height() - borderl; ++l) //iterate through lines //y-axis
		{
			Viewer::pop();	
					
	 		const P2D c = centers.nodeInWorld(k,l); //col,row
			const int t = centers.type(I,k,l); //k=col, l=row
			
			//crop image aroud the center
			double X = c[0], Y = c[1]; 
	 		
	 		Image roi = extract_roi(blurred, X, Y, roiw, roih); //crop
 			cv::normalize(roi, roi, 0, 255, cv::NORM_MINMAX); //normalize
 			
			PRINT_DEBUG("------ Processing node ("<< k << ", " << l <<")"
				"at (" << c.transpose() <<") in image,"
				"at ("<<c[0]-X<<","<<c[1]-Y<<") in roi"
			);
 			
 			//const auto& [meanx, meany, sigma, alpha] = estimation_gaussian_least_squares(roi, {X-x, Y-y, 0., 0.}, true, 200);
 			const auto& [meanx, meany, sigma, alpha] = estimation_gaussian_moments(roi); UNUSED(alpha);
	 		
	 		const double r = sigma99p * sigma; DEBUG_VAR(r);
	 		data.emplace_back(k, l, P2D{X+meanx,Y+meany}, r, t);
	 			
 			RENDER_DEBUG_2D(
	  			Viewer::context().layer(Viewer::layer()++).name("main:radii(0:red,1:green,2:blue)").pen_color(colors[t]).pen_width(5),
	  			Disk{P2D{X+meanx, Y+meany}, r}
			);			
		} //end iterate through lines
	}//end iterate through columns 
}

void compute_coefs(
	const std::vector<std::vector<P2D>>& data, 
	std::vector<LineCoefficients>& coefs
)
{
	coefs = estimation_lines_with_slope_constraint_least_squares(data);
	
	GUI(
		int i = 0;
		for (const auto&points : data)
		{
			const auto& [m,c] = coefs[i];
			PRINT_DEBUG("Line("<<i<<"): y = "<< m << " * x + " << c);	
			Image graph;
			display_data(points, coefs[i++], graph);
			RENDER_DEBUG_2D(Viewer::context().layer(Viewer::layer()++).name("graph("+std::to_string(i-1)+")"), graph);
			
		}
  
		Image graph;
		display_all_data(data, coefs, graph);
		RENDER_DEBUG_2D(Viewer::context().layer(Viewer::layer()++).name("graph(all)"), graph); 
    );
}

InternalParameters
preprocess(
	const std::vector<ImageWithInfo>& imgs, 
	const MIA& grid, 
	double pxl2metric,
	std::size_t I_,
	int mode, double fmatchingnumber
)
{
	const std::size_t I = (I_ == 0) ? 1 : I_;
	const double sgn = (mode == PlenopticCamera::Mode::Galilean) ? -1.0 : 1.0;
	
	std::vector<std::vector<P2D>> data(I);
	for (auto& d: data) d.reserve(grid.size()*imgs.size());
		
	for (const auto& [img, fnumber, frame] : imgs)
	{
		UNUSED(frame);
		
		PRINT_INFO("=== Computing radii in image f/" << fnumber);
		
		std::vector<MicroImage> microimages;
		compute_radii(img, grid, microimages, I);

		GUI(
			PRINT_DEBUG("Displaying histograms of radii repartitions");
			hist_radii(microimages);
			Viewer::update();
			
			wait();
		);
		
		//for each microimage compute the pair (invfnumber, radius)
		for (const auto&mi: microimages)
		{
			data[mi.type].emplace_back(
					1./fnumber, 
					sgn * pxl2metric * mi.radius //see note in subsection (6.2)
			);
		}
	}
	
	for (auto& d: data) d.shrink_to_fit();
	export_radii(data);
	
	PRINT_INFO("=== Computing lines coefficients");
	std::vector<LineCoefficients> coefs;
	compute_coefs(data, coefs);
	
	PRINT_INFO("=== Computing internal parameters");
	InternalParameters params;
	{
		constexpr double lambda = 0.9938; //FIXME: lambda should be estimated from Eq.(28)
		
		params.m 		= coefs[0].m ; // Eq.(10)
		params.scale	= pxl2metric;
		params.dc 		= pxl2metric * grid.diameter();
		params.dC 		= lambda * params.dc; // Eq.(3)
		params.lambda	= lambda;

		params.q.resize(I);
		params.q_prime.resize(I);
		for (std::size_t i = 0; i < I; ++i) 
		{
			params.q[i] = coefs[i].c ; // Eq.(10)
			params.q_prime[i] = coefs[i].c + params.dc / 2.; // Eq.(10)
		}
		
		//For debug only
		for (std::size_t i = 0; i < I; ++i) 
			PRINT_DEBUG("r(1/" << fmatchingnumber << ")|" << i << " = " << (params.m / 4. + params.q[i]) / pxl2metric);
		for (std::size_t i = 0; i < I; ++i) 
			PRINT_DEBUG("N|"<<i<<" = " << std::fabs(params.m) / ( params.dc / 2.0 - std::fabs(params.q[i])));

		double N = 0.0;
		for (std::size_t i = 0; i < I; ++i) N += (std::fabs(params.m) / ( params.dc / 2.0 - std::fabs(params.q[i])));
		N = N / I;
		
		PRINT_WARN("Estimated N must be near the fnumber respecting the matching principle.");
		PRINT_WARN("N = " << N << ", diff = " << (fmatchingnumber-N));
		
		params.N = fmatchingnumber;
		params.I = I_;
	}
	
	return params;
}
