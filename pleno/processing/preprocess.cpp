#include "preprocess.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/fast_math.hpp>

#include "graphic/gui.h"
#include "graphic/viewer_2d.h"

#include "io/printer.h"

#include "processing/estimation.h"
#include "processing/improcess.h"
#include "processing/tools/stats.h"
#include "processing/tools/lens.h"

////////////////////////////////////////////////////////////////////////////////
// DISPLAYS - Histograms
////////////////////////////////////////////////////////////////////////////////
void hist_data(const std::vector<double>& data, Image& dst, double min, double max, int binSize, int height, int nbbin)
{		
	float step = (max - min) / nbbin;
	std::vector<int> hist(nbbin);
	for(const auto&d : data)
	{
		if(d<min or d>max) continue;
		int i =  (d - min) / step;
		hist[i] +=1;	
	}

    int max_value = *std::max_element(hist.begin(), hist.end());
    int rows = 0;
    int cols = 0;
    float scale = 1;
    if (height == 0) {
        rows = max_value + 10;
    }
    else {
        rows = height; 
        scale = float(height) / (max_value + 10);
    }
    cols = hist.size() * binSize;
    dst = Image::zeros(rows, cols, CV_8UC3);
    for (std::size_t i = 0; i < hist.size(); ++i)
    {
        const int h = rows - int(scale * hist[i]);
        cv::rectangle(dst, cv::Point(i*binSize, h), cv::Point((i + 1)*binSize - 1, rows), (i % 2) ? cv::Scalar(0, 100, 255) : cv::Scalar(0, 0, 255), CV_FILLED);
    }

	const double rmean = mean(data);
	const double rmedian = median(data);
	
	const double col_mean = (rmean - min) / step * binSize;
	const double col_median = (rmedian - min) / step * binSize; 
	
	cv::line(dst, cv::Point(col_mean, 20), cv::Point(col_mean, rows), cv::Scalar(255,0,0));
	cv::line(dst, cv::Point(col_median, 10), cv::Point(col_median, rows), cv::Scalar(0,255,0));
	
	cv::putText(dst, 
            "Mean= " + std::to_string(rmean),
            cv::Point(col_mean+5, 20), // Coordinates
            cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
            0.5, // Scale. 2.0 = 2x bigger
            cv::Scalar(255,0,0), // BGR Color
            1 // Line Thickness (Optional)
	); 
	cv::putText(dst, 
            "Median= " + std::to_string(rmedian),
            cv::Point(col_median+5,10), // Coordinates
            cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
            0.5, // Scale. 2.0 = 2x bigger
            cv::Scalar(0,255,0), // BGR Color
            1 // Line Thickness (Optional)
	); 
}

void hist_radii(const std::vector<MicroImage>& data)
{
	const std::size_t I = [&data]{
		int i=0;
		for(const auto& mi : data) if(mi.type > i) i = mi.type;
		return std::size_t(i+1);
	}();
	DEBUG_VAR(I);
	
	std::vector<std::vector<double>> radii(I);
    for(auto& r : radii) r.reserve(data.size());
	
    for(const auto&mi : data)
    {
    	radii[mi.type].emplace_back(mi.radius);
    }
    
    for(auto& r : radii) r.shrink_to_fit();
    
/////////////////////////////////////Compute histograms///////////////////////////////////////////////////   
    const auto& [min, max] = [&data]() -> std::pair<double, double> {
		double min=1e12, max=-1e12;
		for(const auto&mi : data) {
			const double d = mi.radius;
			if(d < min) min = d;
			if(d > max) max = d;
		}
		return {std::floor(min), std::ceil(max)};
	}(); 
	
    Images hist(I);
    for(std::size_t i=0; i<I; ++i)
    { 
    	hist_data(radii[i], hist[i], min, max, 3, 200);
   	}
   	
	Image histo = hist[0];
	for(std::size_t i=1; i<I; ++i) 
	{
    	vconcat(histo, hist[i], histo);
    }
    RENDER_DEBUG_2D(Viewer::context().layer(Viewer::layer()).name("radii_histo"), histo); 
}

////////////////////////////////////////////////////////////////////////////////
// DISPLAYS - data
////////////////////////////////////////////////////////////////////////////////
void display_data(const std::vector<P2D>& pts, const LineCoefficients& coefs, Image& out)
{
	const std::size_t rows = 500;
	const std::size_t cols = 600;
	
	const float xmax = 0.3;
	const float xmin = 0.0;
	const float ymin = 0.01;
	const float ymax = 0.1;
	
	const auto & [m,c] = coefs;
	const float cscaled = rows * (1.f - (c - ymin) / (ymax - ymin) + ymin ); 
	const float endscaled = rows * (1.f - ((m * xmax + c) - ymin) / (ymax - ymin) + ymin);
	
	out = Image::zeros(rows, cols, CV_8UC3);
	
	for(const auto& p : pts)
	{
		cv::Point2f pscaled;
		pscaled.x = cols * (p[0] - xmin) / (xmax - xmin);
		pscaled.y = rows * (1.f - (p[1] - ymin) / (ymax - ymin)); 
			
		cv::circle(out, pscaled, 2, cv::Scalar(0, 255, 255));
	}
	
	cv::line(out, cv::Point(0, cscaled), cv::Point(cols, endscaled), cv::Scalar(255,0,0));
}

void display_all_data(const std::vector<std::vector<P2D>>& data, const std::vector<LineCoefficients>& coefs, Image& out)
{
	const std::size_t rows = 500;
	const std::size_t cols = 800;
	
	const float xmax = 0.3;
	const float xmin = 0.0;
	const float ymin = 0.02;
	const float ymax = 0.08;
	
	out = Image::zeros(rows, cols, CV_8UC3);
	
	cv::Scalar colors_pts[] = {cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255)};
	int i=0;
	for(const auto& pts : data)
	{	
		for(const auto& p : pts)
		{
			cv::Point2f pscaled;
			pscaled.x = cols * (p[0] - xmin) / (xmax - xmin) + 3.*i;
			pscaled.y = rows * (1.f - (p[1] - ymin) / (ymax - ymin)); 
				
			cv::circle(out, pscaled, 1, colors_pts[i]);
		}
		i++;
	}
#if 1	
	cv::Scalar colors_lines[] = {cv::Scalar(255, 255, 0), cv::Scalar(0, 255, 255), cv::Scalar(255, 0, 255)};
	i=0;
	for(const auto & [m,c] : coefs)
	{		
		const float cscaled = rows * (1.f - (c - ymin) / (ymax - ymin) + ymin ); 
		const float endscaled = rows * (1.f - ((m * xmax + c) - ymin) / (ymax - ymin) + ymin);
		DEBUG_VAR(endscaled);
	
		cv::line(out, cv::Point(0, cscaled), cv::Point(cols, endscaled), colors_lines[i++]);
	}
#endif
}

////////////////////////////////////////////////////////////////////////////////
// Processing
////////////////////////////////////////////////////////////////////////////////
void compute_radii(const Image& img, const MIA& centers, std::vector<MicroImage>& data, std::size_t I)
{
	assert(I > 0u);
	
	static constexpr double sigma99p = 2.357022604; // 2.575829; //2.33;//  2.575829 sigma ---> 99%
	
	static const decltype(v::red) colors[3] = {v::red, v::green, v::blue}; //FIXME: load palette

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
	constexpr int roiw=22;
    constexpr int roih=22;
    
    constexpr std::size_t EXCLUDED_BORDER_SIZE = 50; //70
    
    Viewer::stash();
    
    for(std::size_t k = 0 + EXCLUDED_BORDER_SIZE; k < centers.width() - EXCLUDED_BORDER_SIZE ; ++k) //iterate through columns //x-axis
    {
    	for(std::size_t l = 0 + EXCLUDED_BORDER_SIZE ; l < centers.height()/2. /*- EXCLUDED_BORDER_SIZE * 1.5 */; ++l) //iterate through lines //y-axis
		{
			Viewer::pop();	
					
	 		const auto& c = centers.nodeInWorld(k,l); //col,row
			const int t = lens_type(I,k,l); //k=col, l=row
			
			//crop image aroud the center
			float X = c[0], Y = c[1]; 
	 		
	 		Image roi = extract_roi(blurred, X, Y, roiw, roih); //crop
 			cv::normalize(roi, roi, 0, 255, cv::NORM_MINMAX); //normalize
 			
			PRINT_DEBUG("------ Processing node ("<< k << ", " << l <<")"
				"at (" << c.transpose() <<") in image,"
				"at ("<<c[0]-X<<","<<c[1]-Y<<") in roi"
			);
 			
 			//const auto& [meanx, meany, sigma, alpha] = estimation_gaussian_least_squares(roi, {X-x, Y-y, 0., 0.}, true, 200);
 			const auto& [meanx, meany, sigma, alpha] = estimation_gaussian_moments(roi);
	 		
	 		const double r = sigma99p * sigma; DEBUG_VAR(r);
	 		data.emplace_back(MicroImage{k,l,P2D{X+meanx,Y+meany},r,t});
	 			
 			RENDER_DEBUG_2D(
	  			Viewer::context().layer(Viewer::layer()++).name("main:radii(0:red,1:green,2:blue)").pen_color(colors[t]).pen_width(5),
	  			Disk{P2D{X+meanx, Y+meany},r}
			);			
		} //end iterate through lines
	}//end iterate through columns 
}

void compute_coefs(const std::vector<std::vector<P2D>>& data, std::vector<LineCoefficients>& coefs)
{
	coefs = estimation_lines_with_slope_constraint_least_squares(data);
	
	GUI(
		int i=0;
		for(const auto&points : data)
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

void export_radii(const std::vector<std::vector<P2D>>& data)
{
	std::ofstream ofs("radii-"+std::to_string(getpid())+".csv");
	if (!ofs.good())
		throw std::runtime_error(std::string("Cannot open file errors.csv"));
	
	ofs << "type,invf,r\n";
	
	for(int type = 0; type < int(data.size()); ++type)
	{
		for(const auto& d: data[type])
		{
			std::ostringstream oss;
			
			oss << type << ","
				<< d[0]<< ","
				<< d[1] << "\n"; 
			
			ofs << oss.str();
		}
	}
}

InternalParameters
preprocess(
	const std::vector<ImageWithInfo>& imgs, 
	const MIA& grid, 
	double pxl2metric,
	std::size_t I
)
{
	std::vector<std::vector<P2D>> data(I);
	for(auto& d: data) d.reserve(grid.size()*imgs.size());
		
	for(const auto& [img, fnumber] : imgs)
	{
		PRINT_INFO("=== Computing radii in image f/" << fnumber);
		
		std::vector<MicroImage> microimages;
		compute_radii(img, grid, microimages, I);

		GUI(
			PRINT_DEBUG("Displaying histograms of radii repartitions");
			hist_radii(microimages);
			Viewer::update();
			
			std::getchar(); 
		);
		
		//for each microimage compute the pair (invfnumber, radius)
		for(const auto&mi: microimages)
		{
			data[mi.type].emplace_back(
					1.f/fnumber, 
					- pxl2metric * mi.radius //FIXME: add minus sign
			);
		}
	}
	
	for(auto& d: data) d.shrink_to_fit();
	export_radii(data);
	
	PRINT_INFO("=== Computing lines coefficients");
	std::vector<LineCoefficients> coefs;
	compute_coefs(data, coefs);
	
	PRINT_INFO("=== Computing internal parameters");
	InternalParameters params;
	{
		constexpr double magicratio = 0.9931;
		
		params.m 		= coefs[0].m ;
		params.scale	= pxl2metric;
		params.kappa 	= pxl2metric * ( (grid.edge_length()[0] + grid.edge_length()[1]) / 2. ); //(eq. 40) //TODO: update eqt n°
		params.kappa_approx = magicratio * params.kappa; //(eq. 51)

		params.c.resize(I);
		params.c_prime.resize(I);
		for (std::size_t i = 0; i < I; ++i) 
		{
			params.c[i] = coefs[i].c ;
			params.c_prime[i] = coefs[i].c + params.kappa / 2.; //(eq. 46)
		}
		
		for (std::size_t i = 0; i < I; ++i) 
			PRINT_DEBUG("r(1/4)|"<<i<<" = " << (params.m / 4. + params.c[i]) / pxl2metric);
		for (std::size_t i = 0; i < I; ++i) 
			PRINT_DEBUG("N|"<<i<<" = " << std::fabs(params.m) / ( params.kappa / 2.0 - std::fabs(params.c[i])));

		double N = 0.0;
		for (std::size_t i = 0; i < I; ++i) N += (std::fabs(params.m) / ( params.kappa / 2.0 - std::fabs(params.c[i])));
		params.N = N / double(I);
		
		params.I = I;
	}
	
	return params;
}
