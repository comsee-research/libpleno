#include "caracterize.h"

#include <numeric>
#include <array>

#include "graphic/gui.h"
#include "io/printer.h"

#include "processing/improcess.h"
#include "processing/tools/rotation.h"
/*
 * @orientation_histogram compute an histogram based on orientation and magnitude image gradients
 */
std::vector<double>
hist_orientation(
	const Image& orientation, 
	const Image& magnitude,
    const double stepsize,
	const Image& mask    
)
{
    const int num_bins = int(360.0 / stepsize);

	std::vector<double> histogram(num_bins, 0.);

	const int width 	= orientation.cols;
	const int height 	= orientation.rows;
	
	const double *o, *w;
	const uchar *m;
	for(int y = 0 ; y < height ; ++y) //for each row
	{
		o = orientation.ptr<const double>(y);
		w = magnitude.ptr<const double>(y);
		m = mask.ptr<const uchar>(y);
		
		for(int x = 0 ; x < width ; ++x) //for each column
		{
			if(m[x] == 0) continue; //if off the mask skip it

			const int bin = int(o[x] / stepsize);
        	histogram.at(bin) += w[x];
		}
	}

#if 0 //For debug purpose only	
	GUI(
		for (size_t i = 0; i < histogram.size(); ++i)
        {
        	PRINT_DEBUG("magnitude at " << i*stepsize + stepsize/2 <<"° = " << histogram.at(i));
			Viewer::context().layer(Viewer::layer()).name("histogram")
                    .pen_width(2)
                    .pen_color(v::blue)
                    .add_line(i, 0, i, -histogram.at(i));
        }
		Viewer::update();
	);
#endif
	
	return histogram;
}

/*
 * @orientation_histogram compute an histogram based on orientation and magnitude image gradients
 */
std::vector<double>
hist_orientation_modulo_180(
	const Image& orientation, 
	const Image& magnitude,
    const double stepsize,
	const Image& mask    
)
{
    const int num_bins = int(180.0 / stepsize);

	std::vector<double> histogram(num_bins, 0.);

	const int width 	= orientation.cols;
	const int height 	= orientation.rows;
	
	const double *o, *w;
	const uchar *m;
	for(int y = 0 ; y < height ; ++y) //for each row
	{
		o = orientation.ptr<const double>(y);
		w = magnitude.ptr<const double>(y);
		m = mask.ptr<const uchar>(y);
		
		for(int x = 0 ; x < width ; ++x) //for each column
		{
			if(m[x] == 0) continue; //if off the mask skip it

			const int bin =  (int(o[x]) % 180) / int(stepsize);
        	histogram.at(bin) += w[x];
		}
	}

#if 0 //For debug purpose only	
	GUI(
		for (size_t i = 0; i < histogram.size(); ++i)
        {
        	PRINT_DEBUG("magnitude at " << i*stepsize + stepsize/2 <<"° = " << histogram.at(i));
			Viewer::context().layer(Viewer::layer()).name("histogram")
                    .pen_width(2)
                    .pen_color(v::blue)
                    .add_line(i, 0, i, -histogram.at(i));
        }
		Viewer::update();
	);
#endif
	
	return histogram;
}


std::vector<double>
compute_polar_histogram(
    const Image& img,
    const Image& mask,
    const double scale
)
{
    //Convert the image to DBL format
    Image dbl_input;
    img.convertTo(dbl_input, CV_64FC1);

    //Computing gradients the image
    Image x_grad, y_grad;
    x_gradient<double>(dbl_input, x_grad);
    y_gradient<double>(dbl_input, y_grad);

    //convert the image gradients from cartesian to polar coordinate system 
    cv::Mat grad_orientation, grad_magnitude;
    cv::cartToPolar(x_grad, y_grad, grad_magnitude, grad_orientation, true /*use_degree */);
	
	//DEBUG_VAR(grad_magnitude);
	//DEBUG_VAR(grad_orientation);
	
    //compute the histogram
    return hist_orientation(grad_orientation, grad_magnitude, scale, mask);
    //return hist_orientation_modulo_180(grad_orientation, grad_magnitude, scale, mask);
}

/**
 * @Brief 	Filter peaks according to the max angle distance separating them. 
 *			Pack together too closed peaks, and mean the magnitude.
 **/
void filter_peaks(Peaks& peaks, const double maxdist)
{
	Peaks filtered_peaks; //peaks = (angle, magnitude)
	filtered_peaks.reserve(peaks.size());
	
    double nbr_of_angles = 1.;
    double cumulated_value = peaks.at(0).magnitude;

    for (size_t i = 1; i < peaks.size(); ++i)
    {
        //if peaks are close enough (according to max_distance_between_2_peaks)
        if (peaks.at(i).angle - peaks.at(i - 1).angle < maxdist)
        {
            cumulated_value += peaks.at(i).magnitude;
            ++nbr_of_angles;
        }
        else
        {
            // push_back the average of cumulated angles magnitude
            filtered_peaks.emplace_back( 
            	Peak{peaks.at(i - 1).angle, cumulated_value / nbr_of_angles} 
            );
            
            // reset values
            cumulated_value = peaks.at(i).magnitude;
            nbr_of_angles = 1.;
        }
    }
    //FIXME: not really convinced by this
    // push_back the average of cumulated angles
    // at the end of the loop, we take into account the last elements of peaks;
    filtered_peaks.emplace_back( 
    	Peak{ peaks.at(peaks.size() - 1).angle, cumulated_value / nbr_of_angles}
    );

    peaks.swap(filtered_peaks);
    peaks.shrink_to_fit();
}

MicroImageModel 
compute_model(const Image& mi, const Image& mask)
{
	assert(mi.channels() == 1 and mask.channels() == 1);
	constexpr double scale = 20.0;
	
	std::vector<double> histogram = compute_polar_histogram(mi, mask, scale);
	
	MicroImageModel model;
	
    // getting highest peaks of the histogram
    constexpr double max_distance_between_2_peaks = 20.;
    constexpr double threshold_on_magnitude_per_angle = 255.; //at least two pixel at the same gradient
    
    Peaks peaks; //vector containing peaks (angle, magnitude)
    
    // get peaks
    for (size_t i = 0; i < histogram.size(); ++i)
    {
        if (histogram.at(i) > threshold_on_magnitude_per_angle)
        {
            double angle_to_store = i * scale + 90.; //reoriente gradient as it's perpendicular to the edge
            angle_to_store = rad_to_deg(restrict_to_circle(deg_to_rad(angle_to_store)));

            peaks.emplace_back(
            	Peak{angle_to_store, histogram.at(i)}
            );
        }
    }
    
    if ( peaks.size() > 1u ) filter_peaks(peaks, max_distance_between_2_peaks);
    	
	auto satisfy_orthogonality = [&max_distance_between_2_peaks](const double diff) {
		return (diff >= 90 - max_distance_between_2_peaks and diff <  90 + max_distance_between_2_peaks)
			or (diff >= 270 - max_distance_between_2_peaks and diff <  270 + max_distance_between_2_peaks);
	};
        	    
	switch(peaks.size())
	{
		case 0u: //if no peak then it's a FULL
			model.type = FULL;
    		model.colors[0] = model.colors[1] = static_cast<double>(mi.at<uchar>(mi.rows / 2, mi.cols / 2));
    	break;
		
		case 1u: //if there is only one peak then it's a BORDER
			model.type = BORDER;
        	model.lines_angles[0] = deg_to_rad(peaks[0].angle);
		break;
		
		case 2u: //if there is 2 peaks left, we check if there are opposed
		{	if(satisfy_orthogonality(peaks[1].angle - peaks[0].angle))
			{
				model.type = CORNER;
			    model.lines_angles[0] = deg_to_rad(peaks[0].angle);
			    model.lines_angles[1] = deg_to_rad(peaks[1].angle);
			}
			else //FIXME: check if we are always a border in this configuration
			{
				//model.type = NONE;
				model.type = BORDER;
        		model.lines_angles[0] = deg_to_rad(peaks[0].angle);
			}
        }
		break;
		
		// if there is more than 3 peaks after filtering
       	// they represent more than 2 angles so ther is a CORNER
		default: 
			model.type = CORNER;
	        model.lines_angles[0] = deg_to_rad(peaks[0].angle);
	        model.lines_angles[1] = deg_to_rad(peaks[1].angle);
        break;
	}
	
	return model;
}

MicroImageType caracterize(const Image& mi, const Image& mask)
{
	return compute_model(mi, mask).type;
}
