#include "generate.h"

#include "unused.h"

void generate_image(
	Image& raw,
	const CameraModel_t& model,
	const Plate& scene,
	bool blur
)
{
	const auto& sensor = model.sensor();
	const auto& mia = model.mia();
	
	const int W 	= static_cast<int>(sensor.width());
	const int H 	= static_cast<int>(sensor.height());
	const double R 	= mia.radius() + 1;
	
	raw = Image{H, W, CV_8UC3, cv::Scalar(0, 0, 0)};

#pragma omp parallel for	
	//for each micro-image
	for (int k_ = 0; k_ < static_cast<int>(mia.width()); ++k_) //NB: int index for omp loop
	{
		std::size_t k = static_cast<std::size_t>(k_);
		for (std::size_t l = 0; l < mia.height(); ++l)
		{	
			const P2D c = mia.nodeInWorld(k,l);
			
			const int umin = std::max(static_cast<int>(c[0] - R), 0);
			const int umax = std::min(static_cast<int>(c[0] + R), W);
			const int vmin = std::max(static_cast<int>(c[1] - R), 0);
			const int vmax = std::min(static_cast<int>(c[1] + R), H);
			
			double rho = 0.;
			int n = 0;
			
			Image mask = Image{vmax - vmin, umax - umin, CV_8UC3, cv::Scalar(0, 0, 0)};
			
			//for each pixel
			for (int u = umin; u < umax; ++u) //col
			{
				for (int v = vmin; v < vmax; ++v) //row
				{
					const P2D pixel = {u-0.5, v-0.5};
					if ((pixel - c).norm() > R) continue; //out of distance
					
					mask.at<cv::Vec3b>(v-vmin, u-umin) = cv::Vec3b{
						255, 255, 255
					};	
					
					const P2D kl =  model.mi2ml(k, l);
					
					//raytrace
					Ray3D ray; //in CAMERA frame
					if (model.raytrace(pixel, kl[0], kl[1], ray))
					{						
						//check intersection with plate
						P3D P = line_plane_intersection(scene.planeInWorld(), ray); //in CAMERA frame
						if (scene.is_inside(P)) 
						{
							const P2D pp = to_coordinate_system_of(scene.pose(), P).head<2>(); //in PLATE frame			
							const RGBA color = scene.get_color(pp.x(), pp.y());
							
							raw.at<cv::Vec3b>(v, u) = cv::Vec3b{
								static_cast<uchar>(color.b), 
								static_cast<uchar>(color.g), 
								static_cast<uchar>(color.r)
							};		
							
							if (blur)
							{
								//get blur level
								double radius = 0.;
								model.project(P, kl[0], kl[1], radius);	
								
								rho += std::fabs(radius);
								++n;
							}						
						}	
					}								
				}
			}
			//if there is some pixel to blur
			if (blur and n != 0)
			{
				rho /= n; //mean radius
				
				//reblur micro-image				
				const double sigma = model.params().kappa * rho;
				cv::Rect window{umin, vmin, umax - umin, vmax - vmin};
				
				//APPROX_GAUSSIAN_BLUR
				Image temp = raw(window).clone();				
				cv::GaussianBlur(temp, temp, cv::Size{0,0}, sigma, sigma);
				temp.copyTo(raw(window), mask);
			}		
		}
	}	
}

void generate_image(
	Image& raw,
	const CameraModel_t& model,
	const Plate& scene,
	std::size_t nrays,
	bool vignetting 
)
{
	const auto& sensor = model.sensor();
	const auto& mia = model.mia();
	
	const int W 	= static_cast<int>(sensor.width());
	const int H 	= static_cast<int>(sensor.height());
	const double R 	= mia.radius();// + 1;
	
	Image rawd = Image{H, W, CV_64FC3, cv::Scalar(0., 0., 0.)};

#pragma omp parallel for	
	//for each micro-image
	for (int k_ = 0; k_ < static_cast<int>(mia.width()); ++k_) //NB: int index for omp loop
	{
		std::size_t k = static_cast<std::size_t>(k_);
		for (std::size_t l = 0; l < mia.height(); ++l)
		{	
			const auto c = mia.nodeInWorld(k,l);
			
			const int umin = std::max(static_cast<int>(c[0] - R), 0);
			const int umax = std::min(static_cast<int>(c[0] + R), W);
			const int vmin = std::max(static_cast<int>(c[1] - R), 0);
			const int vmax = std::min(static_cast<int>(c[1] + R), H);
			
			//for each pixel
			for (int u = umin; u < umax; ++u) //col
			{
				for (int v = vmin; v < vmax; ++v) //row
				{
					//CENTER SUBPIXEL
					const P2D pixel = {u+0.5, v+0.5};
					if ((pixel - c).norm() > R) continue; //out of distance
					
					const P2D kl = model.mi2ml(k, l);
					
					RGBA color{0., 0., 0., 0.};
					
					//raytrace
					Rays3D rays; //in CAMERA frame
					if (model.raytrace(pixel, kl[0], kl[1], nrays, rays))
					{						
						for (const auto& ray : rays)
						{
							//check intersection with plate
							P3D P = line_plane_intersection(scene.planeInWorld(), ray); //in CAMERA frame
							if (scene.is_inside(P)) 
							{
								const P2D pp = to_coordinate_system_of(scene.pose(), P).head<2>(); //in PLATE frame						
								const RGBA lcolor = scene.get_color(pp.x(), pp.y());
								
								const double alpha = ray.color().a; //NOTE: alpha should be between [0, 1]
								
								color.r += lcolor.r * alpha; //between [0, 255]
								color.g += lcolor.g * alpha; //between [0, 255]
								color.b += lcolor.b * alpha; //between [0, 255]
								color.a += alpha; //between [0, 1]
							}	
						}
						
						const std::size_t n = vignetting ? (nrays+1) : color.a;
						
						rawd.at<cv::Vec3d>(v, u) = cv::Vec3d{
							static_cast<double>(color.b / n), 
							static_cast<double>(color.g / n), 
							static_cast<double>(color.r / n)
						};	
					}								
				}
			}	
		}
	}
	
	rawd.convertTo(raw, CV_8UC3, 1.);	
}
