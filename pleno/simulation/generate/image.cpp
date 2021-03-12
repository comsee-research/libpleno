#include "generate.h"

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
	const double R 	= mia.radius();
	
	raw = Image{H, W, CV_8UC3, cv::Scalar{0, 0, 0}};

#pragma omp parallel for	
	//for each micro-image
	for (std::size_t k = 0; k < mia.width(); ++k)
	{
		for (std::size_t l = 0; l < mia.height(); ++l)
		{	
			const auto c = mia.nodeInWorld(k,l);
			
			const int umin = std::max(static_cast<int>(c[0] - R), 0);
			const int umax = std::min(static_cast<int>(c[0] + R), W);
			const int vmin = std::max(static_cast<int>(c[1] - R), 0);
			const int vmax = std::min(static_cast<int>(c[1] + R), H);
			
			double rho = 0.;
			int n = 0;
			
			Image mask = Image{vmax-vmin, umax - umin, CV_8UC3, cv::Scalar{0, 0, 0}};
			
			//for each pixel
			for (int u = umin; u < umax; ++u)
			{
				for (int v = vmin; v < vmax; ++v)
				{
					const P2D pixel = {u, v};
					if ((pixel - c).norm() > R) continue; //out of distance
					
					mask.at<cv::Vec3b>(v-vmin, u-umin) = cv::Vec3b{
						1, 1, 1
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
							const P2D pp = to_coordinate_system_of(scene.pose(), P).head(2); //in PLATE frame
												
							RGBA color = scene.get_color(pp.x(), pp.y());
							
							raw.at<cv::Vec3b>(v, u) = cv::Vec3b{
								static_cast<uchar>(color.b), 
								static_cast<uchar>(color.g), 
								static_cast<uchar>(color.r)
							};		
							
							//get blur level
							double radius = 0.;
							model.project(P, kl[0], kl[1], radius);	
							
							rho += std::fabs(radius);
							++n;						
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
				
			#if 0 //APPROX_GAUSSIAN_BLUR
				cv::GaussianBlur(raw(window), raw(window), cv::Size{0,0}, sigma, sigma);
			#else
				Image temp = raw(window).mul(mask);
				cv::GaussianBlur(mask, mask, cv::Size{0,0}, sigma, sigma); //blur mask
				cv::GaussianBlur(temp, temp, cv::Size{0,0}, sigma, sigma); //blur masked image	
				cv::divide(temp, mask, raw(window)); //divide the blurred masked image by the blurred mask 
			#endif
			
			}		
		}
	}	
}
