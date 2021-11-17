#include "pointcloud.h"

#include <omp.h>
#include <algorithm> // std::swap_iter, std::minmax_element

#include "io/printer.h" //PRINT_ERR, PRINT_DEBUG
#include "processing/algorithms/neighbour_search.h" //NNS

//******************************************************************************
//******************************************************************************
PointCloud::PointCloud(std::size_t n) 
{ 
	if (n != reserve(n)) 
	{
		PRINT_ERR("Cannot reserve enought space for " << n << " points."); 
	} 
}

PointCloud::PointCloud(const PointCloud& o) 
: features_{o.features_}, pixels_{o.pixels_}, colors_{o.colors_} 
{ 
	PRINT_DEBUG("Copied pointcloud."); 
}

PointCloud::PointCloud(PointCloud&& o) 
: features_{std::move(o.features_)}, pixels_{std::move(o.pixels_)}, colors_{std::move(o.colors_)} 
{ 
	PRINT_DEBUG("Moved pointcloud."); 
}

//******************************************************************************
//******************************************************************************
PointCloud::PointCloud(const DepthMap& dm, const PlenopticCamera& model, const Image& image)
{
	DEBUG_ASSERT((image.type() == CV_8UC3 or image.type() == CV_8UC1), "Conversion need image format U8C3/1");
	
	const DepthMap mdm = dm.to_metric(model);
	
	const auto& sensor 	= model.sensor();
	const auto& mia 	= model.mia();
	
	const int W 	= static_cast<int>(sensor.width());
	const int H 	= static_cast<int>(sensor.height());
	const double R 	= mia.radius();// + 1;
	
	reserve(static_cast<std::size_t>(W * H));

	//for each micro-image
	for (std::size_t k= 0; k < mia.width(); ++k)
	{
		for (std::size_t l = 0; l < mia.height(); ++l)
		{	
			if (mdm.is_coarse_map() and mdm.depth(k, l) == DepthInfo::NO_DEPTH) continue;
			
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
					//get depth
					const double depth = mdm.is_refined_map() ? mdm.depth(u,v) : mdm.depth(k,l);
					if (depth == DepthInfo::NO_DEPTH) continue;
									
					//get center pixel to raytrace from
					const P2D pixel = {u+0.5, v+0.5};
					if ((pixel - c).norm() > R) continue; //out of distance
					
					const P2D kl = model.mi2ml(k, l); //get ml indexes
					
					//raytrace
					Ray3D ray; //in CAMERA frame
					if (model.raytrace(pixel, kl[0], kl[1], ray))
					{
						//get depth plane
						PlaneCoefficients plane; plane << 0., 0., 1., -depth;
						
						//get position
						const P3D point = line_plane_intersection(plane, ray);
						
						//get color
						const RGBA color = [&]() -> RGBA {
							if (image.type() == CV_8UC3)
							{
								const auto bgr = image.at<cv::Vec3b>(v,u); //(row,col) access
								return RGBA{
									static_cast<double>(bgr[2]), 
									static_cast<double>(bgr[1]), 
									static_cast<double>(bgr[0]), 
									255.
								}; 
							}
							else if (image.type() == CV_8UC1)
							{
								const double g = static_cast<double>(image.at<uchar>(v,u)); //(row,col) access
								return RGBA{g, g, g, 255.}; 
							}
							return RGBA{};							
						}();
						
						//add to pointcloud
						add(point, pixel, color);					
					}
				}
			}
		}
	}
	
	shrink();
}

//******************************************************************************
//******************************************************************************	
//accessors
P3DS& PointCloud::features() { return features_; }
const P3DS& PointCloud::features() const { return features_; }

P3D& PointCloud::feature(std::size_t i) { assert(i < features().size()); return features()[i]; }
const P3D& PointCloud::feature(std::size_t i) const { assert(i < features().size()); return features()[i]; }

P2DS& PointCloud::pixels() { return pixels_; }
const P2DS& PointCloud::pixels() const { return pixels_; }

P2D& PointCloud::pixel(std::size_t i) { assert(i < pixels().size()); return pixels()[i]; }
const P2D& PointCloud::pixel(std::size_t i) const { assert(i < pixels().size()); return pixels()[i]; }

Colors& PointCloud::colors() { return colors_; }
const Colors& PointCloud::colors() const { return colors_; }

RGBA& PointCloud::color(std::size_t i) { assert(i < colors().size()); return colors()[i]; }
const RGBA& PointCloud::color(std::size_t i) const { assert(i < colors().size()); return colors()[i]; }

std::size_t PointCloud::size() const { return features().size(); }
std::size_t PointCloud::nbPoints() const { return size(); }
std::size_t PointCloud::capacity() const { return features().capacity(); }

//modifiers	
std::size_t PointCloud::shrink() { features().shrink_to_fit(); pixels().shrink_to_fit(); colors().shrink_to_fit(); return size(); }
std::size_t PointCloud::reserve(std::size_t sz) { features().reserve(sz); pixels().reserve(sz); colors().reserve(sz); return capacity(); }
std::size_t PointCloud::resize(std::size_t sz) { features().resize(sz); pixels().resize(sz); colors().resize(sz); return size(); }

//add
void PointCloud::add(const P3D& data, const P2D& pix, const RGBA& col) 
{
	features().emplace_back(data);
	pixels().emplace_back(pix);
	colors().emplace_back(col);
} 

//swap	
void PointCloud::swap(std::size_t i, std::size_t j) 
{
	assert(i < size()); assert(j < size());
	
	std::iter_swap(features().begin()+i, 	features().begin()+j);
	std::iter_swap(pixels().begin()+i, 		pixels().begin()+j);
	std::iter_swap(colors().begin()+i, 		colors().begin()+j);
}	

//remove
void PointCloud::remove(std::size_t i) 
{
	assert(i < size());
	
	std::iter_swap(features().begin()+i, 	features().end()-1);
	std::iter_swap(pixels().begin()+i, 		pixels().end()-1);
	std::iter_swap(colors().begin()+i, 		colors().end()-1);
	
	features().pop_back();
	pixels().pop_back();
	colors().pop_back();		
}

//const_iterator		
P3DS::const_iterator PointCloud::begin() const { return features_.cbegin(); }
P3DS::const_iterator PointCloud::end() const { return features_.cend(); }		

//min/max
std::pair<double, double> PointCloud::minmax() const
{
	const auto [min, max] = std::minmax_element(
		std::begin(*this), std::end(*this), 
		[](const P3D& p, const P3D& q) -> bool { return p.z() < q.z(); }
	);
	return {min->z(), max->z()};
}

double PointCloud::min() const { return minmax().first; }
double PointCloud::max() const { return minmax().second; }

//transform
void PointCloud::transform(const Pose& pose) 
{
	//for each point apply transformation
	for(P3D& p : features())
	{
		const P3D temp = to_coordinate_system_of(pose, p);
		p = temp;		
	}
}

//distance
double PointCloud::distance(const PointCloud& reading, DistanceType dt, double threshold/* mm */) const 
{
	const int sz = static_cast<int>(size());
	double dist = 0.;
	double N = 0.;
	auto acc = [](const P3D& p) -> const P3D& { return p; };
	
	if (dt == DistanceType::Chamfer)
	{
	#pragma omp parallel for reduction(+:dist) reduction(+:N)		
		for(int i = 0 ; i < sz; ++i)
		{	
			const P3D& p = feature(i);
			const P3D q = NNS::find(reading, p, acc);
			const double d = (p - q).squaredNorm();
	
			if (d < threshold) { dist += d; ++N; } 			
		}
		
		dist /= (N + 1e-12);	
	}
	else if (dt == DistanceType::Hausdorff)
	{	
	#pragma omp parallel for 
		for(int i = 0 ; i < sz; ++i)
		{	
			const P3D& p = feature(i);
			const P3D q = NNS::find(reading, p, acc);
			const double d = (p - q).squaredNorm();	
			
			#pragma omp critical
			if (d < threshold and d > dist) dist = d;		
		}	
	}
	else if (dt == DistanceType::Euclidean)
	{
	#pragma omp parallel for reduction(+:dist) reduction(+:N)		
		for(int i = 0 ; i < sz; ++i)
		{	
			const P3D& p = feature(i);
			const P3D q = NNS::find(reading, p, acc);
			const double d = (p - q).squaredNorm();
	
			if (d < threshold) { dist += d; ++N; } 			
		}
		
		dist /= (N + 1e-12);
		dist = std::sqrt(dist);
	}
	
	return dist;			
}

//******************************************************************************
//******************************************************************************
void save(v::OutputArchive& archive, const PointCloud& pc)
{
	archive
		("features", pc.features_)
		("pixels", pc.pixels_)
		("colors", pc.colors_);
}

void load(v::InputArchive& archive, PointCloud& pc)
{
	archive
		("features", pc.features_)
		("pixels", pc.pixels_)
		("colors", pc.colors_);
}

//******************************************************************************
//******************************************************************************
