#pragma once

#include "types.h" //Image, PnD, RGBA

#include "geometry/pose.h" //Pose
#include "geometry/plane.h" //PlaneCoefficients

#include "io/cfg/scene.h" //PlateConfig

struct Plate {
private:
	Pose pose_;
	
	double width_; //width in mm
	double height_; //height in mm
	
	double scale_; //mm per pixel 
	
	Image texture_; //CV_32FC3, getRectSubPix not working with 64F...

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	Plate(const PlateConfig& config) : 	
		pose_{config.pose()}, 
		width_{config.width()}, height_{config.height()}, scale_{config.scale()}
	{
		Image txtr = cv::imread(config.texture(), cv::IMREAD_COLOR);
		PRINT_DEBUG("Load texture from: " << config.texture());
		PRINT_DEBUG("Texture info: (" << txtr.cols << ", " << txtr.rows << ", " << txtr.channels() << ")");
		
		//if no dimensions are given, take image as dimension
		if (height() < 0. or width() < 0.)
		{
			height() 	= txtr.rows;
			width() 	= txtr.cols;
		}
		
		Image txture;
		txtr.convertTo(txture, CV_32FC3, 1./255.);
		
		//rescale image with scale
		int wpixel = static_cast<int>(width() / scale());
		int hpixel = static_cast<int>(height() / scale());
	
		cv::resize(txture, texture_, cv::Size{wpixel, hpixel}, 0, 0, cv::INTER_LINEAR);
		
		PRINT_DEBUG("Resize texture info: (" << texture().cols << ", " << texture().rows << ", " << texture().channels() << ")");
	}

//accessors	
	Pose& pose() { return pose_; }
	const Pose& pose() const { return pose_; }
	
	double& width() { return width_; }
	double width() const { return width_; }
	
	double& height() { return height_; }
	double height() const { return height_; }
	
	double& scale() { return scale_; }
	double scale() const { return scale_; }
	
	Image& texture() { return texture_; }
	const Image& texture() const { return texture_; }

//helper
	void rescale(double scl = 1.)
	{
		//rescale image with scale
		int wpixel = static_cast<int>(scl * width() / scale());
		int hpixel = static_cast<int>(scl * height() / scale());
	
		cv::resize(texture(), texture(), cv::Size{wpixel, hpixel}, 0, 0, cv::INTER_LINEAR);
	}

//functions
	bool is_inside(const P3D& pw) const 
	{
		const P3D p = to_coordinate_system_of(pose(), pw); //PLATE FRAME
		return (/*.z() == 0. and */ p.x() > 0. and p.x() < width() and p.y() > 0. and p.y() < height());
	}
	
	RGBA get_color(double x, double y) const //x, y in mm
	{
		const double u = x / scale();
		const double v = y / scale(); 
		cv::Mat patch;
    	cv::getRectSubPix(texture(), cv::Size{1,1}, cv::Point2d{u,v}, patch);
    
    	const cv::Vec3f pixel = patch.at<cv::Vec3f>(0,0); //BGR?
    	
    	return RGBA{
    		static_cast<double>(pixel[2]) * 255., 
    		static_cast<double>(pixel[1]) * 255., 
    		static_cast<double>(pixel[0]) * 255., 
    		255.
    	};
	}
	
	// the plane coefficients
	PlaneCoefficients plane() const
	{
		return plane_from_3_points(P3D{0.0, 0.0, 0.0},
		                           P3D{width(), 0.0, 0.0},
		                           P3D{width(), height(), 0.0}
		);
	};

	// the plane coefficients in WORLD coordinate system
	PlaneCoefficients planeInWorld() const
	{
		return plane_from_3_points(from_coordinate_system_of(pose(), P3D{0., 0., 0.}),
		                           from_coordinate_system_of(pose(), P3D{width(), 0., 0.}),
		                           from_coordinate_system_of(pose(), P3D{0., height(), 0.})
		);
	};
};

using Plates = AlignedVector<Plate>;
