#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "types.h"

LineCoefficients line_parameters_from_slope_and_one_point(double s, const P2D& p);
LineCoefficients line_parameters_from_angle_and_one_point(double a, const P2D& p);

/**
 * @Brief enum to determine the position of a 2d point from a 2d line
 */
enum PointLinePosition : std::int8_t
{
	DEFAULT 		  = -1,
    POINT_IS_ON_LINE  = 0,
    POINT_IS_AT_LEFT  = 1,
    POINT_IS_AT_RIGHT = 2,
    POINT_IS_AT_ABOVE = 3,
    POINT_IS_AT_BELOW = 4
};

/**
 *  @Brief return if a 2d point if placed at the left or the right of a 2d line
 *  @Return -1 default
 *  @Return 0 if the point is on the line
 *  @Return 1 if the point is at the left the line
 *  @Return 2 if the point is at the right the line
 *  if the line is horizontal:
 *  @Return 3 if the point is above the line
 *  @Return 4 if the point is below the line
 */
PointLinePosition point_line_position(const P2D& p, const LineCoefficients& l);


enum MicroImageType : std::uint8_t
{
    NONE = 0,
    FULL = 1,
    BORDER = 2,
    CORNER = 3
};

namespace std {
	std::string to_string(MicroImageType);
}

struct MicroImageModel
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
    MicroImageModel(MicroImageType t = NONE, double width = 0., double height = 0.);

    ~MicroImageModel();

    MicroImageType type;
    double w_, h_;
    P2D center;
    
    //TODO: remplacer les slopes du model par des direction (un peu comme un ray<T>)
    std::array<double, 2> lines_slope; //this model is a vertical line
    std::array<double, 2> lines_angles; //this model is a vertical line
    std::array<double, 2> colors;

    LineCoefficients parameters_of_lines(size_t i) const;
	
	double& width();
	double width() const;
	void width(double w);
	
	double& height();
	double height() const;
	void height(double h);
	
    void resize(double sw, double sh);
    Image generate() const;

};

using MIModel = MicroImageModel;

