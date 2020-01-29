#include "microimage.h"

namespace std {

	std::string to_string(MicroImageType type)
	{
        switch(type)
        {
            case NONE   : return "NONE";
            case FULL   : return "FULL";
            case BORDER : return "BORDER";
            case CORNER : return "CORNER";
            default		: return "";
        }
	}	
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// SOME TOOLS USED WITH MODEL
////////////////////////////////////////////////////////////////////////////////////////////////////
LineCoefficients line_parameters_from_slope_and_one_point(double s, const P2D& p)
{
    return LineCoefficients{ s, std::fma(-s, p.x(), p.y()) };
}


LineCoefficients line_parameters_from_angle_and_one_point(double a, const P2D& p)
{
    const double slope = std::tan(a);
    return LineCoefficients{ slope, std::fma(-slope, p.x(), p.y()) };
}

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
PointLinePosition point_line_position(const P2D& p, const LineCoefficients& l)
{
    const double val = std::fma(l.m, p.x(), l.c); //ax+b

    if( p.y() == val )      return POINT_IS_ON_LINE;

    //the line if hozizontal
    if( l.m == 0 )
    {
        if( l.c < p.y() ) return POINT_IS_AT_ABOVE;
        else                return POINT_IS_AT_BELOW;
    }
    else if( l.m < 0.0 )
    {
        if( p.y() < val )   return POINT_IS_AT_RIGHT;
        else                return POINT_IS_AT_LEFT;
    }
    else //FIXME: check here if returns are correct (it's odd that's the sames as above)
    {
        if( p.y() < val )   return POINT_IS_AT_RIGHT;
        else                return POINT_IS_AT_LEFT;
    }

    return DEFAULT;
}


////////////////////////////////////////////////////////////////////////////////////////////////////
// MODEL
////////////////////////////////////////////////////////////////////////////////////////////////////

MicroImageModel::MicroImageModel(MicroImageType t, double width, double height)
    : type(t), w_(width), h_(height)
{
    lines_angles = {0.0, M_PI / 2.0};
    colors = {255.0, 0.0};
    center = {w_ / 2.0, h_ / 2.0};
};

MicroImageModel::~MicroImageModel(){};

LineCoefficients MicroImageModel::parameters_of_lines(size_t i) const
{
    // return line_parameters_from_slope_and_one_point(this->lines_slope.at(i), this->center);
    return line_parameters_from_angle_and_one_point(lines_angles.at(i), center);
}

double& MicroImageModel::width() {return w_;}
double MicroImageModel::width() const {return w_;}
void MicroImageModel::width(double w) {w_ = w;}

double& MicroImageModel::height() {return h_;}
double MicroImageModel::height() const {return h_;}
void MicroImageModel::height(double h) {h_ = h;}

void MicroImageModel::resize(double sw, double sh)
{
    w_ *= sw; h_ *= sh;
    center[0] *= sw;
    center[1] *= sh;
};

Image MicroImageModel::generate() const
{
   assert(this->type == CORNER);
   
   Image image = cv::Mat(h_, w_, CV_64FC1, (this->colors[0] + this->colors[1]) / 2.0);

   // this is simple but incomplete: does not use angles!!!!!!!
   // works only with int coordinates
   const P2D floored_ctr{std::floor(center[0]), std::floor(center[1])};
   const P2D ceiled_ctr{std::ceil(center[0]), std::ceil(center[1])};

   for (int row = 0; row < image.rows; ++row)
   {
       for (int col = 0; col < image.cols; ++col)
       {
           if ((col < floored_ctr[0] and row < floored_ctr[1]) or (col >= ceiled_ctr[0] and row >= ceiled_ctr[1]))
           {
               image.at<double>(row, col) = this->colors[0];
           }
           else if ((col >= ceiled_ctr[0] and row < floored_ctr[1]) or (col < floored_ctr[0] and row >= ceiled_ctr[1]))
           {
               image.at<double>(row, col) = this->colors[1];
           }
       }
   }
   
   return image;
}
