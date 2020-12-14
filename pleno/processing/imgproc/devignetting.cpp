#include "devignetting.h"

////////////////////////////////////////////////////////////////////////////////
/* 
	Images must be 8-bits format
 */ 
void devignetting(const Image& raw, const Image& white, Image& unvignetted)
{    
    unvignetted = raw.clone();
   
    for (int row = 0; row < unvignetted.rows; ++row)
    {
        for (int col = 0; col < unvignetted.cols; ++col)
        {            
            auto & w = white.at<cv::Vec3b>(row, col);              
            auto & u = unvignetted.at<cv::Vec3b>(row, col);
            u[0] = static_cast<uchar>( std::min(255., u[0] * 255. / (w[0] + 1e-3)) );
            u[1] = static_cast<uchar>( std::min(255., u[1] * 255. / (w[1] + 1e-3)) );
            u[2] = static_cast<uchar>( std::min(255., u[2] * 255. / (w[2] + 1e-3)) );
        }
    }
}

/* 
	Images must be 16-bits format
 */ 
void devignetting_u16(const Image& raw, const Image& white, Image& unvignetted)
{    
    unvignetted = raw.clone();
   
    for (int row = 0; row < unvignetted.rows; ++row)
    {
        for (int col = 0; col < unvignetted.cols; ++col)
        {            
            auto & w = white.at<cv::Vec3w>(row, col);              
            auto & u = unvignetted.at<cv::Vec3w>(row, col);
            u[0] = static_cast<ushort>( std::min(65535., u[0] * 65535. / (w[0] + 1e-3)) );
            u[1] = static_cast<ushort>( std::min(65535., u[1] * 65535. / (w[1] + 1e-3)) );
            u[2] = static_cast<ushort>( std::min(65535., u[2] * 65535. / (w[2] + 1e-3)) );
        }
    }
}

