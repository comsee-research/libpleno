#include "viewer_2d.h"

/**
 * @Brief libv_wrapper is a wrapper taking an opencv image to convert it into libv format
 * TODO: becarefull if the image if BGR or RGB
 */
void libv_wrapper(v::ViewerContext& v, const Image& input, int col, int row)
{
    if (input.type() == 0)
        v.add_image(col, row, v::Wrapper<v::core::ImageU8cp> (const_cast<cv::Mat&>(input)) );
    else if (input.type() == 16)
        v.add_image(col, row, v::Wrapper<v::core::ImageRGBU8cp> (const_cast<cv::Mat&>(input)) );
}

/**
 * @Brief display an opencv image
 * TODO: becarefull if the image if BGR or RGB
 */
void viewer_2d(v::ViewerContext& v, const Image& input, int col, int row)
{
    libv_wrapper(v, input, col, row);
    v.update();
}

void viewer_2d(v::ViewerContext& v, const std::vector<cv::Point2f>& ps)
{
    for (auto& p : ps)
        v.add_point(p.x, p.y);
    v.update();
}

/**
 * @Brief display some P2DS
 */
void viewer_2d(v::ViewerContext& v, const P2D& p)
{
    v.add_point(p[0], p[1]);
    v.update();
}

/**
 * @Brief display some P2DS
 */
void viewer_2d(v::ViewerContext& v, const P2DS& ps)
{
    for (auto& p : ps)
        v.add_point(p[0], p[1]);
    v.update();
}

void viewer_2d(v::ViewerContext& v, const Ray2D& r, double d)
{
    auto point = r(d);
    v.add_line(r.origin()[0], r.origin()[1], point[0], point[1])
     .update();
}

void viewer_2d(v::ViewerContext& v, const Disk& d)
{
    v.add_point(d.center[0], d.center[1])
     .add_circle(d.center[0], d.center[1], d.radius)
     .update();
}

/**
 * @Brief viewer_2d draw a 2d grid
 */
void viewer_2d(v::ViewerContext& v, const GridMesh2D& gm)
{
    for (auto node : gm)
        v.add_point(node[0], node[1]);
    v.update();
}
