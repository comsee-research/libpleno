#include "morph.h"

////////////////////////////////////////////////////////////////////////////////
void erode(const Image& input, Image& output, int crossSize)
{
    cv::Mat element = getStructuringElement(cv::MORPH_CROSS,
                                            cv::Size(2 * crossSize + 1, 2 * crossSize + 1),
                                            cv::Point(crossSize, crossSize) );

    cv::erode(input, output, element);
}

void dilate(const Image& input, Image& output, int crossSize)
{
    cv::Mat element = getStructuringElement(cv::MORPH_CROSS,
                                            cv::Size(2 * crossSize + 1, 2 * crossSize + 1),
                                            cv::Point(crossSize, crossSize) );

    cv::dilate(input, output, element);
}

////////////////////////////////////////////////////////////////////////////////
std::vector<std::vector<cv::Point>> detect_shapes(Image& ioSource)
{
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(ioSource, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

    cv::RNG rng (12345);
    cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
    cv::drawContours(ioSource, contours, -1, color, 0, 8, hierarchy, INT_MAX);

    return contours;
}

void fit_polygons(std::vector<std::vector<cv::Point>>& contours, std::vector<cv::Point2f>& center)
{
    std::vector<std::vector<cv::Point>> contours_polygon(contours.size());
    std::vector<cv::Rect> boundRect(contours.size());
    std::vector<float> radius(contours.size());

    for (size_t c = 0; c < contours.size(); ++c )
    {
        cv::approxPolyDP(cv::Mat(contours[c]), contours_polygon[c], 3, true);
        boundRect[c] = boundingRect(cv::Mat(contours_polygon[c]));
        cv::minEnclosingCircle(cv::Mat(contours_polygon[c]), center[c], radius[c]);
    }
}

