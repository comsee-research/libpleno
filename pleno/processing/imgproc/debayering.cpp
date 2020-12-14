#include "debayering.h"

////////////////////////////////////////////////////////////////////////////////
void debayering(const Image& raw, Image& debayered)
{    
    debayered = raw.clone();
    cv::cvtColor(debayered, debayered, cv::COLOR_BayerGR2BGR);
}

