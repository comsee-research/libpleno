#pragma once

#include <random>

#include "types.h"

inline P2D uniform_sample_disk(const P2D& origin = {0., 0.}, double scale = 1.)
{
	static thread_local std::random_device rd;
    static thread_local std::mt19937 mt(rd());

    std::uniform_real_distribution<double> dist(0., 1.);
    
    const double radius = scale * std::sqrt(dist(mt)); // dist(mt);
	const double theta = dist(mt) * 2. * M_PI;
	
	return P2D{origin.x() + radius * std::cos(theta), origin.y() + radius * std::sin(theta)};
}

inline P2D concentric_sample_disk(const P2D& origin = {0., 0.}, double scale = 1.)
{
	constexpr double pi_div_4 = M_PI / 4.;
	constexpr double pi_div_2 = M_PI / 2.;
	 
	static thread_local std::random_device rd;
    static thread_local std::mt19937 mt(rd());

    std::uniform_real_distribution<double> dist(0., 1.);
    
    //map to [-1;1]x[-1;1]
    const double x = 2. * dist(mt) - 1.;
    const double y = 2. * dist(mt) - 1.;
    
    //handle degeneracy at the origin
    if (x == 0. and y == 0.) return P2D{0., 0.};
    
    //apply concentric mapping
    double theta, radius;
    if (std::fabs(x) > std::fabs(y))
    {
    	radius = scale * x;
    	theta = pi_div_4 * (y / x);
    }
    else
    {
    	radius = scale * y;
    	theta =  pi_div_2 - pi_div_4 * (x / y);
    }
    
    return P2D{origin.x() + radius * std::cos(theta), origin.y() + radius * std::sin(theta)};
}


