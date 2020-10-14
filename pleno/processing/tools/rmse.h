#pragma once

#include "types.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
// https://en.wikipedia.org/wiki/Root-mean-square_deviation
struct RMSE
{
    double value = 0.0;
    int size = 0;
    
    void add(double d) { size+=1; value += d * d; }
    void add(const P2D& p) { value += p.squaredNorm(); size+=1; }
    void add(const P3D& p) { value += p.squaredNorm(); size+=1; }

    double get() const { 
    	return size == 0 ? 0. : std::sqrt(sum() / static_cast<double>(size));
    }
    double sum() const {
    	return value; // / 2;
    }
    
    RMSE& operator+=(const RMSE& rhs){
		  this->value += rhs.value;
		  this->size += rhs.size;
		  return *this;
	}
};
