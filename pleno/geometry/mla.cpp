#include "mla.h"

#include "processing/tools/lens.h"

MicroLensesArray::MicroLensesArray(const MLAConfig& config) 
{
	this->geometry() = Geometry(config.mesh().geometry());
	this->width() = config.mesh().width();
	this->height() = config.mesh().height();
	this->pitch() = config.mesh().pitch();
	this->pose() = config.mesh().pose();
	
	init(config.focal_lengths().size());
	
	for (std::size_t i = 0; i < I(); ++i) 
		this->focals_[i].f = config.focal_lengths()[i];
}

void MicroLensesArray::init(std::size_t I) 
{ 
	this->I_ = I; this->focals_.resize(I); 
}

std::size_t MicroLensesArray::I() const 
{ 
	return I_; 
}

double MicroLensesArray::f(std::size_t i) const 
{ 
	assert(i<I()); 
	return focals_[i].f; 
}
double& MicroLensesArray::f(std::size_t i) 
{ 
	assert(i<I()); 
	return focals_[i].f; 
}

FocalLength MicroLensesArray::f(std::size_t k, std::size_t l) const 
{
	assert(I() != 0u and k < width() and l < height());
	const int t = type(k, l);
	return focals_[t];
}

FocalLength& MicroLensesArray::f(std::size_t k, std::size_t l) 
{
	assert(I() != 0u and k < width() and l < height());
	const int t = type(k, l);
	return focals_[t];
}

double MicroLensesArray::radius() const 
{ 
	return (pitch()[0] + pitch()[1]) / 4.; 
}
double MicroLensesArray::diameter() const 
{ 
	return (pitch()[0] + pitch()[1]) / 2.; 
}

int MicroLensesArray::type(std::size_t k, std::size_t l) const 
{ 
	if (geometry() == HexagonalRowsAligned) return lens_type(I(), k ,l);
	else return lens_type(I(), l, k);
} 	
