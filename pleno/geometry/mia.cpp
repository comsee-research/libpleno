#include "mia.h"

#include "processing/tools/lens.h"
#include "processing/imgproc/trim.h"

//******************************************************************************
//******************************************************************************
MicroImage::MicroImage(
	std::size_t k_, std::size_t l_, const P2D& c, double r, int t
) :	k{k_}, l{l_}, center{c}, radius{r}, type{t}
{ 
}

MicroImage::MicroImage(
	std::size_t k_, std::size_t l_, const P2D& c, double r, int t, const Image& i
) :	k{k_}, l{l_}, center{c}, radius{r}, type{t}, mi{i.clone()} //deep copy
{ 
}

MicroImage::MicroImage(const MicroImage& o)
:	k{o.k}, l{o.l}, center{o.center}, radius{o.radius}, type{o.type}, mi{o.mi.clone()} //deep copy
{ 
}

MicroImage::MicroImage(MicroImage&& o)
:	k{o.k}, l{o.l}, center{std::move(o.center)}, radius{o.radius}, type{o.type}, mi{o.mi} //move ownership
{ 
}

double MicroImage::contrast() const 
{
	Image m = mi.clone();
	trim_binarize(m, radius - MIA::border());
	
	cv::Scalar mean, std;
	cv::meanStdDev(mi, mean, std, m); 
	
	const double contrast = std[0];
	return contrast;
}

//******************************************************************************
//******************************************************************************
MicroImagesArray::MicroImagesArray(const MIAConfig& config) 
{
	this->geometry() = Geometry(config.mesh().geometry());
	this->width() = config.mesh().width();
	this->height() = config.mesh().height();
	this->pitch() = config.mesh().pitch();
	this->pose() = config.mesh().pose();
}

double MicroImagesArray::border() 
{ 
	return MI_BORDER; 
}

double MicroImagesArray::radius() const 
{ 
	return (pitch()[0] + pitch()[1]) / 4.; 
}

double MicroImagesArray::diameter() const 
{ 
	return (pitch()[0] + pitch()[1]) / 2.; 
}

//k,l are expressed in MIA space
int MicroImagesArray::type(std::size_t I, std::size_t k, std::size_t l) const 
{ 
	if (geometry() == HexagonalRowsAligned) return lens_type(I, k ,l);
	else return lens_type(I, l, k);
} 

MicroImage MicroImagesArray::mi(const Image& scene, std::size_t k, std::size_t l, std::size_t I) const 
{
	const int W = std::ceil(diameter());
	const auto center = nodeInWorld(k,l);
	
	Image temp;
	cv::getRectSubPix(
		scene, cv::Size{W,W}, 
		cv::Point2d{center[0], center[1]}, temp
	);
	
	return MicroImage{
		k, l,
		nodeInWorld(k,l),
		radius(),
		type(I, k, l),
		temp
	};
}

MicroImage MicroImagesArray::mi(std::size_t k, std::size_t l, std::size_t I) const 
{
	return MicroImage{
		k, l,
		nodeInWorld(k,l),
		radius(),
		type(I, k, l)
	};
}	

void MicroImagesArray::extract(MicroImage& mi, const Image& from) const
{
	const int W = std::ceil(diameter());

	cv::getRectSubPix(
		from, cv::Size{W,W}, 
		cv::Point2d{mi.center[0], mi.center[1]}, mi.mi
	);
}

std::pair<std::size_t, std::size_t> MicroImagesArray::uv2kl(double u, double v) const 
{
    static constexpr double sin60 = std::sin(60.0 / 180.0 * M_PI);
    std::size_t k = 0u, l = 0u;
	
	const auto p00 = nodeInWorld(0,0);
	
	if (geometry() == Orthogonal)
	{
		k = static_cast<std::size_t>(std::floor(0.5 + (u - p00[0]) / diameter()));
		l = static_cast<std::size_t>(std::floor(0.5 + (v - p00[1]) / diameter()));
	}
	if (geometry() == HexagonalRowsAligned)
	{
		//find row
		l = static_cast<std::size_t>(std::floor(0.5 + (v - p00[1]) / (diameter() * sin60)));
		const auto p0l = nodeInWorld(0,l);
		k = static_cast<std::size_t>(std::floor(0.5 + (u - p0l[0]) / diameter()));
	}
	else if (geometry() == HexagonalColsAligned) 
	{
		//find col
		k = static_cast<std::size_t>(std::floor(0.5 + (u - p00[0]) / (diameter() * sin60)));
		const auto pk0 = nodeInWorld(k,0);
		l = static_cast<std::size_t>(std::floor(0.5 + (v - pk0[1]) / diameter()));
	}
	
	return {k , l};
}
