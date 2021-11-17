#pragma once

#include <utility> //std::pair

#include "types.h"

#include "geometry/camera/plenoptic.h"
#include "geometry/rgba.h"
#include "geometry/pose.h"

#include "io/archive.h"

#include "depthmap.h"

class DepthMap;

//******************************************************************************
class PointCloud 
{	
private:
	P3DS 		features_;
	P2DS		pixels_;
	Colors	 	colors_; 

//******************************************************************************	
public:
	PointCloud(std::size_t n = 0.);
	PointCloud(const DepthMap& dm, const PlenopticCamera& model, const Image& image);
	
	PointCloud(const PointCloud& o);
	PointCloud(PointCloud&& o);
	
//accessors
	P3DS& features();
	const P3DS& features() const;
	
	P3D& feature(std::size_t i);
	const P3D& feature(std::size_t i) const;

	P2DS& pixels();
	const P2DS& pixels() const;

	P2D& pixel(std::size_t i);
	const P2D& pixel(std::size_t i) const;
	
	Colors& colors();
	const Colors& colors() const;
	
	RGBA& color(std::size_t i);
	const RGBA& color(std::size_t i) const;
	
	std::size_t size() const;
	std::size_t nbPoints() const;
	std::size_t capacity() const;

//modifiers	
	std::size_t shrink();
	std::size_t reserve(std::size_t sz);
	std::size_t resize(std::size_t sz);

//add
	void add(const P3D& data, const P2D& pix = P2D{-1.,-1.}, const RGBA& col = RGBA{255.,255.,255.,255.});

//swap	
	void swap(std::size_t i, std::size_t j);

//remove
	void remove(std::size_t i);
	
//const_iterator		
	P3DS::const_iterator begin() const;
	P3DS::const_iterator end() const;
	
//min/max	
	std::pair<double, double> minmax() const;
	double min() const;
	double max() const;
	
//transform
	void transform(const Pose& pose);
	
//distance
	enum DistanceType : std::uint8_t { Chamfer = 0, Hausdorff = 1, Euclidean = 2 };
	double distance(const PointCloud& reading, DistanceType dt = DistanceType::Chamfer, double threshold = 10. /* mm */) const;

protected:
	//******************************************************************************
	friend void save(v::OutputArchive& archive, const PointCloud& pc);
	friend void load(v::InputArchive& archive, PointCloud& pc);	
};

//******************************************************************************
//******************************************************************************
void save(v::OutputArchive& archive, const PointCloud& pc);
void load(v::InputArchive& archive, PointCloud& pc);

//******************************************************************************
//******************************************************************************
