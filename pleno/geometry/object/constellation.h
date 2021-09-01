#pragma once

#include "types.h"
#include "io/cfg/scene.h"

struct PointsConstellation 
{
	using Point 	= P3D;
	using Points 	= std::vector<Point>;
	
	Points constellation;
	
	PointsConstellation(const ConstellationConfig& config = {})
		: constellation{config.points()} 
	{}
	
	std::size_t size() const { return constellation.size(); }
	
	P3D& 	get(std::size_t i) { assert(i < size()); return constellation[i]; }
	P3D		get(std::size_t i) const { assert(i < size()); return constellation[i]; }
	
	void 	add(const P3D& p) { constellation.push_back(p); }
	void 	add(double x, double y, double z) { constellation.emplace_back(x,y,z); }	

//iterator	
	Points::iterator begin() { return constellation.begin(); }
	Points::iterator end() { return constellation.end(); }
	
	Points::const_iterator begin() const { return constellation.cbegin(); }
	Points::const_iterator end() const { return constellation.cend(); }		
};
