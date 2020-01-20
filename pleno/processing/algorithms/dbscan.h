#pragma once

#include <vector>
#include <stack>

#include "types.h"
#include "neighbour_search.h"


//DBSCAN clustering algorithm
template<typename Point_t>
struct DBSCAN 
{
//--types
	using DP = std::vector<Point_t,  Eigen::aligned_allocator<Point_t>>;
	using Index = typename NNS::Index;
	using Label = int;
	
private:
	struct PointWithIndex {
		Point_t point;
		Index 	id;
	};
			
private:
	static constexpr Label NOISE = -2;
	static constexpr Label UNCLASSED = -1;
	
public:	
//--attributes
	double epsilon;
	std::size_t minPts;
	
	std::vector<std::vector<Index>> clusters;
	
private:
	const DP * p_pts;
	std::size_t nbPts ;
	NNS::IndexMatrix indices;
	NNS::DistanceMatrix dist;	
	
	std::vector<bool> isSelected;
	
public:
//--ctor
	DBSCAN(const DP& pts, double eps = 0.2, std::size_t min = 5);
	DBSCAN(double eps = 0.2, std::size_t min = 5);
//--dtor
	~DBSCAN();
	
	//void clusterize(const DP& pts, /*subset of points */const std::vector<Index>& idx, bool doKnn=true);
	void clusterize(const DP& pts);
	void clusterize();
	
private:
	void retreiveNeighbors(const DP& pts, std::size_t k);
	void appendNeighbors(std::stack<Index>& nn, Index q);
	
	void setMask(const std::vector<Index>& idx);
	
	inline bool isCore(Index id) ;
	inline bool isConsistent(std::size_t i /*i-th neighbor*/, std::size_t q);

};

template struct DBSCAN<P2D>;
template struct DBSCAN<P3D>;

#include "dbscan.hpp"
