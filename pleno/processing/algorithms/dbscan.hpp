#include "dbscan.h"

#include <algorithm>

//--ctor
template<typename Point_t>
DBSCAN<Point_t>::DBSCAN(double eps, std::size_t min) 
	: epsilon{eps}, minPts{min} {}
	
template<typename Point_t>
DBSCAN<Point_t>::DBSCAN(const DP& pts, double eps, std::size_t min) 
	: epsilon{eps}, minPts{min}, p_pts{&pts} {}
//--dtor
template<typename Point_t>
DBSCAN<Point_t>::~DBSCAN(){}
	
template<typename Point_t>
void DBSCAN<Point_t>::clusterize()	
{
	clusterize(*p_pts);
}	
	
template<typename Point_t>
//void DBSCAN<Point_t>::clusterize(const DP& pts, const std::vector<Index>& idx, bool doKnn)
void DBSCAN<Point_t>::clusterize(const DP& pts)
{
	nbPts = pts.size();
	
	//clear clusters
	clusters.clear();
	
	const std::size_t k = 50; //FIXME: should be a parameter

#if 0
	//get all neighbors with distance through knn
	if(doKnn) retreiveNeighbors(pts, k);
	
	//set mask for subset point
	setMask(idx);
#else	
	//get all neighbors with distance through knn
	retreiveNeighbors(pts, std::min(k, nbPts));
#endif
	//clear label
	std::vector<Label> labels(nbPts, UNCLASSED);
	
	//for(Index id : idx)
	for(Index id = 0; id < Index(nbPts); ++id)
	{
		if(labels[id] != UNCLASSED) continue; //already processed point
		
		if(not isCore(id)) //check density
		{
			labels[id] = NOISE; //mark as noise
			continue;
		}
		
		const std::size_t c = clusters.size();
		labels[id] = Label(c); //labelize cluster id
		
		std::vector<Index> ids; //current cluster
		ids.push_back(id);
		
		std::stack<Index> neighbors; //contains reachable neighbors
		appendNeighbors(neighbors, id);
		
		//For each neighbor
		while(not neighbors.empty())
		{
			const Index q = neighbors.top();
			neighbors.pop();
			
			if(labels[q] == NOISE) //change noise to border point 
			{
				labels[q] = c;
				ids.push_back(q);
			}
			
			if(labels[q] != UNCLASSED) continue; //already processed point
			
			labels[q] = c; //labelize cluster id
			ids.push_back(q);
			
			if(isCore(q)) //append neighbors of neighbors
				appendNeighbors(neighbors, q);
		}
		
		//save cluster
		clusters.push_back(ids);
	}
}

//--methods
template<typename Point_t>
void DBSCAN<Point_t>::setMask(const std::vector<Index>& idx)
{
	isSelected = std::vector<bool>(nbPts, false);	
	for(Index id : idx) isSelected[id] = true;
}

template<typename Point_t>
void DBSCAN<Point_t>::retreiveNeighbors(const DP& pts, std::size_t k)
{
	std::vector<PointWithIndex> pts_with_index;
	pts_with_index.reserve(nbPts);
	
	Index id = 0;
	for(const auto& p : pts)
		pts_with_index.emplace_back(PointWithIndex{p, id++});

	indices = NNS::IndexMatrix::Constant(k, nbPts, NNS::InvalidIndex);
	dist = NNS::DistanceMatrix::Zero(k, nbPts);

	NNS::knn(pts_with_index, indices, dist, k, 
		[](auto& pwi){return pwi.point;}, 
		[](auto& pwi){ return pwi.id; }
	);
}
	
template<typename Point_t>
void DBSCAN<Point_t>::appendNeighbors(std::stack<Index>& nn, Index q)
{
	const std::size_t nbknn = indices.rows();
	
	for(std::size_t i = 0; i < nbknn; ++i)
	{
		if(not isConsistent(i,q)) continue; //not append if not reachable
		
		const Index qnn = indices(i, q);
		if(qnn == NNS::InvalidIndex) break;
		nn.push(qnn);
	}
}
	
template<typename Point_t>
inline bool DBSCAN<Point_t>::isCore(Index id) 
{
	const std::size_t nbknn = indices.rows();

	std::size_t count = 0;
	for(std::size_t i = 0; i < nbknn and count < minPts; ++i)
		if(isConsistent(i, id)) ++count;

	return count >= minPts;
}
	
template<typename Point_t>
inline bool DBSCAN<Point_t>::isConsistent(std::size_t i /*i-th neighbor*/, std::size_t q)
{
	return (indices(i,q) != NNS::InvalidIndex) 
		//and isSelected[indices(i,q)] 
		and (dist(i, q) <= epsilon);
}
