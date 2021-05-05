#pragma once

#include <functional>
//#include <omp.h>

//TODO: move to tools.h
auto return_same = [](auto& a){ return a; };

using Farthest = std::greater<void>;
using Nearest = std::less<void>;

template <typename SearchType = Nearest> //, SearchType& f = SearchType() >
struct NeighbourSearch {
	using Index 			= int;
	using DistanceMatrix 	= Eigen::Matrix<double, Eigen::Dynamic /* k */, Eigen::Dynamic /* nbpts */>;
	using IndexMatrix 		= Eigen::Matrix<Index, Eigen::Dynamic /* k */, Eigen::Dynamic /* nbpts */>;
	
	static constexpr Index InvalidIndex = -1; 
	static constexpr SearchType f = SearchType{};

	
	template<typename PointsWithIndex, typename AccessorPoint, typename AccessorId>
	static void knn(
		const PointsWithIndex &data,
		IndexMatrix &indices, 
		DistanceMatrix &dist, 
		std::size_t k = 30, 
		AccessorPoint acc = return_same, 
		AccessorId id = return_same 
	)
	{
		assert(dist.rows() == indices.rows() and dist.cols() == indices.cols());
		assert(k <= dist.rows()); 
	
		const std::size_t nbpts = dist.cols();
		
#pragma omp parallel for		
		for(std::size_t i = 0 ; i < nbpts ; ++i)
		{
			const auto &ref = acc(data[i]); 
			
			auto comp = [&ref, &acc](const auto& p1, const auto& p2)
			{
				const double d1 = (acc(p1) - ref).norm();// std::hypot(acc(p1)[0] - ref[0], acc(p1)[1] - ref[1]);
				const double d2 = (acc(p2) - ref).norm();// std::hypot(acc(p2)[0] - ref[0], acc(p2)[1] - ref[1]);
				return f(d1, d2);// ou f((t1 - point).norm(),(t2 - point).norm());
			};
			
			PointsWithIndex pts{std::begin(data), std::end(data)};
			
			//coarse sort
			std::nth_element(std::begin(pts), std::begin(pts) + k, std::end(pts), comp);
			//sort according to distance
			std::sort(std::begin(pts), std::begin(pts) + k, comp);
			
			//fill outputs
			for(std::size_t nn = 0; nn < k ; ++nn)			
			{
				indices(nn, i) 	= id(pts[nn]);
				dist(nn, i)		= std::hypot(acc(pts[nn])[0] - ref[0], acc(pts[nn])[1] - ref[1]);
			}		
		}
	}
	
	
	template<typename Points, typename Accessor, typename Point = decltype(Accessor{}.operator()(typename Points::value_type{}))>
	static Point find(const Points& pts, const Point& ref, Accessor acc = return_same)
	{
		Point neighbors;

		auto comp = [&ref, &acc](const auto& p1, const auto& p2)
		{
			const double d1 = (acc(p1) - ref).norm();// std::hypot(acc(p1)[0] - ref[0], acc(p1)[1] - ref[1]);
			const double d2 = (acc(p2) - ref).norm();// std::hypot(acc(p2)[0] - ref[0], acc(p2)[1] - ref[1]);
			return f(d1, d2);// ou f((t1 - point).norm(),(t2 - point).norm());
		};
		
		auto min_it = std::min_element(std::begin(pts), std::end(pts), comp);

		return Point{acc(*min_it)};
	};

};

using NNS = NeighbourSearch<Nearest>;
using FNS = NeighbourSearch<Farthest>;

