#pragma once

#include "types.h"
#include "geometry/pose.h"

// The geometry of the grid: Orthogonal, Hexagonal Rows Aligned, Hexagonal Cols Aligned
enum Geometry: std::int8_t {Orthogonal = 0, HexagonalRowsAligned = 1, HexagonalColsAligned = 2};
std::ostream& operator<<(std::ostream& o, const Geometry& geometry);

////////////////////////////////////////////////////////////////////////////////////////////////////
template<std::size_t Dim>
class GridMesh_ {
public:
	using Point = typename Eigen::Matrix<double, Dim, 1>;
	using Node = Point;
	using Pose_t = Pose_<Dim>;

	struct iterator
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		const GridMesh_<Dim>& gm;
		size_t current;
		
		iterator(const GridMesh_<Dim>& gridmesh, size_t index) 
			: gm{gridmesh}, current{index} {}

		bool operator!=(const iterator& g) const;
		void operator++();
		Node operator*() const;
	};
private:
    Pose_t pose_; // the pose of the grid (from node 0)
    P2D edge_length_; // the mean distance between two nodes
    
    size_t width_, height_; // the size of the grid (number of nodes)
    Geometry geometry_;
    	
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    GridMesh_( const P2D& e = P2D::Zero(),
               size_t w = 0,
               size_t h = 0,
               Geometry g = Geometry::Orthogonal);

    ~GridMesh_();

    const Pose_t& pose() const;
          Pose_t& pose();

    const P2D& edge_length() const;
          P2D& edge_length();

    size_t width() const;
    size_t& width();

    size_t height() const;
    size_t& height();

    Geometry geometry() const;
    Geometry& geometry();

    size_t nodeNbr() const;
    size_t size() const;

    // returns 3D coordinates of a desired node
    Node node(size_t col, size_t row) const;
    Node node(size_t i) const;

    Node nodeInWorld(size_t col, size_t row) const;
    Node nodeInWorld(size_t i) const;

};

template<std::size_t Dim>
typename GridMesh_<Dim>::iterator 
begin(const GridMesh_<Dim>& gm)
{
    return typename GridMesh_<Dim>::iterator(gm, 0);
};
template<std::size_t Dim>
typename GridMesh_<Dim>::iterator 
end(const GridMesh_<Dim>& gm)
{
    return typename GridMesh_<Dim>::iterator(gm, gm.nodeNbr());
};

template<std::size_t Dim>
inline std::ostream& 
operator<<(std::ostream& o, const GridMesh_<Dim>& g)
{
    o << "Geometry: " << g.geometry() << "\n";
    o << "Dimensions: [" << g.width() << ", " << g.height() << "]\n";
    o << "Edge length: [" << g.edge_length()[0] << ", " << g.edge_length()[1] << "]\n";
    o << "Pose:\n" << g.pose();

    return o;
}

using GridMesh = GridMesh_<3ul>;
using GridMesh3D = GridMesh_<3ul>;
using GridMesh2D = GridMesh_<2ul>;
