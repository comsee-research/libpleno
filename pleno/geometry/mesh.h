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
	using Point = PnD<Dim>;
	using Node = Point;
	using Pose_t = Pose_<Dim>;

	struct iterator
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		const GridMesh_<Dim>& gm;
		std::size_t current;
		
		iterator(const GridMesh_<Dim>& gridmesh, std::size_t index) 
			: gm{gridmesh}, current{index} {}

		bool operator!=(const iterator& g) const;
		void operator++();
		Node operator*() const;
	};
private:
    Pose_t pose_; // the pose of the grid (from node 0)
    P2D pitch_; // the mean distance between two nodes
    
    std::size_t width_, height_; // the size of the grid (number of nodes)
    Geometry geometry_;
    	
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    GridMesh_( const P2D& e = P2D::Zero(),
               std::size_t w = 0,
               std::size_t h = 0,
               Geometry g = Geometry::Orthogonal);

    ~GridMesh_();

    const Pose_t& pose() const;
          Pose_t& pose();

    const P2D& pitch() const;
          P2D& pitch();

    std::size_t width() const;
    std::size_t& width();

    std::size_t height() const;
    std::size_t& height();

    Geometry geometry() const;
    Geometry& geometry();

    std::size_t nodeNbr() const;
    std::size_t size() const;

    // returns 3D coordinates of a desired node
    Node node(std::size_t col, std::size_t row) const;
    Node node(std::size_t i) const;

    Node nodeInWorld(std::size_t col, std::size_t row) const;
    Node nodeInWorld(std::size_t i) const;

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
    o << "Pitch: [" << g.pitch()[0] << ", " << g.pitch()[1] << "]\n";
    o << "Pose:\n" << g.pose();

    return o;
}

using GridMesh = GridMesh_<3ul>;
using GridMesh3D = GridMesh_<3ul>;
using GridMesh2D = GridMesh_<2ul>;
