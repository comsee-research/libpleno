#include "mesh.h"

#include "processing/tools/matrix.h"

std::ostream& operator<<(std::ostream& o, const Geometry& geometry)
{
    switch(geometry)
    {
    	case Orthogonal: o << "Orthogonal"; break;
    	case HexagonalRowsAligned: o << "Hexagonal Rows-aligned"; break;
    	case HexagonalColsAligned: o << "Hexagonal Columns-aligned"; break;
    	default: o << "Unknown geometry"; break; 
    }
    return o;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<std::size_t Dim>
GridMesh_<Dim>::GridMesh_(
	const P2D& e, size_t w, size_t h, Geometry g)
: edge_length_(e), width_(w), height_(h), geometry_(g)
{}

template<std::size_t Dim>
GridMesh_<Dim>::~GridMesh_()
{}

template<std::size_t Dim>
const typename GridMesh_<Dim>::Pose_t& GridMesh_<Dim>::pose() const
{
    return pose_;
}
template<std::size_t Dim>
typename GridMesh_<Dim>::Pose_t& GridMesh_<Dim>::pose()
{
    return pose_;
}
template<std::size_t Dim>
const P2D& GridMesh_<Dim>::edge_length() const
{
    return edge_length_;
}
template<std::size_t Dim>
P2D& GridMesh_<Dim>::edge_length()
{
    return edge_length_;
}
template<std::size_t Dim>
size_t GridMesh_<Dim>::width() const
{
    return width_;
}
template<std::size_t Dim>
size_t& GridMesh_<Dim>::width()
{
    return width_;
}
template<std::size_t Dim>
size_t GridMesh_<Dim>::height() const
{
    return height_;
}
template<std::size_t Dim>
size_t& GridMesh_<Dim>::height()
{
    return height_;
}
template<std::size_t Dim>
Geometry GridMesh_<Dim>::geometry() const
{
    return geometry_;
}
template<std::size_t Dim>
Geometry& GridMesh_<Dim>::geometry()
{
    return geometry_;
}

template<std::size_t Dim>
size_t GridMesh_<Dim>::nodeNbr() const
{
    return this->width_ * this->height_;
}
template<std::size_t Dim>
size_t GridMesh_<Dim>::size() const
{
    return this->width_ * this->height_;
}

// returns 2D coordinates of a desired node
template<std::size_t Dim>
typename GridMesh_<Dim>::Node 
GridMesh_<Dim>::node(size_t col, size_t row) const
{        
    static constexpr double sin60 = std::sin(60.0 / 180.0 * M_PI);
   
	DEBUG_ASSERT(col < width() and row < height(), "GridMesh::node: wrong indexes ("<<col<<", "<<row<<")");
	
    typename GridMesh_<Dim>::Node node;
    node.setZero();
    
    const double colOffset = ((this->geometry_ == HexagonalRowsAligned and row%2 == 0) ? 0.5 : 0.0) ;
    const double rowOffset = ((this->geometry_ == HexagonalColsAligned and col%2 == 0) ? 0.5 : 0.0) ;
    
	node[0] = this->edge_length_[0] * (double(col) + colOffset) * ((this->geometry_ == HexagonalColsAligned) ? sin60 : 1.0);
 	node[1] = this->edge_length_[1] * (double(row) + rowOffset) * ((this->geometry_ == HexagonalRowsAligned) ? sin60 : 1.0);

    return node;
}

template<std::size_t Dim>
typename GridMesh_<Dim>::Node 
GridMesh_<Dim>::node(size_t i) const
{
    DEBUG_ASSERT(i < this->nodeNbr(), "GridMesh::node: wrong node value (" << i << ").");

    const std::array<size_t, 2> coords = index_to_colRow(this->width_, i);
    return node(coords[0], coords[1]);
}

// returns 2D coordinates of a desired node in world coordinate
template<std::size_t Dim>
typename GridMesh_<Dim>::Node 
GridMesh_<Dim>::nodeInWorld(size_t col, size_t row) const
{
	DEBUG_ASSERT(col < width() and row < height(), "GridMesh::node: wrong indexes ("<< col <<", "<< row <<")");

    //initalize the grid at the position of the first node
    return from_coordinate_system_of(this->pose_, node(col, row));
}
template<std::size_t Dim>
typename GridMesh_<Dim>::Node 
GridMesh_<Dim>::nodeInWorld(size_t i) const
{
    DEBUG_ASSERT(i < this->nodeNbr(), "GridMesh::node: wrong node value (" << i << ").");

    const std::array<size_t, 2> coords = index_to_colRow(this->width_, i);
    return nodeInWorld(coords[0], coords[1]);
}


template<std::size_t Dim>
bool 
GridMesh_<Dim>::iterator::operator!=(const GridMesh_<Dim>::iterator& g) const
{
    return g.current != current;
}

template<std::size_t Dim>
void 
GridMesh_<Dim>::iterator::operator++()
{
    ++current;
}
template<std::size_t Dim>
typename GridMesh_<Dim>::Node 
GridMesh_<Dim>::iterator::operator*() const
{
    return gm.nodeInWorld(current);
}

template class GridMesh_<3ul>;
template class GridMesh_<2ul>;
