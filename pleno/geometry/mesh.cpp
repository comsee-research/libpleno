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
: _edge_length(e), _width(w), _height(h), _geometry(g)
{}

template<std::size_t Dim>
GridMesh_<Dim>::~GridMesh_()
{}

template<std::size_t Dim>
const typename GridMesh_<Dim>::Pose_t& GridMesh_<Dim>::pose() const
{
    return _pose;
}
template<std::size_t Dim>
typename GridMesh_<Dim>::Pose_t& GridMesh_<Dim>::pose()
{
    return _pose;
}
template<std::size_t Dim>
const P2D& GridMesh_<Dim>::edge_length() const
{
    return _edge_length;
}
template<std::size_t Dim>
P2D& GridMesh_<Dim>::edge_length()
{
    return _edge_length;
}
template<std::size_t Dim>
size_t GridMesh_<Dim>::width() const
{
    return _width;
}
template<std::size_t Dim>
size_t& GridMesh_<Dim>::width()
{
    return _width;
}
template<std::size_t Dim>
size_t GridMesh_<Dim>::height() const
{
    return _height;
}
template<std::size_t Dim>
size_t& GridMesh_<Dim>::height()
{
    return _height;
}
template<std::size_t Dim>
Geometry GridMesh_<Dim>::geometry() const
{
    return _geometry;
}
template<std::size_t Dim>
Geometry& GridMesh_<Dim>::geometry()
{
    return _geometry;
}

template<std::size_t Dim>
size_t GridMesh_<Dim>::nodeNbr() const
{
    return this->_width * this->_height;
}
template<std::size_t Dim>
size_t GridMesh_<Dim>::size() const
{
    return this->_width * this->_height;
}

// returns 2D coordinates of a desired node
template<std::size_t Dim>
typename GridMesh_<Dim>::Node 
GridMesh_<Dim>::node(size_t col, size_t row) const
{        
    static constexpr double sin60 = std::sin(60.0 / 180.0 * M_PI);
   
	DEBUG_ASSERT(col < width() and row < height(), "GridMesh::node: wrong indexes");
	
    typename GridMesh_<Dim>::Node node;
    
    const double colOffset = ((this->_geometry == HexagonalRowsAligned and row%2 == 0) ? 0.5 : 0.0) ;
    const double rowOffset = ((this->_geometry == HexagonalColsAligned and col%2 == 0) ? 0.5 : 0.0) ;
    
	node[0] = this->_edge_length[0] * (double(col) + colOffset) * ((this->_geometry == HexagonalColsAligned) ? sin60 : 1.0);
 	node[1] = this->_edge_length[1] * (double(row) + rowOffset) * ((this->_geometry == HexagonalRowsAligned) ? sin60 : 1.0);

    return node;
}

template<std::size_t Dim>
typename GridMesh_<Dim>::Node 
GridMesh_<Dim>::node(size_t i) const
{
    DEBUG_ASSERT(i < this->nodeNbr(), "GridMesh::node: wrong node value (" << i << ").");

    const std::array<size_t, 2> coords = index_to_colRow(this->_width, i);
    return node(coords[0], coords[1]);
}

// returns 2D coordinates of a desired node in world coordinate
template<std::size_t Dim>
typename GridMesh_<Dim>::Node 
GridMesh_<Dim>::nodeInWorld(size_t col, size_t row) const
{
	DEBUG_ASSERT(col < width() and row < height(), "GridMesh::node: wrong indexes ("<< col <<", "<< row <<")");

    //initalize the grid at the position of the first node
    return from_coordinate_system_of(this->_pose, node(col, row));
}
template<std::size_t Dim>
typename GridMesh_<Dim>::Node 
GridMesh_<Dim>::nodeInWorld(size_t i) const
{
    DEBUG_ASSERT(i < this->nodeNbr(), "GridMesh::node: wrong node value (" << i << ").");

    const std::array<size_t, 2> coords = index_to_colRow(this->_width, i);
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
