#include "mesh.h"

#include "processing/tools/matrix.h"

std::ostream& operator<<(std::ostream& o, const Geometry& geometry)
{
    switch(geometry)
    {
    	case Orthogonal: o << "Orthogonal"; break;
    	case Hexagonal:  o << "Hexagonal"; break;
    	default: break;    
    }
    return o;
}

std::ostream& operator<<(std::ostream& o, const Orientation& orientation)
{
    switch(orientation)
    {
    	case Horizontal: o << "Horizontal"; break;
    	case Vertical:  o << "Vertical"; break;
    	default: break;    
    }
    return o;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
template<std::size_t Dim>
GridMesh_<Dim>::GridMesh_(const P2D& e,
                       size_t w,
                       size_t h,
                       Geometry g,
                       Orientation o)
: _edge_length(e), _width(w), _height(h), _geometry(g), _orientation(o)
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
Orientation GridMesh_<Dim>::orientation() const
{
    return _orientation;
}
template<std::size_t Dim>
Orientation& GridMesh_<Dim>::orientation()
{
    return _orientation;
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
    constexpr double sin60 = std::sin(60.0 / 180.0 * M_PI);
   
    using Node = typename GridMesh_<Dim>::Node;
    
    if (col >= this->_width)
        std::cerr << "Error: GridMesh::node: wrong col index (" << col << ")"
                  << std::endl;
    if (row >= this->_height)
        std::cerr << "Error: GridMesh::node: wrong row index (" << row << ")"
                  << std::endl;
	

    Node p = Node::Zero();
    
    if (col >= this->_width or row >= this->_height)
    {
    	PRINT_ERR("GridMesh::node: wrong indexes");    
    }
    else 
    {
    	p[0] = this->_edge_length[0] 
    		* (double(col) + ((this->_geometry == Orthogonal) ? row : (this->_orientation == Horizontal and row%2 == 0) ? 0.5 : 0.0 )) 
    		* (this->_orientation == Vertical ? sin60 : 1.0);
    
   	 	p[1] = this->_edge_length[1] 
    		* (double(row) + ((this->_geometry == Orthogonal) ? col : (this->_orientation == Vertical and col%2 == 0) ? 0.5 : 0.0 )) 
    		* (this->_orientation == Horizontal ? sin60 : 1.0);
	}
	
#if 0
    if (this->_geometry == Orthogonal)
    {
        p[0] = this->_edge_length[0] * ( double(col) + double(row) );
        p[1] = this->_edge_length[1] * ( double(row) + double(col) );

        return p;
    }
    if (this->_geometry == Hexagonal)
    {
        if (this->_orientation == Horizontal)
        {
            double val = 0.0;
            if (row%2 == 0)
                val = 0.5;

            p[0] = (double(col) + val) * this->_edge_length[0];
            p[1] = double(row) * this->_edge_length[1] * sin60;

            return p;

        }
        if (this->_orientation == Vertical)
        {
            double val = 0.0;
            if (col%2 == 0)
                val = 0.5;

            p[0] = double(col) * this->_edge_length[0] * sin60;
            p[1] = (double(row) + val) * this->_edge_length[1];

            return p;
        }
        else
        {
            std::cerr << "Error: GridMesh::node: "
                      << "wrong orientation value (" << this->_orientation << ")."
                      << std::endl;
        }
    }
    else
    {
        std::cerr << "Error: GridMesh::node: "
                  << "wrong geometry value (" << this->_geometry << ")."
                  << std::endl;
    }
#endif
    return p;
}
template<std::size_t Dim>
typename GridMesh_<Dim>::Node 
GridMesh_<Dim>::node(size_t i) const
{
    if (i >= this->nodeNbr())
        std::cerr << "Error: GridMesh::node: "
                  << "wrong node value (" << i << ")."
                  << std::endl;

    const std::array<size_t, 2> coords = index_to_colRow(this->_width, i);
    return node(coords[0], coords[1]);
}

// returns 2D coordinates of a desired node in world coordinate
template<std::size_t Dim>
typename GridMesh_<Dim>::Node 
GridMesh_<Dim>::nodeInWorld(size_t col, size_t row) const
{
    if (col >= this->_width)
        std::cerr << "Error: GridMesh::nodeInWorld: wrong col index (" << col << ")"
                  << std::endl;
    if (row >= this->_height)
        std::cerr << "Error: GridMesh::nodeInWorld: wrong row index (" << row << ")"
                  << std::endl;

    //initalize the grid at the position of the first node
    return from_coordinate_system_of(this->_pose, node(col, row));
}
template<std::size_t Dim>
typename GridMesh_<Dim>::Node 
GridMesh_<Dim>::nodeInWorld(size_t i) const
{
    if (i >= this->nodeNbr())
        std::cerr << "Error: GridMesh::nodeInWorld: "
                  << "wrong node value (" << i << ")."
                  << std::endl;

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
