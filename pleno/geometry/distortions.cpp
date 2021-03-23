#include "distortions.h"

#include "io/printer.h"

double Distortions::radius(const P2D& p) const
{
    //TODO: check performance over std::sqrt
    return std::hypot(p[0], p[1]); 
}

std::vector<double> Distortions::radius(const P2DS& ps) const
{
    std::vector<double> radii;
    radii.reserve(ps.size());
    
    for(const P2D& p :  ps)
    	radii.emplace_back(radius(p));
    	
    return radii;
}

/*
    @Brief apply radial distortions to a point
    p has to be expressed in the coordinate reliative coordinate system of the principal point
*/
void Distortions::apply_radial(P2D& p) const
{
    const double r = radius(p);

    // ∆x_rad = x_I · ( A_0 r^2 + A_1 r^4 + A_2 r^6 + ...)
    p *= ( this->radial()[0] * r * r
         + this->radial()[1] * r * r * r * r
         + this->radial()[2] * r * r * r * r * r * r);
}

/*
    @Brief apply tangential distortions to a point
    p has to be expressed in the coordinate reliative coordinate system of the principal point
*/
void Distortions::apply_tangential(P2D& p) const
{
    const double r = radius(p);

    // ∆x_tan = B_0 . (r^2 + 2 . x_I^2) + 2 . B_1 . x_I . y_I . (1 + B_2 * r^2 + B_3 . r^4 + ...)
    // ∆y_tan = B_1 . (r^2 + 2 . y_I^2) + 2 . B_0 . x_I . y_I . (1 + B_2 * r^2 + B_3 . r^4 + ...)
    p = { this->tangential()[0] * (r * r + 2.0 * p[0] * p[0]) + 2.0 * this->tangential()[1] * p[0] * p[1],
          this->tangential()[1] * (r * r + 2.0 * p[1] * p[1]) + 2.0 * this->tangential()[0] * p[0] * p[1]};
}

#if defined(USE_DEPTH_DISTORTION) && USE_DEPTH_DISTORTION
/*
    @Brief apply depth distortions to a point
    p has to be expressed in the coordinate reliative coordinate system of the principal point
*/
void Distortions::apply_depth(P3D& p) const
{
    const double r = radius(p.head<2>());

    // ∆z =  (1 + D_0 . z) . (D_1 . r^2 + D_2 . r^4 + ...)
    p[2] = (1 + depth[0] * p[3]) * (depth[1] * r * r + depth[2] * r * r * r * r);
}

void Distortions::apply_depth(P3DS& ps) const
{
    for(P3D& p : ps)
    	apply_depth(p);
}
#endif

void Distortions::apply_radial(P2DS& ps) const
{
    for(P2D& p : ps)
    	apply_radial(p);
}

void Distortions::apply_tangential(P2DS& ps) const
{
    for(P2D& p : ps)
    	apply_tangential(p);
}



/*
    @Brief apply distortions to a point
    p has to be expressed in the coordinate reliative coordinate system of the principal point
*/
void Distortions::apply(P3D& p) const
{
    // les points dans le repère du point_principal
    P2D p_rad = p.head<2>();
    P2D p_tan = p.head<2>();

    //on calcule les delta rad et tan
    apply_radial(p_rad);
    apply_tangential(p_tan);
    
#if defined(USE_DEPTH_DISTORTION) && USE_DEPTH_DISTORTION
    P3D p_depth = p;
    apply_depth(p_depth);
    
    p += p_depth;
#endif

    //on applique le shift sur le pixel
    //x_Id = x_I + ∆x_rad + ∆x_tan
    p += P3D{p_rad[0], p_rad[1], 0.0}
       + P3D{p_tan[0], p_tan[1], 0.0};
}

/*
    @Brief apply distortions to a point
    p has to be expressed in the coordinate reliative coordinate system of the principal point
*/
void Distortions::apply(P3DS& ps) const
{
    for(P3D& p : ps)
    	apply(p);
}

/*
 * @Brief apply_distortions apply distortions to a point expressed in CAMERA
**/
void Distortions::unapply(P3D& /*p*/) const
{
    PRINT_WARN("Distortions::unapply not implemented yet");
}

std::ostream& operator<<(std::ostream& o, const Distortions& dist)
{
    const P3D& r = dist.radial();
    const P2D& t = dist.tangential();
    
#if defined(USE_DEPTH_DISTORTION) && USE_DEPTH_DISTORTION
    const auto& d = dist.depth();    
	o << "depth = {" << d[0] << ", " << d[1] << "};\n";
#endif
	
    o << "radial = {" << r[0] << ", " << r[1] << ", " << r[2] << "};\n";
    o << "tangential = {" << t[0] << ", " << t[1] << "};\n";

    return o;
}
