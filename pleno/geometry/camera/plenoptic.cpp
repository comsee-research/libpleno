#include "plenoptic.h"

#include "sampler.h"

#include "processing/tools/rotation.h"

#include "io/printer.h"
#include "io/cfg/camera.h"


//******************************************************************************
//******************************************************************************
//******************************************************************************
//Accessors	
PlenopticCamera::Mode
PlenopticCamera::mode() const
{
    double f_ = main_lens().focal();
    double d_ = D();
    
    //If we consider micro-lenses focal lengths, use f<d as definition for mode
    if (focused())
    {
    	f_ = mla().f(0);
    	d_ = d();    
    }
    
    if (unfocused()) { return Unfocused; }  
    else if ( f_ < d_ ) { return Keplerian; }
	else /* if (f_ > d_) */ { return Galilean; }
}

const QuadraticFunction& PlenopticCamera::scaling() const { return scaling_; }
QuadraticFunction& PlenopticCamera::scaling() { return scaling_; }

const MicroLensesArray& PlenopticCamera::mla() const { return mla_; }
MicroLensesArray& PlenopticCamera::mla() { return mla_; }

const MicroImagesArray& PlenopticCamera::mia() const { return mia_; }
MicroImagesArray& PlenopticCamera::mia() { return mia_; }     

const InternalParameters& PlenopticCamera::params() const { return params_; }
InternalParameters& PlenopticCamera::params() { return params_; }   

double PlenopticCamera::distance_focus() const { return dist_focus_; } 
double& PlenopticCamera::distance_focus() { return dist_focus_; }   

const ThinLensCamera& PlenopticCamera::main_lens() const { return main_lens_; }
ThinLensCamera& PlenopticCamera::main_lens() { return main_lens_; }

const Distortions& PlenopticCamera::main_lens_distortions() const { return distortions_; }
Distortions& PlenopticCamera::main_lens_distortions() { return distortions_; }

const Distortions& PlenopticCamera::main_lens_invdistortions() const { return invdistortions_; }
Distortions& PlenopticCamera::main_lens_invdistortions() { return invdistortions_; }

double PlenopticCamera::focal() const { return main_lens().focal(); }
double& PlenopticCamera::focal() { return main_lens().focal(); }   

double PlenopticCamera::aperture() const { return main_lens().aperture(); }
double& PlenopticCamera::aperture() { return main_lens().aperture(); }   

double PlenopticCamera::mlaperture() const { return mla().diameter(); }

std::size_t PlenopticCamera::I() const { return mla().I(); }

double PlenopticCamera::d(std::size_t k, std::size_t l) const 
{ 
	return - sensor().pose().translation().z() - D(k,l); 
}
double PlenopticCamera::D(std::size_t k, std::size_t l) const 
{ 
	const P3D pkl = mla().nodeInWorld(k,l);
	return std::fabs(pkl.z()); 
}

double PlenopticCamera::focal_plane(std::size_t i, std::size_t k, std::size_t l) const 
{ 
	if (not focused()) return scaling()(v2obj(std::max(2., (focal() - D(k,l)) / d(k,l) + 1e-6) , k, l)); 
	else return scaling()(mla2obj((mla().f(i) * d(k,l)) / (d(k,l) - mla().f(i) + (focal() > D() ? -1. : 1. ) * 1e-24))); 
}

bool PlenopticCamera::focused() const { return I() > 0; }
bool PlenopticCamera::multifocus() const { return I() > 1; }
bool PlenopticCamera::unfocused() const { return (I() > 0) and (d() > mla().f(0)); }

//******************************************************************************
//******************************************************************************
//******************************************************************************
PlenopticCamera::PrincipalPoint 
PlenopticCamera::pp() const
{
	P3D PP = main_lens().pose().translation(); //CAMERA
	PP = to_coordinate_system_of(sensor().pose(), PP); // SENSOR
	
	P2D pp = sensor().metric2pxl(PP).head<2>(); //IMAGE XY
	xy2uv(pp); //IMAGE UV
		
	return pp;
}
    
PlenopticCamera::PrincipalPoint
PlenopticCamera::mlpp(std::size_t k, std::size_t l) const
{
	const P2D idx = ml2mi(k, l);
	
	const P2D ckl = mia().nodeInWorld(idx[0], idx[1]); //IMAGE
	const double ratio = d() / (D() + d());
	
	const PrincipalPoint mpp = this->pp();
	
	const P2D ppkl = ratio * (mpp - ckl) + ckl;
	
	return ppkl;
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
void PlenopticCamera::init(
	const Sensor& sensor, 
	const MicroImagesArray& mia, 
	const InternalParameters& params, 
	double F, double aperture, double h,
	PlenopticCamera::Mode mode 
)
{		
	mia_ 	= mia; //already calibrated
	params_ = params; //already computed
	
	//I
	const std::size_t I = params.I;
	
	//DISTANCE FOCUS
	dist_focus_ = h;
	//POSE
	pose_.translation() = Translation::Zero();
	pose_.rotation()	= Rotation::Identity();
	
	//MAIN LENS
	main_lens_.focal() 				= F;
	main_lens_.aperture() 			= aperture;
	main_lens_.pose().translation() = pose_.translation();
	main_lens_.pose().rotation()	= pose_.rotation();
		
	//DISTORTIONS
	distortions_.radial() 		<< 0., 0., 0.;
	distortions_.tangential() 	<< 0., 0.;
	distortions_.depth() 		<< 0., 0., 0.;
	
	invdistortions_.radial() 		<< 0., 0., 0.;
	invdistortions_.tangential() 	<< 0., 0.;
	invdistortions_.depth() 		<< 0., 0., 0.;
	
	//SCALING
	scaling_.a = 0.; //coef z²
	scaling_.b = 1.; //coef z
	scaling_.c = 0.; //coef 1
		
	/* 	
		Init of d and D is kinda tricky: 
			- In Keplerian configuration, F < D whatever the focus distance.
			- In Galilean configuration, normally we should have F > D, 
				- but when the main lens focus distance is at infinity
				- the main lens focuses on the TCP (i.e., v = 2)
				- or when h decrease, D increase,
				- so in most cases, we will still have F < D
	*/
	double d = 0., D = 0.;
	const double m = std::fabs(params_.m);
	const double H = (h / 2.) * (1. - std::sqrt(1. - 4. * (F / h))); DEBUG_VAR(H); // eq.(18)
	
	switch(mode)
	{
		case Unfocused:
			d = m * 2.; D = F;
		break;
		
		case Keplerian: 
		{
			d = (2. * m * H) / (F - 4. * m); // eq.(17)
			D = H + 2. * d; // eq.(17)
		}
		break;
		
		case Galilean:
		{
			d = (2. * m * H) / (F + 4. * m); // eq.(17)
			D = H - 2. * d; // eq.(17)
		}
		break;
	}
	
	//std::atan2( mia_.pose().rotation()(1, 0), mia_.pose().rotation()(0, 0) );
	const double theta_z 	= get_rotation_angle(mia_.pose().rotation()); 
	const double t_x		= sensor.pxl2metric(mia_.pose().translation()[0]);
	const double t_y		= sensor.pxl2metric(mia_.pose().translation()[1]);
	const double lambda		= (D / (D + d)); // eq.(3)
	const double dC 		= params_.dc * lambda ; // eq.(3)
	
	//re-set internals
	params_.lambda 	= lambda;
	params_.dC 		= dC;
	params_.N 		= aperture;
	
	//SENSOR
	sensor_ = sensor;
	sensor_.pose().translation()[0] = sensor_.pxl2metric(-(sensor_.width() / 2.)); //set x coordinate
	sensor_.pose().translation()[1] = sensor_.pxl2metric(-(sensor_.height() / 2.)); //set y coordinate
	sensor_.pose().translation()[2] = -(D + d); //set z coordinate
	
	//MLA
	mla_.geometry() 	= mia_.geometry();
	mla_.width() 		= mia_.width();
	mla_.height() 		= mia_.height();
	mla_.pitch() 		= P2D{dC, dC};
	
	mla_.pose().rotation() << 	std::cos(theta_z),	std::sin(theta_z),		0.,
								-std::sin(theta_z),	std::cos(theta_z),		0.,
								0.,					0.,						1.;
														
	mla_.pose().translation()[0] = -t_x + sensor_.pose().translation()[0]; //set x coordinate
	mla_.pose().translation()[1] = -t_y + sensor_.pose().translation()[1]; //set y coordinate
	mla_.pose().translation()[2] = -D; //set z coordinate
	
	//set focal lengths
	mla_.init(I); 
	for (std::size_t i = 0; i < I; ++i) 
		mla().f(i) = (1. / params_.q_prime[i]) * params_.dC * (d / 2.);  // eq.(19)
	
	DEBUG_VAR(D);
	DEBUG_VAR(d);
	DEBUG_VAR(lambda);
	DEBUG_VAR(dC);
	PRINT_DEBUG("theta_z=" << std::setprecision(15) << theta_z << std::setprecision(6));
	DEBUG_VAR(t_x);
	DEBUG_VAR(t_y);
}



//******************************************************************************
//******************************************************************************
//******************************************************************************
//Helper functions
bool PlenopticCamera::is_on_disk(const P2D& p, double radius) const {
	return p.norm() <= radius ; //FIXME: std::sqrt(2.)
}

bool PlenopticCamera::is_inside_mi(const P2D& p, std::size_t k, std::size_t l) const {
	const P2D idx = ml2mi(k,l);
	const P2D c = mia().nodeInWorld(idx[0], idx[1]);
		
	return (p - c).norm() <= (mia().radius() - mia().border());
}

bool PlenopticCamera::hit_main_lens(const Ray3D& ray) const {
	// compute the intersection point between the ray and the lens
	P3D p = line_plane_intersection(main_lens().plane(), ray);
	// Testing if the ray hit the lens
	return is_on_disk(p.head<2>(), main_lens().radius());
}

//Helper projection function	
bool PlenopticCamera::project_through_main_lens(const P3D& p3d_cam, P3D& projection) const
{
	bool is_projected = true;
    
	// the 3d point projected through the main lens
    projection = to_coordinate_system_of(main_lens().pose(), p3d_cam); // THINLENS
    is_projected = main_lens().project(projection, projection);

    // applying main_lens distortions
    main_lens_distortions().apply(projection); // THINLENS
	
    // change to current CAMERA coordinate system
    projection = from_coordinate_system_of(main_lens().pose(), projection); // CAMERA
    
    return is_projected;
}

bool PlenopticCamera::project_through_micro_lens(const P3D& p, std::size_t k, std::size_t l, P2D& projection) const
{
	bool is_projected = true;
	
	// computing a ray linking the micro-lens center and p
    const P3D Ckl_cam = mla().nodeInWorld(k,l); //from_coordinate_system_of(mla().pose(), mla().node(k,l)); //
    Ray3D ray;
    ray.config(Ckl_cam, p); // CAMERA

    //FIXME: only chief ray, testing if the ray hits the main lens
    //is_projected = hit_main_lens(to_coordinate_system_of(main_lens().pose(), ray)); 
    	
    // computing intersection between sensor and ray
    P3D p_sensor = line_plane_intersection(sensor().planeInWorld(), ray); // CAMERA
    p_sensor = to_coordinate_system_of(sensor().pose(), p_sensor); // SENSOR

	projection = sensor().metric2pxl(p_sensor).head<2>(); // IMAGE XY   	
	xy2uv(projection); //IMAGE UV
	
	// check if projection is within the micro-image (k,l)
	is_projected = is_inside_mi(projection, k, l);
	
	return (is_projected and hit_the_sensor(projection));
}	

bool PlenopticCamera::project_radius_through_micro_lens(
	const P3D& p, std::size_t k, std::size_t l, double& radius
) const
{
	if (not focused()) 
	{
		PRINT_ERR("PlenopticCamera::project_radius_through_micro_lens: Can't get radius when MLA is acting as a pinhole array.");
		return false;
	}
	
    // computing radius
//const P3D Ckl_mla 	= mla().node(k,l);
//const P3D p_mla 	= to_coordinate_system_of(mla().pose(), p); // MLA
//const P3D p_kl_mla 	= (p_mla - Ckl_mla); // NODE(K,L)
//const double a_ = (p_kl_mla.z()); //distance according to the tilted direction
    
    const double a_ = p.z() + D(k,l); //orthogonal distance center-proj
	const double d_ = d(k, l); //orthogonal distance center-sensor
	const double D_ = D(k, l); //orthogonal distance center-lens
	const double f_ = mla().f(k,l); //focal length
 
    const double r = params().dc * (D_ / (D_ + d_)) * (d_ / 2.) * ((1. / f_) - (1. / a_) - (1. / d_)); // eq.(12)
    
    radius = sensor().metric2pxl(r);
    
    return true;
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
bool PlenopticCamera::project(const P3D& /*p3d_cam*/, P2D& /*pixel*/) const
{
	PRINT_WARN("PlenopticCamera::project: No micro-lens' index specified.");
	return false;
}
    
bool PlenopticCamera::project(
	const P3D& p3d_cam,
    std::size_t k, std::size_t l,
    P3D& bap
) const
{
	bap.setZero();
	
	P3D p; p.setZero();
    const bool is_projected_through_main_lens = project_through_main_lens(p3d_cam, p);
    
    P2D pixel; pixel.setZero();
	const bool is_projected_through_micro_lens = project_through_micro_lens(p, k, l, pixel);	

	bap.head<2>() = pixel;
	
	if (not focused()) 
	{
		PRINT_ERR("PlenopticCamera::project: Can't get radius when MLA is acting as a pinholes array.");
		return false;
	}
	
    double radius = 0.0;
    const bool is_radius_projected = project_radius_through_micro_lens(p, k, l, radius);
    
    bap[2]		= radius;

    return is_projected_through_main_lens and is_radius_projected and is_projected_through_micro_lens;
}

bool PlenopticCamera::project(
	const P3D& p3d_cam,
    std::size_t k, std::size_t l,
    P2D& pixel
) const
{
	pixel.setZero();
	
	P3D p; p.setZero();
	const bool is_projected_through_main_lens = project_through_main_lens(p3d_cam, p);
	const bool is_projected_through_micro_lens = project_through_micro_lens(p, k, l, pixel);		
	   
    return is_projected_through_main_lens and is_projected_through_micro_lens;
}

bool PlenopticCamera::project(
	const P3D& p3d_cam,
    std::size_t k, std::size_t l,
    double& rho
) const
{
	if (not focused()) 
	{
		PRINT_ERR("PlenopticCamera::project: Can't get radius when MLA is acting as a pinholes array.");
		return false;
	}
	
	rho = 0.0;
	
	P3D p; p.setZero();
    const bool is_projected_through_main_lens = project_through_main_lens(p3d_cam, p);
    const bool is_radius_projected = project_radius_through_micro_lens(p, k, l, rho);
	
	return is_projected_through_main_lens and is_radius_projected;
}


bool PlenopticCamera::project(
	const P3D& p3d_cam,
    CBObservations& observations
) const
{
	observations.clear();
	observations.reserve(mla().nodeNbr());
	
	for (std::size_t k = 0; k < mla().width(); ++k) //iterate through columns //x-axis
    {
    	for (std::size_t l = 0; l < mla().height(); ++l) //iterate through lines //y-axis
		{
			P2D corner; corner.setZero();
			if (project(p3d_cam, k, l, corner))
			{
				observations.emplace_back(
					CBObservation{
						int(k), int(l), 
						corner[0], corner[1]
					}
				);			
			}
		}
	}
	
	observations.shrink_to_fit();
	return (observations.size() > 0u);
}

bool PlenopticCamera::project(
	const P3D& p3d_cam,
    BAPObservations& observations
) const
{
	if (not focused()) 
	{
		PRINT_ERR("PlenopticCamera::project: Can't get radius when MLA is acting as a pinholes array.");
		return false;
	}
	
	observations.clear();
	observations.reserve(mla().nodeNbr());

	for (std::size_t k = 0; k < mla().width(); ++k) //iterate through columns //x-axis
    {
    	for (std::size_t l = 0; l < mla().height(); ++l) //iterate through lines //y-axis
		{
			P3D bap; bap.setZero();
			if (project(p3d_cam, k, l, bap))
			{
				observations.emplace_back(
					BAPObservation{
						int(k), int(l), 
						bap[0], bap[1], bap[2], /* u, v, rho */
					}
				);			
			}
		}
	}
	
	observations.shrink_to_fit();
	return (observations.size() > 0u);
}


//******************************************************************************
//******************************************************************************
//******************************************************************************
bool PlenopticCamera::raytrace(const P2D& /*pixel*/, Ray3D& /*ray*/) const
{
	PRINT_WARN("PlenopticCamera::raytrace: No micro-lens' index specified.");
	return false;
}	


bool PlenopticCamera::raytrace(
	const P2D& pixel, std::size_t k, std::size_t l, Ray3D& ray
) const
{	
	//get pixel in camera coordinate	
	P2D pix = pixel; //IMAGE UV
	uv2xy(pix); //IMAGE XY		
	
	P3D p{pix[0], pix[1], 0.0};
	p = sensor().pxl2metric(p); // SENSOR
	p = from_coordinate_system_of(sensor().pose(), p); // CAMERA
	
	//get ml center
	const P3D ckl = mla().nodeInWorld(k,l); //CAMERA
	
	//configure chief ray from pixel to ml center
	Ray3D r; r.config(p, ckl); // CAMERA
	
	//get p'
	P3D p_prime;
	
	if (focused()) 
	{
		//get focal plane
		const double f = mla().f(k, l); //focal
		const double ai = (f * d()) / (d() - f); //focus distance (in MLA space)
		
		const auto plane = plane_from_3_points(
				from_coordinate_system_of(mla().pose(), P3D{0., 0., ai}),
				from_coordinate_system_of(mla().pose(), P3D{1., 0., ai}),
				from_coordinate_system_of(mla().pose(), P3D{0., 1., ai})
		);
		p_prime = line_plane_intersection(plane, r); // CAMERA
	}
	else //unfocused and pinholes array
	{
		p_prime = p;
	}
	
	//unapply distortions
	P3D p_prime_d = p_prime;
	main_lens_invdistortions().apply(p_prime_d);
	
	//get main lens plane 
	const auto mlplane = main_lens().planeInWorld();
	
	r.config(p_prime, ckl); // CAMERA
	const P3D p_onlens = line_plane_intersection(mlplane, r);
	
	//re-configure ray
	r.config(p_prime_d, p_onlens);		
	
	const double cosTheta = 1.; //r.direction().z(); //normalized()
	r.color().a = cosTheta * cosTheta * cosTheta * cosTheta;
		
//raytrace in main lens
	return main_lens().raytrace(r, ray);
}

bool PlenopticCamera::raytrace(
	const P2D& pixel, std::size_t k, std::size_t l, std::size_t n, Rays3D& rays
) const
{	
	rays.reserve(n+1);

	//get pixel in camera coordinate	
	P2D pix = pixel; //IMAGE UV
	uv2xy(pix); //IMAGE XY		
	
	P3D p{pix[0], pix[1], 0.0};
	p = sensor().pxl2metric(p); // SENSOR
	p = from_coordinate_system_of(sensor().pose(), p); // CAMERA
	
	//get ml center
	const P3D ckl_mla = mla().node(k,l); //MLA
	const P3D ckl = mla().nodeInWorld(k,l); //CAMERA

	//configure chief ray from pixel to ml center
	Ray3D r; r.config(p, ckl); // CAMERA
	
	//get p'
	P3D p_prime;
	if (focused() and not unfocused()) 
	{
		//get focal plane
		const double f = mla().f(k, l); //focal
		const double ai = (f * d()) / (d() - f); //focus distance (in MLA space)
		
		const auto plane = plane_from_3_points(
				from_coordinate_system_of(mla().pose(), P3D{0., 0., ai}),
				from_coordinate_system_of(mla().pose(), P3D{1., 0., ai}),
				from_coordinate_system_of(mla().pose(), P3D{0., 1., ai})
		);
		p_prime = line_plane_intersection(plane, r); // CAMERA
	}
	else //raytrace only from pixel
	{
		p_prime = p;
	}
	
	//unapply distortions
	P3D p_prime_d = p_prime;
	main_lens_invdistortions().apply(p_prime_d);
	
	const auto mlplane = main_lens().planeInWorld();
	
	//raytrace chief ray
	{	
		r.config(p_prime, ckl); // CAMERA
		const P3D p_onlens = line_plane_intersection(mlplane, r);
		r.config(p_prime_d, p_onlens);		
		
		const double cosTheta = 1.; //r.direction().z(); //normalized()
		r.color().a = cosTheta * cosTheta * cosTheta * cosTheta;
		
		Ray ray;
		if (main_lens().raytrace(r, ray))
		{		 	
		 	rays.emplace_back(ray);
		}
	}
	
	//raytrace rays on lens
	for (std::size_t i = 0; i < n; ++i)
	{
		// Get point on lens
		const P2D c_ondisk = concentric_sample_disk(ckl_mla.head<2>(), mla().radius());
		P3D c = P3D{c_ondisk.x(), c_ondisk.y(), 0.}; // MLA
		c = from_coordinate_system_of(mla().pose(), c); // CAMERA
		
		if (not unfocused())
		{
			r.config(p_prime, c); // CAMERA
			const P3D p_onlens = line_plane_intersection(mlplane, r);
					
			r.config(p_prime_d, p_onlens);		
		}
		else // unfocused, generate parallel rays
		{
			//change origin, keep direction of principal ray
			r.config(p_prime, ckl); // CAMERA
			r.origin() = c; //camera
			
			const P3D p_onlens = line_plane_intersection(mlplane, r);
					
			r.config(c, p_onlens);
		}		
		
		const double cosTheta = 1.; //r.direction().z(); //normalized()
		r.color().a = cosTheta * cosTheta * cosTheta * cosTheta;
					
		//raytrace in main lens
		Ray ray;
		if (main_lens().raytrace(r, ray))
		{
			rays.emplace_back(ray);
		}
	}
	
	rays.shrink_to_fit();
	
	return (rays.size() > 0);
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
//Space convertion (obj, mla, virtual)
double PlenopticCamera::v2mla(double x, std::size_t k, std::size_t l) const { return -x * d(k,l); }
double PlenopticCamera::mla2v(double x, std::size_t k, std::size_t l) const { return -x / d(k,l); }

double PlenopticCamera::obj2mla(double x, std::size_t k, std::size_t l) const //{ return D(k,l) - (focal() * x) / (x - focal()); }
{
	const double z = - (focal() * x) / (x - focal());
	
	//apply distortions 
	P3D delta = mla().nodeInWorld(k,l); //CAMERA
	delta.z() = z;
	
	main_lens_distortions().apply_depth(delta);
	
	return D(k,l) + (z + delta.z());  
}

double PlenopticCamera::mla2obj(double x, std::size_t k, std::size_t l) const //{ return (focal() * (D(k,l)-x)) / (D(k,l) -x - focal()); }
{ 
	const double z = x - D(k,l);
	
	//unapply distortions
	P3D delta = mla().nodeInWorld(k,l); //CAMERA
	delta.z() = z;
	main_lens_invdistortions().apply_depth(delta);	
	
	return scaling()(
		(focal() * (z + delta.z())) / (z + delta.z() + focal())
	); 
}

double PlenopticCamera::v2obj(double x, std::size_t k, std::size_t l) const { return mla2obj(v2mla(x, k, l), k, l); }
double PlenopticCamera::obj2v(double x, std::size_t k, std::size_t l) const { return mla2v(obj2mla(x, k, l), k, l); }
	
//******************************************************************************
//******************************************************************************
//******************************************************************************
//Space convertion	(Micro-Images Space / Micro-Lenses Space)
void PlenopticCamera::mi2ml(P2D& pij) const 
{ 
	pij[0] = mla().width()-1 - pij[0]; pij[1] = mla().height()-1 - pij[1];
}
void PlenopticCamera::ml2mi(P2D& pkl) const 
{ 
	mi2ml(pkl);
}

void PlenopticCamera::mi2ml(double& index) const 
{ 
	index = mla().nodeNbr()-1 - index;
}
void PlenopticCamera::ml2mi(double& index) const 
{ 
	mi2ml(index);
}

P2D PlenopticCamera::mi2ml(std::size_t k, std::size_t l) const
{
	P2D pij{k, l}; //MI
	mi2ml(pij); //ML
	return pij;
}

P2D PlenopticCamera::ml2mi(std::size_t k, std::size_t l) const
{
	return mi2ml(k,l);
}

template<typename Observations>
void PlenopticCamera::mi2ml(Observations& obs) const 
{
	std::for_each(
		obs.begin(), obs.end(),
		[*this](auto& ob) { 
			//MI to ML
			P2D pij = mi2ml(ob.k, ob.l);
			
			ob.k = pij[0];
			ob.l = pij[1];
		}	
	);
}

template void PlenopticCamera::mi2ml(MICObservations& obs) const;
template void PlenopticCamera::mi2ml(CBObservations& obs) const;
template void PlenopticCamera::mi2ml(BAPObservations& obs) const;
template void PlenopticCamera::mi2ml(MIObservations& obs) const;

template<typename Observations>
void PlenopticCamera::ml2mi(Observations& obs) const 
{
	std::for_each(
		obs.begin(), obs.end(),
		[*this](auto& ob) { 
			//ML to MI
			P2D pkl = ml2mi(ob.k, ob.l);
			
			ob.k = pkl[0];
			ob.l = pkl[1];
		}	
	);
}

template void PlenopticCamera::ml2mi(MICObservations& obs) const;
template void PlenopticCamera::ml2mi(CBObservations& obs) const;
template void PlenopticCamera::ml2mi(BAPObservations& obs) const;
template void PlenopticCamera::ml2mi(MIObservations& obs) const;

//******************************************************************************
//******************************************************************************
//******************************************************************************
P2D PlenopticCamera::disparity(
	std::size_t k, std::size_t l, std::size_t nk, std::size_t nl, double v
) const
{
	//mi k,l indexes are in mi space, convert to mla space to access micro-lenses
	const P2D idxi = mi2ml(k, l);
	const P2D idxj = mi2ml(nk, nl); 
	
#if 1 //TRUE DISPARITY	
	const P2D deltac = (mia().nodeInWorld(k,l) - mia().nodeInWorld(nk, nl));
	
	const double D_ = (D(idxi(0), idxi(1)) + D(idxj(0), idxj(1))) / 2.;
	const double d_ = (d(idxi(0), idxi(1)) + d(idxj(0), idxj(1))) / 2.;
	
	const double lambda = D_ / (D_ + d_);
	
	const P2D disp = (deltac) * (
		((1. - lambda) * v + lambda) / (v)
	); 
#else
	const P3D mli = mla().node(idxi(0), idxi(1)); 
	const P3D mlj = mla().node(idxj(0), idxj(1));

	const P2D deltaC = (mlj - mli).head<2>();
	
	const P2D disp = (deltaC) / (v * sensor().scale());
#endif
	return disp; //in pixel
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
std::ostream& operator<<(std::ostream& os, const PlenopticCamera::Mode& mode)
{
	switch (mode)
	{
		case PlenopticCamera::Mode::Unfocused: os << "Unfocused (f = d)"; break;
		case PlenopticCamera::Mode::Keplerian: os << "Keplerian (f < d --> F < D)"; break;
		case PlenopticCamera::Mode::Galilean: os << "Galilean (f > d)"; break;
	}
	return os;
}

std::ostream& operator<<(std::ostream& os, const PlenopticCamera& pcm)
{	
	os 	<< "Plenoptic Camera:" << std::endl
		<< "\tinternal configuration = " << pcm.mode() << "," << std::endl
		<< "\th = " << pcm.distance_focus() << "," << std::endl
		<< "\tI = " << pcm.I() << "," << std::endl
		<< "\tscaling = " << pcm.scaling().a << " * z² + " << pcm.scaling().b <<" * z + "<< pcm.scaling().c << "," << std::endl
		<< "\tpose = {" << std::endl << pcm.pose() << "}," << std::endl
		<< "\tsensor = {" << std::endl << pcm.sensor() << "}," << std::endl
		<< "\tmia = {" << std::endl << pcm.mia() << "}," << std::endl
		<< "\tmla = {" << std::endl << pcm.mla() << "}," << std::endl
		<< "\tlens = {" << std::endl << pcm.main_lens() << "}," << std::endl
		<< "\tdistortions = {" << std::endl << pcm.main_lens_distortions() << "}," << std::endl
		<< "\tinvdistortions = {" << std::endl << pcm.main_lens_invdistortions() << "}," << std::endl
		<< "\td = " << pcm.d() << "," << std::endl
		<< "\tD = " << pcm.D() << "," << std::endl
		<< "\tprincipal point = {" << pcm.pp().transpose() << "}," << std::endl;
	
	if (pcm.multifocus())
	{
		os << "\tf = {"; 
		std::size_t i = 0; for(; i < pcm.I()-1; ++i) os << pcm.mla().f(i) <<", ";
		os << pcm.mla().f(i) << "}," << std::endl;
		
		os << "\tfocal_plane = {"; 
		i = 0; for(; i < pcm.I()-1; ++i) os << pcm.focal_plane(i) <<" ("<< pcm.obj2v(pcm.focal_plane(i)) << "), ";
		os << pcm.focal_plane(i) <<" ("<< pcm.obj2v(pcm.focal_plane(i)) << ")}" << std::endl;
	}
	else
	{
		os << "\tfocal_plane = " << pcm.focal_plane(0) <<" ("<< pcm.obj2v(pcm.focal_plane(0)) << ")";
	}
		
	return os;
}

void save(std::string path, const PlenopticCamera& pcm)
{
	PlenopticCameraConfig config;
	
	// Configuring the Sensor
    config.sensor().pose().rotation() = pcm.sensor().pose().rotation();
    config.sensor().pose().translation() = pcm.sensor().pose().translation();
    config.sensor().width() = pcm.sensor().width();
    config.sensor().height() = pcm.sensor().height();
    config.sensor().scale() = pcm.sensor().scale();
    
     // Configuring the MicroImagesArray
    config.mia().mesh().pose().rotation() = pcm.mia().pose().rotation();
    config.mia().mesh().pose().translation() = pcm.mia().pose().translation();
    config.mia().mesh().width() = pcm.mia().width();
    config.mia().mesh().height() = pcm.mia().height();
    config.mia().mesh().pitch() = pcm.mia().pitch();
    config.mia().mesh().geometry() = pcm.mia().geometry();
    
     // Configuring the MicroLensesArray
    config.mla().mesh().pose().rotation() = pcm.mla().pose().rotation();
    config.mla().mesh().pose().translation() = pcm.mla().pose().translation();
    config.mla().mesh().width() = pcm.mla().width();
    config.mla().mesh().height() = pcm.mla().height();
    config.mla().mesh().pitch() = pcm.mla().pitch();
    config.mla().mesh().geometry() = pcm.mla().geometry();
    
    config.mla().focal_lengths().resize(pcm.I());
    for (std::size_t i = 0; i < pcm.I(); ++i) config.mla().focal_lengths()[i] = pcm.mla().f(i);

	// Configuring the Main Lens
    config.main_lens().pose().rotation() = pcm.main_lens().pose().rotation();
    config.main_lens().pose().translation() = pcm.main_lens().pose().translation();
    config.main_lens().f() = pcm.main_lens().focal();
    config.main_lens().aperture() = pcm.main_lens().aperture();
    config.main_lens().diameter() = pcm.main_lens().diameter();

    // Configuring the Distortions
    config.distortions().radial() = pcm.main_lens_distortions().radial();
    config.distortions().tangential() = pcm.main_lens_distortions().tangential();
    config.distortions().depth() = pcm.main_lens_distortions().depth();
    config.distortions().model() = pcm.main_lens_distortions().model();
    
    config.distortions_inverse().radial() = pcm.main_lens_invdistortions().radial();
    config.distortions_inverse().tangential() = pcm.main_lens_invdistortions().tangential();
    config.distortions_inverse().depth() = pcm.main_lens_invdistortions().depth();
    config.distortions_inverse().model() = pcm.main_lens_invdistortions().model();
    
    // Configuring the Focus distance
    config.dist_focus() = pcm.distance_focus();
    
    // Configuring the scaling function
    config.scaling().a() = pcm.scaling().a;
    config.scaling().b() = pcm.scaling().b;
    config.scaling().c() = pcm.scaling().c;
    
    // Configuring additional information
    config.mode() 	= pcm.mode();
    config.d() 		= pcm.d();
    config.D() 		= pcm.D();
    config.pp() 	= pcm.pp();
    config.Rxyz()	= pcm.mla().pose().rotation().eulerAngles(0,1,2);
    
    if (pcm.multifocus())
    {
    	config.focal_planes().resize(pcm.I());
    	for (std::size_t i = 0; i < pcm.I(); ++i) config.focal_planes()[i] = pcm.focal_plane(i);
    }
    else
    {
    	config.focal_planes().push_back(pcm.focal_plane(0));
    }

	v::save(path, config);
}

void load(std::string path, PlenopticCamera& pcm)
{
	PlenopticCameraConfig config;
	v::load(path, config);
	
	// Configuring the Sensor
    pcm.sensor().pose().rotation() = config.sensor().pose().rotation(); 
    pcm.sensor().pose().translation() = config.sensor().pose().translation(); 
    pcm.sensor().width() = config.sensor().width(); 
    pcm.sensor().height() = config.sensor().height(); 
    pcm.sensor().scale() = config.sensor().scale(); 
    
     // Configuring the MicroImagesArray
    pcm.mia() = MicroImagesArray{config.mia()};
    
     // Configuring the MicroLensesArray
    pcm.mla() = MicroLensesArray{config.mla()};
    
	// Configuring the Main Lens
    pcm.main_lens().pose().rotation() = config.main_lens().pose().rotation(); 
    pcm.main_lens().pose().translation() = config.main_lens().pose().translation(); 
    pcm.main_lens().focal() = config.main_lens().f(); 
    pcm.main_lens().aperture() = config.main_lens().aperture();  

    // Configuring the Distortions
    pcm.main_lens_distortions().radial() = config.distortions().radial(); 
    pcm.main_lens_distortions().tangential() = config.distortions().tangential(); 
    pcm.main_lens_distortions().depth() = config.distortions().depth(); 
    pcm.main_lens_distortions().model() = Distortions::DepthDistortionModel(config.distortions().model()); 
    
    pcm.main_lens_invdistortions().radial() = config.distortions_inverse().radial(); 
    pcm.main_lens_invdistortions().tangential() = config.distortions_inverse().tangential(); 
    pcm.main_lens_invdistortions().depth() = config.distortions_inverse().depth(); 
    pcm.main_lens_invdistortions().model() = Distortions::DepthDistortionModel(config.distortions_inverse().model()); 
    
    // Configuring the Focus distance
    pcm.distance_focus() = config.dist_focus();
    
    // Configuring the scaling function
    pcm.scaling().a = config.scaling().a();
    pcm.scaling().b = config.scaling().b();
    pcm.scaling().c = config.scaling().c();
}


