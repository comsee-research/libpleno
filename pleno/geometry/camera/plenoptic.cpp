#include "plenoptic.h"

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
    if (I() > 0u)
    {
    	f_ = mla().f(0);
    	d_ = d();    
    }
        
    if ( f_ < d_ ) { return Keplerian; }
	else if (f_ > d_) { return Galilean; }
    else { return Unfocused; }
}

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

double PlenopticCamera::focal() const { return main_lens().focal(); }
double& PlenopticCamera::focal() { return main_lens().focal(); }   

double PlenopticCamera::aperture() const { return main_lens().aperture(); }
double& PlenopticCamera::aperture() { return main_lens().aperture(); }   

double PlenopticCamera::mlaperture() const { return mla().diameter(); }

std::size_t PlenopticCamera::I() const { return mla().I(); }

double PlenopticCamera::d() const { return std::fabs(mla().pose().translation()[2] - sensor().pose().translation()[2]); }
double PlenopticCamera::D() const { return std::fabs(mla().pose().translation()[2]); }

double PlenopticCamera::focal_plane(std::size_t i) const 
{ 
	if (I() == 0u) return v2obj(2.); 
	else return mla2obj((mla().f(i) * d()) / (d() - mla().f(i))); 
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
PlenopticCamera::PrincipalPoint 
PlenopticCamera::pp() const
{
	P3D PP = main_lens().pose().translation(); //CAMERA
	PP = to_coordinate_system_of(sensor().pose(), PP); // SENSOR
	
	P2D pp = sensor().metric2pxl(PP).head(2); //IMAGE XY
	xy2uv(pp); //IMAGE UV
		
	return pp;
}
    
PlenopticCamera::PrincipalPoint
PlenopticCamera::mlpp(std::size_t k, std::size_t l) const
{
	P2D mi_index{k,l}; ml2mi(mi_index);
	
	const auto& ckl = mia().nodeInWorld(mi_index[0], mi_index[1]); //IMAGE
	const double ratio = d() / (D() + d());
	
	const auto mpp = this->pp();
	
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
	pose_.translation() = Pose::Vector::Zero();
	pose_.rotation()	= Pose::Matrix::Identity();
	
	//MAIN LENS
	main_lens_.focal() = F;
	main_lens_.aperture() = aperture;
	main_lens_.pose().translation() = pose_.translation();
	main_lens_.pose().rotation()	= pose_.rotation();
		
	//DISTORTIONS
	distortions_.radial() << 0., 0., 0.;
	distortions_.tangential() << 0., 0.;
	
	/* 	
		Init of d and D is kinda tricky: 
			- In Keplerian configuration, F < D whatever the focus distance.
			- In Galilean configuration, normally we should have F > D, 
				- but when the main lens focus distance is at infinity
				- the main lens focuses on the TCP (i.e., v = 2)
				- or when h decrease, D increase,
				- so in most cases, we will still have F > D
	*/
	double d,D;
	const double m = std::fabs(params_.m);
	const double H = (h / 2.) * (1. - std::sqrt(1. - 4. * (F / h))); DEBUG_VAR(H); // eq.(18)
	
	switch(mode)
	{
		case Unfocused:
			d = m * 2.; D = F;
		break;
		
		case Keplerian: 
		{
			d = (2. * m * H) / (F + 4. * m); // eq.(17)
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
	params_.lambda = lambda;
	params_.dC = dC;
	params_.N = aperture;
	
	//SENSOR
	sensor_ = sensor;
	sensor_.pose().translation()[0] = sensor_.pxl2metric(- (sensor_.width()  / 2.)); //set x coordinate
	sensor_.pose().translation()[1] = sensor_.pxl2metric(- (sensor_.height() / 2.)); //set y coordinate
	sensor_.pose().translation()[2] = - (D + d); //set z coordinate
	
	//MLA
	mla_.geometry() 	= mia_.geometry();
	mla_.width() 		= mia_.width();
	mla_.height() 		= mia_.height();
	mla_.pitch() 		= P2D{dC, dC};
	
	mla_.pose().rotation() << 	std::cos(theta_z),	std::sin(theta_z),		0.,
								-std::sin(theta_z),	std::cos(theta_z),		0.,
								0.,					0.,						1.;
														
	mla_.pose().translation()[0] = t_x + sensor_.pose().translation()[0]; //set x coordinate
	mla_.pose().translation()[1] = t_y + sensor_.pose().translation()[1]; //set y coordinate
	mla_.pose().translation()[2] = - D; //set z coordinate
	
	//set focal lengths
	mla_.init(I); 
	for(std::size_t i=0; i<I; ++i ) mla().f(i) = (1. / params_.q_prime[i]) * params_.dC * (d / 2.);  // eq.(19)
	
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
    P3D Ckl_cam = from_coordinate_system_of(mla().pose(), mla().node(k,l)); //mla_.nodeInWorld(k,l);
    Ray3D ray;
    ray.config(Ckl_cam, p); // CAMERA

    // testing if the ray hits the main lens
    is_projected = hit_main_lens(to_coordinate_system_of(main_lens().pose(), ray));
    	
    // computing intersection between sensor and ray
    P3D p_sensor = line_plane_intersection(sensor().planeInWorld(), ray); // CAMERA
    p_sensor = to_coordinate_system_of(sensor().pose(), p_sensor); // SENSOR

	projection = sensor().metric2pxl(p_sensor).head(2); // IMAGE XY   	
	xy2uv(projection); //IMAGE UV
	
	return (is_projected and hit_the_sensor(projection));
}	

bool PlenopticCamera::project_radius_through_micro_lens(
	const P3D& p, std::size_t k, std::size_t l, double& radius
) const
{
	if(mla().I() == 0u) 
	{
		PRINT_ERR("PlenopticCamera::project_radius_through_micro_lens: Can't get radius when MLA is acting as a pinhole array.");
		return false;
	}
	
    // computing radius
    P3D Ckl_mla = mla().node(k,l);
    P3D p_mla = to_coordinate_system_of(mla().pose(), p); // MLA
    P3D p_kl_mla = (p_mla - Ckl_mla); // NODE(K,L)
    
    const double a = (p_kl_mla[2]);
	const double d_ = d();
	const double D_ = D();

	P2D mi_idx = ml2mi(k,l);
	const double f = mla().f(mi_idx[0], mi_idx[1]).f;
 
    const double r = params().dc * (D_ / (D_ + d_)) * (d_ / 2.) * ((1. / f) - (1. / a) - (1. / d_)); // eq.(12)
    
    radius = sensor().metric2pxl(r);
    
    return true;
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
bool PlenopticCamera::project(
	const P3D& p3d_cam,
    std::size_t k, std::size_t l,
    P3D& bap
) const
{
	if(mla().I() == 0u) 
	{
		PRINT_ERR("PlenopticCamera::project: Can't get radius when MLA is acting as a pinholes array.");
		return false;
	}
	
	bap.setZero();
	
	P3D p; p.setZero();
    bool is_projected_through_main_lens = project_through_main_lens(p3d_cam, p);
    
    double radius = 0.0;
    bool is_radius_projected = project_radius_through_micro_lens(p, k, l, radius);

    P2D pixel; pixel.setZero();
	bool is_projected_through_micro_lens = project_through_micro_lens(p, k, l, pixel);	

	bap.head(2) = pixel;
    bap[2]	= radius;

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
	bool is_projected_through_main_lens = project_through_main_lens(p3d_cam, p);
	bool is_projected_through_micro_lens = project_through_micro_lens(p, k, l, pixel);		
	   
    //TODO: check if point is projected in the correct micro-image
    return is_projected_through_main_lens and is_projected_through_micro_lens;
}

bool PlenopticCamera::project(
	const P3D& p3d_cam,
    std::size_t k, std::size_t l,
    double& rho
) const
{
	if(mla().I() == 0u) 
	{
		PRINT_ERR("PlenopticCamera::project: Can't get radius when MLA is acting as a pinholes array.");
		return false;
	}
	
	rho = 0.0;
	
	P3D p; p.setZero();
    bool is_projected_through_main_lens = project_through_main_lens(p3d_cam, p);
    bool is_radius_projected = project_radius_through_micro_lens(p, k, l, rho);
	
	return is_projected_through_main_lens and is_radius_projected;
}


bool PlenopticCamera::project(
	const P3D& p3d_cam,
    CBObservations& observations
) const
{
	observations.clear();
	observations.reserve(mla().nodeNbr());

#pragma omp parallel for	
	for(std::size_t k = 0; k < mla().width() ; ++k) //iterate through columns //x-axis
    {
    	for(std::size_t l = 0 ; l < mla().height() ; ++l) //iterate through lines //y-axis
		{
			P2D corner; corner.setZero();
			if(project(p3d_cam, k, l, corner))
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
	if(mla().I() == 0u) 
	{
		PRINT_ERR("PlenopticCamera::project: Can't get radius when MLA is acting as a pinholes array.");
		return false;
	}
	
	observations.clear();
	observations.reserve(mla().nodeNbr());

#pragma omp parallel for	
	for(std::size_t k = 0; k < mla().width() ; ++k) //iterate through columns //x-axis
    {
    	for(std::size_t l = 0 ; l < mla().height() ; ++l) //iterate through lines //y-axis
		{
			P3D bap; bap.setZero();
			if(project(p3d_cam, k, l, bap))
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
//Space convertion (obj, mla, virtual)
double PlenopticCamera::v2mla(double x) const { return -x * d(); }
double PlenopticCamera::mla2v(double x) const { return -x / d(); }

double PlenopticCamera::obj2mla(double x) const { return D() - (focal() * x) / (x - focal()); }
double PlenopticCamera::mla2obj(double x) const { return (focal() * (D()-x)) / (D() - x - focal()); }

double PlenopticCamera::v2obj(double x) const { return mla2obj(v2mla(x)); }
double PlenopticCamera::obj2v(double x) const { return mla2v(obj2mla(x)); }
	
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

template void PlenopticCamera::mi2ml(MICObservations& obs) const ;
template void PlenopticCamera::mi2ml(CBObservations& obs) const ;
template void PlenopticCamera::mi2ml(BAPObservations& obs) const ;

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

template void PlenopticCamera::ml2mi(MICObservations& obs) const ;
template void PlenopticCamera::ml2mi(CBObservations& obs) const ;
template void PlenopticCamera::ml2mi(BAPObservations& obs) const ;
	
//******************************************************************************
//******************************************************************************
//******************************************************************************
std::ostream& operator<<(std::ostream& os, const PlenopticCamera::Mode& mode)
{
	switch(mode)
	{
		case PlenopticCamera::Mode::Unfocused: os << "Unfocused (F = D --> f = d)"; break;
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
		<< "\tpose = {" << std::endl << pcm.pose() << "}," << std::endl
		<< "\tsensor = {" << std::endl << pcm.sensor() << "}," << std::endl
		<< "\tmia = {" << std::endl << pcm.mia() << "}," << std::endl
		<< "\tmla = {" << std::endl << pcm.mla() << "}," << std::endl
		<< "\tlens = {" << std::endl << pcm.main_lens() << "}," << std::endl
		<< "\tdistortions = {" << std::endl << pcm.main_lens_distortions() << "}," << std::endl
		<< "\tprincipal point = {" << pcm.pp() << "}";
	
	if(pcm.mla().I() != 0u)
	{
		os << ","<< std::endl << "\tf = {"; 
		std::size_t i=0; for(; i<pcm.mla().I()-1; ++i) os << pcm.mla().f(i) <<", ";
		os << pcm.mla().f(i) << "}" << std::endl;
		
		os << ","<< std::endl << "\tfocal_plane = {"; 
		i=0; for(; i<pcm.mla().I()-1; ++i) os << pcm.focal_plane(i) <<", ";
		os << pcm.focal_plane(i) << "}" << std::endl;
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
    
    config.mla().focal_lengths().resize(pcm.mla().I());
    for(std::size_t i=0; i<pcm.mla().I(); ++i) config.mla().focal_lengths()[i] = pcm.mla().f(i);

	// Configuring the Main Lens
    config.main_lens().pose().rotation() = pcm.main_lens().pose().rotation();
    config.main_lens().pose().translation() = pcm.main_lens().pose().translation();
    config.main_lens().f() = pcm.main_lens().focal();
    config.main_lens().aperture() = pcm.main_lens().aperture();
    config.main_lens().diameter() = pcm.main_lens().diameter();

    // Configuring the Distortions
    config.distortions().radial() = pcm.main_lens_distortions().radial();
    config.distortions().tangential() = pcm.main_lens_distortions().tangential();
    
    // Configuring the Focus distance
    config.dist_focus() = pcm.distance_focus();
    
    // Configuring the Mode
    config.mode() = pcm.mode();

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
    
    // Configuring the Focus distance
    pcm.distance_focus() = config.dist_focus();
}


