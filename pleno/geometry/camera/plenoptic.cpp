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
    if (main_lens().focal() < std::abs(mla().pose().translation()[2])) { return Keplerian; }
	else if (main_lens().focal() > std::abs(mla().pose().translation()[2])) { return Galilean; }
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

std::size_t PlenopticCamera::I() const { return mla().I(); }

//******************************************************************************
//******************************************************************************
//******************************************************************************
PlenopticCamera::PrincipalPoint 
PlenopticCamera::pp() const
{
	P2D PP = sensor().pose().translation().head(2); //SENSOR
	PP = sensor().metric2pxl(PP); PP *= -1.; //IMAGE XY
	xy2uv(PP); //IMAGE UV
		
	return PP;
}
    
PlenopticCamera::PrincipalPoint
PlenopticCamera::mlpp(std::size_t k, std::size_t l) const
{
	P2D mi_index{k,l}; ml2mi(mi_index);
	
	const auto& ckl = mia().nodeInWorld(mi_index[0], mi_index[1]); //IMAGE
	const double d = std::fabs(mla().pose().translation()[2] - sensor().pose().translation()[2]);
	const double D = std::fabs(mla().pose().translation()[2]);
	const double ratio = d / (D + d);
	
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
	
	double d,D;
	const double m = std::fabs(params_.m);
	switch(mode)
	{
		case Unfocused:
			d = m * 2.; D = F;
		break;
		case Keplerian: 
		{
			//d = (2. * params_.m * F) / (F - 2. * params_.m); D = F + d; 
			const double H = (h / 2.) * (1. - sqrt(1. - 4. * (F / h))); DEBUG_VAR(H);
			d = (2. * m * H) / (F + 4. * m);
			D = H - 2. * d;
			//D = H;
			//d = 2. * D * params_.m / F;
		}
		break;
		case Galilean:
		{
			//d = (2. * F * params_.m) / (2. * params_.m + F); D = F - d;
			const double H = std::fabs((h / 2.) * (1. - sqrt(1. + 4. * (F / h)))); DEBUG_VAR(H);
			d = (2. * m * H) / (F + 4. * m);
			D = H - 2. * d;
			//D = H;
			//d = 2. * D * params_.m / F;
		}
		break;
	}

	const double theta_z 	= get_rotation_angle(mia_.pose().rotation()); //std::atan2( mia_.pose().rotation()(1, 0), mia_.pose().rotation()(0, 0) );
	const double t_x		= sensor.pxl2metric(mia_.pose().translation()[0]);
	const double t_y		= sensor.pxl2metric(mia_.pose().translation()[1]);
	const double kappa_approx = params_.kappa * (D / (D + d)) ;
	
	//re-set kappa_approx
	params_.kappa_approx = kappa_approx;
	
	//SENSOR
	sensor_ = sensor;
	sensor_.pose().translation()[0] = sensor_.pxl2metric(- (sensor_.width()  / 2.)); //set x coordinate
	sensor_.pose().translation()[1] = sensor_.pxl2metric(- (sensor_.height() / 2.)); //set y coordinate
	sensor_.pose().translation()[2] = - (D + d); //set z coordinate
	
	//MLA
	mla_.geometry() 	= mia_.geometry();
	mla_.width() 		= mia_.width();
	mla_.height() 		= mia_.height();
	mla_.edge_length() 	= P2D{kappa_approx, kappa_approx};
	
	mla_.pose().rotation() << 	std::cos(theta_z),	std::sin(theta_z),		0.,
								-std::sin(theta_z),	std::cos(theta_z),		0.,
								0.,					0.,						1.;
														
	mla_.pose().translation()[0] = t_x + sensor_.pose().translation()[0]; //set x coordinate
	mla_.pose().translation()[1] = t_y + sensor_.pose().translation()[1]; //set y coordinate
	mla_.pose().translation()[2] = - D; //set z coordinate
	
	//set focal lengths
	mla_.init(I); for(std::size_t i=0; i<I; ++i ) mla().f(i) = (1. / params_.c_prime[i]) * params_.kappa_approx * (d / 2.); 
	
	DEBUG_VAR(D);
	DEBUG_VAR(d);
	DEBUG_VAR(kappa_approx);
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
    is_projected = is_projected and hit_main_lens(to_coordinate_system_of(main_lens().pose(), ray));
    	
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
    P3D Cklmla_ = mla().node(k,l);
    P3D pmla_ = to_coordinate_system_of(mla().pose(), p); // MLA
    P3D p_klmla_ = (pmla_ - Cklmla_); // NODE(K,L)
    
    const double a = (p_klmla_[2]);
	const double d = std::fabs(mla().pose().translation()[2] - sensor().pose().translation()[2]);
	const double D = std::fabs(mla().pose().translation()[2]);

	P2D mi_index{k,l}; ml2mi(mi_index);
	const double f = mla().f(mi_index[0], mi_index[1]).f;
 
    const double r = params().kappa * (D / (D + d)) * (d / 2.) * ((1. / f) - (1. / a) - (1. / d));
    
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
	
	P3D p;
    bool is_projected_through_main_lens = project_through_main_lens(p3d_cam, p);
    
    double radius;
    bool is_radius_projected = project_radius_through_micro_lens(p, k, l, radius);

    P2D pixel;
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
	P3D p;
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
	
	P3D p;
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
			P2D corner;
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
			P3D bap;
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

template<typename Observations>
void PlenopticCamera::mi2ml(Observations& obs) const 
{
	std::for_each(
		obs.begin(), obs.end(),
		[*this](auto& ob) { 
			//MI to ML
			P2D pij{ob.k, ob.l}; //MI
			this->mi2ml(pij); //ML
			
			ob.k = pij[0];
			ob.l = pij[1];
		}	
	);
}

template void PlenopticCamera::mi2ml(MICObservations& obs) const ;
template void PlenopticCamera::mi2ml(CBObservations& obs) const ;
template void PlenopticCamera::mi2ml(BAPObservations& obs) const ;

template<typename Observations_t>
void PlenopticCamera::ml2mi(Observations_t& obs) const 
{
	std::for_each(
		obs.begin(), obs.end(),
		[*this](auto& ob) { 
			//MI to ML
			P2D pkl{ob.k, ob.l}; //ML
			this->ml2mi(pkl); //MI
			
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
		case PlenopticCamera::Mode::Unfocused: os << "Unfocused (F = D)"; break;
		case PlenopticCamera::Mode::Keplerian: os << "Keplerian (F < D)"; break;
		case PlenopticCamera::Mode::Galilean: os << "Galilean (F > D)"; break;
	}
	return os;
}

std::ostream& operator<<(std::ostream& os, const PlenopticCamera& pcm)
{	
	os 	<< "Plenoptic Camera:" << std::endl
		<< "\tmode = " << pcm.mode() << "," << std::endl
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
    config.mia().mesh().pitch() = pcm.mia().edge_length();
    config.mia().mesh().geometry() = pcm.mia().geometry();
    
     // Configuring the MicroLensesArray
    config.mla().mesh().pose().rotation() = pcm.mla().pose().rotation();
    config.mla().mesh().pose().translation() = pcm.mla().pose().translation();
    config.mla().mesh().width() = pcm.mla().width();
    config.mla().mesh().height() = pcm.mla().height();
    config.mla().mesh().pitch() = pcm.mla().edge_length();
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


