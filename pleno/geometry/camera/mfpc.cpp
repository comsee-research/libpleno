#include "camera.h"

#include <iomanip>

#include <libv/geometry/plane_equation.hpp>

#include "io/printer.h"
#include "io/cfg/camera.h"


//******************************************************************************
//******************************************************************************
//******************************************************************************
MultiFocusPlenopticCamera::Mode
MultiFocusPlenopticCamera::mode() const
{
    if (_main_lens.f() < std::abs(_mla.pose().translation()[2]))
        return Keplerian;
	else if (_main_lens.f() > std::abs(_mla.pose().translation()[2]))
        return Galilean;
    else
    	return Unfocused;
}

//OLD INTERFACE   
const GridMesh3D& MultiFocusPlenopticCamera::micro_lenses() const { return _mla; }
GridMesh3D& MultiFocusPlenopticCamera::micro_lenses() { return _mla; }

const GridMesh2D& MultiFocusPlenopticCamera::raw_image() const { return _mia; }
GridMesh2D& MultiFocusPlenopticCamera::raw_image() { return _mia; }

//NEW INTERFACE
const MicroLensesArray& MultiFocusPlenopticCamera::mla() const { return _mla; }
MicroLensesArray& MultiFocusPlenopticCamera::mla() { return _mla; }

const MicroImagesArray& MultiFocusPlenopticCamera::mia() const { return _mia; }
MicroImagesArray& MultiFocusPlenopticCamera::mia() { return _mia; }     

const InternalParameters& MultiFocusPlenopticCamera::params() const { return _params; }
InternalParameters& MultiFocusPlenopticCamera::params() { return _params; }   

double MultiFocusPlenopticCamera::distance_focus() const { return _dist_focus; } 
double& MultiFocusPlenopticCamera::distance_focus() { return _dist_focus; }   

//COMMON INTERFACE
const Sensor& MultiFocusPlenopticCamera::sensor() const { return _sensor; }
Sensor& MultiFocusPlenopticCamera::sensor() { return _sensor; }

const ThinLensCameraModel& MultiFocusPlenopticCamera::main_lens() const { return _main_lens; }
ThinLensCameraModel& MultiFocusPlenopticCamera::main_lens() { return _main_lens; }

const Pose& MultiFocusPlenopticCamera::pose() const { return _pose; }
Pose& MultiFocusPlenopticCamera::pose() { return _pose; }

const Distortions& MultiFocusPlenopticCamera::main_lens_distortions() const { return _distortions; }
Distortions& MultiFocusPlenopticCamera::main_lens_distortions() { return _distortions; }


//******************************************************************************
//******************************************************************************
//******************************************************************************
void MultiFocusPlenopticCamera::init(
	const Sensor& sensor_, 
	const MicroImagesArray& mia_, 
	const InternalParameters& params_, 
	double F, double aperture, double h,
	MultiFocusPlenopticCamera::Mode mode 
)
{		
	_mia 	= mia_; //already calibrated
	_params = params_; //already computed
	
	//DISTANCE FOCUS
	_dist_focus = h;
	//POSE
	_pose.translation() = Pose::Vector::Zero();
	_pose.rotation()	= Pose::Matrix::Identity();
	
	//MAIN LENS
	_main_lens.f() = F;
	_main_lens.aperture() = aperture;
	_main_lens.pose().translation() = _pose.translation();
	_main_lens.pose().rotation()	= _pose.rotation();
		
	//DISTORTIONS
	_distortions.radial() << 0., 0., 0.;
	_distortions.tangential() << 0., 0.;
	
	double d,D;
	const double m = std::fabs( _params.m);
	switch(mode)
	{
		case Unfocused:
			d = _params.m * 2.; D = F;
		break;
		case Keplerian: 
		{
			//d = (2. * _params.m * F) / (F - 2. * _params.m); D = F + d; 
			const double H = (h / 2.) * (1. - sqrt(1. - 4. * (F / h))); DEBUG_VAR(H);
			d = (2. * m * H) / (F + 4. * m);
			D = H - 2. * d;
			//D = H;
			//d = 2. * D * _params.m / F;
		}
		break;
		case Galilean:
		{
			//d = (2. * F * _params.m) / (2. * _params.m + F); D = F - d;
			const double H = std::fabs((h / 2.) * (1. - sqrt(1. + 4. * (F / h)))); DEBUG_VAR(H);
			d = (2. * m * H) / (F + 4. * m);
			D = H - 2. * d;
			//D = H;
			//d = 2. * D * _params.m / F;
		}
		break;
	}

	const double theta_z 	= std::atan2( _mia.pose().rotation()(1, 0), _mia.pose().rotation()(0, 0) );
	const double t_x		= sensor_.pxl2metric(_mia.pose().translation()[0]);
	const double t_y		= sensor_.pxl2metric(_mia.pose().translation()[1]);
	const double kappa_approx = _params.kappa * (D / (D + d)) ;
	
	//re-set kappa_approx
	_params.kappa_approx = kappa_approx;
	
	//SENSOR
	_sensor = sensor_;
	_sensor.pose().translation()[0] = _sensor.pxl2metric(- (_sensor.width()  / 2.)); //set x coordinate
	_sensor.pose().translation()[1] = _sensor.pxl2metric(- (_sensor.height() / 2.)); //set y coordinate
	_sensor.pose().translation()[2] = - (D + d); //set z coordinate
	
	//MLA
	_mla.geometry() 	= _mia.geometry();
	_mla.orientation() 	= _mia.orientation();
	_mla.width() 		= _mia.width();
	_mla.height() 		= _mia.height();
	_mla.edge_length() 	= P2D{kappa_approx, kappa_approx};
	
	_mla.pose().rotation() << 	std::cos(theta_z),	std::sin(theta_z),		0.,
								-std::sin(theta_z),	std::cos(theta_z),		0.,
								0.,					0.,						1.;
														
	_mla.pose().translation()[0] = t_x + _sensor.pose().translation()[0]; //set x coordinate
	_mla.pose().translation()[1] = t_y + _sensor.pose().translation()[1]; //set y coordinate
	_mla.pose().translation()[2] = - D; //set z coordinate

	_mla.f(0) = (1. / _params.c_prime[0] ) * kappa_approx * (d / 2.); 
	_mla.f(1) = (1. / _params.c_prime[1] ) * kappa_approx * (d / 2.); 
	_mla.f(2) = (1. / _params.c_prime[2] ) * kappa_approx * (d / 2.); 	 		
	
	DEBUG_VAR(D);
	DEBUG_VAR(d);
	DEBUG_VAR(kappa_approx);
	PRINT_DEBUG("theta_z=" << std::setprecision(15) << theta_z << std::setprecision(6));
	DEBUG_VAR(t_x);
	DEBUG_VAR(t_y);
}
  
MultiFocusPlenopticCamera::PrincipalPoint 
MultiFocusPlenopticCamera::pp() const
{
//FIXME:TEST_INVERSION
	P2D PP = _sensor.pose().translation().head(2); //SENSOR
	PP = _sensor.metric2pxl(PP); PP *= -1.; //IMAGE XY
	xy2uv(PP); //IMAGE UV
	
	const double u = PP[0];
	const double v = PP[1];
	
	return {u,v};
}
    
MultiFocusPlenopticCamera::PrincipalPoint
MultiFocusPlenopticCamera::mlpp(std::size_t k, std::size_t l) const
{
	P2D mi_index{k,l}; ml2mi(mi_index);
	
	const auto& ckl = _mia.nodeInWorld(mi_index[0], mi_index[1]); //IMAGE
	const double d = std::fabs(_mla.pose().translation()[2] - _sensor.pose().translation()[2]);
	const double D = std::fabs(_mla.pose().translation()[2]);
	const double ratio = d / (D + d);
	
	const auto PP = this->pp();	
	const P2D mpp = P2D{PP.u, PP.v};
	
	const P2D ppkl = ratio * (mpp - ckl) + ckl;
	
	return {ppkl[0], ppkl[1]};
}


//******************************************************************************
//******************************************************************************
//******************************************************************************
bool MultiFocusPlenopticCamera::project(
	const P3D& p3d_cam,
    std::size_t k, std::size_t l,
    P3D& bap
) const
{
	auto hit_the_sensor = [&sensor = _sensor](const P2D& p) {
		return p[0] >= 0.0 and p[1] >= 0.0 and p[0] < sensor.width() and p[1] < sensor.height();
	};
    
    auto is_on_disk = [](const P2D& p, double disk_diameter) {
    	return p.norm() <= disk_diameter / 2.0 ;
    };
    
    auto hit_main_lens = [&is_on_disk, &lens = _main_lens](const Ray3D& ray) {
		// compute the intersection point between the ray and the lens
		P3D p = line_plane_intersection(Eigen::Vector4d{0.0, 0.0, 1.0, 0.0}, ray);
		// Testing if the ray hit the lens
		return is_on_disk(p.head(2), lens.diameter());
	};

    bool is_projected = true;

    // the 3d point projected through the main lens
    P3D p = to_coordinate_system_of(_main_lens.pose(), p3d_cam); // THINLENS
    is_projected = main_lens().project(p, p);

    // applying main_lens distortions
    _distortions.apply(p); // THINLENS
	
    // change to current CAMERA coordinate system
    p = from_coordinate_system_of(_main_lens.pose(), p); // CAMERA
    
    // computing radius
    P3D Ckl_mla = _mla.node(k,l);
    P3D p_mla = to_coordinate_system_of(_mla.pose(), p); // MLA
    P3D p_kl_mla = (p_mla - Ckl_mla); // NODE(K,L)
    
    const double a = (p_kl_mla[2]);
	const double d = std::fabs(_mla.pose().translation()[2] - _sensor.pose().translation()[2]);
	const double D = std::fabs(_mla.pose().translation()[2]);
	//const double A = (_mla.edge_length()[0] + _mla.edge_length()[1]) / 2.;

	P2D mi_index{k,l}; ml2mi(mi_index);
	const double mik = mi_index[0]; const double mil = mi_index[1];
	const double f =_mla.f(MultiFocusPlenopticCamera::type(mik,mil));
 
    const double r = _params.kappa * (D / (D + d)) * (d / 2.) * ((1. / f) - (1. / a) - (1. / d));

    // computing a ray linking the micro-lens center and p
    P3D Ckl_cam = from_coordinate_system_of(_mla.pose(), _mla.node(k,l)); //_mla.nodeInWorld(k,l);
    Ray3D ray;
    ray.config(Ckl_cam, p); // CAMERA

    // testing if the ray hits the main lens
    is_projected = is_projected and hit_main_lens(to_coordinate_system_of(_main_lens.pose(), ray));
    	
    // computing intersection between sensor and ray
    p = line_plane_intersection(_sensor.planeInWorld(), ray); // CAMERA
    p = to_coordinate_system_of(_sensor.pose(), p); // SENSOR

	P2D corner = _sensor.metric2pxl(p).head(2); // IMAGE XY   	
	xy2uv(corner); //IMAGE UV
	
	bap.head(2) = corner;
    bap[2]	= _sensor.metric2pxl(r);

    is_projected = is_projected and hit_the_sensor(bap.head(2));
    
    //TODO: check if point is projected in the correct micro-image

    return is_projected;
}

bool MultiFocusPlenopticCamera::project(
	const P3D& p3d_cam,
    std::size_t k, std::size_t l,
    P2D& pixel
) const
{
	P3D bap;
	bool is_projected = project(p3d_cam, k, l, bap);
	
	pixel = bap.head(2);
	
	return is_projected;
}

bool MultiFocusPlenopticCamera::project(
	const P3D& p3d_cam,
    std::size_t k, std::size_t l,
    double& rho
) const
{
	P3D bap;
	bool is_projected = project(p3d_cam, k, l, bap);
	
	rho = bap[2];
	
	return is_projected;
}

bool MultiFocusPlenopticCamera::project(
	const P3D& p3d_cam,
    BAPObservations& observations
) const
{
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
						bap[0], bap[1], bap[2]	, /* u, v, rho */
					}
				);			
			}
		}
	}
	
	observations.shrink_to_fit();
	return observations.size() > 0u;
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
void MultiFocusPlenopticCamera::uv2xy(P2D& puv) const 
{ 
	puv[0] = sensor().width()-1 - puv[0]; puv[1] = sensor().height()-1 - puv[1];
}
void MultiFocusPlenopticCamera::xy2uv(P2D& pxy) const 
{ 
	pxy[0] = sensor().width()-1 - pxy[0]; pxy[1] = sensor().height()-1 - pxy[1];
}

void MultiFocusPlenopticCamera::mi2ml(P2D& pij) const 
{ 
	pij[0] = mla().width()-1 - pij[0]; pij[1] = mla().height()-1 - pij[1];
}
void MultiFocusPlenopticCamera::ml2mi(P2D& pkl) const 
{ 
	pkl[0] = mla().width()-1 - pkl[0]; pkl[1] = mla().height()-1 - pkl[1];
}

void MultiFocusPlenopticCamera::mi2ml(double& index) const 
{ 
	index = mla().nodeNbr()-1 - index;
}
void MultiFocusPlenopticCamera::ml2mi(double& index) const 
{ 
	index = mla().nodeNbr()-1 - index;
}

template<typename Observations_t>
void MultiFocusPlenopticCamera::mi2ml(Observations_t& obs) const 
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

template void MultiFocusPlenopticCamera::mi2ml(MICObservations& obs) const ;
template void MultiFocusPlenopticCamera::mi2ml(CBObservations& obs) const ;
template void MultiFocusPlenopticCamera::mi2ml(BAPObservations& obs) const ;

template<typename Observations_t>
void MultiFocusPlenopticCamera::ml2mi(Observations_t& obs) const 
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

template void MultiFocusPlenopticCamera::ml2mi(MICObservations& obs) const ;
template void MultiFocusPlenopticCamera::ml2mi(CBObservations& obs) const ;
template void MultiFocusPlenopticCamera::ml2mi(BAPObservations& obs) const ;
	
//******************************************************************************
//******************************************************************************
//******************************************************************************
std::ostream& operator<<(std::ostream& os, const MultiFocusPlenopticCamera::Mode& mode)
{
	switch(mode)
	{
		case MultiFocusPlenopticCamera::Mode::Unfocused: os << "Unfocused (F = D)"; break;
		case MultiFocusPlenopticCamera::Mode::Keplerian: os << "Keplerian (F < D)"; break;
		case MultiFocusPlenopticCamera::Mode::Galilean: os << "Galilean (F > D)"; break;
	}
	return os;
}

std::ostream& operator<<(std::ostream& os, const MultiFocusPlenopticCamera& mfpc)
{	
	os 	<< "{" << std::endl
		<< "\tmode = " << mfpc.mode() << "," << std::endl
		<< "\th = " << mfpc.distance_focus() << "," << std::endl
		<< "\tpose = {" << std::endl << mfpc.pose() << "}," << std::endl
		<< "\tsensor = {" << std::endl << mfpc.sensor() << "}," << std::endl
		<< "\tmia = {" << std::endl << mfpc.mia() << "}," << std::endl
		<< "\tmla = {" << std::endl << mfpc.mla() << "}," << std::endl
		<< "\tlens = {" << std::endl << mfpc.main_lens() << "}," << std::endl
		<< "\tdistortions = {" << std::endl << mfpc.main_lens_distortions() << "}," << std::endl
		<< "\tprincipal point = {" << std::endl << "(" << mfpc.pp().u << ", " << mfpc.pp().v << ")" << std::endl << "}," << std::endl
		<< "\tf = {" << mfpc.mla().f(0) <<", " << mfpc.mla().f(1) << ", " << mfpc.mla().f(2) << "}" << std::endl
		<< "};" ;
		
	return os;
}

void save(std::string path, const MultiFocusPlenopticCamera& pcm)
{
	MultiFocusPlenopticCameraConfig config;
	
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
    config.mia().mesh().orientation() = pcm.mia().orientation();
    
     // Configuring the MicroLensesArray
    config.mla().mesh().pose().rotation() = pcm.mla().pose().rotation();
    config.mla().mesh().pose().translation() = pcm.mla().pose().translation();
    config.mla().mesh().width() = pcm.mla().width();
    config.mla().mesh().height() = pcm.mla().height();
    config.mla().mesh().pitch() = pcm.mla().edge_length();
    config.mla().mesh().geometry() = pcm.mla().geometry();
    config.mla().mesh().orientation() = pcm.mla().orientation();
    
    config.mla().focal_lengths()[0] = pcm.mla().f(0);
    config.mla().focal_lengths()[1] = pcm.mla().f(1);
    config.mla().focal_lengths()[2] = pcm.mla().f(2);

	// Configuring the Main Lens
    config.main_lens().pose().rotation() = pcm.main_lens().pose().rotation();
    config.main_lens().pose().translation() = pcm.main_lens().pose().translation();
    config.main_lens().f() = pcm.main_lens().f();
    config.main_lens().aperture() = pcm.main_lens().aperture();
    config.main_lens().diameter() = pcm.main_lens().diameter();

    // Configuring the Distortions
    config.distortions().radial() = pcm.main_lens_distortions().radial();
    config.distortions().tangential() = pcm.main_lens_distortions().tangential();
    
    // Configuring the Focus distance
    config.dist_focus() = pcm.distance_focus();

	v::save(path, config);
}

void load(std::string path, MultiFocusPlenopticCamera& pcm)
{
	MultiFocusPlenopticCameraConfig config;
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
    pcm.main_lens().f() = config.main_lens().f(); 
    pcm.main_lens().aperture() = config.main_lens().aperture();  

    // Configuring the Distortions
    pcm.main_lens_distortions().radial() = config.distortions().radial(); 
    pcm.main_lens_distortions().tangential() = config.distortions().tangential(); 
    
    // Configuring the Focus distance
    pcm.distance_focus() = config.dist_focus();
}


