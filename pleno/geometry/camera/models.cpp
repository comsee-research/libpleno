#include "models.h"

#include <cmath>
#include <iostream>

// PinholeCameraModel::PinholeCameraModel(const double fl, const double sk, const double sl, const P2D& ctr)
// : focal(fl), k(sk), l(sl), center(ctr)
// {}

PinholeCameraModel::PinholeCameraModel(const double f, const Sensor& s)
: focal(f), _sensor(s)
{}

PinholeCameraModel::~PinholeCameraModel()
{}

const Sensor& PinholeCameraModel::sensor() const
{
    return _sensor;
}

Sensor& PinholeCameraModel::sensor()
{
    return _sensor;
}

bool PinholeCameraModel::project(const P3D& p3d_cam, P2D& pixel) const
{
    auto hit_the_sensor = [&sensor = sensor()](const P2D& p) {
		return p[0] >= 0.0 and p[1] >= 0.0 and p[0] < sensor.width() and p[1] < sensor.height();
	};
    
    Ray3D ray;
    ray.config(P3D{0.0, 0.0, 0.0}, p3d_cam); // CAMERA

    P3D p3d = line_plane_intersection(sensor().planeInWorld(), ray); // CAMERA
    p3d = to_coordinate_system_of(sensor().pose(), p3d); // SENSOR

    pixel = sensor().metric2pxl(p3d).head(2);

    return hit_the_sensor(pixel);
}

bool PinholeCameraModel::project(const P3DS& p3ds_cam, P2DS& pixels) const
{
    auto hit_the_sensor = [&sensor = sensor()](const P2D& p) {
		return p[0] >= 0.0 and p[1] >= 0.0 and p[0] < sensor.width() and p[1] < sensor.height();
	};
	
    pixels.clear();

    const P3D camera_center {0.0, 0.0, 0.0};
    AlignedVector<Ray3D> rays (p3ds_cam.size());
    for (std::size_t i = 0; i < rays.size(); ++i)
        rays[i].config(camera_center, p3ds_cam[i]); // CAMERA

    P3DS p3ds (p3ds_cam.size());
    for (std::size_t i = 0; i < p3ds.size(); ++i)
        p3ds[i] = line_plane_intersection(sensor().planeInWorld(), rays[i]); // CAMERA

    for (std::size_t i = 0; i < p3ds.size(); ++i)
        p3ds[i] = to_coordinate_system_of(sensor().pose(), p3ds[i]); // SENSOR

    for (std::size_t i = 0; i < p3ds.size(); ++i)
    {
        P2D p = sensor().metric2pxl(p3ds[i]).head(2);
        if (hit_the_sensor(p))
            pixels.push_back(p);
    }
}

/*
 * A ray originating from the camera
**/
bool PinholeCameraModel::raytrace(const P2D& pixel, Ray3D& ray) const
{
    auto hit_the_sensor = [&sensor = sensor()](const P2D& p) {
		return p[0] >= 0.0 and p[1] >= 0.0 and p[0] < sensor.width() and p[1] < sensor.height();
	};
	
    bool is_projected = hit_the_sensor(pixel);

    P3D p_metric = sensor().pxl2metric(P3D{pixel[0], pixel[1], 0.0}); // SENSOR
    p_metric = from_coordinate_system_of(sensor().pose(), p_metric); // CAMERA

    ray.config(p_metric, P3D{0.0, 0.0, 0.0});

    return is_projected;
}

std::ostream& operator<<(std::ostream& o, const PinholeCameraModel& pcm)
{
    // o << "Focal (mm): " << pcm.focal << "\n";
    // o << "Scale (k, l) (pixels/mm): " << pcm.k << ", " << pcm.l << "\n";
    // o << "Skew: " << pcm.skew << "\n";
    // o << "Center (pixels): " << pcm.center.transpose();

    return o;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

double ThinLensCameraModel::diameter() const
{
    return std::fabs(this->f()) / this->aperture();
}

/*
 *  @Brief projection through thinLens using the thin lens camera model
 *  p3d in camera coordinates system
 *  returns p3d
**/
bool ThinLensCameraModel::project(const P3D& p3d_cam, P3D& p3d_out) const
{
    assert(this->diameter() >= 0.0 && "ThinLensCameraModel::project: diameter >= 0.0");

    //the distance according to z axis
    const double dist2pt = std::fabs(p3d_cam[2]);

    if (dist2pt == 0.0)
    {
        std::cerr << "Warning: ThinLensCameraModel::project: "
                  << "the point is on the lens plane!" << std::endl;
        return false;
    }

    if (dist2pt == this->f())
    {
        std::cerr << "Warning: ThinLensCameraModel::project: "
                  << "the point is on the focal plane!" << std::endl;
        return false;
    }

    // projecting the point
    p3d_out = -this->f() / (dist2pt - this->f()) * p3d_cam;

    return true;
}

/*
 *  @Brief projection through thinLens using the thin lens camera model
 *  entry: vector of p3d in camera coordinates system
 *  output: vector of p3d in camera coordinates system
**/
bool ThinLensCameraModel::project(const P3DS& p3ds_cam, P3DS& p3ds_out) const
{
    p3ds_out.clear();
    assert(this->diameter() >= 0.0 && "ThinLensCameraModel::project: diameter >= 0.0");

    // the distance according to z axis
    std::vector<double> dist2pt (p3ds_cam.size());
    for (size_t i = 0; i < dist2pt.size(); ++i)
        dist2pt[i] = std::abs(p3ds_cam[i][2]);

    std::vector<bool> valids (dist2pt.size());
    for (size_t i = 0; i < dist2pt.size(); ++i)
        if (dist2pt[i] != 0.0 and dist2pt[i] != this->f())
            valids[i] = true;
        else
            valids[i] = false;

    for (size_t i = 0; i < valids.size(); ++i)
        if (valids[i])
            p3ds_out.push_back(p3ds_cam[i]);

    // projecting the point
    std::vector<double> values (valids.size());
    for (size_t i = 0; i < values.size(); ++i)
        if (valids[i])
            values[i] = -this->f() / (dist2pt[i] - this->f());

    for (size_t p = 0; p < p3ds_out.size(); ++p)
        p3ds_out[p] =  values[p] * p3ds_out[p];

    return p3ds_out.size() != 0 ? true : false;
}

/*
 *  @Brief raytrace through thinLens using the thin lens camera model
 *  ray in camera coordinate system
 *  ray originating from the camera
**/
bool ThinLensCameraModel::raytrace(const Ray3D& ray_in, Ray3D& ray_out) const
{
    auto is_on_disk = [](const P2D& p, double disk_diameter) {
    	return p.norm() <= disk_diameter / 2.0 ;
    };
    
    // project the origin of the ray_in using the ThinLensCameraModel
    P3D projected_point;
    if ( this->project(ray_in.origin(), projected_point) )
    {
        // compute the intersection point between the ray and the lens
        ray_out.origin() = line_plane_intersection(Eigen::Vector4d{0.0, 0.0, 1.0, 0.0}, ray_in);

        // Testing if the ray hit the lens
        if ( is_on_disk(ray_out.origin().head(2), this->diameter()) )
        // if ( Disk{P2D{0,0}, this->diameter() / 2.0}.is_inside(ray_out.origin.head(2)) )
        {
            ray_out.config(ray_out.origin(), projected_point);
            return true;
        }
        else
        {
            // std::cout << "Warning: ThinLensCameraModel::raytrace: "
            //           << "the ray does not hit the lens!" << std::endl;
            return false;
        }
    }

    return false;
}

/*
 *  @Brief raytrace through thinLens using the thin lens camera model
 *  ray in camera coordinate system
 *  ray originating from the camera
**/
bool ThinLensCameraModel::raytrace(const Rays3D& rays_in,
                                         Rays3D& rays_out) const
{
    auto is_on_disk = [](const P2D& p, double disk_diameter) {
    	return p.norm() <= disk_diameter / 2.0 ;
    };
    rays_out.clear();

    const Eigen::Vector4d lens_plane_equation {0.0, 0.0, 1.0, 0.0};

    for (const auto& r : rays_in)
    {
        // project the origin of the rays_in using the ThinLensCameraModel
        P3D projected_point;
        if ( this->project(r.origin(), projected_point) )
        {
            Ray3D ray;

            ray.origin() = line_plane_intersection(lens_plane_equation, r);

            // Testing if the ray hit the lens
            if (is_on_disk(ray.origin().head(2), this->diameter()))
            {
                ray.config(ray.origin(), projected_point);

                rays_out.push_back(ray);
            }
        }
    }

    return rays_out.size() != 0 ? true : false;
}

std::ostream& operator<<(std::ostream& o, const ThinLensCameraModel& tcm)
{
    o << "Focal (mm): " << tcm.f() << "\n";
    o << "Diameter (mm): " << tcm.diameter() << "\n";
    o << "Pose: " << tcm.pose();

    return o;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
OrthographicCameraModel::OrthographicCameraModel(const double f, const Sensor& s)
: focal(f), _sensor(s)
{}

OrthographicCameraModel::~OrthographicCameraModel(){}

bool OrthographicCameraModel::project(const P3D& p3d_cam, P2D& pixel) const
{
    // bool is_visible = is_p3d_visible(p3d_cam);
    //TODO do we do that ???? think about the galilean mode

    //projecting the point
    // pixel[0] = (focal / k) * p3d_cam[0] / p3d_cam[2] + center[0];
    // pixel[1] = (focal / l) * p3d_cam[1] / p3d_cam[2] + center[1];

    // return is_visible;
    return false;
}

/*
 * A ray originating from the camera
**/
bool OrthographicCameraModel::raytrace(const P2D& pixel, Ray3D& ray) const
{
    auto hit_the_sensor = [&sensor = sensor()](const P2D& p) {
		return p[0] >= 0.0 and p[1] >= 0.0 and p[0] < sensor.width() and p[1] < sensor.height();
	};
	
	bool is_projected = hit_the_sensor(pixel);

    P3D p_metric = sensor().pxl2metric(P3D{pixel[0], pixel[1], 0.0}); // SENSOR
    p_metric = from_coordinate_system_of(sensor().pose(), p_metric); // CAMERA

    P3D out_point = p_metric;
    out_point[2] += 1.0;

    ray.config(p_metric, out_point);

    return is_projected;
}

const Sensor& OrthographicCameraModel::sensor() const
{
    return _sensor;
}

Sensor& OrthographicCameraModel::sensor()
{
    return _sensor;
}

std::ostream& operator<<(std::ostream& o, const OrthographicCameraModel& ocm)
{
    o << "Focal (mm): " << ocm.focal << "\n";
    o << "Sensor : " << ocm.sensor() << "\n";

    return o;
}
