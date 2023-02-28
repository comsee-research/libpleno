#include <cstdlib>
#include <iostream>

#include <libv/geometry/ray.hpp>

using namespace v;

template<typename T>
int test_constructor()
{
    Eigen::Matrix<T, 3, 1> direction(36.f, 64.f, 0);
    Eigen::Matrix<T, 3, 1> unit_direction(36.f / 100.f, 64.f / 100.f, 0);

    Ray<T, 3> ray1(direction), ray2(unit_direction);

    if (ray1 != ray2) {
        std::cout << ray1 << std::endl;
        std::cout << ray2 << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

#define LIBV_CHECK(x) if( (x) ) return EXIT_SUCCESS; else { std::cerr << "CTest: Check fails in file " << __FILE__ << " at line " << __LINE__ << std::endl; return EXIT_FAILURE; }


int main()
{

    // constructor test
    {
      Ray3f ray_float;
      Ray3d ray_double;

      LIBV_CHECK(  test_constructor<float >() == EXIT_SUCCESS )
      LIBV_CHECK(  test_constructor<double>() == EXIT_SUCCESS )

    }

    // Scaling test
    {
        Eigen::Vector3d direction(1., 2., 3.);
        Eigen::Vector3d origin   (4., 5., 6.);
        Ray3d ray(direction, origin);

        Ray3d scaledRay = ray * 10.;

        Ray3d check(direction, Eigen::Vector3d(origin.x()*10., origin.y()*10., origin.z()*10) );
        if (scaledRay != check)
        {
            std::cout << scaledRay << std::endl;
            std::cout << check     << std::endl;
            return EXIT_FAILURE;
        }
        LIBV_CHECK( scaledRay == check)
    }

    // Apply a geometric transformation to a ray
    {
        Eigen::Vector3d direction(1., 2., 3.);
        Eigen::Vector3d origin   (4., 5., 6.);
        Ray3d ray(direction, origin);

        Eigen::Transform<double, 3 ,Eigen::AffineCompact> H;
        Eigen::Vector3d translation = Eigen::Vector3d::Random();
        H.translate(translation);
        Ray3d transformedRay = H * ray;
        LIBV_CHECK( transformedRay.direction == ray.direction )

        LIBV_CHECK( transformedRay.origin == ray.origin + translation )

    }

    return EXIT_SUCCESS;

}
