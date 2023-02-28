#ifdef USE_CERES

#include "ceres/rotation.h"
#include "ceres/jet.h"
#include <Eigen/Dense>
#include <TooN/TooN.h>
#include <cmath>
#include <libv/lma/color/console.hpp>
#include <libv/lma/string/string_utiliy.hpp>
#include <libv/lma/time/tictoc.hpp>
#include <libv/lma/numeric/ad/rt/ad.hpp>
#include <libv/lma/numeric/ad/ct/adct.hpp>
#include <typeinfo>

  typedef Eigen::Matrix<double,9,1> Camera;
  typedef Eigen::Matrix<double,3,1> Point3d;
  typedef Eigen::Matrix<double,2,1> Point2d;
  
  template<class T>
  void analytical_derivative(const Camera& camera_, const Point3d& point_, const Eigen::Vector2d& obs, Eigen::Matrix<double,2,12>& j)
  {
    const T camera[3] = {T(camera_[0],0),T(camera_[1],1),T(camera_[2],2)};
    const T point[3] = {T(point_[0],9),T(point_[1],10),T(point_[2],11)};
    T p[3];
    ceres::AngleAxisRotatePoint(camera, point, p);
    p[0] += T(camera_[3],3);
    p[1] += T(camera_[4],4);
    p[2] += T(camera_[5],5);
    auto xp = - p[0] / p[2];
    auto yp = - p[1] / p[2];
    const T l1(camera_[7],7);
    const T l2(camera_[8],8);
    auto r2 = xp*xp + yp*yp;
    auto distortion = T(1.0) + r2  * (l1 + l2  * r2);
    T focal(camera_[6],6);
    auto jacobx = focal * distortion * xp - T(obs.x());
    auto jacoby = focal * distortion * yp - T(obs.y());
    for(int i = 0 ; i < 12 ; ++i)
    {
      j(0,i) = jacobx.v[i];
      j(1,i) = jacoby.v[i];
    }
  }

  template<class T>
  void analytical_derivative2(const Camera& camera_, const Point3d& point_, const Point2d& obs, TooN::Matrix<2,12,double>& j)
  {
    typedef T Scalar;
    const std::array<T,9> c = {
      T(camera_[0],0),T(camera_[1],1),T(camera_[2],2),
      T(camera_[3],3),T(camera_[4],4),T(camera_[5],5),
      T(camera_[6],6),T(camera_[7],7),T(camera_[8],8)};

    const std::array<T,3> pt = {T(point_[0],9),T(point_[1],10),T(point_[2],11)};
    
    std::array<T,2> p;
    
    const T theta2 = c[0]*c[0] + c[1]*c[1] + c[2]*c[2];
    if (theta2 > Scalar(std::numeric_limits<double>::epsilon()))
    {
      const T
        theta = sqrt(theta2),
        costheta = cos(theta),
        sintheta = sin(theta),
        theta_inverse = 1.0 / theta,
        w[3] = { c[0] * theta_inverse, c[1] * theta_inverse, c[2] * theta_inverse },
        tmp = (w[0] * pt[0] + w[1] * pt[1] + w[2] * pt[2]) * (Scalar(1.0) - costheta),
        p2 =    pt[2] * costheta + (w[0] * pt[1] - w[1] * pt[0]) * sintheta + w[2] * tmp + c[5];
      p[0] = - (pt[0] * costheta + (w[1] * pt[2] - w[2] * pt[1]) * sintheta + w[0] * tmp + c[3]) / p2;
      p[1] = - (pt[1] * costheta + (w[2] * pt[0] - w[0] * pt[2]) * sintheta + w[1] * tmp + c[4]) / p2;
    }
    else
    {
      const T p2 = pt[2] + c[0] * pt[1] - c[1] * pt[0] + c[5];
      p[0] = - (pt[0] + c[1] * pt[2] - c[2] * pt[1] + c[3]) / p2;
      p[1] = - (pt[1] + c[2] * pt[0] - c[0] * pt[2] + c[4]) / p2;
    }
    
    const T
      r2 = p[0]*p[0] + p[1]*p[1],
      fx = c[6] * (Scalar(1.0) + r2  * (c[7] + c[8]  * r2));
      
    const typename T::Array
      jacobx = (fx * p[0] - Scalar(obs.x())).infinite(),
      jacoby = (fx * p[1] - Scalar(obs.y())).infinite();

    for(int i = 0 ; i < 12 ; ++i)
    {
      j(0,i) = jacobx[i];
      j(1,i) = jacoby[i];
    }
  }
  
  template<class T> auto f(const T& a, const T& b, const T& c)
  {
    return - a * 2.5 * b * a * c * a - 2.0 * a * b * a * b / 10.0 + a /2.0 + a * sqrt(b * c * cos( sin( b * b + c) / a * c / 5.0));
  }
  
  inline void test_adrt(double x, double y, double z, double d[3], double& r)
  {
    AdRt::Ad<double,3> a(x,0),b(y,1),c(z,2);
    AdRt::Ad<double,3> derivatives = f(a,b,c);
    d[0] = derivatives.infinite[0]; d[1] = derivatives.infinite[1]; d[2] = derivatives.infinite[2];
    r = derivatives.value;
  }
  
  inline void test_adct(double x, double y, double z, double d[3], double& r)
  {
    adct::Ad<double,3> a(x,0),b(y,1),c(z,2);
    auto expr = f(a,b,c);
    auto derivatives = expr.infinite();
    d[0] = derivatives[0];d[1] = derivatives[1];d[2] = derivatives[2];
    r = expr.value();
  }
  
  inline void test_jet(double x, double y, double z, double d[3], double& r)
  {
    ceres::Jet<double,3> a(x,0),b(y,1),c(z,2);
    auto derivatives = f(a,b,c);
    d[0] = derivatives.v[0];d[1] = derivatives.v[1];d[2] = derivatives.v[2];
    r = derivatives.a;
  }
  


int main()
{
  std::cout << std::endl << std::endl;
  double d[3] = {0,0,0};
  double r = 0;
  
  size_t N = 1000000;
  
  utils::Tic<true> ticAdrt("Adrt");
  for(size_t i = 0 ; i < N; ++i)
    test_adrt(i,i/2,i/3,d,r);
  ticAdrt.disp();
  
  std::cout << color.red() << " adrt dx,dy,dz = " << d[0] << "," << d[1] << "," << d[2] << "," << r << color.reset() << std::endl;
  d[0] = 0; d[1] = 0, d[2] = 0;
  r = 0; 
  
  utils::Tic<true> ticAd("AdCt");
  for(size_t i = 0 ; i < N; ++i)
    test_adct(i,i/2,i/3,d,r);
  ticAd.disp();
  
  std::cout << color.red() << " adct dx,dy,dz = " << d[0] << "," << d[1] << "," << d[2] << "," << r << color.reset() << std::endl;
  
  d[0] = 0; d[1] = 0, d[2] = 0;
  r = 0;
  utils::Tic<true> ticjet("jet");
  for(size_t i = 0 ; i < N; ++i)
    test_jet(i,i/2,i/3,d,r);
  ticjet.disp();
  
  
  std::cout << color.red() << " jet  dx,dy,dz = " << d[0] << "," << d[1] << "," << d[2] << "," << r <<  color.reset() << std::endl;
  

  Camera cam; cam << 0,0,0,10,10,10,10,10,10;
  Point3d pt; pt << 100,200,150;
  Point2d obs; obs << 100,100;
  

  
  {
    Eigen::Matrix<double,2,12> jacob;
    utils::Tic<true> jet_proj("Jet Proj");
    for(size_t i = 0 ; i < N; ++i)
      analytical_derivative<ceres::Jet<double,12>>(cam,pt,obs,jacob);
    jet_proj.disp();
    std::cout << jacob << std::endl;
  }
  
  {
    TooN::Matrix<2,12,double> jacob;
    utils::Tic<true> ad_proj("Ad Proj");
    for(size_t i = 0 ; i < N; ++i)
      analytical_derivative2<adct::Ad<double,12>>(cam,pt,obs,jacob);
    ad_proj.disp();
    std::cout << jacob << std::endl;
  }
//   
}

#else // #ifdef USE_CERES

int main()
{
}

#endif // #ifdef USE_CERES

