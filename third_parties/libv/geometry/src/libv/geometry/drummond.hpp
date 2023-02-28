#ifndef __LIBV_GEOMETRY_DRUMMOND_HPP__
#define __LIBV_GEOMETRY_DRUMMOND_HPP__

#include <array>
#include <vector>
#include <Eigen/Core>
#include <libv/geometry/pose.hpp>
#include <libv/geometry/camera_model.hpp>
#include <libv/geometry/global.hpp>

namespace v {
namespace geometry {

class LIBV_GEOMETRY_EXPORT DrummondEssentialEstimation
{
  static const int Dynamic = -1;
  template<int I, int J> using Mat = Eigen::Matrix<double,I,J>;

public:
  typedef Mat<3,Dynamic> Matrix3dt;
  typedef Mat<Dynamic,3> Matrixd3t;
  typedef Mat<Dynamic,2> Matrixd2t;
  typedef Mat<Dynamic,5> Matrixd5t;
  typedef Mat<3,3>       Matrix3t;
  typedef Mat<Dynamic,1> VectorXt;
  typedef Mat<3,1>       Vector3t;

  DrummondEssentialEstimation(double,int);
  DrummondEssentialEstimation& Estimate(
        const Matrixd2t & matches_1,
        const Matrixd2t & matches_2,
        const v::UnifiedCameraModel & cam,
        int sample_size=5,
        int ransac_max_iter=500,
        double lambda=pow(10,-3));
  Matrix3t GetR() const {return R;}
  Vector3t GetT() const {return T;}
  Matrix3t GetE() const {return E;}

private:
  std::array<Matrix3t,3> G;//generators of so(3) (the lie algebra of SO(3))
  Matrix3t R;
  Matrix3t K;
  Vector3t T;
  Matrix3t E;
  double lm_thresh;
  size_t lm_max_iter;

  void IterativeEstimation(
         const std::vector<int> &,
         const Matrix3dt &,
         const Matrix3dt &,
         const Matrixd3t &,
         const Matrixd3t &,
         Matrix3t &,
         Matrix3t &,
         double);
  double ErrorFunction(const Matrix3dt &,const Matrix3dt &, VectorXt &) const;
  double EpipolarError(const Matrixd3t &,const Matrixd3t &, const Matrix3t& R1, const Matrix3t& R2) const;
  void ComputeJacobian(const Matrix3dt &,const Matrix3dt &, Matrixd5t &);
  double phi(double s) const;
  double atan2_approx(double y,double x) const;

};

}} // namespaces

#endif
