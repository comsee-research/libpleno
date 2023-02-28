#include <vector>
#include <algorithm>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <libv/geometry/rotation.hpp>
#include <libv/geometry/drummond.hpp>

using namespace v::geometry;

DrummondEssentialEstimation::DrummondEssentialEstimation(double lm_thresh_, int lm_max_iter_):
  lm_thresh(lm_thresh_),
  lm_max_iter(lm_max_iter_)
{
  G[0]<<
        0,0,0,
        0,0,-1,
        0,1,0;
  G[1]<<
        0,0,1,
        0,0,0,
        -1,0,0;
  G[2]<<
        0,-1,0,
        1,0,0,
        0,0,0;
}

DrummondEssentialEstimation& DrummondEssentialEstimation::Estimate(
                             const Matrixd2t & matches_1,
                             const Matrixd2t & matches_2,
                             const v::UnifiedCameraModel & model,
                             int sample_size,
                             int ransac_max_iter,
                             double lambda)
{

// TODO
// utiliser camera.model.project() pour remplir m1_3d et m2_3d
// là on ne tient pas compte des distortions et le code est dupliqué
// ↓
  // Matrix of intrinsic parameters
  K <<
      model.focal[0], 0, model.center[0],
      0, model.focal[1], model.center[1],
      0,0,1;
  
  Matrix3t invK=K.inverse();

  Matrixd3t m1_2d(matches_1.rows(),3);
  Matrixd3t m2_2d(matches_1.rows(),3);

  m1_2d << matches_1, VectorXt::Ones(matches_1.rows());
  m2_2d << matches_2, VectorXt::Ones(matches_1.rows());

  Matrix3dt m1_3d=invK*m1_2d.transpose();
  Matrix3dt m2_3d=invK*m2_2d.transpose();

// ↑
  for(int i=0;i<m1_3d.cols();++i)
  {
    m1_3d.col(i).normalize();
    m2_3d.col(i).normalize();
  }

  Matrix3t R1=Matrix3t::Identity();
  Matrix3t R2=Matrix3t::Identity();

  //RANSAC
  std::vector<int> inliers;
  int num_inliers=-1;//TODO inliers.size()

  for(int i=0;i<ransac_max_iter;++i)
  {
    std::vector<int> cur_sample;
    std::vector<int> cur_inliers;
    for(int j=0;j<m1_3d.cols();++j)
      cur_sample.push_back(j);
    std::random_shuffle(cur_sample.begin(),cur_sample.end());

    cur_sample.erase(cur_sample.begin()+sample_size,cur_sample.end());
    IterativeEstimation(cur_sample,m1_3d,m2_3d,m1_2d,m2_2d,R1,R2,lambda);
    Matrix3t E=R2.transpose()*G[2]*R1;
    Matrix3t F=(K.transpose()).inverse() * E * K.inverse();

    for(int j=0;j<m1_2d.rows();++j)
    {
      auto x1=m1_2d.row(j).transpose();
      auto x2=m2_2d.row(j).transpose();
      Vector3t d_epipolar = F * x1;
      double err = std::abs(d_epipolar(0) * x2(0) + d_epipolar(1) * x2(1) + d_epipolar(2))/ d_epipolar.head<2>().norm();

      if(err < 2.)
       cur_inliers.push_back(j);
    }

    if(int(cur_inliers.size())>num_inliers)
    {
      num_inliers=cur_inliers.size();
      inliers=cur_inliers;
    }

  }

  //std::cerr<<"num_inliers_found=="<<inliers.size() <<" / " << matches_1.rows() <<std::endl;

  if(inliers.size() != 0)
  {
      IterativeEstimation(inliers,m1_3d,m2_3d,m1_2d,m2_2d,R1,R2,lambda);
      //Matrix3t R=R1.transpose()*R2;
      E=R2.transpose()*G[2]*R1;
      R=R2.transpose()*R1;
      Matrix3t Tx=E*R.transpose();

      T << Tx(2,1), Tx(0,2), Tx(1,0);
  }
  else
  {
      std::cerr << "aucun inlier !!!" << std::endl;
      R.setIdentity();
      T.setZero();
      E.setIdentity();
  }
  return *this;
}

void DrummondEssentialEstimation::IterativeEstimation(const std::vector<int> & inliers,
                             const Matrix3dt & m1,
                             const Matrix3dt & m2,
                             const Matrixd3t & p1,
                             const Matrixd3t & p2,
                             Matrix3t & R1,
                             Matrix3t & R2,
                             double lambda)
{

  Matrixd3t obs_1(inliers.size(),3);
  Matrixd3t obs_2(inliers.size(),3);

  Matrix3dt P1(3,inliers.size());
  Matrix3dt P2(3,inliers.size());
  for(unsigned int i=0;i<inliers.size();++i)
  {
    P1.col(i)=m1.col(inliers[i]);
    P2.col(i)=m2.col(inliers[i]);
    obs_1.row(i)=p1.row(inliers[i]);
    obs_2.row(i)=p2.row(inliers[i]);
  }

  R1.setIdentity();
  R2.setIdentity();
  Matrix3t prev_R1=R1;
  Matrix3t prev_R2=R2;

  Matrix3dt R1P1=R1*P1;
  Matrix3dt R2P2=R2*P2;
  VectorXt err_vec = VectorXt::Zero(R1P1.cols(),1);
  VectorXt prev_err_vec=err_vec;

  Matrixd5t J = Matrixd5t::Zero(P1.cols(),5);
  Mat<5,5> M = Mat<5,5>::Zero();
  Mat<5,5> W = Mat<5,5>::Zero();
  Mat<5,1> delta = Mat<5,1>::Zero();

  ErrorFunction(R1P1,R2P2,err_vec);
  double prev_nerr =  EpipolarError( obs_1, obs_2, R1, R2);

  //pour debug
  int reject = 0;
  int accept = 0;

  for(size_t num_it = 0; num_it<lm_max_iter; num_it++)
  {
    // store the previous state
    prev_R1=R1;
    prev_R2=R2;
    prev_err_vec=err_vec;

    // solve the system
    ComputeJacobian(R1P1,R2P2,J);
    M=J.transpose()*J+lambda*Mat<5,5>::Identity();
    W=M.transpose()*M;
    delta=W.inverse()*M.transpose()*(-J.transpose()*err_vec);

    // update the R1 and R2
    v::apply_rotation(R1, {delta[0], delta[1], delta[2]});
    v::apply_rotation(R2, {delta[3], delta[4], 0.});

    // compute the current error
    ErrorFunction(R1P1,R2P2,err_vec);
    double nerr =  EpipolarError( obs_1, obs_2, R1, R2);

    //
    if(prev_nerr<nerr)
    {
      R1=prev_R1;
      R2=prev_R2;
      err_vec=prev_err_vec;
      lambda*=10;
      if (lambda > 1e3)
        break;
      reject++;
    }
    else
    {
      lambda/=10;
      if(num_it + 1 < lm_max_iter)
      {
        R1P1=R1*P1;
        R2P2=R2*P2;
        prev_nerr = nerr;
        ErrorFunction(R1P1,R2P2,err_vec);
      }
      accept++;
    }

    if((delta.norm()<1e-10) || (nerr<lm_thresh))
    {
      break;
    }
  }

}

double DrummondEssentialEstimation::phi(double s) const//approximation for sin [-pi/2,pi/2]
{
  if(s>-1 && s<1)
    return ((M_PI/4.0)*s+0.273*s*(1.0-std::abs(s)));
  else if(s>1)
    return M_PI/2.0-((M_PI/4.0)*(1/s)+0.273*(1/s)*(1.0-std::abs(1/s)));
  else if(s<-1)
    return -phi(std::abs(s));
  return -1;
}

double DrummondEssentialEstimation::atan2_approx(double y,double x) const
{
  if(x<0 && y>0)
    return M_PI-phi(std::abs(y/x));
  else if(x<0 && y<0)
    return phi(std::abs(y/x))-M_PI;
  else
    return phi(y/x);
}

double DrummondEssentialEstimation::ErrorFunction(const Matrix3dt & R1P1,const Matrix3dt & R2P2, VectorXt & err_vec) const
{
  double err=0;
  for(int i=0;i<R1P1.cols();++i)
  {
    double err_i=atan2_approx(R1P1(1,i),R1P1(0,i))-atan2_approx(R2P2(1,i),R2P2(0,i));//TODO ramener dans [-π,π]

    while(err_i > M_PI)
        err_i -= 2 * M_PI;
    while(err_i < -M_PI)
        err_i += 2 * M_PI;

    err+=(err_i*err_i);
    err_vec(i)=err_i;
  }
  return sqrt(err);
}

double DrummondEssentialEstimation::EpipolarError(const Matrixd3t & obs_1,const Matrixd3t & obs_2, const Matrix3t & R1, const Matrix3t & R2) const
{
  // Calcul de la matrice essentielle et fondamontale
  Matrix3t E=R2.transpose()*G[2]*R1;
  Matrix3t F=(K.transpose()).inverse() * E * K.inverse();

  // Calcul de l'erreur epipolaire totale
  double err = 0.;
  for(int j=0;j<obs_1.rows();++j)
  {
    auto x1=obs_1.row(j).transpose();
    auto x2=obs_2.row(j).transpose();
    Vector3t d_epipolar = F * x1;
    err += std::abs(d_epipolar(0) * x2(0) + d_epipolar(1) * x2(1) + d_epipolar(2)) / d_epipolar.head<2>().norm();
  }
  return err;
}

void DrummondEssentialEstimation::ComputeJacobian(const Matrix3dt & P1,const Matrix3dt & P2,Matrixd5t & J)//ATTENTION!R1*P1 and R2*P2 should be given to this function
{
  for(int i=0;i<P1.cols();++i)
  {
    const double 
      c1= -P1(1,i) / (P1(0,i)*P1(0,i) + P1(1,i)*P1(1,i)),
      c2=  P1(0,i) / (P1(0,i)*P1(0,i) + P1(1,i)*P1(1,i)),
      c3= -P2(1,i) / (P2(0,i)*P2(0,i) + P2(1,i)*P2(1,i)),
      c4=  P2(0,i) / (P2(0,i)*P2(0,i) + P2(1,i)*P2(1,i));

    for (int j=0;j<3;++j)
    {
      Vector3t GjP1i=G[j]*P1.col(i);
      J(i,j)=c1*(GjP1i.x())+c2*(GjP1i.y());
    }
    for (int j=3;j<5;++j)
    {
      Vector3t GjP2i=G[j-3]*P2.col(i);
      J(i,j)=-(c3*(GjP2i.x())+c4*(GjP2i.y()));
    }
  }
}

