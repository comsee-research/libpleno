
#include <libv/lma/lma.hpp>


  using Vector4d = Eigen::Matrix<double,4,1>;
  using Vector3d = Eigen::Vector3d;
  using Vector2d = Eigen::Vector2d;


  struct F
  {
    bool operator()(const Vector4d& pose, const Vector3d& p3d, Vector2d& error) const
    {
      error[0] = pose.norm() + p3d.norm();
      error[1] = pose.squaredNorm() + p3d.squaredNorm();
      return true;
    }
  };

  struct Verbose : lma::enable_verbose_output//lma::default_callbacks_for_solver 
  {
    Eigen::MatrixXd& mat;
    Verbose(Eigen::MatrixXd& mat_):mat(mat_){}
    
    template<typename Solver, typename EquationNormal>
    void at_end_bundle_adjustment(const Solver& solver, const EquationNormal& eq)
    {
      lma::enable_verbose_output::at_end_bundle_adjustment(solver,eq);
//      std::cout << " SPARSE CONTAINER: " << eq.ba_.h << std::endl;
      using Ba = typename EquationNormal::Ba;
      using Keys = typename Ba::Keys;
      using MatrixTag = typename EquationNormal::MatrixTag;
      mat = lma::to_mat<MatrixTag,Keys,typename Ba::Hessian>(eq.ba_.h,lma::size_tuple<lma::mpl::size<Keys>::value>(eq.ba_.delta));
    }
  };
  
  
int main()
{
  Vector4d pose1 = Vector4d::Random();
  Vector4d pose2 = Vector4d::Random();
  Vector3d p1 = Vector3d::Random();
  Vector3d p2 = Vector3d::Random();
  Vector3d p3 = Vector3d::Random();

  Eigen::MatrixXd mat;
	lma::Solver<F>().add(F(),&pose1,&p1).add(F(),&pose1,&p2).add(F(),&pose2,&p2).add(F(),&pose2,&p3).solve(lma::DENSE_SCHUR,Verbose(mat));
	
  std::cout << " DENSE MATRIX : \n" << mat << std::endl;
}
