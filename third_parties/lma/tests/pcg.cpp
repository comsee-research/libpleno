#include <iostream>
#include <Eigen/IterativeLinearSolvers>

#include <libv/lma/lm/solver/solver.hpp>
#include <libv/lma/lm/solver/verbose.hpp>

using namespace Eigen;

void eigen_pcg(const Eigen::MatrixXd& m, const Eigen::VectorXd& jte)
{
  
  std::cout << "\n\n EIGEN PCG " << std::endl;
  size_t n = jte.size();
  VectorXd x(n), b(jte);
//   typedef SparseMatrix<double> Mat;
  typedef Eigen::MatrixXd Mat;
//   Mat A(n,n);
  Mat A = m;
//   for(size_t i = 0 ; i < m.cols() ; ++i)
//     for(size_t j = 0 ; j < m.rows() ; ++j)
//       A.coeff(j,i) = m(j,i);
      
//   for(size_t i = 0 ; i < n ; ++i)
//     A.coeffRef(i,i) = 1.0;
//   A.setIdentity();
  std::cout << A << std::endl;
  x.setZero();
  std::cout <<"\n X = " <<  x.transpose() << std::endl;
  std::cout <<"\n B = " <<  b.transpose() << std::endl;
  // fill A and b
  ConjugateGradient<Mat> cg;
  cg.compute(A);
  x = cg.solve(b);
  std::cout << "#iterations:     " << cg.iterations() << std::endl;
  std::cout << "estimated error: " << cg.error()      << std::endl;
  // update b, and solve again
//   x = cg.solve(b);
  std::cout << "\n X = " << x.transpose() << std::endl;
}

// typedef Eigen::Matrix<double,2,1> Type0;
// typedef Eigen::Matrix<double,3,1> Type1;
// typedef Eigen::Matrix<double,1,1> Type2;
// 
// struct F
// {
//   bool operator()(const Type0& , const Type1& , const Type2&, Eigen::Matrix<double,2,1>&) const
//   {
//     return true;
//   }
// };

int main()
{
  /*
  Type0 t0;
  Type1 t1;
  Type2 t2;
  std::cout << " Test pcg " << std::endl;
  lma::Solver<F> solver(1,1);
  
//   solver.algo.norm_eq.seuil=0.9999;
//   solver.algo.norm_eq.max_iteration=100000typedef typename SelectAlgo<Container,Container::NbClass,AlgoTag>::type Algorithm;
//       Algorithm algo(config);;
  
  auto i0 = ttt::Indice<Type0*>(0);
  auto i1 = ttt::Indice<Type1*>(0);
  auto i2 = ttt::Indice<Type2*>(0);
  solver.add(F(),&t0,&t1,&t2);//lma::bf::make_vector(i0,i1,i2),F());
//   solver.solve(lma::enable_verbose_output());
//   solver.algo.compute_b(solver.algo.ba_);
//   solver.algo.compute_delta_a(solver.algo.ba_);
  using namespace lma;
  typedef typename SelectAlgo<lma::Solver<F>::Container,lma::Solver<F>::Container::NbClass,ImplicitSchurTag<1>>::type Algorithm;
  Algorithm algo(ImplicitSchurTag<1>(0.9999,100000));
  algo.init(solver.bundle);
  
  lma::bf::at_key<lma::bf::pair<Type0*,Type0*>>(algo.ba_.h())(i0,0) << 
    1,0,
    0,2;
    
  lma::bf::at_key<lma::bf::pair<Type2*,Type2*>>(algo.ba_.h())(i2,0) << 
    1;
  
  lma::bf::at_key<lma::bf::pair<Type0*,Type1*>>(algo.ba_.h())(i0,0) << 
    3.2,0.5,2,
    0,-2.5,-1.5;
    
  lma::bf::at_key<lma::bf::pair<Type1*,Type1*>>(algo.ba_.h())(i1,0) << 
    3,0,0,
    0,4,0,
    0,0,5;
    
  lma::bf::at_key<Type0*>(algo.ba_.jte())(i0) << 1,2;
  lma::bf::at_key<Type1*>(algo.ba_.jte())(i1) << 3,4,5;
  lma::bf::at_key<Type2*>(algo.ba_.jte())(i2) << 1;
  
  
  
  std::cout << std::endl;
  std::cout << " A = \n" << lma::to_mat(algo.ba_.h()) << std::endl;
//   std::cout << " B = " << lma::to_vect(solver.algo.schur_.bs_).transpose() << std::endl;
  std::cout << " B = " << lma::to_vect(algo.ba_.jte()).transpose() << std::endl;
  
  algo.compute_y(algo.ba_);
  algo.compute_b(algo.ba_);
  algo.compute_delta_a(algo.ba_);
  
  std::cout << " X = " << lma::to_vect(algo.ba_.delta()).transpose() << std::endl;
  
  eigen_pcg(lma::to_mat(algo.ba_.h()),lma::to_vect(algo.ba_.jte()));*/
}

