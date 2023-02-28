#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <iostream>
#include <libv/lma/time/tictoc.hpp>
#undef NDEBUG // make assert() work
#include <cassert>

typedef Eigen::MatrixXd Matrix;
typedef Eigen::VectorXd Vector;

Vector llt(Matrix u, Vector x)
{
  for(int i = 0 ; i < u.rows() ; ++i)
  {
    for(int k = 0 ; k < i ; ++k)
      u(i,i) -= u(k,i) * u(k,i);
    
    assert(u(i,i)>0);
    u(i,i) = std::sqrt(u(i,i));
    
    for(int j = i + 1; j < u.cols() ; ++j)
    {
      for(int k = 0 ; k < i ; ++k)
        u(i,j) -= u(k,i) * u(k,j);
      u(i,j) /= u(i,i);
    }
  }

  for(int j = 0 ; j < x.size() ; ++j)
  {
    for(int i = 0 ; i < j ; ++i)
      x(j) -= u(i,j) * x(i);
    x(j) /= u(j,j);
  }
  
  for(int j = x.size() - 1 ; j >=0  ; --j)
  {
    for(int i = j+1 ; i < x.size() ; ++i)
      x(j) -= u(j,i) * x(i);
    x(j) /= u(j,j);
  }

  return x;
}

int main()
{
  size_t n = 10;
  Matrix a(n,n);
  Vector b(n),x(n);

  a = Matrix::Random(n,n);
  a = (a + a.transpose()).eval();

  for(size_t i = 0 ; i < n ; ++i)
  {
    if (a(i,i)<0) a(i,i) = - a(i,i);
    a(i,i) += 10.0;
    x(i) = i+1;
  }
  
  b = a * x;
  
  for(size_t i = 0 ; i < n ; ++i)
    for(size_t j = 0 ; j < n ; ++j)
    {
      if (j<i) a(i,j) = 0;
    }
  std::cout << a << std::endl;
  std::cout << " determinant " << a.determinant() << std::endl;
//   std::cout << "\nb = " << b.transpose() << std::endl;
  
  size_t N = 1000000;
//   size_t N = 1; 
  
  utils::Tic<true> tic("llt");
  for(size_t i = 0 ; i < N; ++i)
    x = llt(a,b);
  tic.disp();
  std::cout << "\nX = " << x.transpose() << std::endl;

  Vector X;
  
  utils::Tic<true> tic2("LLT");
  for(size_t i = 0 ; i < N ; ++i)
  {
    Eigen::LLT<Matrix,Eigen::Upper> LLT(a);
    X = LLT.solve(b);
  }
  tic2.disp();
//   std::cout << "\nX = " << x.transpose() << std::endl;
  
//   std::cout << "A * x = " << (a * x).transpose() << std::endl;
  
//   Matrix L = LLT.matrixL();
//   Matrix U = LLT.matrixU();
//   std::cout << "\nL =\n" << L << std::endl;
//   std::cout << "\nU =\n" << U << std::endl;
  std::cout << "\nX = " << X.transpose() << std::endl;
//   std::cout << " CHECK " << (a*X - b).transpose() << std::endl;
  
  return x == X;
}
