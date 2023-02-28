#include <Eigen/Core>
#include <Eigen/Sparse>
#include <iostream>
#include <libv/lma/time/tictoc.hpp>
template<class T> using AlignedVector = std::vector<T,Eigen::aligned_allocator<T>>;

/*
  dense
  crs
  bcrs
  sbcrs
*/

Eigen::SparseMatrix<double> dense_to_sparse(const Eigen::MatrixXd& m)
{
  typedef Eigen::Triplet<double> Triplet;

  std::vector<Triplet> tripletList;

  for(int i = 0 ; i < m.rows() ; ++i)
    for(int j = 0 ; j < m.cols() ; ++j)
      if (m(i,j)!=0)
        tripletList.emplace_back(i,j,m(i,j));

  Eigen::SparseMatrix<double> s(m.rows(),m.cols());
  s.setFromTriplets(tripletList.begin(), tripletList.end());
  return s;
}

struct Dense
{
  Eigen::MatrixXd m;
  Eigen::VectorXd r,v;

  Dense(int rows, int cols, int block_size):m(rows,cols),r(rows),v(rows)
  {
    for(int i = 0 ; i < rows/block_size ; ++i)
      m.block(i*block_size,i*block_size,block_size,block_size) = Eigen::MatrixXd::Random(block_size,block_size);
    v = v.Random(rows);
  }
};

struct Sparse
{
  Eigen::SparseMatrix<double> m;
  Eigen::SparseVector<double> r,v;

  Sparse(const Dense& dense)
  {
     m = dense_to_sparse(dense.m);
     v = dense_to_sparse(dense.v);
  }
};

template<class Storage> void compute(Storage& st)
{
  st.r = st.m * st.v;
}

template<int BlockSize> struct Blocks
{
  AlignedVector<Eigen::Matrix<double,BlockSize,BlockSize>> m;
  AlignedVector<Eigen::Matrix<double,BlockSize,1>> r,v;

  Blocks(const Dense& dense)
  {
    for(int i = 0 ; i < dense.m.rows()/BlockSize ; ++i)
    {
      m.push_back(dense.m.block<BlockSize,BlockSize>(i,i));
      v.push_back(dense.v.block<BlockSize,1>(i,0));
    }
    r.resize(v.size());
  }

  void eval(int i)
  {
    r[i] = m[i] * v[i];
  }
};

template<int BlockSize> void compute(Blocks<BlockSize>& blocks)
{
  for(size_t i = 0 ; i < blocks.r.size(); ++i)
  {
    blocks.eval(i);
  }
}

int main()
{
  size_t nb_block = 1000;
  static const size_t BlockSize = 6;
  size_t rows = nb_block*BlockSize, cols = nb_block*BlockSize;

  Dense dense(rows,cols,BlockSize);
  Sparse sparse(dense);
  Blocks<BlockSize> blocks(dense);

  utils::Tic<true> ticd("dense");
  compute(dense);
  ticd.disp();

  utils::Tic<true> tics("sparse");
  compute(sparse);
  tics.disp();


  utils::Tic<true> ticb("blocks");
  compute(blocks);
  ticb.disp();
  

  // std::cout << m << std::endl;
  // std::cout << std::endl;
  // std::cout << v.transpose() << std::endl;

  //r = s * v;

  return 0;
}
