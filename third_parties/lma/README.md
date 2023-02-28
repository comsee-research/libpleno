# LMA : Levenberg-Marquardt Algorithm

LMA is a submodule of libv, so its installation depends on libv-core: https://github.com/bezout/libv-core.git

The dependencies are Eigen-3.2, Boost >= 1.51, g++ >= 4.8, c++11.

The following procedure should work for a local installation:
-
git clone https://github.com/bezout/libv-core.git libv-core

mkdir build_libv-core && cd build_libv-core

cmake ../libv-core -DCMAKE_INSTALL_PREFIX=../ROOT

make -j4 && make install

cd ..

git clone https://github.com/bezout/LMA.git LMA

mkdir build_lma && cd build_lma

cmake ../LMA -DCMAKE_PREFIX_PATH=../ROOT -DCMAKE_INSTALL_PREFIX=../ROOT

make && make install

Dependencies Graph:
-
Tool used to generate multiple constraints problem.
See: LMA/examples/dependencies_graph


Available examples :
-
  bal : Bundle Adjustment on http://grail.cs.washington.edu/projects/bal/ datasets.
  
  circle : optimize a circle equation
