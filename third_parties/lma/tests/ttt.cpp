#include <boost/mpl/vector.hpp>
#include <libv/lma/ttt/mpl/naming.hpp>
#include <iostream>


int main()
{
  std::cout << " --> " << ttt::name<boost::mpl::vector<int,char,double>>() << std::endl;
  return EXIT_SUCCESS;
}
