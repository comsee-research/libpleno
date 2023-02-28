#include <libv/lma/ttt/mpl/naming.hpp>
#include <iostream>

#include <boost/mpl/remove_if.hpp>
#include <boost/mpl/vector.hpp>
#include <boost/mpl/equal.hpp>
#include <boost/mpl/less.hpp>
#include <boost/mpl/count.hpp>
#include <iostream>


template<class ... T> struct Struct
{
};

using namespace boost;

template<class ... T> struct get_id : 
  mpl::remove_if<
                mpl::vector<T...>,
                mpl::less< 
                            mpl::count< mpl::vector<T...>, mpl::_1 >,
                            mpl::int_<2>
                          >
              >{};
              


int main()
{
  std::cout << ttt::name<get_id<int,double,char,int,int>::type>();
  Struct<get_id<int,double,char,int,int>::type> s;
  (void)s; // keep the compiler happy
}
