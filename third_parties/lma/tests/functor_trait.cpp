#include <libv/lma/ttt/traits/functor_trait.hpp>
#include <boost/fusion/include/as_vector.hpp>
#include <boost/fusion/mpl.hpp>

struct f1 { bool operator()(double,float, double&) const { return true; } };
struct f2 { bool operator()(const double&,const float&,double&) const { return true; } };
struct f3 { bool operator()(const double*,const float*,double&) const { return true; } };

template<class F, class S> void Test()
{
  BOOST_MPL_ASSERT((
    boost::is_same<
        typename boost::fusion::result_of::as_vector<typename ttt::FunctorTrait<F>::ParametersType>::type,
        S
      >
    ));
}

int main()
{
  Test<decltype(&f1::operator()),boost::fusion::vector2<double,float>>();
  Test<decltype(&f2::operator()),boost::fusion::vector2<double,float>>();
  Test<decltype(&f3::operator()),boost::fusion::vector2<const double*,const float*>>();

  return EXIT_SUCCESS;
}
