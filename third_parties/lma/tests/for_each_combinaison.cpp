#ifdef USE_BRIGAND

#include <libv/lma/ttt/brigand/for_each_combinaison.hpp>

using namespace brigand;

struct Cout
{
  template<class ... T>
  Cout operator()(brigand::type_<brigand::list<T...>>) const
  {
    brigand::for_each<brigand::list<T...>>( *this );
    std::cout << std::endl;
    return *this;
  }

  template<class T>
  Cout operator()(T) const { std::cout << typeid(T).name() << " "; return *this; }

  template<class T>
  Cout operator()(brigand::type_<T>) const { return (*this)(T{});}
  Cout operator()(brigand::type_<double>) const { std::cout << "double "; return *this;}
  Cout operator()(brigand::type_<float>) const { std::cout << "float "; return *this;}
  Cout operator()(brigand::type_<char>) const { std::cout << "char "; return *this;}
  Cout operator()(brigand::type_<int>) const { std::cout << "int "; return *this;}
  Cout operator()(brigand::type_<std::size_t>) const { std::cout << "size_t "; return *this;}
  template<std::size_t I>
  Cout operator()(std::integral_constant<std::size_t,I>) const { std::cout << "<" << I << "> "; return *this;}
  Cout operator()(std::string str) const { std::cout << str << " "; return *this;}
  Cout operator()(const char*& str) const { std::cout << str << " "; return *this;}
  Cout operator,(auto x) const { return this->operator()(x);}
};

int main()
{
  using l1 = list<brigand::size_t<1>,brigand::size_t<2>>;
  using l2 = list<brigand::size_t<3>,brigand::size_t<4>,brigand::size_t<5>>;
  using l3 = list<brigand::size_t<6>,brigand::size_t<7>,brigand::size_t<8>,brigand::size_t<9>>;
  using Lists = list<l1,l2,l3>;

  ttt::for_each_combinaison(
    Lists{},
    [](auto ... l){ for_each(list_(l...), Cout{}); std::cout << std::endl;}
    );

  std::cout << std::endl;
}

#else // #ifdef USE_BRIGAND

int main()
{
}

#endif // #ifdef USE_BRIGAND

