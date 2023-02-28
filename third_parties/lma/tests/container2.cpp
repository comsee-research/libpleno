#include <array>
#include <vector>
#include <iostream>

template<class T, int I> struct Container : std::array<T,I>
{
};

template<class T> struct Container<T,0> : std::vector<T>
{
  typedef std::vector<T> data;
  Container(){}
  Container(size_t n, T value):data(n,value){}
};

int main()
{
  Container<int,2> array;
  Container<int,0> vector(2,0);

  for(auto& x : array)
    x = 1;

  for(auto& x : vector)
    x = 2;

  for(auto& x : array)
    std::cout << x << std::endl;

  for(auto& x : vector)
    std::cout << x << std::endl;
}