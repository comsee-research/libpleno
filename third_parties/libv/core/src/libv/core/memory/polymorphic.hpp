/**

\file
\author Alexis Wilhelm (2013)
\copyright 2013 Institut Pascal

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef LIBV_CORE_MEMORY_POLYMORPHIC_HPP
#define LIBV_CORE_MEMORY_POLYMORPHIC_HPP

#include <cstddef>

#include "../global.hpp"

namespace v {
namespace core {
/// \addtogroup polymorphism
/// \{

/**

A smart pointer that can store polymorphic types in standard containers.
Use this instead of \e T* if dynamic allocation is too expensive and you know the size of the largest subclass of \e T.

In this class we use virtual functions, constructors and destructors in a way that does not require dynamic allocation.
Most of the magic lies in the virtual copy constructor and assignment operator.
Try to figure out how it works and see how much of polymorphism you truly understand!

\tparam T A class with virtual methods.
\tparam size The size of the largest subclass of \e T.
\implements std::TrivialIterator

*/
template<class T, size_t size>
class Polymorphic
{
  template<class> struct Data;
  virtual void copy_into(Polymorphic *) const {}
  mutable char data_[size];
public:

  /// \name Trivial Iterator Concept
  /// \{

  /// \copydoc std::TrivialIterator::value_type
  typedef T value_type;

  /// \copydoc std::TrivialIterator::reference
  typedef T &reference;

  /// \copydoc std::TrivialIterator::pointer
  typedef T *pointer;

  reference
  operator*(void) const
  {
    return data<T>();
  }

  pointer
  operator->(void) const
  {
    return &**this;
  }

  /// \copydoc std::TrivialIterator::operator==()
  bool
  operator==(const Polymorphic &other) const
  {
    return this == &other;
  }

  /// \copydoc std::TrivialIterator::operator!=()
  bool
  operator!=(const Polymorphic &other) const
  {
    return !(*this == other);
  }

  /// \copydoc std::Assignable::constructor()
  /// This function calls \e T's virtual copy constructor.
  Polymorphic(const Polymorphic &other)
  {
    other.copy_into(this);
  }

  /// \copydoc std::Assignable::operator=()
  /// This function calls \e T's virtual assignment operator.
  Polymorphic &
  operator=(const Polymorphic &other)
  {
    this->~Polymorphic();
    other.copy_into(this);
    return *this;
  }

  /// \}

  Polymorphic(void) {}
  virtual ~Polymorphic(void) {}

  /// Wrap an object.
  /// \tparam X A subclass of \e T.
  /// \param x The object to be wrapped.
  /// \returns A fake pointer to a copy of \e x.
  template<class X> static Data<X>
  create(const X &x)
  {
    return Data<X>(x);
  }

protected:

  /// Reinterpret contents as an instance of \e X.
  /// \tparam X A subclass of \e T.
  /// \returns A reference to \e X.
  template<class X> X &
  data(void) const
  {
    union { char *lead; X *gold; } alchemy;
    alchemy.lead = data_;
    return *alchemy.gold;
  }
};

/// Helper class for Polymorphic.
/// Holds dynamic type information.
template<class T, size_t size>
template<class X>
struct Polymorphic<T, size>::Data
  : Polymorphic
{
  /// Constructor.
  Data(const X &x)
  {
    new(&data<X>()) X(x);
  }

  ~Data(void)
  {
    data<X>().~X();
  }

private:

  void
  copy_into(Polymorphic *other) const
  {
    new(other) Data(data<X>());
  }
};

/// \}
}}

#endif
