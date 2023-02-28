/**

\file
\author Alexis Wilhelm (2012-2013)
\copyright 2012-2013 Institut Pascal

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

#ifndef LIBV_CORE_IMAGE_ITERATOR_HPP
#define LIBV_CORE_IMAGE_ITERATOR_HPP

#include <iterator>

namespace v {
namespace core {
namespace image_ {

/// An iterator pointing to an element of an image.
/// \implements std::RandomAccessIterator
template<class T>
class image_iterator
{
  typedef image_iterator this_type;

public:

  /// \name Random Access Iterator Concept
  /// \{

  /// \copydoc std::RandomAccessIterator::value_type
  typedef typename T::image_type value_type;

  /// \copydoc std::RandomAccessIterator::pointer
  typedef const T *pointer;

  /// \copydoc std::RandomAccessIterator::reference
  typedef T reference;

  /// \copydoc std::RandomAccessIterator::difference_type
  typedef typename T::difference_type difference_type;

  /// \copydoc std::RandomAccessIterator::iterator_category
  typedef std::random_access_iterator_tag iterator_category;

  reference
  operator*(void) const
  {
    return data_;
  }

  pointer
  operator->(void) const
  {
    return &data_;
  }

  reference
  operator[](difference_type n) const
  {
    return *(*this + n);
  }

  this_type &
  operator++(void)
  {
    return *this += 1;
  }

  this_type
  operator++(int)
  {
    const this_type copy(*this);
    ++*this;
    return copy;
  }

  this_type &
  operator+=(difference_type n)
  {
    data_.data_ += n * step_;
    return *this;
  }

  /// \copydoc std::RandomAccessIterator::operator+(this_type, difference_type)
  friend this_type
  operator+(this_type p, difference_type n)
  {
    return p += n;
  }

  /// \copydoc operator+
  friend this_type
  operator+(difference_type n, this_type p)
  {
    return p += n;
  }

  this_type &
  operator--(void)
  {
    return *this -= 1;
  }

  this_type
  operator--(int)
  {
    const this_type copy(*this);
    --*this;
    return copy;
  }

  this_type &
  operator-=(difference_type n)
  {
    return *this += -n;
  }

  this_type
  operator-(difference_type n) const
  {
    return *this + -n;
  }

  template<class X> difference_type
  operator-(const X &other) const
  {
    return (data_.data() - other.data_.data()) / step_;
  }

  template<class X> bool
  operator==(const X &other) const
  {
    return data_.data() == other.data_.data();
  }

  template<class X> bool
  operator!=(const X &other) const
  {
    return !(*this == other);
  }

  template<class X> bool
  operator<(const X &other) const
  {
    return data_.data() < other.data_.data();
  }

  template<class X> bool
  operator>(const X &other) const
  {
    return data_.data() > other.data_.data();
  }

  template<class X> bool
  operator<=(const X &other) const
  {
    return !(*this > other);
  }

  template<class X> bool
  operator>=(const X &other) const
  {
    return !(*this < other);
  }

  /// \copydoc std::DefaultConstructible::constructor
  image_iterator(void)
  {
  }

  /// \copydoc std::Assignable::constructor
  image_iterator(const this_type &other)
  {
    copy_construct(other);
  }

  /// \copydoc image_iterator(const this_type &)
  template<class X>
  image_iterator(const X &other)
  {
    copy_construct(other);
  }

  /// \copydoc std::RandomAccessIterator::operator=()
  this_type &
  operator=(const this_type &other)
  {
    return assign(other);
  }

  /// \copydoc operator=()
  template<class X> this_type &
  operator=(const X &other)
  {
    return assign(other);
  }

  /// \}

  /// Initialize an iterator.
  image_iterator(const reference &data, difference_type step)
    : data_(data)
    , step_(step)
  {
  }

private:

  template<class X> void
  copy_construct(const X &other)
  {
    new(this) this_type(other.data_, other.step_);
  }

  template<class X> this_type &
  assign(const X &other)
  {
    this->~this_type();
    new(this) this_type(other);
    return *this;
  }

  template<class> friend class image_iterator;
  reference data_;
  difference_type step_;
};

}}}

#endif
