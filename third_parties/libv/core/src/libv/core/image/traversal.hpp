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

#ifndef LIBV_CORE_IMAGE_TRAVERSAL_HPP
#define LIBV_CORE_IMAGE_TRAVERSAL_HPP

#include <type_traits>
#include <libv/core/type_traits/raw_type.hpp>

/// Traverse an image and provide access to each pixel.
#define V_FOR_EACH_PIXEL(_iterator, _image)\
  for(v::core::image_::image_traversal_pixel<V_DECLTYPE(_image)> _iterator(_image); _iterator.row < _iterator.image.height(); ++_iterator.row)\
  for(_iterator.column = 0; _iterator.column < _iterator.image.width(); ++_iterator.column)\
  if(_iterator.commit())\

/// Traverse an image and provide access to each scalar.
#define V_FOR_EACH_SCALAR(_iterator, _image)\
  for(v::core::image_::image_traversal_scalar<V_DECLTYPE(_image)> _iterator(_image); _iterator.row < _iterator.image.height(); ++_iterator.row)\
  for(_iterator.column = 0; _iterator.column < _iterator.image.width(); ++_iterator.column)\
  for(_iterator.plane = 0; _iterator.plane < _iterator.image.depth(); ++_iterator.plane)\
  if(_iterator.commit())\

namespace v {
namespace core {
namespace image_ {

/// Behave just like a pointer to an image.
/// \implements std::TrivialIterator
template<class T>
struct image_pointer
{
  /// \name Trivial Iterator Concept
  /// \{

  /// \copydoc std::TrivialIterator::pointer
  typedef const T *pointer;

  /// \copydoc std::TrivialIterator::reference
  typedef T reference;

  /// \copydoc std::DefaultConstructible::constructor
  image_pointer(void)
  {
  }

  reference
  operator*(void) const
  {
    return data;
  }

  pointer
  operator->(void) const
  {
    return &data;
  }

  /// \}

protected:

  /// Initialize this pointer with a part of an image.
  image_pointer(const T &data)
    : data(data)
  {
  }

  /// Re-initialize this pointer with a new part of an image.
  void
  update(const T &new_data)
  {
    new(&data) T(new_data);
  }

private:

  T data;
};

/// An iterator pointing to an image, to be traversed with #V_FOR_EACH_PIXEL.
template<class T>
struct image_traversal_pixel
  : image_pointer<typename raw_type<T>::type::reference_to_pixel>
{
  /// \copydoc image_traversal_scalar::image
  T &image;

  /// \copydoc image_traversal_scalar::row
  size_t row;

  /// \copydoc image_traversal_scalar::column
  size_t column;

  /// Initialize a traversal for an image.
  image_traversal_pixel(T &image)
    : image(image)
    , row(0)
    , column(0)
  {
  }

  /// \copydoc image_traversal_scalar::commit
  bool
  commit(void)
  {
    this->update(image.pixel(row, column));
    return true;
  }
};

/// An iterator pointing to an image, to be traversed with #V_FOR_EACH_SCALAR.
/// \implements std::TrivialIterator
template<class T>
class image_traversal_scalar
{
  template<class X>
  struct traits
  {
    typedef typename raw_type<X>::type::pointer_to_scalar pointer;
    typedef typename raw_type<X>::type::reference_to_scalar reference;
  };

  template<class X>
  struct traits<const X>
  {
    typedef typename raw_type<X>::type::const_pointer_to_scalar pointer;
    typedef typename raw_type<X>::type::const_reference_to_scalar reference;
  };

public:
  /// \name Trivial Iterator Concept
  /// \{

  typedef typename traits<typename std::remove_reference<T>::type>::pointer pointer;
  typedef typename traits<typename std::remove_reference<T>::type>::reference reference;

  reference
  operator*(void) const
  {
    return *element;
  }

  pointer
  operator->(void) const
  {
    return &**this;
  }

  /// \}

  /// The traversed image.
  T &image;

  /// The current row.
  size_t row;

  /// The current column.
  size_t column;

  /// The current column.
  size_t plane;

  /// Initialize a traversal for an image.
  image_traversal_scalar(T &image)
    : image(image)
    , row(0)
    , column(0)
    , plane(0)
  {
  }

  /// Commit the changes made to the coordinates.
  /// \return \c true so we can use it in a condition.
  bool
  commit(void)
  {
    element = &image.scalar(row, column, plane);
    return true;
  }

private:

  pointer element;
};

}}}

#endif
