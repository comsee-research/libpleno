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

#ifndef LIBV_CORE_IMAGE_IMAGE_HPP
#define LIBV_CORE_IMAGE_IMAGE_HPP

#include <type_traits>
#include <limits>
#include <stdexcept>
#include <libv/core/memory/aligned_allocator.hpp>
#include <libv/core/array_initializer.hpp>
#include <libv/core/assert.hpp>
#include <libv/core/type_traits/enable_if.hpp>
#include <libv/core/tag.hpp>

#include "format.hpp"
#include "iterator.hpp"
#include "layout.hpp"
#include "size.hpp"
#include "storage.hpp"
#include "traversal.hpp"

namespace v {
namespace core {
/// \addtogroup image
/// \{

/// Define a const type, given the corresponding non-const type.
#define _DEFINE_CONST_TYPE(x)\
  /** \copydoc x */\
  typedef typename const_this_type::x const_##x;

/// An image.
/// \implements std::DefaultConstructible
/// \implements std::RandomAccessContainer
template<class T>
class Image
  : public image_::image_layout<Image<T>, typename T::scalar_type, typename std::conditional<T::has_value_semantic, const typename T::scalar_type, typename T::scalar_type>::type &, T::flags & IMAGE_LAYOUT_MASK, T::plane_count, T::row_count == 1 && T::column_count == 1>
  , protected image_::image_size<1, T::row_count>
  , protected image_::image_size<2, T::column_count>
  , protected image_::image_size<3, T::plane_count>
  , protected image_::image_size<-1, T::row_step>
{
  template<class> friend class Image;
  template<class> friend class image_::image_iterator;
  template<class> friend struct image_::image_traversal_pixel;
  template<class> friend class image_::image_traversal_scalar;
  typedef T format;
  typedef Image this_type;
  typedef typename std::conditional<T::has_value_semantic, Image<typename T::template with_scalar_type<const typename T::scalar_type>::type>, this_type>::type const_this_type;
  typedef image_::image_size<1, T::row_count> the_row_count;
  typedef image_::image_size<2, T::column_count> the_column_count;
  typedef image_::image_size<3, T::plane_count> the_plane_count;
  typedef image_::image_size<-1, T::row_step> the_row_step;
  typename image_::image_storage_type<typename T::scalar_type, T::byte_count>::type data_;

  template<class X>
  struct is_convertible
  {
    static const bool value = (!T::row_count || T::row_count == X::row_count) && (!T::column_count || T::column_count == X::column_count) && (!T::plane_count || T::plane_count == X::plane_count);
  };

public:

  /// Same as \e this, but with \e _new_scalar_type as the scalar type.
  template<class _new_scalar_type>
  struct with_scalar_type
  {
    /// Result
    typedef Image<typename T::template with_scalar_type<_new_scalar_type>::type> type;
  };

  /// Type of scalar.
  typedef typename T::scalar_type scalar_type;

  /// Type of reference to a scalar.
  typedef scalar_type &reference_to_scalar;

  /// Type of pointer to a scalar.
  typedef scalar_type *pointer_to_scalar;

  /// Type of scalar iterator.
  typedef pointer_to_scalar scalar_iterator;

  typedef typename std::remove_cv<scalar_type>::type mutable_scalar_type;

  /// Type of image.
  typedef Image<ImageFormat<scalar_type, T::naked_flags | IMAGE_SEMANTIC_VALUE, T::row_count, T::column_count, T::plane_count, T::row_step, T::column_step> > image_type;

  /// Type of proxy.
  typedef Image<ImageFormat<scalar_type, T::naked_flags | IMAGE_SEMANTIC_REFERENCE, T::row_count, T::column_count, T::plane_count, T::row_step, T::column_step> > proxy_type;

  /// Type of view.
  typedef Image<ImageFormat<scalar_type, T::naked_flags | IMAGE_SEMANTIC_POINTER, T::row_count, T::column_count, T::plane_count, T::row_step, T::column_step> > view_type;

  /// Type of row.
  typedef Image<ImageFormat<scalar_type, T::naked_flags | IMAGE_SEMANTIC_VALUE, 1, T::column_count, T::plane_count, -1, -1> > row_type;

  /// Type of reference to a row.
  typedef typename row_type::proxy_type reference_to_row;

  /// Type of pointer to a row.
  typedef const reference_to_row *pointer_to_row;

  /// Type of row iterator.
  typedef image_::image_iterator<reference_to_row> row_iterator;

  /// Type of column.
  typedef Image<ImageFormat<scalar_type, T::naked_flags | IMAGE_SEMANTIC_VALUE, T::row_count, 1, T::plane_count, T::row_step, -1> > column_type;

  /// Type of reference to a column.
  typedef typename column_type::proxy_type reference_to_column;

  /// Type of pointer to a column.
  typedef const reference_to_column *pointer_to_column;

  /// Type of column iterator.
  typedef image_::image_iterator<reference_to_column> column_iterator;

  /// Type of pixel.
  typedef Image<ImageFormat<scalar_type, T::naked_flags | IMAGE_SEMANTIC_VALUE, 1, 1, T::plane_count, 1, 1> > pixel_type;

  /// Type of reference to a pixel.
  typedef typename pixel_type::proxy_type reference_to_pixel;

  /// Type of pointer to a pixel.
  typedef const reference_to_pixel *pointer_to_pixel;

  /// Type of pixel iterator.
  typedef image_::image_iterator<reference_to_pixel> pixel_iterator;

  _DEFINE_CONST_TYPE(pointer_to_scalar)
  _DEFINE_CONST_TYPE(pointer_to_pixel)
  _DEFINE_CONST_TYPE(pointer_to_row)
  _DEFINE_CONST_TYPE(pointer_to_column)
  _DEFINE_CONST_TYPE(reference_to_scalar)
  _DEFINE_CONST_TYPE(reference_to_pixel)
  _DEFINE_CONST_TYPE(reference_to_row)
  _DEFINE_CONST_TYPE(reference_to_column)
  _DEFINE_CONST_TYPE(scalar_iterator)
  _DEFINE_CONST_TYPE(pixel_iterator)
  _DEFINE_CONST_TYPE(row_iterator)
  _DEFINE_CONST_TYPE(column_iterator)
  _DEFINE_CONST_TYPE(proxy_type)
  _DEFINE_CONST_TYPE(view_type)

  /// \name Random Access Container Concept
  /// \{

  /// \copydoc std::RandomAccessContainer::difference_type
  typedef ptrdiff_t difference_type;

  /// \copydoc std::RandomAccessContainer::size_type
  typedef size_t size_type;

  /// \copydoc std::RandomAccessContainer::value_type
  typedef typename std::conditional<T::dimension == 1, scalar_type, typename std::conditional<T::row_count != 1, row_type, pixel_type>::type>::type value_type;

  /// \copydoc std::RandomAccessContainer::reference
  typedef typename std::conditional<T::dimension == 1, reference_to_scalar, typename std::conditional<T::row_count != 1, reference_to_row, reference_to_pixel>::type>::type reference;

  /// \copydoc std::RandomAccessContainer::pointer
  typedef typename std::conditional<T::dimension == 1, pointer_to_scalar, typename std::conditional<T::row_count != 1, pointer_to_row, pointer_to_pixel>::type>::type pointer;

  /// \copydoc std::RandomAccessContainer::iterator
  /// \todo what if rows ≠ 1 && columns = 1 && planes = 1 && row_step ≠ ±1 ?
  typedef typename std::conditional<T::dimension == 1, pointer_to_scalar, image_::image_iterator<reference> >::type iterator;

  /// \copydoc std::RandomAccessContainer::reverse_iterator
  typedef std::reverse_iterator<iterator> reverse_iterator;

  _DEFINE_CONST_TYPE(pointer)
  _DEFINE_CONST_TYPE(reference)
  _DEFINE_CONST_TYPE(iterator)
  _DEFINE_CONST_TYPE(reverse_iterator)

  size_type
  size(void) const
  {
    return T::row_count != 1 ? height() : T::column_count != 1 ? width() : depth();
  }

  /// \copydoc std::RandomAccessContainer::max_size
  /// \todo use row_step and other dimensions
  static size_type
  max_size(void)
  {
    return std::numeric_limits<size_type>::max() / sizeof(scalar_type);
  }

  bool
  empty(void) const
  {
    return !size();
  }

  iterator
  begin(void)
  {
    return begin(type_tag<iterator>());
  }

  const_iterator
  begin(void) const
  {
    return begin(type_tag<iterator>());
  }

  iterator
  end(void)
  {
    return begin() + size();
  }

  const_iterator
  end(void) const
  {
    return begin() + size();
  }

  reverse_iterator
  rbegin(void)
  {
    return reverse_iterator(end());
  }

  const_reverse_iterator
  rbegin(void) const
  {
    return const_reverse_iterator(end());
  }

  reverse_iterator
  rend(void)
  {
    return reverse_iterator(begin());
  }

  const_reverse_iterator
  rend(void) const
  {
    return const_reverse_iterator(begin());
  }

  reference
  operator[](size_type i)
  {
    return begin()[difference_type(i)];
  }

  const_reference
  operator[](size_type i) const
  {
    return begin()[difference_type(i)];
  }

  template<class X> bool
  operator==(const X &other) const
  {
    if(height() != other.height() || width() != other.width() || depth() != other.depth())
    {
      return false;
    }
    V_FOR_EACH_SCALAR(p, *this)
    {
      if(*p != other.scalar(p.row, p.column, p.plane))
      {
        return false;
      }
    }
    return true;
  }

  template<class X> bool
  operator!=(const X &other) const
  {
    return !(*this == other);
  }

  /// \}

  /// \name Constructors and destructor
  /// \{

  /// \copydoc std::DefaultConstructible::constructor
  Image(void)
  {
    new(this) this_type(0, 0, 0, 0, 0, numeric_tag<T::byte_count>());
  }

  /// \copydoc std::Assignable::constructor
  Image(const this_type &other)
  {
    copy_construct(other);
  }

  /// \copydoc Image(const this_type &)
  template<class X>
  Image(const Image<X> &other V_ENABLE_IF_TPL(is_convertible<X>::value)
  {
    copy_construct(other);
  }

  /// \copydoc Image(const this_type &)
  template<class X> explicit
  Image(const Image<X> &other V_ENABLE_IF_TPL(!is_convertible<X>::value)
  {
    copy_construct(other);
  }

  /// Constructor for pointer semantic or reference semantic.
  /// \param data An existing memory buffer.
  /// \param rows The height of the image.
  /// \param columns The width of the image.
  /// \param row_step The distance between two consecutive rows.
  Image V_ENABLE_IF(!T::has_value_semantic && !T::row_count && !T::column_count && T::plane_count, pointer_to_scalar) data, size_type rows, size_type columns, size_type row_step)
  {
    new(this) this_type(data, rows, row_step, columns, 0, numeric_tag<T::byte_count>());
  }

  /// Constructor for value semantic.
  /// \param rows The height of the image.
  /// \param columns The width of the image.
  Image V_ENABLE_IF(T::has_value_semantic && !T::row_count && !T::column_count && T::plane_count && T::row_step, size_type) rows, size_type columns)
  {
    new(this) this_type(0, rows, 0, columns, 0, numeric_tag<T::byte_count>());
  }

  /// Constructor for a static image.
  /// \param initializer An array initializer built with new_array().
  template<class X>
  Image(const ArrayInitializer<X, T::byte_count> &initializer)
  {
    initializer.apply(begin_scalar());
  }

  /// Destructor
  ~Image(void)
  {
    if(T::has_value_semantic)
    {
      destruct(numeric_tag<T::byte_count>());
    }
  }

  /// \copydoc std::Assignable::operator=()
  this_type &
  operator=
  ( const this_type &other ///< An other image.
  )
  {
    return assign(other);
  }

  /// \copydoc operator=()
  template<class X> this_type &
  operator=
  ( const X &other ///< An other image.
  )
  {
    return assign(other);
  }

  /**

  Swap this image with another image.

  Assigning a temporary container requires 2 allocations, 2 deallocations and 1 copy.
  Swapping it, however, requires only 1 allocation and 1 deallocation, saving 1 allocation, 1 deallocation and 1 copy.
  Thus, you should never write this to reinitialize an image:
  \code my_image = Image(rows, columns); \endcode
  but you should prefer this:
  \code Image(rows, columns).swap(my_image); \endcode

  \param other An other image.

  */
  template<class X> void
  swap(Image<X> &other V_ENABLE_IF_TPL(T::row_count == X::row_count && T::column_count == X::column_count && T::plane_count == X::plane_count && ((T::has_value_semantic && X::has_value_semantic) || (!T::has_value_semantic && !X::has_value_semantic)) && !T::byte_count && !X::byte_count)
  {
    std::swap(data_, other.data_);
    the_row_count::swap(static_cast<typename Image<X>::the_row_count &>(other));
    the_column_count::swap(static_cast<typename Image<X>::the_column_count &>(other));
    the_plane_count::swap(static_cast<typename Image<X>::the_plane_count &>(other));
    the_row_step::swap(static_cast<typename Image<X>::the_row_step &>(other));
  }

  /// \}

  /// The height of the image.
  /// \returns The row count.
  size_type
  height(void) const
  {
    return the_row_count::get();
  }

  /// The width of the image.
  /// \returns The column count.
  size_type
  width(void) const
  {
    return the_column_count::get();
  }

  /// The depth of the image.
  /// \returns The plane count.
  size_type
  depth(void) const
  {
    return the_plane_count::get();
  }

  /// A pointer to the underlying array.
  /// \returns A pointer to the underlying array.
  pointer_to_scalar
  data(void)
  {
    return data_;
  }

  /// \copydoc data
  const_pointer_to_scalar
  data(void) const
  {
    return data_;
  }

  /// The distance between two consecutive rows.
  /// If \f$ row step < 0 \f$ it is \f$ width × depth \f$, aligned on \f$ -row step \f$ bytes.
  /// \returns The distance between two consecutive rows.
  size_type
  row_step(void) const
  {
#ifdef __clang__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdivision-by-zero"
#endif
    return T::row_step < 0 ? -T::row_step * ((width() * depth() * sizeof(scalar_type) - 1) / std::min(size_type(1), size_type(-T::row_step)) + 1) / sizeof(scalar_type) : the_row_step::get();
#ifdef __clang__
#pragma GCC diagnostic pop
#endif
  }

  /// Get a row.
  /// \param i The index of a row.
  /// \returns A reference to the \f$ i \f$-th row.
  reference_to_row
  row(size_type i)
  {
    return reference_to_row(data() + i * row_step(), height(), row_step(), width(), depth(), numeric_tag<0>());
  }

  /// \copydoc row
  const_reference_to_row
  row(size_type i) const
  {
    return const_reference_to_row(data() + i * row_step(), height(), row_step(), width(), depth(), numeric_tag<0>());
  }

  /// Get several rows.
  /// \param i The vertical offset of the view.
  /// \param height The height of the view.
  /// \returns A view on the sub-image.
  proxy_type
  rows(size_type i, size_type height)
  {
    return proxy_type(row(i).data(), height, row_step(), width(), depth(), numeric_tag<0>());
  }

  /// \copydoc rows
  const_proxy_type
  rows(size_type i, size_type height) const
  {
    return const_proxy_type(row(i).data(), height, row_step(), width(), depth(), numeric_tag<0>());
  }

  /// Iterate over each row.
  /// \return A row iterator.
  row_iterator
  begin_row(void)
  {
    return row_iterator(row(0), row_step());
  }

  /// \copydoc begin_row
  const_row_iterator
  begin_row(void) const
  {
    return const_row_iterator(row(0), row_step());
  }

  /// \copydoc begin_row
  row_iterator
  end_row(void)
  {
    return begin_row() + height();
  }

  /// \copydoc end_row
  const_row_iterator
  end_row(void) const
  {
    return begin_row() + height();
  }

  /// The distance between two consecutive columns.
  /// \returns The distance between two consecutive columns.
  size_type
  column_step(void) const
  {
    return depth();
  }

  /// Get a column.
  /// \param j The index of a column.
  /// \returns A reference to the \f$ i \f$-th column.
  reference_to_column
  column(size_type j)
  {
    return reference_to_column(data() + j * depth(), height(), row_step(), width(), depth(), numeric_tag<0>());
  }

  /// \copydoc column
  const_reference_to_column
  column(size_type j) const
  {
    return const_reference_to_column(data() + j * depth(), height(), row_step(), width(), depth(), numeric_tag<0>());
  }

  /// Get several columns.
  /// \param j The horizontal offset of the view.
  /// \param width The width of the view.
  /// \returns A view on the sub-image.
  proxy_type
  columns(size_type j, size_type width)
  {
    return proxy_type(column(j).data(), height(), row_step(), width, depth(), numeric_tag<0>());
  }

  /// \copydoc columns
  const_proxy_type
  columns(size_type j, size_type width) const
  {
    return const_proxy_type(column(j).data(), height(), row_step(), width, depth(), numeric_tag<0>());
  }

  /// Iterate over each column.
  /// \return A column iterator.
  column_iterator
  begin_column(void)
  {
    return column_iterator(column(0), depth());
  }

  /// \copydoc begin_column
  const_column_iterator
  begin_column(void) const
  {
    return const_column_iterator(column(0), depth());
  }

  /// \copydoc begin_column
  column_iterator
  end_column(void)
  {
    return begin_column() + width();
  }

  /// \copydoc end_column
  const_column_iterator
  end_column(void) const
  {
    return begin_column() + width();
  }

  /// The distance between two consecutive planes.
  /// \returns The distance between two consecutive planes.
  size_type
  plane_step(void) const
  {
    return 1;
  }

  /// Get a pixel.
  /// \param i The index of a row.
  /// \param j The index of a column.
  /// \returns A reference to the pixel at \f$ (i,j) \f$.
  reference_to_pixel
  pixel(size_type i, size_type j)
  {
    return row(i).column(j);
  }

  /// \copydoc pixel
  const_reference_to_pixel
  pixel(size_type i, size_type j) const
  {
    return row(i).column(j);
  }

  /// \copydoc pixel
  reference_to_pixel
  V_OPERATOR_ENABLE_IF(T::dimension == 3, size_type) i, size_type j)
  {
    return (*this)[i][j];
  }

  /// \copydoc pixel
  const_reference_to_pixel
  V_OPERATOR_ENABLE_IF(T::dimension == 3, size_type) i, size_type j) const
  {
    return (*this)[i][j];
  }

  /// \copydoc pixel
  reference_to_pixel
  at V_ENABLE_IF(T::dimension == 3, size_type) i, size_type j)
  {
    return at(i).at(j);
  }

  /// \copydoc pixel
  const_reference_to_pixel
  at V_ENABLE_IF(T::dimension == 3, size_type) i, size_type j) const
  {
    return at(i).at(j);
  }

  /// Get several pixels.
  /// \param i The vertical offset of the view.
  /// \param j The horizontal offset of the view.
  /// \param height The height of the view.
  /// \param width The width of the view.
  /// \returns A view on the sub-image.
  proxy_type
  pixels(size_type i, size_type j, size_type height, size_type width)
  {
    return rows(i, height).columns(j, width);
  }

  /// \copydoc pixels
  const_proxy_type
  pixels(size_type i, size_type j, size_type height, size_type width) const
  {
    return rows(i, height).columns(j, width);
  }

  /// Iterate over each pixel.
  /// \return A pixel iterator.
  pixel_iterator
  begin_pixel(void)
  {
    V_PRECONDITION(pixels_are_contiguous())
    return pixel_iterator(pixel(0, 0), depth());
  }

  /// \copydoc begin_pixel
  const_pixel_iterator
  begin_pixel(void) const
  {
    V_PRECONDITION(pixels_are_contiguous())
    return const_pixel_iterator(pixel(0, 0), depth());
  }

  /// \copydoc begin_pixel
  pixel_iterator
  end_pixel(void)
  {
    return begin_pixel() + height() * width();
  }

  /// \copydoc end_pixel
  const_pixel_iterator
  end_pixel(void) const
  {
    return begin_pixel() + height() * width();
  }

  /// Whether pixels can be traversed with a constant step.
  /// \returns True if the pixels are contiguous.
  bool
  pixels_are_contiguous(void) const
  {
    return row_step() == width() * column_step();
  }

  /// Get a scalar.
  /// \param i The index of a row.
  /// \param j The index of a column.
  /// \param k The index of a plane.
  /// \returns A reference to the scalar at \f$ (i,j,k) \f$.
  reference_to_scalar
  scalar(size_type i, size_type j, size_type k)
  {
    return pixel(i, j).data()[k];
  }

  /// \copydoc scalar
  const_reference_to_scalar
  scalar(size_type i, size_type j, size_type k) const
  {
    return pixel(i, j).data()[k];
  }

  /// \copydoc scalar
  reference_to_scalar
  V_OPERATOR_ENABLE_IF(T::dimension == 3, size_type) i, size_type j, size_type k)
  {
    return (*this)[i][j][k];
  }

  /// \copydoc scalar
  const_reference_to_scalar
  V_OPERATOR_ENABLE_IF(T::dimension == 3, size_type) i, size_type j, size_type k) const
  {
    return (*this)[i][j][k];
  }

  /// Get a scalar.
  /// \param i The index of a row.
  /// \param j The index of a column.
  /// \returns A reference to the scalar at \f$ (i,j) \f$.
  reference_to_scalar
  V_OPERATOR_ENABLE_IF(T::dimension == 2, size_type) i, size_type j)
  {
    return (*this)[i][j];
  }

  /// Get a scalar.
  /// \param i The index of a row.
  /// \param j The index of a column.
  /// \returns A reference to the scalar at \f$ (i,j) \f$.
  const_reference_to_scalar
  V_OPERATOR_ENABLE_IF(T::dimension == 2, size_type) i, size_type j) const
  {
    return (*this)[i][j];
  }

  /// \copydoc scalar
  reference_to_scalar
  at V_ENABLE_IF(T::dimension == 3, size_type) i, size_type j, size_type k)
  {
    return at(i).at(j).at(k);
  }

  /// \copydoc scalar
  const_reference_to_scalar
  at V_ENABLE_IF(T::dimension == 3, size_type) i, size_type j, size_type k) const
  {
    return at(i).at(j).at(k);
  }

  /// Get a scalar.
  /// \param i The index of a row.
  /// \param j The index of a column.
  /// \returns A reference to the scalar at \f$ (i,j) \f$.
  reference_to_scalar
  at V_ENABLE_IF(T::dimension == 2, size_type) i, size_type j)
  {
    return at(i).at(j);
  }

  /// Get a scalar.
  /// \param i The index of a row.
  /// \param j The index of a column.
  /// \returns A reference to the scalar at \f$ (i,j) \f$.
  const_reference_to_scalar
  at V_ENABLE_IF(T::dimension == 2, size_type) i, size_type j) const
  {
    return at(i).at(j);
  }

  /// Iterate over each scalar.
  /// \return A scalar iterator.
  scalar_iterator
  begin_scalar(void)
  {
    V_PRECONDITION(scalars_are_contiguous())
    return &scalar(0, 0, 0);
  }

  /// \copydoc begin_scalar
  const_scalar_iterator
  begin_scalar(void) const
  {
    V_PRECONDITION(scalars_are_contiguous())
    return &scalar(0, 0, 0);
  }

  /// \copydoc begin_scalar
  scalar_iterator
  end_scalar(void)
  {
    return begin_scalar() + height() * width() * depth();
  }

  /// \copydoc end_scalar
  const_scalar_iterator
  end_scalar(void) const
  {
    return begin_scalar() + height() * width() * depth();
  }

  /// Whether scalars can be traversed with a constant step.
  /// \returns True if the scalars are contiguous.
  bool
  scalars_are_contiguous(void) const
  {
    return pixels_are_contiguous() && column_step() == depth() * plane_step();
  }

  /// Get an element.
  /// \param i The index of a row, column or plane.
  /// \returns A reference to the \f$ i \f$-th element.
  reference
  operator()(size_type i)
  {
    return (*this)[i];
  }

  /// \copydoc operator()(size_type)
  const_reference
  operator()(size_type i) const
  {
    return (*this)[i];
  }

  /// \copydoc operator()(size_type)
  reference
  at(size_type i)
  {
    if(i >= size())
    {
      throw std::out_of_range("image::at");
    }
    return (*this)[i];
  }

  /// \copydoc operator()(size_type)
  const_reference
  at(size_type i) const
  {
    if(i >= size())
    {
      throw std::out_of_range("image::at");
    }
    return (*this)[i];
  }

  /**

  Crop this image to the given dimensions.
  The value of pixels in the intersection of the image and the cropped region will be kept intact.
  Pixels outside this intersection are not initialized.
  This intersection can be empty, but then you should consider just reallocating the image using swap() instead of cropping it.

  \param i The vertical offset of the cropped region. Can be negative.
  \param j The horizontal offset of the cropped region. Can be negative.
  \param rows The new height of the image. Can be greater than height().
  \param columns The new width of the image. Can be greater than width().

  */
  void
  crop(difference_type i, difference_type j, size_type rows, size_type columns)
  {
    image_type new_image(rows, columns);
    const size_type
      i1 = std::max(i, ptrdiff_t(0)),
      j1 = std::max(j, ptrdiff_t(0)),
      i2 = -std::min(i, ptrdiff_t(0)),
      j2 = -std::min(j, ptrdiff_t(0));
    if(i1 < height() && j1 < width())
    {
      rows = std::min(rows, height() - i1);
      columns = std::min(columns, width() - j1);
      new_image.pixels(i2, j2, rows, columns) = pixels(i1, j1, rows, columns);
    }
    swap(new_image);
  }

private:

  iterator
  begin(type_tag<pointer_to_scalar>)
  {
    return data();
  }

  const_iterator
  begin(type_tag<pointer_to_scalar>) const
  {
    return data();
  }

  iterator
  begin(tag)
  {
    return iterator(reference(*this), difference_type(T::row_count != 1 ? row_step() : depth()));
  }

  const_iterator
  begin(tag) const
  {
    return const_iterator(const_reference(*this), difference_type(T::row_count != 1 ? row_step() : depth()));
  }

  Image(pointer_to_scalar data, size_type rows, size_type _row_step, size_type columns, size_type planes, numeric_tag<0>)
    : data_(data)
  {
    the_row_count::set(rows);
    the_row_step::set(_row_step);
    the_column_count::set(columns);
    the_plane_count::set(planes);

    if(T::has_value_semantic)
    {
      data_ = AlignedAllocator<scalar_type>::allocate(height() * row_step() * depth());

      for(scalar_iterator p = begin_scalar(); p != end_scalar(); ++p)
      {
        AlignedAllocator<mutable_scalar_type>::construct(const_cast<mutable_scalar_type *>(&*p), scalar_type());
      }
    }
  }

  Image(pointer_to_scalar, size_type, size_type, size_type, size_type, tag)
  {
  }

  void
  destruct(numeric_tag<0>)
  {
    for(scalar_iterator p = begin_scalar(); p != end_scalar(); ++p)
    {
      AlignedAllocator<mutable_scalar_type>::destroy(const_cast<mutable_scalar_type *>(&*p));
    }

    AlignedAllocator<scalar_type>::deallocate(data());
  }

  void
  destruct(tag)
  {
  }

  template<class X> void
  copy_construct(const X &other)
  {
    new(this) this_type(const_cast<pointer_to_scalar>(other.data()), other.height(), other.row_step(), other.width(), other.depth(), numeric_tag<T::byte_count>());

    if(T::has_value_semantic)
    {
      copy_data(other);
    }
  }

  template<class X> this_type &
  assign(const X &other)
  {
    if(T::has_reference_semantic)
    {
      copy_data(other);
    }
    else
    {
      this->~this_type();
      new(this) this_type(other);
    }
    return *this;
  }

  template<class X> void
  copy_data(const X &other)
  {
    for(size_t i = 0; i < height(); ++i)
    {
      std::copy(other.row(i).data(), other.row(i).data() + width() * depth(), const_cast<mutable_scalar_type *>(row(i).data()));
    }
  }
};

/// \}
}}

#undef _DEFINE_CONST_TYPE
#endif
