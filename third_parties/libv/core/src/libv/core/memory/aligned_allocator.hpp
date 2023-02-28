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

#ifndef LIBV_CORE_MEMORY_ALIGNED_ALLOCATOR_HPP
#define LIBV_CORE_MEMORY_ALIGNED_ALLOCATOR_HPP

#include <cstdlib>
#include <new>

#include "../global.hpp"

namespace v {
namespace core {

/**

An allocator of aligned memory blocks.

*/
template<typename T, int n = 16>
struct AlignedAllocator
{
  /// Type of pointer.
  typedef T *pointer;

  /// Type of constant pointer.
  typedef const T *const_pointer;

  /// Type of constant reference.
  typedef const T &const_reference;

  /// Type of size.
  typedef size_t size_type;

  /// Type of block.
  typedef T value_type;

  /// Type of allocator for an other type of block.
  template<typename _other_type>
  struct rebind
  {
    typedef AlignedAllocator<_other_type> other;
  };

  /**

  Allocate a block.

  \return A pointer to the newly allocated block.

  */
  static pointer allocate
  ( size_type count ///< The number of elements in the allocated block.
  , const void * = 0
  )
  {
    const size_type size = ((count * sizeof(T) - 1) / n + 1) * n;
    void *p = 0;
    #if defined(_MSC_VER)
    p = _aligned_malloc(size, n);
    #else
    if(posix_memalign(&p, n, size)) p = 0;
    #endif
    if(!p) throw std::bad_alloc();
    return static_cast<pointer>(p);
  }

  /**

  Initialize an object.

  */
  static void construct
  ( pointer p
  , const_reference prototype
  )
  {
    new(p) T(prototype);
  }

  /**

  Finalize an object.

  */
  static void destroy
  ( pointer p
  )
  {
    p->~T();
  }

  /**

  Deallocate a block.

  */
  static void deallocate
  ( const_pointer p ///< A pointer to the block to be deallocated.
  , size_type = 0
  )
  {
    #if defined(_MSC_VER)
    _aligned_free
    #else
    std::free
    #endif
      (const_cast<void *>(static_cast<const void *>(p)));
  }
};

template <typename T1, int n1, typename T2, int n2>
bool operator==(const AlignedAllocator<T1, n1>&, const AlignedAllocator<T2, n2>&)
{
  return (n1 == n2);
}

template <typename T1, int n1, typename T2, int n2>
bool operator!=(const AlignedAllocator<T1, n1>&, const AlignedAllocator<T2, n2>&)
{
  return (n1 != n2);
}

}}

#endif
