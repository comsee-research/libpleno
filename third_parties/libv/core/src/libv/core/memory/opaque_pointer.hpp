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

#ifndef LIBV_CORE_MEMORY_OPAQUE_POINTER_HPP
#define LIBV_CORE_MEMORY_OPAQUE_POINTER_HPP

#include "../noncopyable.hpp"
#include "../global.hpp"

#include <utility>

namespace v {
namespace core {
/// \addtogroup opaque
/// \{

/**

Declare an opaque pointer on a \e Private nested class.
The nested class can be defined in another source file.
Private members can be accessed through the \c that() method.

You must initialize this pointer in the constructor's initialization list with \c that_(new Private).

Derived classes may declare their own \e Private nested class with V_REDECLARE_PRIVATE.

\post The class contains the following new members: \code

  struct Private; // the private structure (declared, not defined)
  Private &that(void); // mutable getter
  const Private &that(void) const; // constant getter

\endcode

*/
#define V_DECLARE_PRIVATE\
\
  V_REDECLARE_PRIVATE\
  v::OpaquePointer<Private> that_;\

/**

Declare a \e Private nested class.
The nested class must derive from the nested class super::Private declared in the parent class with V_DECLARE_PRIVATE.

You must initialize the parent class in the constructor's initialization list with \c super(new Private).

*/
#define V_REDECLARE_PRIVATE\
\
  protected:\
  struct Private;\
  friend struct Private;\
\
  /** The private members. */\
  /** \return A reference to the private members. */\
  Private &\
  that(void)\
  {\
    return *reinterpret_cast<Private *>(that_.get());\
  }\
\
  /** \copydoc that */\
  const Private &\
  that(void) const\
  {\
    return *reinterpret_cast<Private *>(that_.get());\
  }\

/**

Define a \e Config nested class and declare an opaque pointer on a \e Private nested class.
This macro is provided for convenience, since many algorithms will need both the \e Config and \e Private classes (and most of the time \e Private will be a subclass of \e Config).

\see V_DECLARE_PRIVATE
\see V_DEFINE_CONFIG

*/
#define V_DECLARE_PRIVATE_CONFIG\
  V_DECLARE_PRIVATE\
  public:\
  V_DEFINE_CONFIG\

/**

A pointer to a private structure.
This works just like a \c scoped_ptr, but \e T can be an incomplete type.

*/
template<class T>
struct OpaquePointer: private noncopyable
{
  /// The managed pointer.
  T *get() const
  {
    return ptr_;
  }

  /// Constructor
  explicit OpaquePointer(T *ptr)
  : ptr_(ptr)
  , destructor_(delete_)
  {
  }

  OpaquePointer(OpaquePointer &&other)
  : ptr_(other.ptr_)
  , destructor_(other.destructor_)
  {
    other.ptr_ = 0;
  }

  OpaquePointer &operator=(OpaquePointer &&other)
  {
    this->~OpaquePointer();
    new(this) OpaquePointer(std::move(other));
    return *this;
  }

  ~OpaquePointer(void)
  {
    if(ptr_)
    {
      destructor_(ptr_);
    }
  }

private:

  static void delete_(T *ptr)
  {
    delete ptr;
  }

  T *ptr_ = 0;
  void (*const destructor_)(T *);
};

/// \}
}}

#ifdef DOXYGEN
/// \cond false

// Astonishingly, Doxygen is less confused with this definition than without.
// We need this for Doxygen to draw the collaboration graph properly.
#define PrivatePointer

/// \endcond
#endif
#endif
