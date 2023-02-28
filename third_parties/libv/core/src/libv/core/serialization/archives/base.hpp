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

#ifndef LIBV_CORE_SERIALIZATION_ARCHIVES_BASE_HPP
#define LIBV_CORE_SERIALIZATION_ARCHIVES_BASE_HPP

#include <numeric>
#include <string>
#include <type_traits>

#include "../types.hpp"

namespace v {
namespace core {
/// \addtogroup serialization_archives
/// \{

/// Define overloads for each fundamental type.
#define _DEFINE_FUNDAMENTAL_LOAD(T)\
\
  virtual void serialize(T &) = 0;\
  virtual void serialize(T *, const size_t *, size_t) = 0;\
\
  friend void load(InputArchive &archive, T &value)\
  {\
    archive.serialize(value);\
  }\
\
  friend void load(InputArchive &archive, T *values, const size_t *extents, size_t rank)\
  {\
    archive.serialize(values, extents, rank);\
  }\

/**

An input archive.

*/
struct InputArchive
{
  /**

  Start loading a list.

  */
  virtual void begin_list() = 0;

  /**

  Stop loading a list.

  */
  virtual void end_list() = 0;

  /**

  The extents of the current list.

  */
  virtual void get_extents
  ( size_t *extents ///< A list of extents.
  , size_t rank ///< The size of \e extents.
  ) = 0;

  /**

  Load a (\e key, \e value) pair.

  \return A reference to \e this.

  */
  template<class T>
  InputArchive &operator()
  ( const std::string &key ///< [in] A key.
  , T &value ///< [out] A value.
  )
  {
    if(begin(key))
    {
      load(*this, value);
      end();
    }
    return *this;
  }

  /**

  Load an anonymous value.

  \return A reference to \e this.

  */
  template<class T>
  InputArchive &operator()
  ( T &value ///< [out] A value.
  )
  {
    begin();
    load(*this, value);
    end();
    return *this;
  }

  /**

  Load the next (\e key, \e value) pair.

  \return A reference to \e this.

  */
  template<class T>
  InputArchive &operator()
  ( std::string *key ///< [in] A key.
  , T &value ///< [out] A value.
  )
  {
    begin(key);
    load(*this, value);
    end();
    return *this;
  }

private:

  virtual bool begin(const std::string &) = 0;
  virtual bool begin(std::string *) = 0;
  virtual bool begin() = 0;
  virtual void end() = 0;

  V_FOR_EACH_FUNDAMENTAL_TYPE(_DEFINE_FUNDAMENTAL_LOAD)
};

/**

Load an array from an input archive.

*/
template<class T>
void load
( InputArchive &archive
, T &value
, typename std::enable_if<bool(SerializableTraits<T>::rank)>::type * = nullptr
)
{
  size_t extents[SerializableTraits<T>::rank];
  archive.get_extents(extents, SerializableTraits<T>::rank);
  load(archive, &value, extents, 0);
}

/**

Load an array from an input archive.

*/
template<class T>
void load
( InputArchive &archive ///< An archive.
, T *values ///< An array of values.
, const size_t *extents ///< The extents of \e values.
, size_t rank ///< The rank of \e values.
)
{
  const size_t size = accumulate(extents, extents + rank, 1, std::multiplies<size_t>());
  archive.begin_list();
  for(size_t i = 0; i < size; ++i)
  {
    archive(values[i]);
  }
  archive.end_list();
}

/// Define overloads for each fundamental type.
#define _DEFINE_FUNDAMENTAL_SAVE(T)\
\
  virtual void serialize(T) = 0;\
  virtual void serialize(const T *, const size_t *, size_t) = 0;\
\
  friend void save(OutputArchive &archive, T value)\
  {\
    archive.serialize(value);\
  }\
\
  friend void save(OutputArchive &archive, const T *values, const size_t *extents, size_t rank)\
  {\
    archive.serialize(values, extents, rank);\
  }\

/**

An output archive.

*/
struct OutputArchive
{
  /**

  Start saving a list.

  */
  virtual void begin_list(size_t) = 0;

  /**

  Stop saving a list.

  */
  virtual void end_list() = 0;

  /**

  Save a (\e key, \e value) pair.

  \return A reference to \e this.

  */
  template<class T>
  OutputArchive &operator()
  ( const std::string &key ///< A key.
  , const T &value ///< A value.
  )
  {
    begin(key);
    save(*this, value);
    end();
    return *this;
  }

  /**

  Save an anonymous value.

  \return A reference to \e this.

  */
  template<class T>
  OutputArchive &operator()
  ( const T &value ///< A value.
  )
  {
    begin();
    save(*this, value);
    end();
    return *this;
  }

private:

  virtual void begin(const std::string &) = 0;
  virtual void begin() = 0;
  virtual void end() = 0;

  V_FOR_EACH_FUNDAMENTAL_TYPE(_DEFINE_FUNDAMENTAL_SAVE)
};

/**

Save a range in an output archive.

*/
template<class T>
void save_range
( OutputArchive &archive ///< An archive.
, T begin ///< An iterator to the begining of the range.
, size_t size ///< The size of the range.
)
{
  archive.begin_list(size);
  for(; size; --size, ++begin) archive(*begin);
  archive.end_list();
}

/**

Save an array in an output archive.

*/
template<class T>
void save
( OutputArchive &archive ///< An archive.
, const T &value ///< An array.
, typename std::enable_if<bool(SerializableTraits<T>::rank)>::type * = nullptr
)
{
  size_t extents[SerializableTraits<T>::rank];
  save(archive, &value, extents, 0);
}

/**

Save an array in an output archive.

*/
template<class T>
void save
( OutputArchive &archive ///< An archive.
, const T *values ///< An array of values.
, const size_t *extents ///< The extents of \e values.
, size_t rank ///< The rank of \e values.
, typename std::enable_if<!(bool(SerializableTraits<T>::rank) && bool(SerializableTraits<T>::size)) && !std::is_fundamental<T>::value>::type * = nullptr
)
{
  save_range(archive, values, accumulate(extents, extents + rank, 1, std::multiplies<size_t>()));
}

/// \}
}}

#undef _DEFINE_FUNDAMENTAL_LOAD
#undef _DEFINE_FUNDAMENTAL_SAVE
#endif
