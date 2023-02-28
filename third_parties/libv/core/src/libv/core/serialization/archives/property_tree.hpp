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

#ifndef LIBV_CORE_SERIALIZATION_ARCHIVES_PROPERTY_TREE_HPP
#define LIBV_CORE_SERIALIZATION_ARCHIVES_PROPERTY_TREE_HPP

#include <boost/property_tree/ptree.hpp>
#include <stack>

#include "base.hpp"

namespace v {
namespace core {
/// \addtogroup serialization_archives
/// \{

/// Define overloads for each fundamental type.
#define _DEFINE_FUNDAMENTAL_LOAD(T)\
  void serialize(T &);\
  void serialize(T *, const size_t *, size_t);\

/**

An input archive using the Boost Property Tree library.

*/
struct LIBV_CORE_EXPORT PropertyTreeInputArchive
: InputArchive
{
  PropertyTreeInputArchive(const boost::property_tree::ptree &);
private:
  bool begin(const std::string &);
  bool begin(std::string *);
  bool begin();
  void end();
  void begin_list();
  void end_list();
  void get_extents(size_t *, size_t);
  void load_(char *, const size_t *, size_t);
  template<class T> void load_(T &);
  template<class T> void load_(T *, const size_t *, size_t);
  std::stack<const boost::property_tree::ptree *> pointers;
  std::stack<boost::property_tree::ptree::const_iterator> iterators;

  V_FOR_EACH_FUNDAMENTAL_TYPE(_DEFINE_FUNDAMENTAL_LOAD)
};

/// Define overloads for each fundamental type.
#define _DEFINE_FUNDAMENTAL_SAVE(T)\
  void serialize(T);\
  void serialize(const T *, const size_t *, size_t);\

/**

An output archive using the Boost Property Tree library.

*/
struct LIBV_CORE_EXPORT PropertyTreeOutputArchive
: OutputArchive
{
  PropertyTreeOutputArchive(boost::property_tree::ptree &);
  void default_key(const std::string &);
private:
  void begin(const std::string &);
  void begin();
  void end();
  void begin_list(size_t);
  void end_list();
  void save_(const char *, const size_t *, size_t);
  template<class T> void save_(T);
  template<class T> void save_(const T *, const size_t *, size_t);
  std::stack<boost::property_tree::ptree *> pointers;
  std::string default_key_;

  V_FOR_EACH_FUNDAMENTAL_TYPE(_DEFINE_FUNDAMENTAL_SAVE)
};

/// \}
}}

#undef _DEFINE_FUNDAMENTAL_LOAD
#undef _DEFINE_FUNDAMENTAL_SAVE
#endif
