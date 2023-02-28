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

#include <libv/core/auto_load.hpp>
#include <libv/core/serialization/archives/simple_text.hpp>
#include <libv/core/serialization/serializable.hpp>
#include <boost/format.hpp>
#include <algorithm>
#include <cstdio>
#include <fstream>
#include <iostream>

namespace v {
namespace core {
namespace {

template<class T>
static void load_(std::istream &stream, T &value)
{
  stream >> value;
}

template<class T>
static void load_(std::istream &stream, T *values, const size_t *extents, size_t rank)
{
  size_t size = accumulate(extents, extents + rank, 1, std::multiplies<size_t>());
  for(size_t i = 0; i < size; ++i) load_(stream, values[i]);
}

static void load_(std::istream &stream, char &value)
{
  static const size_t size = 1;
  load_(stream, &value, &size, 1);
}

static void load_(std::istream &stream, char *values, const size_t *extents, size_t rank)
{
  std::string buffer;
  load_(stream, buffer);
  size_t size = accumulate(extents, extents + rank, 1, std::multiplies<size_t>());
  unsigned c;

  for(std::string::const_iterator p = buffer.begin(); p < buffer.end() && size; ++p, --size)
  {
    if(*p == '%')
    {
      std::sscanf(&*(++p)++, "%2x", &c);
      *values++ = c;
    }
    else *values++ = *p;
  }
}

template<class T>
static void save_(std::ostream &stream, T value)
{
  stream << value << std::endl;
}

static size_t save_(std::ostream &stream, const size_t *extents, size_t rank)
{
  size_t size = 1;

  for(size_t i = 0; i < rank; ++i)
  {
    save_(stream, extents[i]);
    size *= extents[i];
  }

  return size;
}

template<class T>
static void save_(std::ostream &stream, const T *values, const size_t *extents, size_t rank)
{
  size_t size = save_(stream, extents, rank);

  for(size_t i = 0; i < size; ++i)
  {
    save_(stream, values[i]);
  }
}

static void save_(std::ostream &stream, char value)
{
  static const size_t size = 1;
  save_(stream, &value, &size, 1);
}

static void save_(std::ostream &stream, const char *values, const size_t *extents, size_t rank)
{
  size_t size = save_(stream, extents, rank);
  std::ostringstream s;

  for(size_t i = 0; i < size; ++i)
  {
    if(std::isalnum(values[i])) s << values[i];
    else s << boost::format("%%%02x") % size_t(values[i]);
  }

  save_(stream, s.str());
}

static void check_format(std::string file)
{
  file = file.substr(file.rfind('.') + 1);
  transform(file.begin(), file.end(), file.begin(), tolower);
  if(file != "txt") throw std::logic_error(file);
}

static struct load_simple_txt: register_codec<const std::string, Serializable> {
  void operator()(const std::string &input, Serializable &output) const {
    check_format(input);
    std::ifstream stream(input.c_str());
    SimpleTextInputArchive archive(stream);
    v::load(archive, output);
  }} load_txt;

static struct save_simple_txt: register_codec<const Serializable, const std::string> {
  void operator()(const Serializable &input, const std::string &output) const {
    check_format(output);
    std::ofstream stream(output.c_str());
    SimpleTextOutputArchive archive(stream);
    v::save(archive, input);
  }} save_txt;

}

/**

Initialize this archive with a stream.

*/
SimpleTextInputArchive::SimpleTextInputArchive
( std::istream &stream ///< A reference to a stream.
)
: stream(stream)
{
}

bool SimpleTextInputArchive::begin(const std::string &)
{
  return true;
}

bool SimpleTextInputArchive::begin(std::string *)
{
  return true;
}

bool SimpleTextInputArchive::begin()
{
  return true;
}

void SimpleTextInputArchive::end()
{
}

void SimpleTextInputArchive::begin_list()
{
}

void SimpleTextInputArchive::end_list()
{
}

void SimpleTextInputArchive::get_extents(size_t *extents, size_t rank)
{
  load_(stream, extents, &rank, 1);
}

/**

Initialize this archive with a stream.

*/
SimpleTextOutputArchive::SimpleTextOutputArchive
( std::ostream &stream ///< A reference to a stream.
)
: stream(stream)
{
}

void SimpleTextOutputArchive::begin(const std::string &)
{
}

void SimpleTextOutputArchive::begin()
{
}

void SimpleTextOutputArchive::end()
{
}

void SimpleTextOutputArchive::begin_list(size_t size)
{
  save_(stream, size);
}

void SimpleTextOutputArchive::end_list()
{
}

/// Define overloads for each fundamental type.
#define _DEFINE_FUNDAMENTAL_TYPE(T)\
\
  void SimpleTextInputArchive::serialize(T &value)\
  {\
    load_(stream, value);\
  }\
\
  void SimpleTextInputArchive::serialize(T *values, const size_t *extents, size_t rank)\
  {\
    load_(stream, values, extents, rank);\
  }\
\
  void SimpleTextOutputArchive::serialize(T value)\
  {\
    save_(stream, value);\
  }\
\
  void SimpleTextOutputArchive::serialize(const T *values, const size_t *extents, size_t rank)\
  {\
    save_(stream, values, extents, rank);\
  }\

V_FOR_EACH_FUNDAMENTAL_TYPE(_DEFINE_FUNDAMENTAL_TYPE)

}}
