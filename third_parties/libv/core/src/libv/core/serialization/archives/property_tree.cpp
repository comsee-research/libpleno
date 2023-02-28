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
#include <libv/core/serialization/archives/property_tree.hpp>
#include <libv/core/serialization/serializable.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/xml_parser.hpp>

using namespace boost::property_tree;

namespace v {
namespace core {
namespace {

template<class T, class F>
static void load_recursive(const ptree &tree, T *values, const size_t *extents, size_t rank, const F &function, std::stack<ptree::const_iterator> &begin)
{
  std::stack<ptree::const_iterator> end;
  end.push(tree.end());
  begin.push(tree.begin());
  for(;;)
  {
    if(begin.top() == end.top())
    {
      end.pop();
      begin.pop();
      if(end.empty()) break;
      else ++begin.top();
    }
    else if(end.size() == rank)
    {
      function(begin.top()->second, values, extents[end.size() - 1]);
      ++begin.top();
    }
    else
    {
      end.push(begin.top()->second.end());
      begin.push(begin.top()->second.begin());
    }
  }
}

template<class T>
static void load_value(const ptree &tree, T *&values, size_t)
{
  try
  {
    *values = tree.get_value<T>();
    ++values;
  }
  catch(ptree_bad_data &)
  {
  }
}

static void load_string(const ptree &tree, char *&values, size_t size)
{
  copy(tree.data().begin(), tree.data().begin() + size, values);
  values += size;
}

template<class T, class F>
static void save_recursive(const T *values, const size_t *extents, size_t rank, const F &function, std::stack<ptree *> pointers, const std::string &default_key)
{
  size_t i = 1;
  for(;;)
  {
    if(pointers.top()->size() == extents[i - 1])
    {
      pointers.pop();
      --i;
      if(!i) break;
    }
    else if(i == rank)
    {
      function(*pointers.top(), values, extents[i - 1], default_key);
    }
    else
    {
      pointers.push(&pointers.top()->push_back({default_key, {}})->second);
      ++i;
    }
  }
}

template<class T>
static void save_value(ptree &tree, const T *&values, size_t, const std::string &default_key)
{
  tree.push_back({default_key, {}})->second.put_value(*values);
  ++values;
}

static void save_string(ptree &tree, const char *&values, size_t size, const std::string &)
{
  tree.put_value(std::string(values, values + size));
  values += size;
}

static struct load_boost_property_tree_ini: register_codec<const std::string, Serializable> {
  void operator()(const std::string &input, Serializable &output) const {
    ptree tree;
    read_ini(input, tree);
    PropertyTreeInputArchive archive(tree);
    v::load(archive, output);
  }} load_ini;

static struct load_boost_property_tree_xml: register_codec<const std::string, Serializable> {
  void operator()(const std::string &input, Serializable &output) const {
    ptree tree;
    read_xml(input, tree, xml_parser::trim_whitespace);
    PropertyTreeInputArchive archive(tree.front().second);
    v::load(archive, output);
  }} load_xml;

static struct load_boost_property_tree_json: register_codec<const std::string, Serializable> {
  void operator()(const std::string &input, Serializable &output) const {
    ptree tree;
    read_json(input, tree);
    PropertyTreeInputArchive archive(tree);
    v::load(archive, output);
  }} load_json;

// low priority because read_info accepts binary files
static struct load_boost_property_tree_info: register_codec<const std::string, Serializable, -1000> {
  void operator()(const std::string &input, Serializable &output) const {
    std::string format = input.substr(input.rfind('.') + 1);
    transform(format.begin(), format.end(), format.begin(), tolower);
    if(format != "info" && format != "ini") throw std::logic_error(format);
    ptree tree;
    read_info(input, tree);
    PropertyTreeInputArchive archive(tree);
    v::load(archive, output);
  }} load_info;

static struct save_boost_property_tree: register_codec<const Serializable, const std::string> {
  void operator()(const Serializable &input, const std::string &output) const {
    ptree tree;
    PropertyTreeOutputArchive archive(tree);
    std::string format = output.substr(output.rfind('.') + 1);
    transform(format.begin(), format.end(), format.begin(), tolower);
    if(format == "ini")
    {
      archive.default_key("*");
      v::save(archive, input);
      return write_ini(output, tree, 0, std::locale::classic());
    }
    if(format == "xml")
    {
      archive.default_key("_");
      archive(input);
      return write_xml(output, tree, std::locale::classic());
    }
    if(format == "js" || format == "json")
    {
      v::save(archive, input);
      return write_json(output, tree, std::locale::classic());
    }
    if(format == "info")
    {
      archive.default_key("*");
      v::save(archive, input);
      return write_info(output, tree, std::locale::classic(), info_writer_make_settings('\t', 1));
    }
    throw std::logic_error(format);
  }} save_any;

}

/**

Initialize this archive with a property tree.

*/
PropertyTreeInputArchive::PropertyTreeInputArchive
( const ptree &tree ///< A property tree.
)
{
  pointers.push(&tree);
}

bool PropertyTreeInputArchive::begin(const std::string &key)
{
  try
  {
    pointers.push(&pointers.top()->get_child(key));
    return true;
  }
  catch(ptree_bad_path &)
  {
    return false;
  }
}

bool PropertyTreeInputArchive::begin(std::string *key)
{
  *key = iterators.top()->first;
  return begin();
}

bool PropertyTreeInputArchive::begin()
{
  pointers.push(&iterators.top()->second);
  ++iterators.top();
  return true;
}

void PropertyTreeInputArchive::end()
{
  pointers.pop();
}

void PropertyTreeInputArchive::get_extents(size_t *extents, size_t rank)
{
  const ptree *p = pointers.top();
  for(size_t i = 0; i < rank; ++i)
  {
    if(p->empty())
    {
      extents[i] = p->data().size();
    }
    else
    {
      extents[i] = p->size();
      p = &p->front().second;
    }
  }
}

void PropertyTreeInputArchive::begin_list()
{
  iterators.push(pointers.top()->begin());
}

void PropertyTreeInputArchive::end_list()
{
  iterators.pop();
}

void PropertyTreeInputArchive::load_(char *values, const size_t *extents, size_t rank)
{
  if(rank == 1)
  {
    load_string(*pointers.top(), values, *extents);
  }
  else
  {
    load_recursive(*pointers.top(), values, extents, rank - 1, load_string, iterators);
  }
}

template<class T>
void PropertyTreeInputArchive::load_(T &value)
{
  try
  {
    value = pointers.top()->get_value<T>();
  }
  catch(ptree_bad_data &)
  {
  }
}

template<class T>
void PropertyTreeInputArchive::load_(T *values, const size_t *extents, size_t rank)
{
  load_recursive(*pointers.top(), values, extents, rank, load_value<T>, iterators);
}

/**

Initialize this archive with a property tree.

*/
PropertyTreeOutputArchive::PropertyTreeOutputArchive
( ptree &tree ///< A reference to a property tree.
)
{
  pointers.push(&tree);
}

void PropertyTreeOutputArchive::default_key(const std::string &key)
{
  default_key_ = key;
}

void PropertyTreeOutputArchive::begin(const std::string &key)
{
  pointers.push(&pointers.top()->push_back({key, {}})->second);
}

void PropertyTreeOutputArchive::begin()
{
  begin(default_key_);
}

void PropertyTreeOutputArchive::end()
{
  pointers.pop();
}

void PropertyTreeOutputArchive::begin_list(size_t)
{
}

void PropertyTreeOutputArchive::end_list()
{
}

void PropertyTreeOutputArchive::save_(const char *values, const size_t *extents, size_t rank)
{
  if(rank == 1)
  {
    save_string(*pointers.top(), values, *extents, default_key_);
  }
  else
  {
    save_recursive(values, extents, rank - 1, save_string, pointers, default_key_);
  }
}

template<class T>
void PropertyTreeOutputArchive::save_(T value)
{
  pointers.top()->put_value(value);
}

template<class T>
void PropertyTreeOutputArchive::save_(const T *values, const size_t *extents, size_t rank)
{
  save_recursive(values, extents, rank, save_value<T>, pointers, default_key_);
}

/// Define overloads for each fundamental type.
#define _DEFINE_FUNDAMENTAL_TYPE(T)\
\
  void PropertyTreeInputArchive::serialize(T &value)\
  {\
    load_(value);\
  }\
\
  void PropertyTreeInputArchive::serialize(T *values, const size_t *extents, size_t rank)\
  {\
    load_(values, extents, rank);\
  }\
\
  void PropertyTreeOutputArchive::serialize(T value)\
  {\
    save_(value);\
  }\
\
  void PropertyTreeOutputArchive::serialize(const T *values, const size_t *extents, size_t rank)\
  {\
    save_(values, extents, rank);\
  }\

V_FOR_EACH_FUNDAMENTAL_TYPE(_DEFINE_FUNDAMENTAL_TYPE)

}}
