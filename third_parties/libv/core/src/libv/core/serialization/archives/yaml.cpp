#include <libv/core/serialization/archives/yaml.hpp>
#if defined LIBV_CORE_HAS_YAML_ARCHIVES
#include <libv/core/auto_load.hpp>
#include <libv/core/serialization/serializable.hpp>
#include <fstream>

namespace v {
namespace core {
namespace {

template<class T> static void load_recursive(const YAML::Node &, T *&, const size_t *, size_t);
template<class T> static void save_recursive(YAML::Emitter &, const T *&, const size_t *, size_t);

template<class T>
static void load_(const YAML::Node &node, T &value)
{
  assert(node.IsScalar());
  value = node.as<T>();
}

template<class T>
static void load_(const YAML::Node &node, T *&values, const size_t *extents, size_t rank)
{
  if(rank)
  {
    load_recursive(node, values, extents, rank);
  }
  else
  {
    load_(node, *values);
    ++values;
  }
}

static void load_(const YAML::Node &node, char *&values, const size_t *extents, size_t rank)
{
  if(rank > 1)
  {
    load_recursive(node, values, extents, rank);
  }
  else
  {
    assert(node.IsScalar());
    assert(!rank || node.Scalar().size() == *extents);
    copy(node.Scalar().begin(), node.Scalar().end(), values);
    values += node.Scalar().size();
  }
}

template<class T>
static void load_recursive(const YAML::Node &node, T *&values, const size_t *extents, size_t rank)
{
  assert(node.IsSequence());
  for(auto &p: node)
  {
    load_(p, values, extents + 1, rank - 1);
  }
}

template<class T>
static void save_(YAML::Emitter &out, T value)
{
  out << YAML::Value << value;
}

template<class T>
static void save_(YAML::Emitter &out, const T *&values, const size_t *extents, size_t rank)
{
  if(rank)
  {
    save_recursive(out, values, extents, rank);
  }
  else
  {
    save_(out, *values);
    ++values;
  }
}

static void save_(YAML::Emitter &out, const char *&values, const size_t *extents, size_t rank)
{
  if(rank > 1)
  {
    save_recursive(out, values, extents, rank);
  }
  else
  {
    assert(rank);
    save_(out, std::string(values, values + *extents));
    values += *extents;
  }
}

template<class T>
static void save_recursive(YAML::Emitter &out, const T *&values, const size_t *extents, size_t rank)
{
  out << YAML::BeginSeq;
  for(size_t i = 0; i < *extents; ++i)
  {
    save_(out, values, extents + 1, rank - 1);
  }
  out << YAML::EndSeq;
}

static void check_format(std::string file)
{
  file = file.substr(file.rfind('.') + 1);
  transform(file.begin(), file.end(), file.begin(), tolower);
  if(file != "yaml" && file != "yml") throw std::logic_error(file);
}

static struct load_yaml: register_codec<const std::string, Serializable> {
  void operator()(const std::string &input, Serializable &output) const {
    check_format(input);
    std::ifstream stream(input.c_str());
    YamlInputArchive archive(stream);
    v::load(archive, output);
  }} load_yaml;

static struct save_yaml: register_codec<const Serializable, const std::string> {
  void operator()(const Serializable &input, const std::string &output) const {
    check_format(output);
    std::ofstream stream(output.c_str());
    YamlOutputArchive archive(stream);
    v::save(archive, input);
  }} save_yaml;

}

/**
  Initialize this archive with a stream.
*/
YamlInputArchive::YamlInputArchive
( std::istream &stream ///< A reference to a stream.
)
{
  nodes.emplace(YAML::Load(stream));
}

bool YamlInputArchive::begin(const std::string &key)
{
  assert(!nodes.empty());
  assert(nodes.top().IsMap());
  nodes.emplace(nodes.top()[key]);
  if(nodes.top())
  {
    return true;
  }
  else
  {
    nodes.pop();
    return false;
  }
}

bool YamlInputArchive::begin(std::string *key)
{
  assert(!nodes.empty());
  assert(!iterators.empty());
  assert(nodes.top().IsMap());
  assert(iterators.top() != nodes.top().end());
  *key = iterators.top()->first.Scalar();
  return begin();
}

bool YamlInputArchive::begin()
{
  assert(!nodes.empty());
  assert(!iterators.empty());
  assert(nodes.top().IsMap() || nodes.top().IsSequence());
  assert(iterators.top() != nodes.top().end());
  nodes.emplace(nodes.top().IsMap() ? iterators.top()->second : *iterators.top());
  ++iterators.top();
  return true;
}

void YamlInputArchive::end()
{
  assert(!nodes.empty());
  nodes.pop();
}

/**
  Il faut impérativement utiliser `node.reset()`. On ne peut pas simplement faire `node = node[0]` parce que ça détruirait le contenu de `node`.
*/
void YamlInputArchive::get_extents(size_t *extents, size_t rank)
{
  assert(!nodes.empty());
  YAML::Node node(nodes.top());
  for(size_t i = 0; i < rank; ++i)
  {
    if(node.IsMap())
    {
      extents[i] = node.size();
      node.reset(node.begin()->second);
    }
    else if(node.IsSequence())
    {
      extents[i] = node.size();
      node.reset(node[0]);
    }
    else if(node.IsScalar())
    {
      assert(rank < 2);
      extents[i] = node.Scalar().size();
      node.reset();
    }
  }
}

void YamlInputArchive::begin_list()
{
  assert(!nodes.empty());
  iterators.emplace(nodes.top().begin());
}

void YamlInputArchive::end_list()
{
  assert(!iterators.empty());
  iterators.pop();
}

/**
  Initialize this archive with a stream.
*/
YamlOutputArchive::YamlOutputArchive
( std::ostream &stream ///< A reference to a stream.
)
: out(stream)
{
  state.emplace(YAML::Value);
}

void YamlOutputArchive::begin(const std::string &key)
{
  assert(!state.empty());
  assert(state.top() == YAML::BeginMap || state.top() == YAML::Key || state.top() == YAML::Value);
  if(state.top() != YAML::BeginMap)
  {
    out << YAML::BeginMap;
    state.top() = YAML::BeginMap;
  }
  out << YAML::Key << key;
  state.emplace(YAML::Key);
}

void YamlOutputArchive::begin()
{
  assert(!state.empty());
  assert(state.top() == YAML::BeginSeq || state.top() == YAML::Key || state.top() == YAML::Value);
  if(state.top() != YAML::BeginSeq)
  {
    out << YAML::BeginSeq;
    state.top() = YAML::BeginSeq;
  }
  state.emplace(YAML::Auto);
}

void YamlOutputArchive::end()
{
  assert(!state.empty());
  assert(state.top() == YAML::BeginMap || state.top() == YAML::BeginSeq || state.top() == YAML::Key || state.top() == YAML::Value);
  if(state.top() == YAML::BeginMap)
  {
    out << YAML::EndMap;
  }
  if(state.top() == YAML::BeginSeq)
  {
    out << YAML::EndSeq;
  }
  if(state.top() == YAML::Key)
  {
    // Si on a une clé mais pas de valeur, il faut ajouter une valeur vide.
    // Sinon le fichier se casse quand on essaie d'enregistrer des listes vides.
    out << YAML::Null;
  }
  state.pop();
}

void YamlOutputArchive::begin_list(size_t)
{
}

void YamlOutputArchive::end_list()
{
}

/**
  Define overloads for each fundamental type.
*/
#define _(T)\
\
  void YamlInputArchive::serialize(T &value)\
  {\
    load_(nodes.top(), value);\
  }\
\
  void YamlInputArchive::serialize(T *values, const size_t *extents, size_t rank)\
  {\
    load_(nodes.top(), values, extents, rank);\
  }\
\
  void YamlOutputArchive::serialize(T value)\
  {\
    save_(out, value);\
    state.top() = YAML::Value;\
  }\
\
  void YamlOutputArchive::serialize(const T *values, const size_t *extents, size_t rank)\
  {\
    save_(out, values, extents, rank);\
    state.top() = YAML::Value;\
  }\

V_FOR_EACH_FUNDAMENTAL_TYPE(_)

}}

#endif
