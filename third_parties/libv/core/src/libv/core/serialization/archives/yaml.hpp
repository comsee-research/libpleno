#pragma once

#include <libv/core/found/yaml_cpp>
#if \
  defined DOXYGEN ||\
  defined LIBV_CORE_YAML_CPP_FOUND &&\
  1
#define LIBV_CORE_HAS_YAML_ARCHIVES
#include <yaml-cpp/yaml.h>
#include <stack>

#include "base.hpp"

namespace v {
namespace core {
/// \addtogroup serialization_archives
/// \{

/**
  A YAML input archive.
*/
struct LIBV_CORE_EXPORT YamlInputArchive
: InputArchive
{
  YamlInputArchive(std::istream &);
private:
  bool begin(const std::string &) final;
  bool begin(std::string *) final;
  bool begin() final;
  void end() final;
  void begin_list() final;
  void end_list() final;
  void get_extents(size_t *, size_t) final;
  std::stack<YAML::Node> nodes;
  std::stack<YAML::const_iterator> iterators;

  /**
    Declare overloads for each fundamental type.
  */
  #define _(T)\
    void serialize(T &) final;\
    void serialize(T *, const size_t *, size_t) final;\

  V_FOR_EACH_FUNDAMENTAL_TYPE(_)
  #undef _
};

/**
  A YAML output archive.
*/
struct LIBV_CORE_EXPORT YamlOutputArchive
: OutputArchive
{
  YamlOutputArchive(std::ostream &);
private:
  void begin(const std::string &) final;
  void begin() final;
  void end() final;
  void begin_list(size_t) final;
  void end_list() final;
  YAML::Emitter out;
  std::stack<YAML::EMITTER_MANIP> state;

  /**
    Declare overloads for each fundamental type.
  */
  #define _(T)\
    void serialize(T) final;\
    void serialize(const T *, const size_t *, size_t) final;\

  V_FOR_EACH_FUNDAMENTAL_TYPE(_)
  #undef _
};

/// \}
}}

#endif
