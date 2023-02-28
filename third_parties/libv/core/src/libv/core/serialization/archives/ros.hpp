#ifndef LIBV_CORE_SERIALIZATION_ARCHIVES_ROS_HPP
#define LIBV_CORE_SERIALIZATION_ARCHIVES_ROS_HPP

#include <libv/core/found/roscpp>
#if defined LIBV_CORE_ROSCPP_FOUND || !defined LIBV_IGNORE_OPTIONAL_DEPENDENCIES

#include <libv/core/serialization/archives/property_tree.hpp>
#include <xmlrpcpp/XmlRpcValue.h>

namespace v {
namespace core {
/// \addtogroup serialization_archives
/// \{

struct LIBV_CORE_EXPORT RosInputArchive
: PropertyTreeInputArchive
{
  RosInputArchive(XmlRpc::XmlRpcValue);
private:
  const boost::property_tree::ptree tree_;
};

/// \}
}
}

#endif
#endif
