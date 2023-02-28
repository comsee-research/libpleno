/**

\file
Définition de la classe RosInputArchive pour importer les configurations fournies par ROS dans les structures Libv.
Les déclarations sont dans ros.hpp.

Ces fonctions étaient initialement définies dans la bibliothèque libv_ros écrite par Gérald Lelong.

\author Gérald Lelong (2018)
\author Alexis WIlhelm (2018)

*/

/**

\class RosInputArchive
An input archive using the XmlRpc++ library from ROS.

Cet objet s'utilise en trois temps :
- déclarer une structure de configurations : `MyConfig config;`
- construire une archive en lecture : `RosInputArchive archive(xmlrpc_value);`
- copier les configurations dans la structure : `load(archive, config);`

\note
On passe par un boost::property_tree et un PropertyTreeInputArchive parce qu'historiquement ça s'est fait comme ça, mais idéalement il faudrait désérialiser directement depuis le XmlRpcValue.

\note
PropertyTreeInputArchive suppose que la durée de vie du property_tree qu'on lui passe est suffisamment longue, donc il ne manipule que des pointeurs.
Ici, le property_tree est construit à partir du XmlRpcValue dans le constructeur de RosInputArchive, donc il faut le copier dans #tree_ pour qu'il survive pendant qu'on utilise notre RosInputArchive.

*/

#include <libv/core/found/roscpp>
#if defined LIBV_CORE_ROSCPP_FOUND

#include <libv/core/serialization/archives/ros.hpp>
#include <ros/console.h>

namespace v {
namespace core {
namespace {

using namespace boost::property_tree;

std::string getString(XmlRpc::XmlRpcValue);
ptree parseStruct(XmlRpc::XmlRpcValue);
ptree parseSequence(XmlRpc::XmlRpcValue);

std::string getString(XmlRpc::XmlRpcValue param)
{
  switch (param.getType())
  {
    case XmlRpc::XmlRpcValue::TypeBoolean:
    {
      return static_cast<bool>(param) ? "true" : "false";
    }

    case XmlRpc::XmlRpcValue::TypeInt:
    {
      return std::to_string(static_cast<int>(param));
    }

    case XmlRpc::XmlRpcValue::TypeDouble:
    {
      std::stringstream stream;
      stream.imbue(std::locale::classic());
      stream << static_cast<double>(param);
      return stream.str();
    }

    case XmlRpc::XmlRpcValue::TypeString:
    {
      return static_cast<std::string>(param);
    }

    case XmlRpc::XmlRpcValue::TypeDateTime:
    {
      struct tm date_time = static_cast<struct tm>(param);
      return std::string(asctime(&date_time));
    }

    case XmlRpc::XmlRpcValue::TypeBase64:
    {
      XmlRpc::XmlRpcValue::BinaryData& binary_data = param;
      return std::string(binary_data.data());
    }

    default:
    {
      ROS_WARN_STREAM("Invalid parameter type found.");
      abort();
    }
  }
}

ptree parseStruct(XmlRpc::XmlRpcValue params)
{
  ptree tree;

  for (auto& it: params)
  {
    const std::string& param_name = it.first;
    XmlRpc::XmlRpcValue& param = it.second;

    switch (param.getType())
    {
      case XmlRpc::XmlRpcValue::TypeArray:
      {
        ptree subptree = parseSequence(param);
        tree.add_child(param_name, subptree);
        break;
      }

      case XmlRpc::XmlRpcValue::TypeStruct:
      {
        ptree subptree = parseStruct(param);
        tree.add_child(param_name, subptree);
        break;
      }

      default:
      {
        const std::string& value = getString(param);
        tree.add(param_name, value);
        break;
      }
    }
  }

  return tree;
}

ptree parseSequence(XmlRpc::XmlRpcValue params)
{
  ptree tree;

  for (int i = 0; i < params.size(); ++i)
  {
    XmlRpc::XmlRpcValue& param = params[i];

    switch (param.getType())
    {
      case XmlRpc::XmlRpcValue::TypeArray:
      {
        tree.push_back({{}, parseSequence(param)});
        break;
      }

      case XmlRpc::XmlRpcValue::TypeStruct:
      {
        tree.push_back({{}, parseStruct(param)});
        break;
      }

      default:
      {
        ptree item;
        item.put_value(getString(param));
        tree.push_back({{}, item});
        break;
      }
    }
  }

  return tree;
}

ptree parse(XmlRpc::XmlRpcValue params)
{
  switch(params.getType())
  {
    case XmlRpc::XmlRpcValue::TypeArray: return parseSequence(params);
    case XmlRpc::XmlRpcValue::TypeStruct: return parseStruct(params);
    default: throw params;
  }
}

}

RosInputArchive::RosInputArchive(XmlRpc::XmlRpcValue params)
: PropertyTreeInputArchive(tree_)
, tree_(parse(params))
{
}

}}

#endif
