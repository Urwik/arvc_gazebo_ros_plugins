#pragma once
#include <filesystem>
#include <gazebo/gazebo.hh>
#include <yaml-cpp/yaml.h>

namespace fs = std::filesystem;
namespace im = ignition::math;

namespace YAML
{
  template<>
  struct convert<im::Vector3d>
  {
    static Node encode(const im::Vector3d& v3d)
    {
      Node node;
      node.push_back(v3d.X());
      node.push_back(v3d.Y());
      node.push_back(v3d.Z());
      return node;
    }

    static bool decode(const Node& node, im::Vector3d& v3d)
    {
      if(!node.IsSequence() || node.size() != 3) {
        return false;
      }

      double x = node[0].as<double>();
      double y = node[1].as<double>();
      double z = node[2].as<double>();

      v3d.Set(x, y, z);

      return true;
    }
  };

  template<>
  struct convert<std::filesystem::path> {
    static Node encode(const std::filesystem::path& path) {
      Node node;
      node = path.string();
      return node;
    }

    static bool decode(const Node& node, std::filesystem::path& path) {
      if(!node.IsScalar()) {
        return false;
      }

      path = node.as<std::string>();
      return true;
    }
  };

    template<>
  struct convert<im::Vector2d>
  {
    static Node encode(const im::Vector2d& v2d)
    {
      Node node;
      node.push_back(v2d.X());
      node.push_back(v2d.Y());
      return node;
    }

    static bool decode(const Node& node, im::Vector2d& v2d)
    {
      if(!node.IsSequence() || node.size() != 2) {
        return false;
      }

      double x = node[0].as<double>();
      double y = node[1].as<double>();

      v2d.Set(x, y);

      return true;
    }
  };
}