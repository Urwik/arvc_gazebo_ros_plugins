#pragma once

#include <filesystem>
#include <iostream>
#include <algorithm>
#include <math.h>
#include <yaml-cpp/yaml.h>

//MULTITHREADING
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <rosgraph_msgs/Log.h>


#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/gui/gui.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/common.hh>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Box.hh>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

#include <sdf/sdf.hh>
#include "arvc_gazebo_ros_plugins/utils.hpp"

namespace fs = std::filesystem;
using namespace std;

#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"  
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"



// Type Definitions ////////////////////////////////////////////////////////////
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloud;

namespace gazebo
{
  // Register this plugin with the simulator

class MoveModel : public WorldPlugin
{

////////////////////////////////////////////////////////////////////////////////
  public: 
  MoveModel();

  
////////////////////////////////////////////////////////////////////////////////
  void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf); 



////////////////////////////////////////////////////////////////////////////////
  void GenerateDataset();



////////////////////////////////////////////////////////////////////////////////
  void ParseArgs(sdf::ElementPtr sdf);

  /////////////////////////////////
  void GetYamlConfig();


  ////////////////////////////////////////////////////////////////////////////////
  bool MobileModelReady();


  ////////////////////////////////////////////////////////////////////////////////
  void MoveMobileModel();




////////////////////////////////////////////////////////////////////////////////
  void SavePointCloud(PointCloud::Ptr cloud);


////////////////////////////////////////////////////////////////////////////////
  void CheckOutputDirs();

////////////////////////////////////////////////////////////////////////////////
  void SetupROS();

////////////////////////////////////////////////////////////////////////////////
  void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input);

////////////////////////////////////////////////////////////////////////////////
  void QueueThread();


  bool ValidPose(ignition::math::AxisAlignedBox _sensor_bbx);






  // VARIABLES
  
  // GAZEBO
  private:
    physics::WorldPtr world;
    physics::ModelPtr mobile_model;
    physics::ModelPtr sensor_model;
    std::vector<physics::ModelPtr> collisionable_models;
    physics::ModelPtr fixed_model;
    event::ConnectionPtr updateConnection;

    // CONFIGURATION
    YAML::Node config;
    std::string RANDMODE;
    int NUM_ENV;
    //  int NUM_MODELS;
    std::filesystem::path output_dir;
    std::filesystem::path pcd_dir;
    std::string world_name;
    std::string fixed_model_name;
    std::string sensor_name;
    std::string sensor_topic;

    bool save_pcd;
    bool pcd_binary;

    uint sensor_offset;

    ignition::math::Vector3d origin;
    float range;

    // ROS
    ros::NodeHandle* ros_node;
    ros::Subscriber ros_sub;
    ros::SubscribeOptions ros_so;
    ros::CallbackQueue ros_cbqueue;
    boost::thread callback_queue_thread;


    // PCL
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud;


    // HELPERS
    bool debug_msgs;
    int env_count;
    bool paused;
    boost::thread generator_thread;
    std::vector<ignition::math::AxisAlignedBox> links_bbx;
    int callback_count;
    float truss_offset;

};

}




namespace YAML 
{
  template<>
  struct convert<ignition::math::Vector3d> 
  {
    static Node encode(const ignition::math::Vector3d& v3d) 
    {
      Node node;
      node.push_back(v3d.X());
      node.push_back(v3d.Y());
      node.push_back(v3d.Z());
      return node;
    }

    static bool decode(const Node& node, ignition::math::Vector3d& v3d) 
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
}