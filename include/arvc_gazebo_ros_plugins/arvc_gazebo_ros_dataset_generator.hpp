#pragma once
// C++
#include <filesystem>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <math.h>
// #include <thread>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/config.hpp>
#include <yaml-cpp/yaml.h>

// GAZEBO
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/common/Plugin.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <gazebo/common/Console.hh>


// Eigen
#include <Eigen/Dense>

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

/// PCL Libraries
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/visualization/pcl_visualizer.h>

// #include "arvc_gazebo_ros_plugins/arvc_dataset_generator_utils.hpp"
#include "console_utils.hpp"

#define RESET "\033[0m"
#define RED "\033[31m"
#define GREEN "\033[32m"
#define YELLOW "\033[33m"
#define BLUE "\033[34m"

typedef pcl::PointXYZI PointI;
typedef pcl::PointCloud<PointI> PointCloudI;

typedef pcl::PointXYZL PointL;
typedef pcl::PointCloud<PointL> PointCloudL;

namespace fs = std::filesystem;
namespace im = ignition::math;

namespace gazebo
{
  class DatasetGenerator : public WorldPlugin
  {

  public:
    /// @brief Constructor
    DatasetGenerator();

    /// @brief Destructor
    ~DatasetGenerator();

  private:
    /// @brief Load the plugin. Executes once at start
    void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

    /// @brief Main function that executes the dataset generation.
    void GenerateDataset();

    /**
     * @brief Parse arguments to configure the plugin. Gets the value of the arguments
     * in the configuration file. This values can also be set inside element <plugin>
     * in the ".world" file.
     * @param sdf sdf element to the ".world"
     */
    void getConfig(sdf::ElementPtr _sdf);

    void insertSensorModel();

    void rotateSensorModel();

    /**
     * @brief Remove models
     * @param models Vector of strings with model names
     */
    void removeModelsByName(std::vector<std::string> models);

    /// @brief Insert labeled cuboid models in random scales and poses
    std::vector<std::string> SpawnRandomParalellepipeds();

    /// @brief Insert unlabeled models as a perturbations to the world
    std::vector<std::string> SpawnRandomEnviroment();


  void moveDownTillCollisionWithGround(std::vector<std::string>  model_name);


    /// @brief Check output directories format, and create if don't exists
    void CheckOutputDirs();

    /// @brief Get last saved cloud by writing time and set env count to this value
    /// to continue from that number
    void ResumeEnvCount();


    bool checkCollisions(std::string model_name_1, std::string model_name_2);


    /// @brief Save last published cloud as a file in ".pcd"
    void SavePointCloud();


    /// @brief Setup ROS configuration
    void SetupROS();

    /// @brief Thred that manages callbacks in ROS
    void QueueThread();

    /// @brief Saves last published PointCloud in a global variable (pcl_cloud)
    void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &input);

    // VARIABLES ////////////////////
  private:
    // GAZEBO
    physics::WorldPtr world;
    physics::ModelPtr sensor_model;

    // ROS
    ros::NodeHandle *ros_node;
    ros::Subscriber ros_sub;
    ros::SubscribeOptions ros_so;
    ros::CallbackQueue ros_cbqueue;
    boost::thread callback_queue_thread;

    // DIRECTORIES
    fs::path pcd_dir;

    // PCL
    PointCloudI::Ptr cloud_I;
    PointCloudL::Ptr cloud_L;

    // HELPERS

    int env_count;
    int laser_retro;
    boost::thread generator_thread;

    // UTILS
    utils::Console console;

    // YAML
    YAML::Node config;
  };
}