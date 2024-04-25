// C++
#include <filesystem>
#include <thread>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <math.h>

// #include <boost/thread.hpp>
// #include <boost/bind.hpp>
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

#include "arvc_gazebo_ros_plugins/arvc_dataset_generator_utils.hpp"
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

    /// @brief Executes once after Load function
    void Init();

    /// @brief Executes every world update event
    void OnUpdate();

    /// @brief Main function that executes the dataset generation.
    void GenerateDataset();

    /**
     * @brief Parse arguments to configure the plugin. Gets the value of the arguments
     * in the configuration file. This values can also be set inside element <plugin>
     * in the ".world" file.
     * @param sdf sdf element to the ".world"
     */
    void ParseArgs(sdf::ElementPtr sdf);

    /// @brief Read configuration file to set parameters to the plugin.
    void GetYamlConfig();

    /// @brief Inserts a camera model from an ".sdf"
    void InsertCameraModel();

    /// @brief Gets a pointer to the camera
    bool GetCameraPointer();

    /// @brief Saves an image of the enviroment
    void TakeScreenShot();

    /// @brief Gets transform between camera and sensor
    im::Pose3d GetCameraSensorTF();

    /// @brief Gets transform between camera and sensor
    void SaveCameraSensorTF();

    /// @brief Gets transform between camera and sensor
    void SaveCameraParams();

    /**
     * @brief Gets a pointer to an sdf file from a path
     * @param sdfPath Absolute path to the sdf file
     * @return pointer to the sdf file
     */
    sdf::SDFPtr GetSDFfile(fs::path sdfPath);

    /**
     * @brief Makes a copy of the model file so it can be transformed as many times
     * as you want from the original model.
     * @param path Absolute path to the model file.
     * @return Absolute path to the copy of the model file. It renames it with
     * suffix "_copy".
     */
    fs::path GetTemporarySDFfile(fs::path sdfPath);

    /**
     * @brief Reset temporary file as its original file
     * @param sdfPath Absolute path to the original sdf file
     * @return absolute path to the temporal sdf file
     */
    fs::path ResetTemporarySDFfile(fs::path sdfPath);

    void InsertModel(int _model_index);

    /// @brief Insert labeled cuboid models in random scales and poses
    std::vector<std::string> SpawnRandomParalellepipeds();

    /// @brief Insert unlabeled models as a perturbations to the world
    std::vector<std::string> SpawnRandomEnviroment();

    std::vector<std::string> SpawnElements(arvc::plugin::model_base[] elements);

    /// @brief Delete all models in the world except os_128, camera, and world
    void removeModels();

    /**
     * @brief Remove models
     * @param models Vector of strings with model names
     */
    void removeModelsByName(std::vector<std::string> models);

    /**
     * @brief Set model name consecutively for each model inserted in the world.
     * For each model inserted its name appends an integer describing its name.
     * @param modelElement sdf::ElementPtr to the model element.
     * @param cnt number of the spawned model
     */
    std::string SetModelName(sdf::ElementPtr modelElement, std::string _model_name, int count);

    /**
     * @brief Set random pose to a model.
     * @param modelElement sdf::ElementPtr to the model element.
     */
    void SetModelPose(sdf::ElementPtr modelElement);

    /**
     * @brief Set random pose to a model.
     * @param modelElement sdf::ElementPtr to the model element.
     */
    void SetModelPose(sdf::ElementPtr modelElement, arvc::plugin::model_base model_cfg);

    void DatasetGenerator::SetModelPosition(sdf::ElementPtr modelElement, arvc::plugin::model_base model_cfg);

    void DatasetGenerator::SetModelOrientation(sdf::ElementPtr modelElement);

    /**
     * Set random scale of a model in all its axes.
     * @param modelElement sdf::ElementPtr to the model element.
     */
    void SetRandomScale(sdf::ElementPtr model, Eigen::Vector3f _min_scale, Eigen::Vector3f _max_scale);

    /**
     * Set random scale of a model in all its axes.
     * @param modelElement sdf::ElementPtr to the model element.
     */
    void SetRandomScale(sdf::ElementPtr model, arvc::plugin::model_base model_cfg);

    /**
     * @brief Set random scale in 3 axis to a model.
     * @param modelElement sdf::ElementPtr to the model element.
     */
    void SetRandomMeshScale(sdf::ElementPtr model);

    /**
     * @brief Set laser retro consecutively for each visual element in the model.
     * @param model sdf::ElementPtr to the model element.
     * @return void.
     */
    void IncreaseVisualLaserRetro(sdf::ElementPtr model);

    /// @brief Moves groud model randomly
    void MoveGroundModel();

    /// @brief Check output directories format, and create if don't exists
    void CheckOutputDirs();

    /// @brief Get last saved cloud by writing time and set env count to this value
    /// to continue from that number
    void ResumeEnvCount();

    /**
     * @brief Check if sensor is currently working.
     * @return true if sensor is working.
     */
    bool SensorReady();

    /**
     * @brief Check if models are correctly spawned in the world
     * @param model_names Vector of strings with model names
     * @return true if all models are spawned correctly
     */
    bool CheckSpawnedModels(std::vector<std::string> model_names);

    /**
     * @brief Check if models are correctly removed from the world
     * @param model_names Vector of strings with model names
     * @return true if all models are removed correctly
     */
    bool CheckDeletedModels(std::vector<std::string> model_names);

    /// @brief Saves last published PointCloud in a global variable (pcl_cloud)
    void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &input);

    /// @brief Save last published cloud as a file in ".pcd"
    void SavePointCloud();

    /**
     * @brief Compute random pose X Y Z R P Y
     * @return Return the random pose
     */
    im::Pose3d ComputeRandomPose();

    /**
     * @brief Compute random pose X Y Z R P Y
     * @return Return the random pose
     */
    im::Pose3d ComputeRandomPose(arvc::plugin::model_base model_cfg);

    /**
     * @brief Compute random pose X Y Z R P Y
     * @return Return the random pose
     */
    im::Pose3d ComputeWorldRandomPose();

    /**
     * @brief Compute random position X Y Z
     * @return Return the random position
     */
    im::Vector3d ComputeEnvRandPosition();

    im::Vector3d DatasetGenerator::ComputeRandomPosition(arvc::plugin::model_base model_cfg);

    /**
     * @brief Compute random rotation R P Y
     * @return Return the random orientation
     */
    im::Vector3d ComputeRandRotation();

    /**
     * @brief Compute random rotation R P Y
     * @return Return the random orientation
     */
    im::Vector3d DatasetGenerator::ComputeRandomRotation(arvc::plugin::model_base model_cfg);

    /**
     * @brief Compute random scale in 3 axis (X, Y, Z)
     * @return the vector with the values of the scale.
     */
    im::Vector3d ComputeRandomScale();

    /**
     * @brief Compute random scale in 3 axis (X, Y, Z)
     * @return the vector with the values of the scale.
     */
    im::Vector3d ComputeRandomScale(im::Vector3d min_scale_, im::Vector3d max_scale_);

    /**
     * @brief Compute random scale in 3 axis (X, Y, Z)
     * @return the vector with the values of the scale.
     */
    im::Vector3d ComputeRandomScale(arvc::plugin::model_base model_cfg);

    /**
     * @brief Check that pose dont lies inside truss structure
     * @return Return true if pose is valid
     */
    bool ReachPositionOffset(im::Pose3d pose);

    /**
     * @brief Remove models that collide with sensor
     *
     */
    std::vector<std::string> RemoveCollideModels(physics::ModelPtr sensor_model);

    /**
     * @brief Apply offset to the passed coordinate.
     * @return the new vector
     */
    im::Vector3d ApplyOffset(im::Vector3d input);

    /**
     * @brief Apply offset to the passed coordinate.
     * @return the new vector
     */
    im::Vector3d ApplyOffset(im::Vector3d input, im::Vector3d offset_);

    /**
     * @brief Apply offset to the passed coordinate.
     * @return the new vector
     */
    im::Vector3d DatasetGenerator::ApplySensorOffset(im::Vector3d position);

    /**
     * @brief Aplly rotation to a model
     * @param model_ptr Pointer to a model in gazebo
     * @param rotation Rotation vector R P Y
     */
    void ApplyRotation(physics::ModelPtr model_ptr, im::Vector3d rotation);

    /**
     * @brief Return the number of files in an existing directory.
     * @param path The absolute path to the directory.
     * @return Integer with the number of files in directory.
     */
    int GetNumOfItems(fs::path path);

    /**
     * @brief Set a random weight for each model which will be used to set the number
     * of copies of each model to insert in the world
     * @param path The absolute path to the directory.
     * @return vector with wei
     */
    Eigen::VectorXf SetModelWeights(fs::path path);

    /// @brief Setup ROS configuration
    void SetupROS();

    /// @brief Thred that manages callbacks in ROS
    void QueueThread();

    // VARIABLES ////////////////////
  private:
    // GAZEBO
    physics::WorldPtr world;
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;
    vector<string> inserted_labeled_models_names;
    vector<string> inserted_environment_models_names;

    // SENSORS
    physics::ModelPtr sensor_model;
    physics::ModelPtr camera_model;
    sensors::CameraSensorPtr camera;
    im::Pose3d camera_pose;
    std::string cam_name;

    // CONFIGURATION
    arvc::plugin::configuration config;
    int num_paralelelpipeds;

    // ENV
    int num_env_models;


    // ROS
    ros::NodeHandle *ros_node;
    ros::Subscriber ros_sub;
    ros::SubscribeOptions ros_so;
    ros::CallbackQueue ros_cbqueue;
    boost::thread callback_queue_thread;

    // DIRECTORIES
    fs::path pcd_dir;
    fs::path img_dir;

    // PCL
    PointCloudI::Ptr cloud_I;
    PointCloudL::Ptr cloud_L;

    // HELPERS

    sdf::ElementPtr inserting_model;
    arvc::plugin::model_base inserting_model_cfg;
    bool ousterReady;
    bool handle_to_cam;
    bool take_screenshot;
    int env_count;
    int laser_retro;
    std::thread generator_thread;
    bool pc_binary;
    std::string sensor_topic;

    // UTILS
    utils::Console console;
  };
}