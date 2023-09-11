// C++
#include <filesystem>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <yaml-cpp/yaml.h>

// GAZEBO
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>


// ROS
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <sensor_msgs/PointCloud2.h>

/// PCL Libraries
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "arvc_gazebo_ros_plugins/arvc_dataset_generator_utils.hpp"

namespace gazebo
{
  class DatasetGenerator : public WorldPlugin
  {
    
    /// @brief Constructor
    public: DatasetGenerator();

    /// @brief Destructor
    public: ~DatasetGenerator();

    /// @brief Load the plugin. Executes once at start
    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

    /// @brief Executes once after Load function
    public: void Init();

    /// @brief Executes every world update event
    private: void OnUpdate();

    /// @brief Main function that executes the dataset generation.
    private: void GenerateDataset();

    /**
     * @brief Parse arguments to configure the plugin. Gets the value of the arguments
     * in the configuration file. This values can also be set inside element <plugin>
     * in the ".world" file.
     * @param sdf sdf element to the ".world"
     */
    private: void ParseArgs(sdf::ElementPtr sdf);

    /// @brief Read configuration file to set parameters to the plugin.
    private: void GetYamlConfig();



    /// @brief Inserts a camera model from an ".sdf"
    private: void InsertCameraModel();

    /// @brief Gets a pointer to the camera
    private: bool GetCameraPointer();
    
    /// @brief Saves an image of the enviroment
    private: void TakeScreenShot();

    /// @brief Gets transform between camera and sensor
    private: ignition::math::Pose3d GetCameraSensorTF();

    /// @brief Gets transform between camera and sensor
    private: void SaveCameraSensorTF();

        /// @brief Gets transform between camera and sensor
    private: void SaveCameraParams();

    /**
     * @brief Gets a pointer to an sdf file from a path
     * @param sdfPath Absolute path to the sdf file
     * @return pointer to the sdf file
     */
    private: sdf::SDFPtr GetSDFfile(std::filesystem::path sdfPath);

    /**
     * @brief Makes a copy of the model file so it can be transformed as many times
     * as you want from the original model.
     * @param path Absolute path to the model file.
     * @return Absolute path to the copy of the model file. It renames it with
     * suffix "_copy".
     */
    private: std::filesystem::path GetTemporarySDFfile(std::filesystem::path sdfPath);

    /**
     * @brief Reset temporary file as its original file
     * @param sdfPath Absolute path to the original sdf file
     * @return absolute path to the temporal sdf file
     */
    private: std::filesystem::path ResetTemporarySDFfile(std::filesystem::path sdfPath);

    /// @brief Insert labeled cuboid models in random scales and poses
    private: std::vector<std::string> SpawnRandomModels();

    /// @brief Insert unlabeled models as a perturbations to the world
    private: std::vector<std::string> SpawnRandomEnviroment();

    /// @brief Delete all models in the world except os_128, camera, and world
    private: void removeModels();
    
    /**
     * @brief Remove models
     * @param models Vector of strings with model names
     */
    public: void removeModelsByName(std::vector<std::string> models);

    /**
     * @brief Set model name consecutively for each model inserted in the world.
     * For each model inserted its name appends an integer describing its name.
     * @param modelElement sdf::ElementPtr to the model element.
     * @param cnt number of the spawned model
     */
    public: std::string SetModelName(sdf::ElementPtr modelElement, int count);

    /**
     * @brief Set random pose to a model.
     * @param modelElement sdf::ElementPtr to the model element.
     */
    private: void SetModelPose(sdf::ElementPtr modelElement);

    /**
     * @brief Set random position to a model.
     * @param modelElement sdf::ElementPtr to the model element.
     */
    private: void SetModelPosition(sdf::ElementPtr modelElement);

    /**
     * @brief Set random scale to a model in all its axes.
     * @param modelElement sdf::ElementPtr to the model element.
     */
    private: void SetRandomScale(sdf::ElementPtr model);

    /**
     * @brief Set random scale in 3 axis to a model.
     * @param modelElement sdf::ElementPtr to the model element.
     */
    private: void SetRandomMeshScale(sdf::ElementPtr model);

    /**
     * @brief Set laser retro consecutively for each visual element in the model.
     * @param model sdf::ElementPtr to the model element.
     * @return void.
     */
    private: void SetLaserRetro(sdf::ElementPtr model);

    /// @brief Moves groud model randomly
    private: void MoveGroundModel();

    /// @brief Check output directories format, and create if don't exists
    private: void CheckOutputDirs();

    /// @brief Get last saved cloud by writing time and set env count to this value
    /// to continue from that number
    private: void ResumeEnvCount();

    /**
     * @brief Check if sensor is currently working.
     * @return true if sensor is working.
     */
    private: bool SensorReady();

    /**
     * @brief Check if models are correctly spawned in the world
     * @param model_names Vector of strings with model names
     * @return true if all models are spawned correctly
     */
    private: bool CheckSpawnedModels(std::vector<std::string> model_names);

    /**
     * @brief Check if models are correctly removed from the world
     * @param model_names Vector of strings with model names
     * @return true if all models are removed correctly
     */
    private: bool CheckDeletedModels(std::vector<std::string> model_names);

    
    /// @brief Saves last published PointCloud in a global variable (pcl_cloud)
    private: void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input);

    /// @brief Save last published cloud as a file in ".pcd" 
    private: void SavePointCloud();


    /**
     * @brief Compute random pose X Y Z R P Y
     * @return Return the random pose
     */
    private: ignition::math::Pose3d ComputeRandomPose();

    /**
     * @brief Compute random pose X Y Z R P Y
     * @return Return the random pose
     */
    private: ignition::math::Pose3d ComputeWorldRandomPose();

    /**
     * @brief Compute random position X Y Z
     * @return Return the random position
     */
    private: ignition::math::Vector3d ComputeEnvRandPosition();

    /**
     * @brief Compute random rotation R P Y
     * @return Return the random orientation
     */
    private: ignition::math::Vector3d ComputeRandRotation();


    /**
     * @brief Compute random scale in 3 axis (X, Y, Z)
     * @return the vector with the values of the scale.
     */
    private: ignition::math::Vector3d ComputeRandomScale();
    
    /**
     * @brief Compute random scale in 3 axis (X, Y, Z)
     * @return the vector with the values of the scale.
     */
    private: ignition::math::Vector3d ComputeRandomScale(ignition::math::Vector3d min_scale_, ignition::math::Vector3d max_scale_);

    /**
     * @brief Check that pose dont lies inside truss structure
     * @return Return true if pose is valid
     */
    private: bool ReachPositionOffset(ignition::math::Pose3d pose);


    /**
     * @brief Remove models that collide with sensor
     * 
     */
    private: std::vector<std::string> RemoveCollideModels(physics::ModelPtr sensor_model);

    /**
     * @brief Apply offset to the passed coordinate.
     * @return the new vector
     */
    private: ignition::math::Vector3d ApplyOffset(ignition::math::Vector3d input);

    /**
     * @brief Apply offset to the passed coordinate.
     * @return the new vector
     */
    private: ignition::math::Vector3d ApplyOffset(ignition::math::Vector3d input, ignition::math::Vector3d offset_);

    /**
     * @brief Aplly rotation to a model
     * @param model_ptr Pointer to a model in gazebo
     * @param rotation Rotation vector R P Y
     */
    private: void ApplyRotation(physics::ModelPtr model_ptr, ignition::math::Vector3d rotation);

    /**
     * @brief Return the number of files in an existing directory.
     * @param path The absolute path to the directory.
     * @return Integer with the number of files in directory.
     */
    private: int GetNumOfItems(std::filesystem::path path);

    /**
     * @brief Set a random weight for each model which will be used to set the number 
     * of copies of each model to insert in the world
     * @param path The absolute path to the directory.
     * @return vector with wei
     */
    private: Eigen::VectorXf SetModelWeights(std::filesystem::path path);

    /// @brief Setup ROS configuration
    private: void SetupROS();

    /// @brief Thred that manages callbacks in ROS
    private: void QueueThread();




    // VARIABLES ////////////////////
    // GAZEBO
    private: physics::WorldPtr world;
    private: physics::ModelPtr model;
    private: physics::ModelPtr sensor_model;
    private: event::ConnectionPtr updateConnection;
    private: physics::ModelPtr camera_model;
    private: sensors::CameraSensorPtr camera;
    private: ignition::math::Pose3d camera_pose;
    private: std::string cam_name;
     

    // CONFIGURATION
    private: arvc::configuration config;
    private: std::string RANDMODE;
    private: int NUM_ENV;
    private: int NUM_MODELS;
    private: std::filesystem::path ENV_DIR;
    private: std::filesystem::path models_dir;
    private: std::filesystem::path output_dir;
    private: std::filesystem::path pcd_dir;
    private: std::filesystem::path images_dir;
    private: std::filesystem::path cam_path;
    private: std::string sensor_name;
    private: std::string sensor_topic;
    private: std::string ground_model_name;
    private: ignition::math::Vector3d pos_offset;
    private: ignition::math::Vector3d neg_offset;
    private: ignition::math::Vector3d pos_dist;
    private: ignition::math::Vector3d neg_dist;
    private: ignition::math::Vector3d min_scale;
    private: ignition::math::Vector3d max_scale;
    private: ignition::math::Vector3d rot_range;
    private: bool debug_msgs;
    private: bool pc_binary;
    private: float sensor_offset;

    //ROS
    private: ros::NodeHandle* ros_node;
    private: ros::Subscriber ros_sub;
    private: ros::SubscribeOptions ros_so;
    private: ros::CallbackQueue ros_cbqueue;
    private: boost::thread callback_queue_thread;


    //PCL
    private: pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud;

    // HELPERS
    private: bool ousterReady;
    private: bool handle_to_cam;
    private: bool take_screenshot;
    private: int env_count;
    private: int laser_retro;
    private: bool paused;
  };
}