// ARVC
#include "arvc_gazebo_ros_plugins/arvc_gazebo_ros_dataset_generator.h"
#include "arvc_gazebo_ros_plugins/arvc_dataset_generator_utils.hpp"

// C++
#include <iostream>
#include <fstream>
#include <algorithm>
#include <math.h>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>


// GAZEBO
#include <sdf/sdf.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/common/Console.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

// ROS
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>


// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/visualization/pcl_visualizer.h>

//Eigen
#include <Eigen/Dense>

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
  GZ_REGISTER_WORLD_PLUGIN(DatasetGenerator)
  /////////////////////////////////
  DatasetGenerator::DatasetGenerator(){
    cout << RED << "Running Plugin Constructor..." << RESET << endl;
    this->env_count = 0;
    this->ousterReady = false;
    this->handle_to_cam = false;

    this->pcl_cloud.reset(new PointCloud);
    this->take_screenshot = false;
    this->laser_retro = 1;
    this->config.simulation.paused = true;
  }

  /////////////////////////////////
  DatasetGenerator::~DatasetGenerator(){
    this->env_count = 0;
    this->ousterReady = false;
    this->handle_to_cam = false;

    this->pcl_cloud.reset(new PointCloud);
    this->take_screenshot = false;
    this->laser_retro = 1;
    this->config.simulation.paused = true;
  }


  //////////////////////////////////////////////////////////////////////////////
  void DatasetGenerator::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
  {
    this->world = _parent;
    this->world->SetPhysicsEnabled(false);

    this->ParseArgs(_sdf);

    this->InsertCameraModel();

    this->SetupROS();

    this->CheckOutputDirs();
    
    this->updateConnection =  event::Events::ConnectWorldUpdateBegin(
                              boost::bind(&DatasetGenerator::OnUpdate, this));

    boost::thread generator_thread(boost::bind(&DatasetGenerator::GenerateDataset, this));

    ROS_INFO(GREEN "ARVC GAZEBO SPAWNMODEL PLUGIN LOADED" RESET);
  }


  //////////////////////////////////////////////////////////////////////////////
  void DatasetGenerator::Init()
  { 
  }


  //////////////////////////////////////////////////////////////////////////////
  void DatasetGenerator::OnUpdate()
  { 
    if(!this->handle_to_cam)
    {
      if(this->GetCameraPointer())
        this->handle_to_cam = true;
    }
  }


  // MAIN FUNCTION
  //////////////////////////////////////////////////////////////////////////////
  void DatasetGenerator::GenerateDataset()
  {
    std::vector<std::string> env_models_;
    std::vector<std::string> models_;
    std::vector<std::string> rmvd_models; 
    int estado = 0;

    while (this->env_count < this->config.out_data.quantity)
    {
      switch (estado)
      {
        
        // CHECK IF SENSOR IS READY
        case 0:
          if(this->SensorReady()){
            this->ResumeEnvCount();
            ROS_INFO(GREEN "STARTING TO SPAWN MODELS..." RESET);
            std::this_thread::sleep_for(std::chrono::milliseconds(3000));
            estado = 1;
          }
          break;

        // INSERT ENVIROMENT MODELS AND LABELED MODELS IN THE WORLD
        case 1:
          ROS_INFO(YELLOW "GENERATING RANDOM ENVIROMENT: %d" RESET, this->env_count);
          this->MoveGroundModel();
          env_models_ = this->SpawnRandomEnviroment();
          models_ = this->SpawnRandomModels();
          this->ApplyRotation(this->sensor_model, this->ComputeRandRotation());
          estado = 2;
          break;
        
        // REMOVING MODELS THAT ARE IN COLLISION WITH THE SENSOR
        case 2:
          ROS_INFO_COND(this->config.simulation.debug_msgs, YELLOW "CALLING REMOVE COLLIDE MODELS" RESET);

          rmvd_models = this->RemoveCollideModels(this->sensor_model);

          if (this->config.simulation.debug_msgs)
          {
            ROS_INFO_COND(this->config.simulation.debug_msgs, BLUE "REMOVED MODELS: " RESET);
            if (rmvd_models.size() > 0)
            {
              for (size_t i = 0; i < rmvd_models.size(); i++)
                std::cout << rmvd_models[i] << std::endl;
            }
          }

          ROS_INFO_COND(this->config.simulation.debug_msgs, GREEN "PASSED REMOVE COLLIDE MODELS" RESET);
          estado = 3;
          break;

        // CHECK THAT ALL MODELS ARE CORRECTLY SPAWNED
        case 3:
          if (this->CheckSpawnedModels(models_) && this->CheckSpawnedModels(env_models_))
          {
            estado = 4;
          }
          break;
        
        // SAVE DATA AND REMOVE MODELS
        case 4:
          this->TakeScreenShot();
          this->SavePointCloud();
          this->removeModels();
          estado = 5;
          break;
        
        // CHECK THAT ALL MODELS ARE REMOVED CORRECTLY
        case 5:
          if(this->CheckDeletedModels(models_) && this->CheckDeletedModels(env_models_))
          {
            this->env_count++;
            estado = 1;
          }
          break;
        
        case 6:
          // this->env_count++;
          // estado = 1;
          std::this_thread::sleep_for(std::chrono::milliseconds(1000));
          break;

        default:
          break;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    ROS_INFO(GREEN "ENVS CREATED CORRECTLY" RESET);

  }

  // CONFIGURATION AND PARSING ARGUMENTS
  //////////////////////////////////////////////////////////////////////////////
/*   void DatasetGenerator::ParseArgs(sdf::ElementPtr sdf)
  {
    ROS_INFO(BLUE "PARSING ARGUMENTS... " RESET);

    this->GetYamlConfig();



    // OUTPUT DIRECTORY
    if (!sdf->HasElement("out_dir")){
      this->output_dir = this->config.out_data.out_dir;
    } else {
      this->output_dir = sdf->GetElement("out_dir")->Get<std::string>();
    }
    std::cout << YELLOW << "OUTPUT DIR: " << RESET << "\n " << this->output_dir.c_str() << std::endl;

    // ENVIROMENT MODELS DIRECTORY
    if (!sdf->HasElement("env_dir")) {
      this->ENV_DIR = this->config["plugin"]["env_dir"].as<std::string>();
    } else {
      this->ENV_DIR = sdf->GetElement("env_dir")->Get<std::string>();
    }
    std::cout << YELLOW << "ENVIROMENTS DIR: " << RESET << "\n " << this->ENV_DIR.c_str() << std::endl;

    // LABELED MODELS DIRECTORY
    if (!sdf->HasElement("mod_dir")) {
      this->models_dir = this->config["plugin"]["mod_dir"].as<std::string>();
    } else {
      this->models_dir = sdf->GetElement("mod_dir")->Get<std::string>();
    }
    std::cout << YELLOW << "MODELS DIR: " << RESET << "\n " << this->models_dir.c_str() << std::endl;

    // CAMERA MODEL ABSOLUTE PATH
    if (!sdf->HasElement("cam_model")) {
      this->cam_path = this->config["plugin"]["cam_model"].as<std::string>();
    } else {
      this->cam_path = sdf->GetElement("cam_model")->Get<std::string>();
    }
    std::cout << YELLOW << "CAMERA MODEL: " << RESET << "\n " << this->cam_path.c_str() << std::endl;

    // WORLD MODEL NAME
    if (!sdf->HasElement("grnd_model")) {
      this->ground_model_name = this->config["plugin"]["ground_model_name"].as<std::string>();
    } else {
      this->ground_model_name = sdf->GetElement("grnd_model")->Get<std::string>("name");
    }
    std::cout << YELLOW << "WORLD MODEL NAME: " << RESET << "\n " << this->ground_model_name.c_str() << std::endl;

    // SENSOR MODEL NAME
    if (!sdf->HasElement("sensor_name")) {
      this->sensor_name = this->config["plugin"]["sensor_name"].as<std::string>();
    } else {
      this->sensor_name = sdf->GetElement("sensor_name")->Get<std::string>("name");
    }
    std::cout << YELLOW << "SENSOR NAME: " << RESET << "\n " << this->sensor_name.c_str() << std::endl;

    // SENSOR TOPIC NAME
    if (!sdf->HasElement("sensor_topic")) {
      this->sensor_topic = this->config["plugin"]["sensor_topic"].as<std::string>();
    } else {
      this->sensor_topic = sdf->GetElement("sensor_topic")->Get<std::string>("name");
    }
    std::cout << YELLOW << "SENSOR TOPIC: " << RESET << "\n " << this->sensor_topic.c_str() << std::endl;

    // NUM OF ENVIROMENTS TO CREATE
    if (!sdf->HasElement("NUM_ENV")) {
      this->NUM_ENV = this->config["plugin"]["num_env"].as<int>();
    } else {
      this->NUM_ENV = sdf->GetElement("num_env")->Get<int>();
    }
    std::cout << YELLOW << "ENVS TO GENERATE: " << RESET << "\n " << this->NUM_ENV << std::endl;

    // NUM OF LABELED MODELS TO SPAWN AROUND THE SENSOR
    if (!sdf->HasElement("num_models")) {
      this->NUM_MODELS = this->config["plugin"]["num_models"].as<int>();
    } else {
      this->NUM_MODELS = sdf->GetElement("num_models")->Get<int>();
    }
    std::cout << YELLOW << "MODELS TO SWAPN PER ENV: " << RESET << "\n " << this->NUM_MODELS << std::endl;

    // OFFSETS AVOID LABELED MODELS TO SPAWN OVER THE SENSOR
    if (!sdf->HasElement("negative_offset")) {
      this->neg_offset = this->config["plugin"]["negative_offset"].as<ignition::math::Vector3d>();
    } else {
      this->neg_offset = sdf->GetElement("negative_offset")->Get<ignition::math::Vector3d>();
    }
    
    if (!sdf->HasElement("positive_offset")) {
      this->pos_offset = this->config["plugin"]["positive_offset"].as<ignition::math::Vector3d>();
    } else {
      this->pos_offset = sdf->GetElement("positive_offset")->Get<ignition::math::Vector3d>();
    }

    // MAX DIST FROM ORIGIN(0, 0, 0) TO SPAWN LABELED MODELS
    if (!sdf->HasElement("positive_dist")) {
      this->pos_dist = this->config["plugin"]["positive_dist"].as<ignition::math::Vector3d>();
    } else {
      this->pos_dist = sdf->GetElement("positive_dist")->Get<ignition::math::Vector3d>();
    }

    // MIN DIST FROM ORIGIN(0, 0, 0) TO SPAWN LABELED MODELS
    if (!sdf->HasElement("negative_dist")) {
      this->neg_dist = this->config["plugin"]["negative_dist"].as<ignition::math::Vector3d>();
    } else {
      this->neg_dist = sdf->GetElement("negative_dist")->Get<ignition::math::Vector3d>();
    }

    // LOWEST SCALE FACTOR FOR LABELED MODELS
    if (!sdf->HasElement("min_scale")) {
      this->min_scale = this->config["plugin"]["min_scale"].as<ignition::math::Vector3d>();
    } else {
      this->min_scale = sdf->GetElement("min_scale")->Get<ignition::math::Vector3d>();
    }

    // HIGHEST SCALE FACTOR FOR LABELED MODELS
    if (!sdf->HasElement("max_scale")) {
      this->max_scale = this->config["plugin"]["max_scale"].as<ignition::math::Vector3d>();
    } else {
      this->max_scale = sdf->GetElement("max_scale")->Get<ignition::math::Vector3d>();
    }

    // DISTRIBUTION USED TO GENERATE RANDOM VALUES, "normal" OR "uniform"
    if (!sdf->HasElement("rand_mode")) {
      this->RANDMODE = this->config["plugin"]["rand_mode"].as<std::string>();
    } else {
      this->RANDMODE = sdf->GetElement("rand_mode")->Get<std::string>();
    }
    std::cout << YELLOW << "RANDOM MODE: " << RESET << "\n " << this->RANDMODE.c_str() << std::endl;


    // SETS THE WAY POINTCLOUD IS SAVED, BINARY OR ASCII
    if (!sdf->HasElement("pc_binary")) {
      this->pc_binary = this->config["plugin"]["pc_binary"].as<bool>();
    } else {
      this->pc_binary = sdf->GetElement("pc_binary")->Get<bool>();
    }
    std::cout << YELLOW << "CLOUD BINARY FORMAT: " << RESET << "\n " << this->pc_binary << std::endl;

    // PLOT EXTRA INFORMATION MESSAVES OVER THE TERMINAL
    if (!sdf->HasElement("debug_msgs")) {
      this->config.simulation.debug_msgs = this->config["plugin"]["debug_msgs"].as<bool>();
    } else {
      this->config.simulation.debug_msgs = sdf->GetElement("debug_msgs")->Get<bool>();
    }
      std::cout << YELLOW << "DEBUG MODE: " << RESET << "\n " << this->config.simulation.debug_msgs << std::endl;

    // 
    if (!sdf->HasElement("sensor_offset")) {
      this->sensor_offset = this->config["plugin"]["sensor_offset"].as<float>();
    } else {
      this->sensor_offset = sdf->GetElement("sensor_offset")->Get<float>();
    }

    // 
    if (!sdf->HasElement("rotation_range")) {
      this->rot_range = this->config["plugin"]["rotation_range"].as<ignition::math::Vector3d>();
    } else {
      this->rot_range = sdf->GetElement("rotation_range")->Get<ignition::math::Vector3d>();
    }
      std::cout << YELLOW << "DEBUG MODE: " << RESET << "\n " << this->config.simulation.debug_msgs << std::endl;
  } */


  //////////////////////////////////////////////////////////////////////////////
  void DatasetGenerator::GetYamlConfig()
  {
    fs::path package_path(ros::package::getPath("arvc_dataset_generator"));
    fs::path config_path = package_path / "config/dataset_generator_config.yaml";
    
    this->config = arvc::plugin::configuration(config_path);
    std::cout << YELLOW << "YAML CONFIG PATH: " << RESET << "\n " << config_path.string().c_str() << std::endl;
  }


  // CAMERA FUNCTIONS
  //////////////////////////////////////////////////////////////////////////////
  void DatasetGenerator::InsertCameraModel()
  {
    sdf::SDFPtr camera_sdf = this->GetSDFfile(this->config.camera.path);
    this->world->InsertModelSDF(*camera_sdf);

    // GET CAMERA NAME
    sdf::ElementPtr root = camera_sdf->Root();
    sdf::ElementPtr model = root->GetElement("model");
    sdf::ElementPtr link = model->GetElement("link");
    this->camera_pose = link->GetElement("pose")->Get<ignition::math::Pose3d>();
    sdf::ElementPtr sensor = link->GetElement("sensor");

    this->cam_name = sensor->GetAttribute("name")->GetAsString();
  }
  

  //////////////////////////////////////////////////////////////////////////////
  bool DatasetGenerator::GetCameraPointer()
  {
    ROS_INFO_COND(this->config.simulation.debug_msgs, YELLOW "TRYING TO GET CAMERA: %s" RESET, this->cam_name.c_str());
    sensors::SensorPtr sensor = sensors::get_sensor(this->cam_name);

    if(!sensor)
      return false;
    else 
    {
      ROS_INFO(BLUE "HANDLE TO CAM OBTAINED CORRECTLY" RESET);
      this->camera = std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
      this->camera_model = this->world->ModelByName("camera");

      return true;
    }
  }


  //////////////////////////////////////////////////////////////////////////////
  void DatasetGenerator::TakeScreenShot()
  {
    ROS_INFO_COND(this->config.simulation.debug_msgs, "TAKING SCREENSHOT");

    std::stringstream ss;
    ss.str("");
    ss << this->img_dir.string() << "/" << std::setfill('0') << std::setw(5)  << this->env_count << ".jpg";
    
    this->camera->SaveFrame(ss.str().c_str());
  }
  

  //////////////////////////////////////////////////////////////////////////////
  ignition::math::Pose3d DatasetGenerator::GetCameraSensorTF()
  {
    ignition::math::Pose3d sensor_pose = this->sensor_model->WorldPose();
    return (this->camera_pose - sensor_pose);
  }


  // GENERATION FUNCTIONS
  //////////////////////////////////////////////////////////////////////////////
  sdf::SDFPtr DatasetGenerator::GetSDFfile(fs::path sdfPath)
  {

    sdf::SDFPtr sdf_file (new sdf::SDF());
    sdf::init(sdf_file);
    sdf::readFile(sdfPath, sdf_file);

    return sdf_file;
  }


  //////////////////////////////////////////////////////////////////////////////
  fs::path DatasetGenerator::GetTemporarySDFfile(fs::path path)
  {
    std::string suffix = "_copy";
    fs::path new_path;

    std::string ext = path.extension();
    std::string in_filename = path.stem();
    std::string out_filename = in_filename + suffix + ext; 
    fs::path root_dir = path.parent_path();
    new_path = root_dir / out_filename;

    fs::copy_file(path, new_path, fs::copy_options::overwrite_existing);

    return new_path;
  }


  //////////////////////////////////////////////////////////////////////////////
  fs::path DatasetGenerator::ResetTemporarySDFfile(fs::path orig_path)
  {
    fs::path new_path = this->GetTemporarySDFfile(orig_path);
    return new_path;
  }


  //////////////////////////////////////////////////////////////////////////////
  std::vector<std::string> DatasetGenerator::SpawnRandomModels()
  {
    ROS_INFO_COND(this->config.simulation.debug_msgs, "SPAWNING MODELS...");

    std::vector<std::string> models;
    std::string model_name;

    for (const fs::directory_entry &entry : fs::directory_iterator(this->models_dir))
    {
      fs::path temp_file = GetTemporarySDFfile(entry.path() / "model.sdf");
      this->laser_retro = 1;

      for (int i = 0; i < this->NUM_MODELS; i++)
      {
        temp_file = this->ResetTemporarySDFfile(entry.path() / "model.sdf");
        sdf::SDFPtr temp_sdfFile = this->GetSDFfile(temp_file);
        sdf::ElementPtr modelElement = temp_sdfFile->Root()->GetElement("model");

        model_name = this->SetModelName(modelElement, i);
        this->SetRandomScale(modelElement);
        this->SetLaserRetro(modelElement);
        this->SetModelPose(modelElement);

        models.push_back(model_name);

        ROS_INFO_COND(this->config.simulation.debug_msgs, "SPAWNING MODEL: %s", model_name.c_str());
        this->world->InsertModelSDF(*temp_sdfFile);
      }
    }
    ROS_INFO_COND(this->config.simulation.debug_msgs, "MODELS SPAWNED CORRECTLY");
    return models;
  }

  /**
   * @brief For each model located in ENV_DIR, inserts a random number of it in 
   * random position (fixed orientation) and scale 
   * 
   * @return Vector with the names of the inserted models
   */
  std::vector<std::string> DatasetGenerator::SpawnRandomEnviroment()
  {
    ROS_INFO_COND(this->config.simulation.debug_msgs, "SPAWNING ENVIROMENT...");

    std::vector<std::string> models;
    std::string model_name;
    
    for (const fs::directory_entry &entry : fs::directory_iterator(this->ENV_DIR))
    {
      int num_models_ = ignition::math::Rand::IntUniform(3, 7);
      fs::path temp_file = GetTemporarySDFfile(entry.path() / "model.sdf");

      for (int i = 0; i < num_models_; i++)
      {
        temp_file = this->ResetTemporarySDFfile(entry.path() / "model.sdf");
        sdf::SDFPtr temp_sdfFile = this->GetSDFfile(temp_file);
        sdf::ElementPtr modelElement = temp_sdfFile->Root()->GetElement("model");

        model_name = this->SetModelName(modelElement, i);
        models.push_back(model_name);

        this->SetModelPosition(modelElement);
        this->SetRandomMeshScale(modelElement);

        ROS_INFO_COND(this->config.simulation.debug_msgs, "SPAWNING MODEL: %s", model_name.c_str());
        this->world->InsertModelSDF(*temp_sdfFile);
      }
    }
    ROS_INFO_COND(this->config.simulation.debug_msgs, "ENVIROMENT SPAWNED CORRECTLY");
    return models;
  }
  
  /////////////////////////////////
  void DatasetGenerator::removeModels()
  {
    ROS_INFO_COND(this->config.simulation.debug_msgs, "DELETING MODELS...");

    this->world->SetPaused(true);

    physics::Model_V actual_models_ = this->world->Models();
    
    for(const auto &model_ : actual_models_)
    {
      std::string name = model_->GetName();
      if(name != this->sensor_name && name != this->ground_model_name && name != this->cam_name)
      {
        ROS_INFO_COND(this->config.simulation.debug_msgs, "DELETING MODEL: %s", name.c_str());
        this->world->RemoveModel(name);
      }
    }   
    this->world->SetPaused(false);
  }
  
  /////////////////////////////////
  void DatasetGenerator::removeModelsByName(std::vector<std::string> models)
  {
    ROS_INFO_COND(this->config.simulation.debug_msgs, "DELETING MODELS...");

    this->world->SetPaused(true);
    for(const std::string &name : models)
    {
      ROS_INFO_COND(this->config.simulation.debug_msgs, "DELETING MODEL: %s", name.c_str());
      this->world->RemoveModel(name);
    }
    this->world->SetPaused(false);

    ROS_INFO_COND(this->config.simulation.debug_msgs, "MODELS DELETED CORRECTLY");
  }

  /////////////////////////////////
  std::string DatasetGenerator::SetModelName(sdf::ElementPtr modelElement, int cnt)
  {
    std::string base_name =  modelElement->Get<std::string>("name");

    std::stringstream ss;
    ss.str("");
    ss << base_name << '_' << cnt;

    std::string model_name = ss.str();

    modelElement->GetAttribute("name")->Set(model_name);
    return model_name;
  }

  /////////////////////////////////
  void DatasetGenerator::SetModelPose(sdf::ElementPtr modelElement)
  {
    sdf::ElementPtr pose_element = modelElement->GetElement("pose");
    ignition::math::Pose3d pose;
    
    do
    {
      pose = this->ComputeRandomPose();
    } while (!this->ReachPositionOffset(pose));
    

    pose_element->Set<ignition::math::Pose3d>(pose);
  }

  /////////////////////////////////
  void DatasetGenerator::SetModelPosition(sdf::ElementPtr modelElement)
  {
    using namespace ignition::math;
    sdf::ElementPtr poseElement = modelElement->GetElement("pose");
    Pose3d orig_pose = poseElement->Get<Pose3d>();
    Pose3d new_pose_;
    Vector3d position = this->ComputeEnvRandPosition();
    new_pose_.Set(position, Vector3d(0.0,0.0,0.0));
    
    poseElement->Set<Pose3d>(new_pose_);
  }
 
  /////////////////////////////////
  void DatasetGenerator::SetRandomScale(sdf::ElementPtr model)
  {
    using namespace ignition::math;

    Vector3d scale = this->ComputeRandomScale();

    sdf::ElementPtr linkElement = model->GetElement("link");
    sdf::ElementPtr visualElement = linkElement->GetElement("visual");

    while (visualElement)
    {
      sdf::ElementPtr sizeElement = visualElement->GetElement("geometry")->GetElement("box")->GetElement("size");
      sdf::ElementPtr poseElement = visualElement->GetElement("pose");

      Vector3d size =  sizeElement->Get<Vector3d>();
      size = size * scale;

      Pose3d pose =  poseElement->Get<Pose3d>();
      pose.Pos() = pose.Pos() * scale;

      sizeElement->Set<Vector3d>(size);
      poseElement->Set<Pose3d>(pose);

      visualElement = visualElement->GetNextElement("visual");
    }

    // Set Scale to Collision element 
    sdf::ElementPtr collisionElement = linkElement->GetElement("collision");
    sdf::ElementPtr sizeElement = collisionElement->GetElement("geometry")->GetElement("box")->GetElement("size");
    sizeElement->Set<Vector3d>(scale);
  }
  
  /////////////////////////////////
  void DatasetGenerator::SetRandomMeshScale(sdf::ElementPtr model)
  {
    using namespace ignition::math;
  
    Vector3d min_scale_(0.8, 0.8, 0.8);
    Vector3d max_scale_(1.5, 1.5, 2);

    sdf::ElementPtr linkElement = model->GetElement("link");
    sdf::ElementPtr visualElement = linkElement->GetElement("visual");
    sdf::ElementPtr scaleElement;

    while (visualElement)
    {
      if(!visualElement->GetElement("geometry")->GetElement("mesh")->GetElement("scale"))
      {
        scaleElement = visualElement->GetElement("geometry")
                                    ->GetElement("mesh")
                                    ->AddElement("scale");
      }
      else
      {
        scaleElement = visualElement->GetElement("geometry")
                                    ->GetElement("mesh")
                                    ->GetElement("scale");
      }

      Vector3d scale = this->ComputeRandomScale(min_scale_, max_scale_);
      Vector3d new_scale = scale;

      scaleElement->Set<Vector3d>(new_scale);

      visualElement = visualElement->GetNextElement("visual");
    }
  }

  /////////////////////////////////
  void DatasetGenerator::SetLaserRetro(sdf::ElementPtr model)
  {
    sdf::ElementPtr linkElement = model->GetElement("link");
    sdf::ElementPtr visualElement = linkElement->GetElement("visual");


    while (visualElement)
    {
      sdf::ElementPtr retroElement = visualElement->GetElement("laser_retro");
      // retroElement->GetValue()->SetFromString(std::to_string(this->laser_retro));
      // retroElement->GetValue()->Set<int>(this->laser_retro);
      retroElement->Set<int>(this->laser_retro);

      visualElement = visualElement->GetNextElement("visual");
      this->laser_retro++;
    }
    
  }

  //////////////////////////////////////////////////////////////////////////////
  /// @brief Moves groud model randomly
  void DatasetGenerator::MoveGroundModel()
  {
    ROS_INFO_COND(this->config.simulation.debug_msgs, "MOVING GROUND MODEL");
    physics::ModelPtr world_model = this->world->ModelByName(this->ground_model_name);
    ignition::math::Pose3d pose = this->ComputeWorldRandomPose();
    
    // To use SetWorldPose is recommendable pause the world
    this->world->SetPaused(true);
    world_model->SetWorldPose(pose);
    this->world->SetPaused(false);
  }


  // CHECK FUNCTIONS
  /////////////////////////////////
  void DatasetGenerator::CheckOutputDirs()
  {
    this->pcd_dir = this->output_dir / "pcd";
    this->images_dir = this->output_dir / "images";

    if(!fs::exists(this->output_dir))
      fs::create_directory(this->output_dir);    

    if(!fs::exists(this->pcd_dir))
      fs::create_directory(this->pcd_dir);

    if(!fs::exists(this->images_dir))
      fs::create_directory(this->images_dir);

    std::cout << YELLOW << "PCD OUTPUT DIR: " << RESET << "\n " << this->pcd_dir.c_str() << std::endl;
    std::cout << YELLOW << "IMAGES OUTPUT DIR: " << RESET << "\n " << this->images_dir.c_str() << std::endl;
  }

  /////////////////////////////////
  void DatasetGenerator::ResumeEnvCount()
  {
    fs::directory_entry last_entry;
    bool first_entry = true;
    int last_num = 0;

    if (!fs::is_empty(this->pcd_dir))
    {
      
      for (const fs::directory_entry entry : fs::directory_iterator(this->pcd_dir))
      {
        if(entry.path().extension() == ".pcd")
        {
          try{
            int actual_num = std::stoi(entry.path().stem());

            if(actual_num > last_num)
              last_num = actual_num;
          }

          catch(const std::exception& e){
            ROS_WARN("CAN'T READ FILE: %s", entry.path().string().c_str());
          }
        }
      }

      this->env_count = last_num + 1;
      ROS_INFO(YELLOW "Starting in Env: %d" RESET, this->env_count);
    }
    else
    {
      this->env_count = 0;
      ROS_INFO(YELLOW "Starting in Env: %d" RESET, this->env_count);
    }
  }

  /**
   * @brief Check if sensor is currently working in Gazebo.
   * @return true if sensor is working.
   */
  bool DatasetGenerator::SensorReady()
  {
    ROS_INFO_COND(this->config.simulation.debug_msgs, "CHECKING IF OUSTER IS READY");
    if(!this->world->ModelByName(this->sensor_name))
      return false;
    else
    {
      ROS_INFO(GREEN "OUSTER IS READY" RESET);
      this->sensor_model = this->world->ModelByName(this->sensor_name);
      return true;
    }
  }

  /////////////////////////////////
  bool DatasetGenerator::CheckSpawnedModels(std::vector<std::string> model_names)
  {
    ROS_INFO_COND(this->config.simulation.debug_msgs, "CHECKING SPAWNED MODELS");

    for(auto model_name : model_names)
    {
      if(!this->world->ModelByName(model_name))
        return false;
    }
    ROS_INFO_COND(this->config.simulation.debug_msgs, "MODELS SPAWNED CORRECTLY");

    return true;
  }

  /////////////////////////////////
  bool DatasetGenerator::CheckDeletedModels(std::vector<std::string> model_names)
  {
    ROS_INFO_COND(this->config.simulation.debug_msgs, "CHECKING DELETE MODELS");
    for(auto model_name : model_names)
    {
      if(this->world->ModelByName(model_name))
        return false;
    }
    ROS_INFO_COND(this->config.simulation.debug_msgs, "MODELS DELETED CORRECTLY");
    return true;
  }

  /////////////////////////////////
  std::vector<std::string> DatasetGenerator::RemoveCollideModels(physics::ModelPtr sensor_model)
  {
    using namespace ignition::math;

    std::vector<std::string> removed_models;

    physics::Model_V models = this->world->Models();
    AxisAlignedBox sensor_bbx = sensor_model->CollisionBoundingBox();

    for (size_t i = 0; i < models.size(); i++)
    {
      if(models[i]->GetName() != this->sensor_model->GetName())
      {
        AxisAlignedBox model_bbx = models[i]->CollisionBoundingBox();
        
        if(sensor_bbx.Intersects(model_bbx))
        {
          removed_models.push_back(models[i]->GetName());
          this->world->RemoveModel(models[i]);
        }
      }
    }
    
    return removed_models;
  }


  // POINT CLOUD FUNCTIONS
  /////////////////////////////////
  void DatasetGenerator::PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
  {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);

    PointCloud::Ptr temp_cloud (new PointCloud);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

    *pcl_cloud = *temp_cloud;
  }

  /////////////////////////////////
  void DatasetGenerator::SavePointCloud()
  {
    ROS_INFO_COND(this->config.simulation.debug_msgs, "SAVING POINTCLOUD...");
    pcl::PCDWriter writer;
    std::stringstream ss;
    ss.str("");
    ss << this->pcd_dir.string() << "/" << std::setfill('0') << std::setw(5)  << this->env_count << ".pcd";

    if(!this->pcl_cloud->empty()){

      if(this->pcl_cloud->points.size() != this->pcl_cloud->width)
      {
        int cloud_size = this->pcl_cloud->points.size();
        this->pcl_cloud->width = cloud_size;
        this->pcl_cloud->height = 1;
      }
      // this->SaveCameraSensorTF();
      // this->SaveCameraParams();
      writer.write<PointT>(ss.str(), *this->pcl_cloud, this->pc_binary);
    }
  }

  /////////////////////////////////
  void DatasetGenerator::SaveCameraSensorTF()
  {
    pcl::visualization::PCLVisualizer vis;
    vis.initCameraParameters();
    ignition::math::Pose3d pose = this->GetCameraSensorTF();
    ignition::math::Vector3d upVector;

    vis.setCameraPosition(this->camera_pose.X(), this->camera_pose.Y(), this->camera_pose.Z(), pose.X(), pose.Y(), pose.Z(), pose.Roll(), pose.Pitch(), pose.Yaw());

    std::stringstream ss;
    ss.str("");
    ss << this->pcd_dir.string() << "/" << std::setfill('0') << std::setw(5)  << this->env_count << ".cam";

    vis.saveCameraParameters(ss.str());
    vis.close();
  }

    /////////////////////////////////
  void DatasetGenerator::SaveCameraParams()
  {
    ignition::math::Pose3d sensor_pose = this->sensor_model->WorldPose();

    Eigen::VectorXd sensor_world_pose(6);
    const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", "\n");

    std::stringstream ss;
    ss.str("");
    ss << this->pcd_dir.string() << "/" << std::setfill('0') << std::setw(5)  << this->env_count << ".pose";

    std::ofstream file(ss.str());
    if(file.is_open())
    {
      file << sensor_world_pose.format(CSVFormat);
      file.close();
    }

  }

 


  // ROS
  /////////////////////////////////
  void DatasetGenerator::SetupROS()
  {
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized()) {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    this->ros_node = new ros::NodeHandle("arvc_gazebo_ros_dataset_generator");

    ros::SubscribeOptions ros_so =
      ros::SubscribeOptions::create<sensor_msgs::PointCloud2>(
          this->sensor_topic, 1,
          boost::bind(&DatasetGenerator::PointCloudCallback, this, _1),
          ros::VoidPtr(), &this->ros_cbqueue);
    
    this->ros_sub = this->ros_node->subscribe(ros_so);
    this->callback_queue_thread = boost::thread(boost::bind(&DatasetGenerator::QueueThread, this));
  }

  /////////////////////////////////
  void DatasetGenerator::QueueThread()
  {
    static const double timeout = 0.01;
    while (this->ros_node->ok())
    {
      this->ros_cbqueue.callAvailable(ros::WallDuration(timeout));
    }
  }

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