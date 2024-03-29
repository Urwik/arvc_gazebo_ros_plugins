// ARVC
#include "arvc_gazebo_ros_plugins/arvc_gazebo_ros_dataset_generator.h"

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
    std::cout << RED << "DatasetGenerator constructor" << RESET << std::endl;
    this->env_count = 0;
    this->ousterReady = false;
    this->handle_to_cam = false;

    this->pcl_cloud.reset(new PointCloud);
    this->take_screenshot = false;
    this->laser_retro = 1;
    this->paused = true;
  }

  /////////////////////////////////
  DatasetGenerator::~DatasetGenerator(){
    this->env_count = 0;
    this->ousterReady = false;
    this->handle_to_cam = false;

    this->pcl_cloud.reset(new PointCloud);
    this->take_screenshot = false;
    this->laser_retro = 1;
    this->paused = false;
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

    // boost::thread generator_thread(boost::bind(&DatasetGenerator::GenerateDataset, this));
    this->generator_thread = boost::thread(boost::bind(&DatasetGenerator::GenerateDataset, this));

    ROS_INFO(GREEN "ARVC GAZEBO SPAWNMODEL PLUGIN LOADED" RESET);
  }


  //////////////////////////////////////////////////////////////////////////////
  void DatasetGenerator::Init()
  { 
    this->env_count = 0;
    this->ousterReady = false;
    this->handle_to_cam = false;

    this->pcl_cloud.reset(new PointCloud);
    this->take_screenshot = false;
    this->laser_retro = 1;
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
    // std::this_thread::sleep_for(std::chrono::milliseconds(7000));

    std::vector<std::string> env_models_;
    std::vector<std::string> models_;
    std::vector<std::string> all_models_;

    int estado = 0;
    int env_count_ = 0;
    gazebo::common::Console::SetQuiet(true);

    while (this->env_count < this->NUM_ENV)
    {
      switch (estado)
      {
      case 0:
        if(this->SensorReady()){
          this->ResumeEnvCount();
          ROS_INFO(GREEN "STARTING TO SPAWN MODELS..." RESET);
          std::this_thread::sleep_for(std::chrono::milliseconds(3000));
          estado = 1;
        }
        break;

      case 1:
        ROS_INFO(YELLOW "GENERATING RANDOM ENVIROMENT: %d" RESET, this->env_count);
        this->MoveGroundModel();
        env_models_ = this->SpawnRandomEnviroment();
        models_ = this->SpawnRandomModels();
        this->ApplyRotation(this->sensor_model, this->ComputeRandRotation());

        all_models_.clear();
        all_models_.resize(env_models_.size() + models_.size());
	      std::set_union(env_models_.begin(), env_models_.end(), models_.begin(), models_.end(), all_models_.begin());

        ROS_INFO_COND(this->debug_msgs, BLUE "MODELS SPAWNED: %d" RESET, (int) all_models_.size());

        estado = 2;
        break;
      
      case 2:
        if (this->CheckSpawnedModels(all_models_))
        {
          if(this->paused)
          {
            ROS_INFO(YELLOW "PAUSED: Press enter to continue ..." RESET);
            std::getchar();
          }
          estado = 3;
        }
        break;
      
      case 3:
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        // this->TakeScreenShot();
        this->SavePointCloud();
        this->removeModelsByName(all_models_);
        estado = 4;
        break;
      
      case 4:
        if(this->CheckDeletedModels(all_models_))
        {
          this->env_count++;
          estado = 1;
        }
        break;
      
      case 5:
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
  /////////////////////////////////
  void DatasetGenerator::ParseArgs(sdf::ElementPtr sdf)
  {
    ROS_INFO(BLUE "PARSING ARGUMENTS... " RESET);

    this->GetYamlConfig();

    // OUTPUT DIRECTORY
    if (!sdf->HasElement("out_dir")){
      this->output_dir = this->config["plugin"]["out_dir"].as<std::string>();
    } else {
      this->output_dir = sdf->GetElement("out_dir")->Get<std::string>();
    }
    std::cout << YELLOW << "OUTPUT DIR: " << RESET << "\n " << this->output_dir.c_str() << std::endl;

    // ENVIROMENT MODELS DIRECTORY
    if (!sdf->HasElement("env_dir")) {
      this->env_dir = this->config["plugin"]["env_dir"].as<std::string>();
    } else {
      this->env_dir = sdf->GetElement("env_dir")->Get<std::string>();
    }
    std::cout << YELLOW << "ENVIROMENTS DIR: " << RESET << "\n " << this->env_dir.c_str() << std::endl;

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
      this->debug_msgs = this->config["plugin"]["debug_msgs"].as<bool>();
    } else {
      this->debug_msgs = sdf->GetElement("debug_msgs")->Get<bool>();
    }
      std::cout << YELLOW << "DEBUG MODE: " << RESET << "\n " << this->debug_msgs << std::endl;

    // PAUSES THE PROGRAM UNTIL USER PRESS ENTER
    if (!sdf->HasElement("paused")) {
      this->paused = this->config["plugin"]["paused"].as<bool>();
    } else {
      this->paused = sdf->GetElement("paused")->Get<bool>();
    }
      std::cout << YELLOW << "PAUSED MODE: " << RESET << "\n " << this->paused << std::endl;
  }

  /////////////////////////////////
  void DatasetGenerator::GetYamlConfig()
  {
    fs::path package_path(ros::package::getPath("arvc_dataset_generator"));
    fs::path config_path = package_path / "config/dataset_generator_config.yaml";

    std::cout << YELLOW << "YAML CONFIG PATH: " << RESET << "\n " << config_path.string().c_str() << std::endl;

    this->config = YAML::LoadFile(config_path.string());
  }


  // CAMERA FUNCTIONS
  /////////////////////////////////
  void DatasetGenerator::InsertCameraModel()
  {
    sdf::SDFPtr camera_sdf = this->GetSDFfile(this->cam_path);
    this->world->InsertModelSDF(*camera_sdf);

    // GET CAMERA NAME
    sdf::ElementPtr root = camera_sdf->Root();
    sdf::ElementPtr model = root->GetElement("model");
    sdf::ElementPtr link = model->GetElement("link");
    this->camera_pose = link->GetElement("pose")->Get<ignition::math::Pose3d>();
    sdf::ElementPtr sensor = link->GetElement("sensor");

    this->cam_name = sensor->GetAttribute("name")->GetAsString();
  }
  
  /////////////////////////////////
  bool DatasetGenerator::GetCameraPointer()
  {
    ROS_INFO_COND(this->debug_msgs, YELLOW "TRYING TO GET CAMERA: %s" RESET, this->cam_name.c_str());
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

  /////////////////////////////////
  void DatasetGenerator::TakeScreenShot()
  {
    ROS_INFO_COND(this->debug_msgs, "TAKING SCREENSHOT");

    std::stringstream ss;
    ss.str("");
    ss << this->images_dir.string() << "/" << std::setfill('0') << std::setw(5)  << this->env_count << ".jpg";
    
    this->camera->Update(true);
    this->camera->SaveFrame(ss.str().c_str());
  }
  
  /////////////////////////////////
  ignition::math::Pose3d DatasetGenerator::GetCameraSensorTF()
  {
    ignition::math::Pose3d sensor_pose = this->sensor_model->WorldPose();
    return (this->camera_pose - sensor_pose);
  }

  // GENERATION FUNCTIONS
  /////////////////////////////////
  sdf::SDFPtr DatasetGenerator::GetSDFfile(fs::path sdfPath)
  {

    sdf::SDFPtr sdf_file (new sdf::SDF());
    sdf::init(sdf_file);
    sdf::readFile(sdfPath, sdf_file);

    return sdf_file;
  }

  /////////////////////////////////
  fs::path DatasetGenerator::GetTemporarySDFfile(fs::path orig_path)
  {
    ROS_INFO_COND(this->debug_msgs, "GENERATING TEMPORARY SDF FILE");

    fs::path new_model_path = orig_path.parent_path() / "temp_model.sdf";
    fs::path new_config_path = orig_path.parent_path() / "temp_model.config";
    fs::path orig_config_path = orig_path.parent_path() / "model.config";

    fs::copy_file(orig_path, new_model_path, fs::copy_options::overwrite_existing);
    fs::copy_file(orig_config_path, new_config_path, fs::copy_options::overwrite_existing);


    return new_model_path;
  }

  /////////////////////////////////
  fs::path DatasetGenerator::ResetTemporarySDFfile(fs::path orig_path)
  {
    ROS_INFO_COND(this->debug_msgs, "RESETING TEMPORARY SDF FILE");

    fs::path new_path;

    return new_path;
  }

  /////////////////////////////////
  std::vector<std::string> 
  DatasetGenerator::SpawnRandomModels()
  {
    ROS_INFO_COND(this->debug_msgs, "SPAWNING MODELS...");

    std::vector<std::string> models;
    std::string model_name;

    for (const fs::directory_entry &entry : fs::directory_iterator(this->models_dir))
    {
      fs::path original_file = entry.path() / "model.sdf";
      this->laser_retro = 1;

      for (int i = 0; i < this->NUM_MODELS; i++)
      {
        fs::path temp_file = this->GetTemporarySDFfile(original_file);
        sdf::SDFPtr temp_sdfFile = this->GetSDFfile(temp_file);
        sdf::ElementPtr modelElement = temp_sdfFile->Root()->GetElement("model");

        model_name = this->SetModelName(modelElement, i);
        models.push_back(model_name);

        this->SetModelPose(modelElement);
        this->SetLaserRetro(modelElement);
        this->SetRandomScale(modelElement);

        ROS_INFO_COND(this->debug_msgs, "SPAWNING MODEL: %s", model_name.c_str());
        this->world->InsertModelSDF(*temp_sdfFile);
        boost::this_thread::sleep(boost::posix_time::milliseconds(10));
      }
    }
    ROS_INFO_COND(this->debug_msgs, "MODELS SPAWNED CORRECTLY");
    return models;
  }

  /////////////////////////////////
  std::vector<std::string> 
  DatasetGenerator::SpawnRandomEnviroment()
  {
    ROS_INFO_COND(this->debug_msgs, "SPAWNING ENVIROMENT...");

    std::vector<std::string> models;
    std::string model_name;
    
    for (const fs::directory_entry &entry : fs::directory_iterator(this->env_dir))
    {
      fs::path original_file = entry.path() / "model.sdf";
      int num_models_ = ignition::math::Rand::IntUniform(3, 7);

      for (int i = 0; i < num_models_; i++)
      {
        fs::path temp_file = GetTemporarySDFfile(original_file);
        sdf::SDFPtr temp_sdfFile = this->GetSDFfile(temp_file);
        sdf::ElementPtr modelElement = temp_sdfFile->Root()->GetElement("model");

        model_name = this->SetModelName(modelElement, i);
        models.push_back(model_name);

        this->SetModelPosition(modelElement);
        this->SetRandomMeshScale(modelElement);

        ROS_INFO_COND(this->debug_msgs, "SPAWNING MODEL: %s", model_name.c_str());
        this->world->InsertModelSDF(*temp_sdfFile);
        boost::this_thread::sleep(boost::posix_time::milliseconds(10));
      }
    }
    ROS_INFO_COND(this->debug_msgs, "ENVIROMENT SPAWNED CORRECTLY");
    return models;
  }
  
  /////////////////////////////////
  void DatasetGenerator::removeModels()
  {
    ROS_INFO_COND(this->debug_msgs, "DELETING MODELS...");

    this->world->SetPaused(true);

    physics::Model_V actual_models_ = this->world->Models();
    
    for(const auto &model_ : actual_models_)
    {
      std::string name = model_->GetName();
      if(name != this->sensor_name && name != this->ground_model_name && name != this->cam_name)
      {
        ROS_INFO_COND(this->debug_msgs, "DELETING MODEL: %s", name.c_str());
        this->world->RemoveModel(name);
      }
    }   
    this->world->SetPaused(false);
  }
  
  /////////////////////////////////
  void DatasetGenerator::removeModelsByName(std::vector<std::string> models)
  {
    ROS_INFO_COND(this->debug_msgs, "DELETING MODELS...");

    this->world->SetPaused(true);
    for(const std::string &name : models)
    {
      ROS_INFO_COND(this->debug_msgs, "DELETING MODEL: %s", name.c_str());
      this->world->RemoveModel(name);
    }
    this->world->SetPaused(false);
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
    ignition::math::Pose3d pose = this->ComputeRandomPose();
    
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

  /////////////////////////////////
  void DatasetGenerator::MoveGroundModel()
  {
    ROS_INFO_COND(this->debug_msgs, "MOVING GROUND MODEL");
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

  /// @brief Get last saved cloud count and continue from that number
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
   * @brief Check if the sensor is ready. It tries to get a model with name saved in "sensor_name"
   * @return true if the sensor is ready
   * 
  */
  bool DatasetGenerator::SensorReady()
  {
    ROS_INFO_COND(this->debug_msgs, "CHECKING IF OUSTER IS READY");
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
    ROS_INFO_COND(this->debug_msgs, "CHECKING SPAWNED MODELS");

    int spawned_models = 0;
    for(auto model_name : model_names)
    {
      if(!this->world->ModelByName(model_name))
      { 
        ROS_INFO_COND(this->debug_msgs, YELLOW "CAN'T FIND MODEL: %s" RESET, model_name.c_str());
        ROS_INFO_COND(this->debug_msgs, YELLOW "FOUND MODELS: %d" RESET, (int) spawned_models);
        return false;
      }
      spawned_models++;
    }
    ROS_INFO_COND(this->debug_msgs, YELLOW "FOUND MODELS: %d" RESET, (int) spawned_models);
    ROS_INFO_COND(this->debug_msgs, GREEN "MODELS SPAWNED CORRECTLY" RESET);

    return true;
  }

  /////////////////////////////////
  bool DatasetGenerator::CheckDeletedModels(std::vector<std::string> model_names)
  {
    ROS_INFO_COND(this->debug_msgs, "CHECKING DELETE MODELS");
    for(auto model_name : model_names)
    {
      if(this->world->ModelByName(model_name))
      {
        ROS_INFO_COND(this->debug_msgs, YELLOW "MODEL STILL REMAINING: %s" RESET, model_name.c_str());
        return false;
      }
    }
    ROS_INFO_COND(this->debug_msgs, GREEN "MODELS DELETED CORRECTLY" RESET);
    return true;
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
    ROS_INFO_COND(this->debug_msgs, "SAVING POINTCLOUD...");
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

  // HELPER FUNCTIONS CALCULATIONS
  /////////////////////////////////
  ignition::math::Pose3d DatasetGenerator::ComputeRandomPose()
  {
    using namespace ignition::math;
    Pose3d pose;
    Vector3d position;
    Vector3d rotation;

    rotation.X() = Rand::DblUniform(0, 2*M_PI);
    rotation.Y() = Rand::DblUniform(0, 2*M_PI);
    rotation.Z() = Rand::DblUniform(0, 2*M_PI);

    if (this->RANDMODE == "uniform")
    {
      position.X() = Rand::DblUniform(this->neg_dist.X(), this->pos_dist.X()); 
      position.Y() = Rand::DblUniform(this->neg_dist.Y(), this->pos_dist.Y()); 
      position.Z() = Rand::DblUniform(this->neg_dist.Z(), this->pos_dist.Z()); 
    }
    else if (this->RANDMODE == "normal")
    {
      position.X() = Rand::DblNormal(0,this->pos_dist.X()); 
      position.Y() = Rand::DblNormal(0,this->pos_dist.Y()); 
      position.Z() = Rand::DblNormal(0,this->pos_dist.Z()); 
    }
    else
    {
      ROS_ERROR("WRONG RANDOM MODE, POSSIBLE OPTIONS ARE: uniform, normal");
    }

    position = this->ApplyOffset(position);
    pose.Set(position, rotation);

    return pose;
  }

  /////////////////////////////////
  ignition::math::Pose3d DatasetGenerator::ComputeWorldRandomPose()
  {
    using namespace ignition::math;
    Pose3d pose;
    Vector3d position;
    Vector3d rotation;

    rotation.X() = 0;
    rotation.Y() = 0;
    rotation.Z() = Rand::DblUniform(0, 2*M_PI);

    if (this->RANDMODE == "uniform")
    {
      position.X() = Rand::DblUniform(-2, 2); 
      position.Y() = Rand::DblUniform(-2, 2); 
      position.Z() = Rand::DblUniform(-0.5, 0.5); 
    }
    else if (this->RANDMODE == "normal")
    {
      position.X() = Rand::DblNormal(0, 1); 
      position.Y() = Rand::DblNormal(0, 1); 
      position.Z() = Rand::DblNormal(0, 0.5); 
    }
    else
      ROS_ERROR("WRONG RANDOM MODE, POSSIBLE OPTIONS ARE: uniform, normal");

    pose.Set(position, rotation);

    return pose;
  }

  /////////////////////////////////
  ignition::math::Vector3d DatasetGenerator::ComputeEnvRandPosition()
  {
    using namespace ignition::math;
    Vector3d position;
    Vector3d offset_(10, 10, 0);

    if (this->RANDMODE == "uniform")
    {
      position.X() = Rand::DblUniform(-30, 30);
      position.Y() = Rand::DblUniform(-30, 30);
      position.Z() = Rand::DblUniform(0, 0.5);
    }
    else if (this->RANDMODE == "normal")
    {
      position.X() = Rand::DblNormal(0, 15); 
      position.Y() = Rand::DblNormal(0, 15);
      position.Z() = Rand::DblNormal(0, 0.25);
    }
    else
      ROS_ERROR("WRONG RANDOM MODE, POSSIBLE OPTIONS ARE: uniform, normal");

    return this->ApplyOffset(position, offset_);
  }

  /////////////////////////////////
  ignition::math::Vector3d DatasetGenerator::ComputeRandRotation()
  {
    using namespace ignition::math;
    Vector3d rotation;
    Vector3d offset_(10, 10, 0);

    rotation.X() = Rand::DblUniform(0, 2*M_PI);
    rotation.Y() = Rand::DblUniform(0, 2*M_PI);
    rotation.Z() = Rand::DblUniform(0, 2*M_PI);

    return rotation;
  }

  /////////////////////////////////
  ignition::math::Vector3d DatasetGenerator::ComputeRandomScale()
  {
    ignition::math::Vector3d scale;
    scale.X() = ignition::math::Rand::DblUniform(this->min_scale.X(), this->max_scale.X());
    scale.Y() = ignition::math::Rand::DblUniform(this->min_scale.Y(), this->max_scale.Y());
    scale.Z() = ignition::math::Rand::DblUniform(this->min_scale.Z(), this->max_scale.Z());

    return scale;
  }
  
  /////////////////////////////////
  ignition::math::Vector3d DatasetGenerator::ComputeRandomScale(ignition::math::Vector3d min_scale_, ignition::math::Vector3d max_scale_)
  {
    ignition::math::Vector3d scale;
    scale.X() = ignition::math::Rand::DblUniform(min_scale_.X(), max_scale_.X());
    scale.Y() = ignition::math::Rand::DblUniform(min_scale_.Y(), max_scale_.Y());
    scale.Z() = ignition::math::Rand::DblUniform(min_scale_.Z(), max_scale_.Z());

    return scale;
  }

  /////////////////////////////////
  ignition::math::Vector3d DatasetGenerator::ApplyOffset(ignition::math::Vector3d input)
  {
    ignition::math::Vector3d output;

    for (size_t i = 0; i < 3; i++)
    {
      if(input[i] < 0)
        output[i] = input[i] + this->neg_offset[i];
      else
        output[i] = input[i] + this->pos_offset[i];
    }
    
    return output;
  }

  /////////////////////////////////
  ignition::math::Vector3d DatasetGenerator::ApplyOffset(ignition::math::Vector3d input, ignition::math::Vector3d offset_)
  {
    ignition::math::Vector3d output;

    for (size_t i = 0; i < 3; i++)
    {
      if(input[i] < 0)
        output[i] = input[i] - offset_[i];
      else
        output[i] = input[i] + offset_[i];
    }
    
    return output;
  }

  /////////////////////////////////
  void DatasetGenerator::ApplyRotation(physics::ModelPtr model_ptr, ignition::math::Vector3d rotation)
  {
    ignition::math::Pose3d pose;

    pose = model_ptr->WorldPose();
    this->world->SetPaused(true);
    pose.Rot().Euler(rotation);
    model_ptr->SetWorldPose(pose);
    this->world->SetPaused(false);
  }

  /////////////////////////////////
  int DatasetGenerator::GetNumOfItems(fs::path path)
  {
    fs::directory_iterator dir_iter(path);
    return int(std::distance(dir_iter, fs::directory_iterator{}));
  }

  /////////////////////////////////
  Eigen::VectorXf DatasetGenerator::SetModelWeights(fs::path path) 
  {
    using namespace ignition::math;
    const int num_models = this->GetNumOfItems(path);
    Eigen::VectorXf model_weights(num_models);

    for (size_t i = 0; i < num_models; i++)
      model_weights(i) = Rand::IntUniform(0,100);

    model_weights = model_weights / model_weights.sum();

    return model_weights;
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