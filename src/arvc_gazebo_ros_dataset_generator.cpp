#include "arvc_gazebo_ros_plugins/arvc_gazebo_ros_dataset_generator.h"
#include "sdf_utils.hpp"
#include "train_utils.hpp"

using namespace std;

namespace gazebo
{
  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(DatasetGenerator)
  /////////////////////////////////

  DatasetGenerator::DatasetGenerator()
  {
    cout << RED << "Running Plugin Constructor..." << RESET << endl;
    this->cloud_I.reset(new PointCloudI);
    this->cloud_L.reset(new PointCloudL);

    this->env_count = 0;
    this->ousterReady = false;
    this->handle_to_cam = false;

    this->take_screenshot = false;
    this->laser_retro = 1;
    this->config.simulation.paused = true;
  }

  /////////////////////////////////
  DatasetGenerator::~DatasetGenerator()
  {
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

    this->generator_thread = std::thread(std::bind(&DatasetGenerator::GenerateDataset, this));

    ROS_INFO(GREEN "ARVC GAZEBO SPAWNMODEL PLUGIN LOADED" RESET);
  }

  // MAIN FUNCTION
  //////////////////////////////////////////////////////////////////////////////
  void DatasetGenerator::GenerateDataset()
  {

    // Wait for the sensor to be ready
    while (!this->sensor_model)
    {
      this->sensor_model = this->world->ModelByName(this->config.sensor.name);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    while (!this->camera_model)
    {
      this->camera_model = this->world->ModelByName(this->config.camera.name);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::vector<std::string> env_models_;
    std::vector<std::string> models_;
    std::vector<std::string> all_models_;

    int estado = 0;
    gazebo::common::Console::SetQuiet(true);

    while (this->env_count < this->config.out_data.quantity)
    {
      switch (estado)
      {
      case 0:
        this->console.info("STARTING TO SPAWN MODELS...");
        this->ResumeEnvCount();
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        estado = 1;
        break;

      case 1:
        this->console.info("GENERATING RANDOM ENVIROMENT..." + std::to_string(this->env_count));
        this->MoveGroundModel();
        env_models_ = this->SpawnRandomEnviroment();
        models_ = this->SpawnRandomModels();
        this->ApplyRotation(this->sensor_model, this->ComputeRandRotation());

        all_models_.clear();
        all_models_.resize(env_models_.size() + models_.size());
        std::set_union(env_models_.begin(), env_models_.end(), models_.begin(), models_.end(), all_models_.begin());

        ROS_INFO_COND(this->config.simulation.debug_msgs, BLUE "MODELS SPAWNED: %d" RESET, (int)all_models_.size());

        estado = 2;
        break;

      case 2:
        if (this->CheckSpawnedModels(all_models_))
        {
          if (this->config.simulation.paused)
          {
            ROS_INFO(YELLOW "PAUSED: Press enter to continue ..." RESET);
            std::getchar();
          }
          estado = 3;
        }
        break;

      case 3:
        std::this_thread::sleep_for(std::chrono::milliseconds(this->config.simulation.data_capture_delay));

        if (this->config.camera.enable)
          this->TakeScreenShot();

        if (this->config.out_data.enable)
          this->SavePointCloud();

        this->removeModelsByName(all_models_);
        estado = 4;
        break;

      case 4:
        if (this->CheckDeletedModels(all_models_))
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

      std::this_thread::sleep_for(std::chrono::milliseconds(this->config.simulation.iteration_delay));
    }
    this->console.info("FINISHED GENERATING DATASET");
  }


  void DatasetGenerator::GetYamlConfig()
  {
    fs::path package_path(ros::package::getPath("arvc_dataset_generator"));
    fs::path config_path = package_path / "config/dataset_generator_config.yaml";

    this->config = arvc::plugin::configuration(config_path);
    std::cout << YELLOW << "YAML CONFIG PATH: " << RESET << "\n " << config_path.string().c_str() << std::endl;
  }


  std::vector<std::string>
  DatasetGenerator::SpawnRandomParalellepipeds()
  {
    
    this->console.debug("SPAWNING RANDOM PARALELEPIPEDS...");

    std::vector<std::string> models;

    for (int i = 0; i < this->config.lab_mod.num_lbld_models; i++)
    {
      this->inserting_model_cfg = this->config.lab_mod.model[i];

      for (int j = 0; i < this->inserting_model_cfg.num_models; j++)
        this->InsertModel(j);
    }

    ROS_INFO_COND(this->config.simulation.debug_msgs, "MODELS SPAWNED CORRECTLY");
    return models;
  }

  std::vector<std::string>
  DatasetGenerator::SpawnRandomEnviroment()
  {
    ROS_INFO_COND(this->config.simulation.debug_msgs, "SPAWNING ENVIROMENT...");

    std::vector<std::string> models;
    std::string model_name;

    // For each enviroment model
    for (int i = 0; i < this->config.env.num_env_models; i++)
    {
      arvc::plugin::model_base actual_model = this->config.env.model[i];

      // Spawn N models of the same type
      for (int j = 0; i < this->num_env_models; j++)
      {



      }
        this->InsertModel(actual_model, j);
    }

    ROS_INFO_COND(this->config.simulation.debug_msgs, "ENVIROMENT SPAWNED CORRECTLY");
    return models;
  }

  void
  DatasetGenerator::InsertModel(int model_idx)
  {

    fs::path original_file = this->inserting_model_cfg.path / "model.sdf";
    fs::path temp_file = this->GetTemporarySDFfile(original_file);

    sdf::SDFPtr temp_sdfFile = this->GetSDFfile(temp_file);
    sdf::ElementPtr modelElement = temp_sdfFile->Root()->GetElement("model");

    string model_name = this->SetModelName(modelElement, this->inserting_model_cfg.name, model_idx);

    if (this->inserting_model_cfg.type == "environment")
    {
      this->SetModelPosition(modelElement, this->inserting_model_cfg);
      this->SetRandomMeshScale(modelElement, this->inserting_model_cfg);
      this->inserted_environment_models_names.push_back(model_name);
    }
    else if (this->inserting_model_cfg.type == "labeled")
    {
      this->SetRandomScale(modelElement, this->inserting_model_cfg);
      this->SetModelPose(modelElement, this->inserting_model_cfg);
      this->IncreaseVisualLaserRetro(modelElement);
      this->inserted_labeled_models_names.push_back(model_name);
    }

    ROS_INFO_COND(this->config.simulation.debug_msgs, "SPAWNING MODEL: %s", model_name.c_str());
    this->world->InsertModelSDF(*temp_sdfFile);
    boost::this_thread::sleep(boost::posix_time::milliseconds(10));
  }

  /////////////////////////////////
  void DatasetGenerator::removeModels()
  {
    ROS_INFO_COND(this->config.simulation.debug_msgs, "DELETING MODELS...");

    this->world->SetPaused(true);

    physics::Model_V actual_models_ = this->world->Models();

    for (const auto &model_ : actual_models_)
    {
      std::string name = model_->GetName();
      if (name != this->sensor_name && name != this->world_name && name != this->cam_name)
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
    for (const std::string &name : models)
    {
      ROS_INFO_COND(this->config.simulation.debug_msgs, "DELETING MODEL: %s", name.c_str());
      this->world->RemoveModel(name);
    }
    this->world->SetPaused(false);
  }
  // CHECK FUNCTIONS
  /////////////////////////////////
  void DatasetGenerator::CheckOutputDirs()
  {
    this->pcd_dir = this->config.out_data.out_dir / "pcd";
    this->img_dir = this->config.out_data.out_dir / "images";

    if (!fs::exists(this->config.out_data.out_dir))
      fs::create_directory(this->config.out_data.out_dir);

    if (!fs::exists(this->pcd_dir))
      fs::create_directory(this->pcd_dir);

    if (!fs::exists(this->img_dir))
      fs::create_directory(this->img_dir);

    std::cout << YELLOW << "PCD OUTPUT DIR: " << RESET << "\n " << this->pcd_dir.c_str() << std::endl;
    std::cout << YELLOW << "IMAGES OUTPUT DIR: " << RESET << "\n " << this->img_dir.c_str() << std::endl;
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
        if (entry.path().extension() == ".pcd")
        {
          try
          {
            int actual_num = std::stoi(entry.path().stem());

            if (actual_num > last_num)
              last_num = actual_num;
          }

          catch (const std::exception &e)
          {
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
    ROS_INFO_COND(this->config.simulation.debug_msgs, "CHECKING IF OUSTER IS READY");
    if (!this->world->ModelByName(this->config.sensor.name))
      return false;
    else
    {
      ROS_INFO(GREEN "%s SENSOR IS READY" RESET, this->config.sensor.name.c_str());
      this->sensor_model = this->world->ModelByName(this->config.sensor.name);
      return true;
    }
  }

  /////////////////////////////////
  bool DatasetGenerator::CheckSpawnedModels(std::vector<std::string> model_names)
  {
    ROS_INFO_COND(this->config.simulation.debug_msgs, "CHECKING SPAWNED MODELS");

    int spawned_models = 0;
    for (auto model_name : model_names)
    {
      if (!this->world->ModelByName(model_name))
      {
        ROS_INFO_COND(this->config.simulation.debug_msgs, YELLOW "CAN'T FIND MODEL: %s" RESET, model_name.c_str());
        ROS_INFO_COND(this->config.simulation.debug_msgs, YELLOW "FOUND MODELS: %d" RESET, (int)spawned_models);
        return false;
      }
      spawned_models++;
    }
    ROS_INFO_COND(this->config.simulation.debug_msgs, YELLOW "FOUND MODELS: %d" RESET, (int)spawned_models);
    ROS_INFO_COND(this->config.simulation.debug_msgs, GREEN "MODELS SPAWNED CORRECTLY" RESET);

    return true;
  }

  /////////////////////////////////
  bool DatasetGenerator::CheckDeletedModels(std::vector<std::string> model_names)
  {
    ROS_INFO_COND(this->config.simulation.debug_msgs, "CHECKING DELETE MODELS");
    for (auto model_name : model_names)
    {
      if (this->world->ModelByName(model_name))
      {
        ROS_INFO_COND(this->config.simulation.debug_msgs, YELLOW "MODEL STILL REMAINING: %s" RESET, model_name.c_str());
        return false;
      }
    }
    ROS_INFO_COND(this->config.simulation.debug_msgs, GREEN "MODELS DELETED CORRECTLY" RESET);
    return true;
  }

  /////////////////////////////////
  std::vector<std::string> DatasetGenerator::RemoveCollideModels(physics::ModelPtr sensor_model)
  {
    using namespace im;

    std::vector<std::string> removed_models;

    physics::Model_V models = this->world->Models();
    AxisAlignedBox sensor_bbx = sensor_model->CollisionBoundingBox();

    for (size_t i = 0; i < models.size(); i++)
    {
      if (models[i]->GetName() != this->sensor_model->GetName())
      {
        AxisAlignedBox model_bbx = models[i]->CollisionBoundingBox();

        if (sensor_bbx.Intersects(model_bbx))
        {
          removed_models.push_back(models[i]->GetName());
          this->world->RemoveModel(models[i]);
        }
      }
    }

    return removed_models;
  }



  void DatasetGenerator::SavePointCloud()
  {
    ROS_INFO_COND(this->config.simulation.debug_msgs, "SAVING POINTCLOUD...");
    pcl::PCDWriter writer;
    std::stringstream ss;
    ss.str("");
    ss << this->pcd_dir.string() << "/" << std::setfill('0') << std::setw(5) << this->env_count << ".pcd";

    pcl::copyPointCloud(*this->cloud_I, *this->cloud_L);


    for (size_t i = 0; i < this->cloud_I->points.size(); i++)
    {
      this->cloud_L->points[i].label = this->cloud_I->points[i].intensity; 
    }
    

    if (!this->cloud_L->empty())
    {

      if (this->cloud_L->points.size() != this->cloud_L->width)
      {
        int cloud_size = this->cloud_L->points.size();
        this->cloud_L->width = cloud_size;
        this->cloud_L->height = 1;
      }
      writer.write<PointL>(ss.str(), *this->cloud_L, this->pc_binary);
    }
  }


  //---- ROS -----------------------------------------------------//
  //--------------------------------------------------------------//

  void DatasetGenerator::SetupROS()
  {
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
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

  void DatasetGenerator::QueueThread()
  {
    static const double timeout = 0.01;
    while (this->ros_node->ok())
    {
      this->ros_cbqueue.callAvailable(ros::WallDuration(timeout));
    }
  }

  void DatasetGenerator::PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &input)
  {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);

    PointCloudI::Ptr temp_cloud(new PointCloudI);
    pcl::fromPCLPointCloud2(pcl_pc2, *this->cloud_I);

    // *pcl_cloud = *temp_cloud;
  }


}

/* namespace YAML
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
} */