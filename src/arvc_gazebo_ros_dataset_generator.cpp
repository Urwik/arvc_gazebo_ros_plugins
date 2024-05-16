#include "arvc_gazebo_ros_plugins/arvc_gazebo_ros_dataset_generator.h"
#include "sdf_utils.hpp"
#include "train_utils.hpp"
#include "yaml_conversions.hpp"

using namespace std;

namespace gazebo
{
  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(DatasetGenerator)
  /////////////////////////////////

  DatasetGenerator::DatasetGenerator()
  {

    int i = 30;
    std::cout << "Delay to enable the attach gdb vscode debug" << std::endl;
    while (i>0) {
      sleep(1);
      std::cout << i << std::endl;
      i--;
    }


    cout << RED << "Running Plugin Constructor..." << RESET << endl;
    this->cloud_I.reset(new PointCloudI);
    this->cloud_L.reset(new PointCloudL);

    this->env_count = 0;
    this->laser_retro = 1;
  }

  /////////////////////////////////
  DatasetGenerator::~DatasetGenerator()
  {
  }

  //////////////////////////////////////////////////////////////////////////////
  void DatasetGenerator::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
  {
    this->world = _parent;

    this->getConfig(_sdf);

    this->SetupROS();

    this->CheckOutputDirs();

    this->generator_thread = std::thread(std::bind(&DatasetGenerator::GenerateDataset, this));

    this->console.info("ARVC GAZEBO SPAWNMODEL PLUGIN LOADED", GREEN);
  }

  // MAIN FUNCTION
  //////////////////////////////////////////////////////////////////////////////
  void DatasetGenerator::GenerateDataset()
  {
    std::mutex mtx;

    this->console.debug("Inserting sensor model");
    this->insertSensorModel();

    // Wait for the sensor to be ready
    while (!this->sensor_model)
    {
      mtx.lock();
      this->sensor_model = this->world->ModelByName(this->config["sensor"]["name"].as<std::string>());
      mtx.unlock();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::vector<std::string> env_models_;
    std::vector<std::string> par_models_;
    std::vector<std::string> all_models_;

    int estado = 0;
    gazebo::common::Console::SetQuiet(true);

    while (this->env_count < this->config["generator"]["items_to_generate"].as<int>())
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
        // this->MoveGroundModel();
        this->rotateSensorModel();
        env_models_ = this->SpawnRandomEnviroment();
        par_models_     = this->SpawnRandomParalellepipeds();

        all_models_.clear();
        all_models_.resize(env_models_.size() + par_models_.size());
        std::set_union(env_models_.begin(), env_models_.end(), par_models_.begin(), par_models_.end(), all_models_.begin());

        this->console.debug("Parallelepipeds spawned: " + std::to_string(par_models_.size()));
        this->console.debug("Enviroment spawned: " + std::to_string(env_models_.size()));

        estado = 2;
        break;

      case 2:
        if (this->CheckSpawnedModels(all_models_))
        {
          if (this->config["generator"]["paused"].as<bool>())
          {
            this->console.info("## PAUSED: Press enter to continue ...", YELLOW);
            std::getchar();
          }
          estado = 3;
        }
        break;

      case 3:
        if (this->config["generator"]["data"]["save"].as<bool>())
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

      std::this_thread::sleep_for(std::chrono::milliseconds(this->config["generator"]["iteration_delay"].as<int>()));
    }
    this->console.info("FINISHED GENERATING DATASET", GREEN);
  }

  void DatasetGenerator::getConfig(sdf::ElementPtr _sdf)
  {
    std::cout << BLUE << "PARSING ARGUMENTS... " << RESET << std::endl;

    if (_sdf->HasElement("yaml_config")) {
    std::string yaml_config = _sdf->GetElement("yaml_config")->Get<std::string>();
    this->config = YAML::LoadFile(yaml_config);
    }
    else {
      std::cout << RED << "NO FOUND param yaml_config inside sdf model plugin declaration" << RESET << std::endl;
    } 
  }

  void DatasetGenerator::insertSensorModel(){

    sdf::SDFPtr sensor_sdf = utils::getSDFfile(this->config["sensor"]["model_path"].as<std::string>());
    sdf::ElementPtr sensor_element = sensor_sdf->Root()->GetElement("model");

    std::string sensor_name = this->config["sensor"]["name"].as<std::string>();
    utils::setModelName(sensor_element, sensor_name);

    sdf::ElementPtr cylinder_elem = sensor_element->GetElement("link")->GetElement("collision")->GetElement("geometry")->GetElement("cylinder");
    float current_radius = cylinder_elem->GetElement("radius")->Get<float>();
    float current_length = cylinder_elem->GetElement("length")->Get<float>();

    float new_radius = this->config["sensor"]["collision_offset"].as<float>() + current_radius;
    float new_length = this->config["sensor"]["collision_offset"].as<float>() + current_length;

    cylinder_elem->GetElement("radius")->Set(new_radius);
    cylinder_elem->GetElement("length")->Set(new_length);

    std::mutex mtx;
    mtx.lock();
    this->world->InsertModelSDF(*sensor_sdf);
    mtx.unlock();
  }

  void DatasetGenerator::rotateSensorModel(){
    std::mutex mtx;

    im::Pose3d new_pose;
    im::Vector3d rotation = utils::computeRandomRotation();

    mtx.lock();
    im::Pose3d orig_pose = this->sensor_model->WorldPose();
    new_pose.Set(orig_pose.Pos(), rotation);
    sensor_model->SetWorldPose(new_pose);
    mtx.unlock();
  }

  void DatasetGenerator::removeModelsByName(std::vector<std::string> models)
  {
    this->console.debug("DELETING MODELS...");

    std::mutex mtx;

    // this->world->SetPaused(true);
    for (const std::string &model_name : models)
    {
      mtx.lock();
      this->console.debug("DELETING MODEL: " + model_name);
      this->world->RemoveModel(model_name);
      mtx.unlock();
    }
    // this->world->SetPaused(false);
  }

  std::vector<std::string> DatasetGenerator::SpawnRandomParalellepipeds()
  {
    this->console.debug("SPAWNING RANDOM PARALELEPIPEDS...");
    int item_count = this->config["paralellepipeds"]["item_count"].as<int>();
    fs::path model_path   = this->config["paralellepipeds"]["model_path"].as<fs::path>();
    im::Vector2d length   = this->config["paralellepipeds"]["length"].as<im::Vector2d>();
    im::Vector2d width    = this->config["paralellepipeds"]["width"].as<im::Vector2d>();
    im::Vector2d height   = this->config["paralellepipeds"]["height"].as<im::Vector2d>();
    im::Vector3d max_pos  = this->config["paralellepipeds"]["position"]["max"].as<im::Vector3d>();
    im::Vector3d min_pos  = this->config["paralellepipeds"]["position"]["min"].as<im::Vector3d>();


    std::vector<std::string> model_names;
    int laser_retro_count = 1;
    for (int i = 0; i < this->config["paralellepipeds"]["item_count"].as<int>(); i++)
    {
      fs::path orig_model_sdf = model_path / "model.sdf";
      fs::path temp_model_sdf = utils::copySDFfile(orig_model_sdf);

      sdf::SDFPtr temp_sdfFile = utils::getSDFfile(temp_model_sdf);
      sdf::ElementPtr model_element = temp_sdfFile->Root()->GetElement("model");
      
      std::string model_name = "paralellepiped_" + std::to_string(i);

      utils::setModelName(model_element, model_name);
      utils::setLaserRetroForVisualElement(model_element, laser_retro_count);
      
      bool collision = true;
      std::mutex mtx;
      do
      {
        im::Pose3d pose = utils::computeRandomPose(min_pos, max_pos);
        im::Vector3d scale = utils::computeRandomScale(length, width, height);

        utils::setModelPose(model_element, pose);
        utils::setModelScale(model_element, scale);

        mtx.lock();        
        this->world->InsertModelSDF(*temp_sdfFile);
        mtx.unlock();

        collision = this->checkCollisions(model_name, this->sensor_model->GetName());

        if (!collision)
        {
          model_names.push_back(model_name);
          laser_retro_count++;
        }
        else
        {
          mtx.lock();
          this->world->RemoveModel(model_name);
          mtx.unlock();
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

      } while (collision);
    }
    this->console.debug("PARALELLEPIPEDS SPAWNED CORRECTLY");
    return model_names;
  }

  std::vector<std::string> DatasetGenerator::SpawnRandomEnviroment()
  {
    this->console.debug("SPAWNING RANDOM ENVIROMENT...");

    int item_count = this->config["environment"]["item_count"].as<int>();
    fs::path model_path   = this->config["environment"]["model_path"].as<fs::path>();
    im::Vector2d length   = this->config["environment"]["length"].as<im::Vector2d>();
    im::Vector2d width    = this->config["environment"]["width"].as<im::Vector2d>();
    im::Vector2d height   = this->config["environment"]["height"].as<im::Vector2d>();
    im::Vector3d max_pos  = this->config["environment"]["position"]["max"].as<im::Vector3d>();
    im::Vector3d min_pos  = this->config["environment"]["position"]["min"].as<im::Vector3d>();


    std::vector<std::string> model_names;

    for (int i = 0; i < item_count; i++)
    {
      fs::path orig_model_sdf = model_path / "model.sdf";
      fs::path temp_model_sdf = utils::copySDFfile(orig_model_sdf);

      sdf::SDFPtr temp_sdfFile = utils::getSDFfile(temp_model_sdf);
      sdf::ElementPtr model_element = temp_sdfFile->Root()->GetElement("model");
      
      std::string model_name = model_element->GetName() + "_" + std::to_string(i);

      utils::setModelName(model_element, model_name);
      
      bool collision = true;
      std::mutex mtx;
      do
      {
        im::Pose3d pose = utils::computeRandomPose(min_pos, max_pos);
        im::Vector3d scale = utils::computeRandomScale(length, width, height);

        utils::setModelPose(model_element, pose);
        utils::setModelScale(model_element, scale);

        mtx.lock();        
        this->world->InsertModelSDF(*temp_sdfFile);
        while (!this->world->ModelByName(model_name))
          std::this_thread::sleep_for(std::chrono::milliseconds(10));

        mtx.unlock();

        collision = this->checkCollisions(model_name, this->sensor_model->GetName());

        if (!collision)
          model_names.push_back(model_name);
        else
        {
          mtx.lock();
          this->world->RemoveModel(model_name);
          mtx.unlock();
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

      } while (collision);
    }
    this->console.debug("PARALELLEPIPEDS SPAWNED CORRECTLY");
    return model_names;
  }



  //---- CHECK FUNCTIONS -----------------------------------------------------//
  void DatasetGenerator::CheckOutputDirs()
  {
    this->console.debug("CHECKING OUTPUT DIRECTORIES...");
    this->pcd_dir = this->config["generator"]["data"]["out_dir"].as<std::string>() + "/pcd";

    if (!fs::exists(this->pcd_dir))
    {
      this->console.debug("Creating output directories");
      fs::create_directories(this->pcd_dir);
    }

  }

  void DatasetGenerator::ResumeEnvCount()
  {
    this->console.debug("RESUMING ENVIRONMENT COUNT...");
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
           this->console.info("Error the pcd file has no correct name: " + entry.path().string());
          }
        }
      }

      this->env_count = last_num + 1;
      this->console.info("Starting in Env: " + std::to_string(this->env_count));
    }
    else
    {
      this->env_count = 0;
      this->console.debug("No previous data found, starting from: " + std::to_string(this->env_count));
    }
  }

  bool DatasetGenerator::CheckSpawnedModels(std::vector<std::string> model_names)
  {
    this->console.debug("Checking if all models were spawned correctly");
    int spawned_models = 0;
    std::mutex mtx;
    for (auto model_name : model_names)
    {
      mtx.lock();
      if (!this->world->ModelByName(model_name)) 
      {
        mtx.unlock();
        this->console.debug("Not found all models, trying in next step");
        return false;
      }
      mtx.unlock();
    }
    this->console.debug("All models were spawned correctly", GREEN);
    return true;
  }

  bool DatasetGenerator::CheckDeletedModels(std::vector<std::string> model_names)
  {
    this->console.debug("Checking if all models were deleted correctly");

    std::mutex mtx;

    for (auto model_name : model_names)
    {
      mtx.lock();
      if (this->world->ModelByName(model_name))
      {
        mtx.unlock();
        this->console.debug("Not all models were deleted, trying in next step");
        return false;
      }
      mtx.unlock();
    }
    this->console.debug("All models were deleted correctly", GREEN);
    return true;
  }

  bool DatasetGenerator::checkCollisions(std::string model_name_1, std::string model_name_2)
  {
    physics::ModelPtr model_a;
    physics::ModelPtr model_b;

    std::mutex mtx;
    mtx.lock();

    do 
    {
      model_a = this->world->ModelByName(model_name_1);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    while (!model_a);

    do 
    {
      model_b = this->world->ModelByName(model_name_2);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    while (!model_b);

    im::AxisAlignedBox model_a_bbx = model_a->CollisionBoundingBox();
    im::AxisAlignedBox model_b_bbx = model_b->CollisionBoundingBox();

    bool intersection = model_a_bbx.Intersects(model_b_bbx);
    mtx.unlock();

    return intersection;

  }



  //---- POINTCLOUD -----------------------------------------------------//
  void DatasetGenerator::SavePointCloud()
  {
    this->console.debug("SAVING POINTCLOUD...");
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
      writer.write<PointL>(ss.str(), *this->cloud_L, this->config["generator"]["data"]["binary"].as<bool>());
    }
  }


  //---- ROS -----------------------------------------------------//
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
            this->config["sensor"]["topic"].as<std::string>(), 1,
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