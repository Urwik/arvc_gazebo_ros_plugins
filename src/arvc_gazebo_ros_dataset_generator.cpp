#include "arvc_gazebo_ros_plugins/arvc_gazebo_ros_dataset_generator.hpp"
#include "sdf_utils.hpp"
#include "train_utils.hpp"
#include "yaml_conversions.hpp"
#include "timer_utils.hpp"

using namespace std;

namespace gazebo
{
  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(DatasetGenerator)
  /////////////////////////////////

  DatasetGenerator::DatasetGenerator()
  {
    // std::cout << "Delay to enable the attach gdb vscode debug" << std::endl;
    // bool gdb_attached = false;
    // while (!gdb_attached) {
    //   sleep(1);
    // }

    cout << RED << "Running Plugin Constructor..." << RESET << endl;
    this->cloud_I.reset(new PointCloudI);
    this->cloud_L.reset(new PointCloudL);

    this->env_count = 0;
    this->laser_retro = 1;
  }

  /////////////////////////////////
  DatasetGenerator::~DatasetGenerator()  {
    if (generator_thread.joinable()) {
    generator_thread.join();
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  void DatasetGenerator::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
  {
    this->world = _parent;

    this->getConfig(_sdf);

    this->SetupROS();

    this->CheckOutputDirs();

    this->generator_thread = boost::thread(boost::bind(&DatasetGenerator::GenerateDataset, this));

    this->console.info("ARVC GAZEBO DATASET GENERATOR PLUGIN LOADED", GREEN);
  }

  //////////////////////////////////////////////////////////////////////////////
  // MAIN FUNCTION
  void DatasetGenerator::GenerateDataset()
  {

    this->world->Reset();
    this->world->SetPhysicsEnabled(this->config["physics"].as<bool>());
    bool physics_enabled = this->world->PhysicsEnabled();
    this->console.debug("Physics enabled: " + std::to_string(physics_enabled));
    this->world->SetPaused(true);

    std::vector<std::string> env_models;
    std::vector<std::string> par_models;
    std::vector<std::string> all_models;

    gazebo::common::Console::SetQuiet(true);

    bool first_run = true;
    int items_to_generate = this->config["generator"]["items_to_generate"].as<int>();
    int env_change_iteration = this->config["environment"]["change_iteration"].as<int>();
    int par_change_iteration = this->config["paralellepipeds"]["change_iteration"].as<int>();
    bool env_move = this->config["environment"]["move"].as<bool>();
    bool par_move = this->config["paralellepipeds"]["move"].as<bool>();
    int env_change_counter = 0;
    int par_change_counter = 0;

    while (this->env_count < items_to_generate)
    {

      if (first_run) {
        this->console.info("FIRST ENVIRONMENT GENERATION STARTING TO SPAWN MODELS...");
        this->ResumeEnvCount();
        this->insertSensorModel();

        // Environment
        env_models = this->SpawnRandomEnviroment();
        // this->world->Step(10);
        this->removeModelsInCollision(env_models, this->sensor_model->GetName());
        // this->moveDownTillCollisionWithGround(env_models); // TODO
        env_change_counter++;

        // Paralellepipeds
        par_models = this->SpawnRandomParalellepipeds();
        // this->world->Step(10);
        this->removeModelsInCollision(par_models, this->sensor_model->GetName());
        par_change_counter++;

        this->world->SetPaused(false);
        // this->world->Step(10);

        if (this->config["generator"]["paused"].as<bool>()) {
          this->console.info("## PAUSED ##: Press enter to continue ...", YELLOW);
          std::getchar();
        }

        if (this->config["generator"]["data"]["save"].as<bool>())
          this->SavePointCloud();

        first_run = false;
      }

      else {
        this->console.info("GENERATING RANDOM ENVIROMENT..." + std::to_string(this->env_count));

        if (env_change_iteration != 0){
          if (env_change_counter == env_change_iteration) {
            this->console.debug("Enviroment change iteration reached");
            this->removeModelsByName(env_models);
            env_models = this->SpawnRandomEnviroment();
            this->removeModelsInCollision(env_models, this->sensor_model->GetName());

            // this->moveDownTillCollisionWithGround(env_models);
            env_change_counter = 0;
            this->console.debug("New Enviroment spawned");
          }
          else
          {
            if (env_move)
              this->moveEnvironmentRandomly(env_models);
          }
        }

        if (par_change_iteration != 0){
          if (par_change_counter == par_change_iteration) {
            this->console.debug("Paralellepipeds change iteration reached");
            this->removeModelsByName(par_models);
            par_models = this->SpawnRandomParalellepipeds();
            this->removeModelsInCollision(par_models, this->sensor_model->GetName());
            par_change_counter = 0;
            this->console.debug("New Paralellepipeds spawned");
          }
          else
          {
            if (par_move)
              this->moveParallellepipedRandomly(par_models);
          }
        }

        this->rotateSensorModel();
        this->console.debug("Sensor rotated");

        this->world->SetPaused(false);
        this->world->Step(10);

        if (this->config["generator"]["paused"].as<bool>()) {
          this->console.info("## PAUSED ##: Press enter to continue ...", YELLOW);
          std::getchar();
        }

        if (this->config["generator"]["data"]["save"].as<bool>())
          this->SavePointCloud();

        env_change_counter++;
        par_change_counter++;
        this->env_count++;
      }
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
      std::cout << RED << "Param yaml_config inside plugin declaration" << RESET << std::endl;
    }

    this->console.enable = this->config["debug"].as<bool>(); 
  }

  void DatasetGenerator::insertSensorModel(){

    this->console.debug("INSERTING SENSOR MODEL...");
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

    boost::mutex mtx;
    mtx.lock();
    this->world->InsertModelSDF(*sensor_sdf);
    // Wait for the sensor to be ready
    this->console.debug("Waiting for sensor model to be ready...");
    this->sensor_model = this->world->ModelByName(sensor_name);
    mtx.unlock();

    while (!this->sensor_model)
    {
      // mtx.lock();
      this->sensor_model = this->world->ModelByName(sensor_name);
      // mtx.unlock();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    this->console.debug("Sensor MODEL ready", GREEN);

  }

  void DatasetGenerator::rotateSensorModel(){
    boost::mutex mtx;

    im::Pose3d new_pose;
    im::Vector3d rotation = utils::computeRandomRotation();

    mtx.lock();
    im::Pose3d orig_pose = this->sensor_model->WorldPose();
    new_pose.Set(orig_pose.Pos(), rotation);
    this->sensor_model->SetWorldPose(new_pose);
    mtx.unlock();
  }


  void DatasetGenerator::removeModelsByName(std::vector<std::string> models)
  {
    this->world->SetPaused(true);

    this->console.debug("DELETING MODELS...");

    if (models.size() == 0)
    {
      this->console.debug("No models to delete");
      return;
    }

    for (const std::string &model_name : models)
    {
      this->console.debug("DELETING MODEL: " + model_name);
      this->world->RemoveModel(model_name);
      while (this->world->ModelByName(model_name))
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    this->world->SetPaused(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  std::vector<std::string> DatasetGenerator::SpawnRandomParalellepipeds()
  {
    this->world->SetPaused(true);
    
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
    for (int i = 0; i < item_count; i++)
    {
      fs::path orig_model_sdf = model_path / "model.sdf";
      fs::path temp_model_sdf = utils::copySDFfile(orig_model_sdf);

      sdf::SDFPtr temp_sdfFile = utils::getSDFfile(temp_model_sdf);
      sdf::ElementPtr model_element = temp_sdfFile->Root()->GetElement("model");
      
      std::string model_name = "paralellepiped_" + std::to_string(i);

      utils::setModelName(model_element, model_name);

      if (this->config["paralellepipeds"]["label_per_face"].as<bool>())
        laser_retro_count = utils::setIncrementalLaserRetroForVisualElement(model_element, laser_retro_count);
      else
        utils::setLaserRetroForVisualElement(model_element, laser_retro_count);
      
      im::Pose3d pose = utils::computeRandomPose(min_pos, max_pos);
      im::Vector3d scale = utils::computeRandomScale(length, width, height);

      utils::setModelPose(model_element, pose);
      utils::setModelScale(model_element, scale);

      this->world->InsertModelSDF(*temp_sdfFile);
      while (!this->world->ModelByName(model_name))
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

      model_names.push_back(model_name);
      laser_retro_count++;

    }
    this->world->SetPaused(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    this->console.debug("PARALELLEPIPEDS SPAWNED CORRECTLY");

    return model_names;
  }

  std::vector<std::string> DatasetGenerator::SpawnRandomEnviroment()
  {
    this->world->SetPaused(true);

    this->console.debug("SPAWNING RANDOM ENVIROMENT...");

    int item_count = this->config["environment"]["item_count"].as<int>();
    fs::path model_path   = this->config["environment"]["model_path"].as<fs::path>();
    im::Vector2d length   = this->config["environment"]["length"].as<im::Vector2d>();
    im::Vector2d width    = this->config["environment"]["width"].as<im::Vector2d>();
    im::Vector2d height   = this->config["environment"]["height"].as<im::Vector2d>();
    im::Vector3d max_pos  = this->config["environment"]["position"]["max"].as<im::Vector3d>();
    im::Vector3d min_pos  = this->config["environment"]["position"]["min"].as<im::Vector3d>();
    im::Vector2d scale_val = this->config["environment"]["scale"].as<im::Vector2d>();


    std::vector<std::string> model_names;

    for (int i = 0; i < item_count; i++)
    {
      fs::path orig_model_sdf = model_path / "model.sdf";
      fs::path temp_model_sdf = utils::copySDFfile(orig_model_sdf);
      this->console.debug("Crea una copia del SDF: " + temp_model_sdf.string());

      sdf::SDFPtr temp_sdfFile = utils::getSDFfile(temp_model_sdf);
      
      if (!temp_sdfFile)
        this->console.debug("Error al cargar el SDF: " + temp_model_sdf.string());
      else
        this->console.debug("Obtiene un puntero al SDF: " + temp_sdfFile->Root()->GetName());

      sdf::ElementPtr model_element = temp_sdfFile->Root()->GetElement("model");
      this->console.debug("Obtiene un puntero al modelo: " + model_element->GetName());

      std::string model_name = model_element->GetName() + "_" + std::to_string(i);
      utils::setModelName(model_element, model_name);
      this->console.debug("Setea el nombre del modelo: " + model_name);

      im::Vector3d position = utils::computeRandomPosition(min_pos, max_pos);
      this->console.debug("Genera una posicion aleatoria: ");
      // im::Vector3d scale = utils::computeRandomScale(length, width, height); // DIFFERENT SCALE FOR EACH AXIS
      im::Vector3d scale = utils::computeRandomScale(scale_val);                // SAME SCALE FOR ALL AXIS

      this->console.debug("Genera una escala aleatoria: ");

      utils::setModelPosition(model_element, position);
      this->console.debug("Setea la pose del modelo: ");
      utils::setMeshScale(model_element, scale);
      this->console.debug("Setea la escala del modelo: ");

      this->world->InsertModelSDF(*temp_sdfFile);
      // Wait for model to be inserted
      while (!this->world->ModelByName(model_name))
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      this->console.debug("Inserta el modelo en el mundo: ");

      model_names.push_back(model_name);
    }
    
    this->world->SetPaused(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    this->console.debug("ENVIRONMENT SPAWNED CORRECTLY");
    return model_names;
  }


  void DatasetGenerator::moveEnvironmentRandomly(std::vector<std::string> model_names){

    this->world->SetPaused(true); 
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    im::Vector3d max_pos  = this->config["environment"]["position"]["max"].as<im::Vector3d>();
    im::Vector3d min_pos  = this->config["environment"]["position"]["min"].as<im::Vector3d>();


    for (std::string tmp_model_name : model_names) {
      physics::ModelPtr tmp_model;

      do {
        tmp_model = this->world->ModelByName(tmp_model_name);
      } while (!tmp_model);

      im::Vector3d position = utils::computeRandomPosition(min_pos, max_pos);

      im::Pose3d tmp_pose = tmp_model->WorldPose();
      tmp_pose.Pos() = position;

      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      tmp_model->SetWorldPose(tmp_pose);
    }

    this->world->SetPaused(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
  }

  void DatasetGenerator::moveParallellepipedRandomly(std::vector<std::string> model_names){

    this->world->SetPaused(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));


    im::Vector3d max_pos  = this->config["paralellepipeds"]["position"]["max"].as<im::Vector3d>();
    im::Vector3d min_pos  = this->config["paralellepipeds"]["position"]["min"].as<im::Vector3d>();

    for (std::string tmp_model_name : model_names) {
      physics::ModelPtr tmp_model;

      do {
        tmp_model = this->world->ModelByName(tmp_model_name);
      } while (!tmp_model);

      im::Pose3d tmp_pose = utils::computeRandomPose(min_pos, max_pos);
     
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      tmp_model->SetWorldPose(tmp_pose);
    }
    
    this->world->SetPaused(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

  }

  void DatasetGenerator::moveDownTillCollisionWithGround(std::vector<std::string> model_name)
  {
    this->console.debug("MOVING DOWN TILL COLLISION WITH GROUND...");

    std::string ground_name = this->config["generator"]["ground_name"].as<std::string>();
    for (const std::string &model_name : model_name) {

 
      physics::ModelPtr model;
      do {
        model = this->world->ModelByName(model_name);
      } while (!model);


      model->PlaceOnEntity(ground_name);

      // im::Pose3d pose = model->WorldPose();
      // im::Vector3d position = pose.Pos();
      // im::Vector3d new_position = position;

      // while (true) {
      //   new_position.Z() -= 0.05;
      //   pose.Set(new_position, pose.Rot());
      //   model->SetWorldPose(pose);
      //   this->console.debug("Moving model down 5cm: " + model_name);

      //   if (this->checkCollisions(model_name, this->config["generator"]["ground_name"].as<std::string>())) {
      //     this->console.debug("Collision detected with ground: " + model_name, GREEN);
      //     break;
      //   }
      // }
    }
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

  bool DatasetGenerator::checkCollisions(std::string model_name_a, std::string model_name_b)
  {
    this->console.debug("-- CHECKING COLLISIONS", YELLOW);
    this->console.debug("model_a: " + model_name_a, RESET);
    this->console.debug("model_b: " + model_name_b, RESET);
    physics::ModelPtr model_a;
    physics::ModelPtr model_b;
    im::AxisAlignedBox model_a_bbx;
    im::AxisAlignedBox model_b_bbx;

    do
    {
      model_a = this->world->ModelByName(model_name_a);
      boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
    } while (!model_a);
    this->console.debug("\tGot model a");
    
    do
    {
      model_b = this->world->ModelByName(model_name_b);
      boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
    } while (!model_b);
    this->console.debug("\tGot model b");

    // boost::this_thread::sleep_for(boost::chrono::milliseconds(1000));

    std::mutex mtx;
    mtx.lock();
    this->console.debug("Getting models bounding boxes:");
    model_a_bbx = model_a->CollisionBoundingBox();
    this->console.debug("\tGot model a bounding box", RESET);
    model_b_bbx = model_b->CollisionBoundingBox();
    this->console.debug("\tGot model b bounding box", RESET);
    mtx.unlock();

    return model_a_bbx.Intersects(model_b_bbx);
  }

  void DatasetGenerator::removeModelsInCollision(std::vector<std::string> model_to_remove, std::string model_fixed){
    this->console.debug("REMOVING MODELS IN COLLISION...");
    for (const std::string &model_name : model_to_remove)
    {
      if (this->checkCollisions(model_name, model_fixed))
      {
        this->console.debug("Collision detected with: " + model_name, RED);
        this->world->RemoveModel(model_name);
        while (this->world->ModelByName(model_name))
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        this->console.debug("Model removed: " + model_name);
      }
    }

  }

  //---- POINTCLOUD -----------------------------------------------------//
  void DatasetGenerator::SavePointCloud() {
    this->console.debug("SAVING POINTCLOUD...");


    pcl::PCDWriter writer;
    std::stringstream ss;
    ss.str("");
    ss << this->pcd_dir.string() << "/" << std::setfill('0') << std::setw(5) << this->env_count << ".pcd";

    this->console.info("Saving point cloud in: " + ss.str(), GREEN);

    int ms_delay = this->config["generator"]["iteration_delay"].as<int>();

    this->world->SetPaused(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(ms_delay));
    
    this->console.debug("Waitting for generated data ready");
    while (this->cloud_L->empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
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
      this->console.info("Point cloud saved", GREEN);
    }
    else
    {
      this->console.info("Cloud is empty", RED);
    }

    this->world->SetPaused(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

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
    std::string topic_name = this->config["sensor"]["topic"].as<std::string>();

    this->console.debug("Subscribing to topic: " + topic_name);
    ros::SubscribeOptions ros_so =
        ros::SubscribeOptions::create<sensor_msgs::PointCloud2>(
            topic_name, 1, boost::bind(&DatasetGenerator::PointCloudCallback, this, _1),
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

    pcl::copyPointCloud(*this->cloud_I, *this->cloud_L);

    for (size_t i = 0; i < this->cloud_I->points.size(); i++)
      this->cloud_L->points[i].label = this->cloud_I->points[i].intensity; 

  }


}