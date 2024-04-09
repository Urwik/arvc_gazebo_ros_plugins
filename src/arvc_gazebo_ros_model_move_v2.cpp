#include "arvc_gazebo_ros_plugins/arvc_gazebo_ros_model_move_v2.hpp"



namespace gazebo {


MoveModel::MoveModel() {
  ROS_INFO(RED "CONSTRUCTOR" RESET);
  this->pcl_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
  this->handle_to_model = false;
  this->env_count = 0;
}



void MoveModel::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
  this->world = _parent;
  this->world->SetPhysicsEnabled(false);

  this->ParseArgs(_sdf);

  this->SetupROS();

  this->CheckOutputDirs();
  
  // this->updateConnection =  event::Events::ConnectWorldUpdateBegin(
  //                           std::bind(&MoveModel::OnUpdate, this));

  this->generator_thread = boost::thread(boost::bind(&MoveModel::GenerateDataset, this));

  ROS_INFO(GREEN "ARVC GAZEBO MoveModel PLUGIN LOADED" RESET);
}



/**
 * @brief Se ejecuta una única vez inmediatamente tras la función Load()
 */
void MoveModel::Init()
{ 
  // gazebo::common::Console::SetQuiet(true);
  this->fixed_model = this->world->ModelByName(this->fixed_model_name);

  // Get all links of the structure
  physics::Link_V links = this->fixed_model->GetLinks();
  for(physics::LinkPtr link : links)
    this->links_bbx.push_back(link->CollisionBoundingBox());
}



/**
 * @brief Hilo que se ejecuta cada vez que se avanza un paso en la simulación
 */
// void OnUpdate(){ 
// }



void MoveModel::GenerateDataset()
{

  while (!this->fixed_model){
    this->world->ModelByName(this->fixed_model_name);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  int estado = 0;

  while (this->env_count < this->NUM_ENV)
  {
    switch (estado)
    {
    case 0:
      if(this->MobileModelReady()){
        ROS_INFO(GREEN "STARTING TO MOVE THE MODEL..." RESET);
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        estado = 1;
      }
      break;

    case 1:
      ROS_INFO( YELLOW "ENVIROMENT %d" RESET, this->env_count);
      this->MoveMobileModel();

      if(this->paused)
      {
        ROS_INFO(YELLOW "PAUSED: Press enter to continue ..." RESET);
        std::getchar();
      }

      estado = 2;
      break;
    
    case 2:
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      this->SavePointCloud(this->pcl_cloud);
      estado = 3;
      break;
      
    case 3:

      this->env_count++;
      estado = 1;
      break;

    default:
      break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  ROS_INFO(GREEN "ENVS CREATED CORRECTLY" RESET);

}



void MoveModel::ParseArgs(sdf::ElementPtr sdf)
{
  std::cout << BLUE << "PARSING ARGUMENTS... " << RESET << std::endl;

  if (sdf->HasElement("yaml_config")) {
    std::string yaml_config = sdf->GetElement("yaml_config")->Get<std::string>();
    this->config = YAML::LoadFile(yaml_config);

    this->output_dir  = this->config["plugin"]["out_dir"].as<std::string>();
    this->NUM_ENV     = this->config["plugin"]["num_env"].as<int>();
    this->pos_dist    = this->config["plugin"]["positive_dist"].as<ignition::math::Vector3d>();
    this->neg_dist    = this->config["plugin"]["negative_dist"].as<ignition::math::Vector3d>();
    this->pc_binary   = this->config["plugin"]["pc_binary"].as<bool>();
    this->RANDMODE    = this->config["plugin"]["rand_mode"].as<std::string>();
    this->debug_msgs  = this->config["plugin"]["debug_msgs"].as<bool>();
    this->paused      = this->config["plugin"]["paused"].as<bool>();
    this->mobile_model_name = this->config["plugin"]["mobile_model_name"].as<string>();
    this->fixed_model_name  = this->config["plugin"]["fixed_model_name"].as<string>();
    this->sensor_topic      = this->config["plugin"]["sensor_topic"].as<string>();
  }
  else
  {


  // PARSE ARGUMENTS
  if (sdf->HasElement("out_dir")) 
    this->output_dir = sdf->GetElement("out_dir")->Get<std::string>();
  else
    std::cout << RED << "ERROR: out_dir not found" << RESET << std::endl;
  

  if (sdf->HasElement("NUM_ENV"))
    this->NUM_ENV = sdf->GetElement("NUM_ENV")->Get<int>();
  else
    std::cout << RED << "ERROR: NUM_ENV not found" << RESET << std::endl; 
  

  if (sdf->HasElement("positive_dist"))
    this->pos_dist = sdf->GetElement("positive_dist")->Get<ignition::math::Vector3d>();
  else
    std::cout << RED << "ERROR: positive_dist not found" << RESET << std::endl;

  if (sdf->HasElement("negative_dist"))
    this->neg_dist = sdf->GetElement("negative_dist")->Get<ignition::math::Vector3d>();
  else
    std::cout << RED << "ERROR: negative_dist not found" << RESET << std::endl;

  if (sdf->HasElement("pc_binary"))
    this->pc_binary = sdf->GetElement("pc_binary")->Get<bool>();
  else
    std::cout << RED << "ERROR: pc_binary not found" << RESET << std::endl;


  if (sdf->HasElement("rand_mode"))
    this->RANDMODE = sdf->GetElement("rand_mode")->Get<std::string>();
  else
    std::cout << RED << "ERROR: rand_mode not found" << RESET << std::endl;


  if (sdf->HasElement("debug_msgs"))
    this->debug_msgs = sdf->GetElement("debug_msgs")->Get<bool>();
  else
    std::cout << RED << "ERROR: debug_msgs not found" << RESET << std::endl;

  // PAUSES THE PROGRAM UNTIL USER PRESS ENTER
  if (sdf->HasElement("paused"))
    this->paused = sdf->GetElement("paused")->Get<bool>();
  else
    std::cout << RED << "ERROR: paused not found" << RESET << std::endl;

  // Gets the model name
  if (sdf->HasElement("mobile_model_name"))
    this->mobile_model_name = sdf->GetElement("mobile_model_name")->Get<string>();
  else
    std::cout << RED << "ERROR: mobile_model_name not found" << RESET << std::endl;

  // Gets the model name
  if (sdf->HasElement("fixed_model_name"))
    this->fixed_model_name = sdf->GetElement("fixed_model_name")->Get<string>();
  else
    std::cout << RED << "ERROR: fixed_model_name not found" << RESET << std::endl;

  // Gets the model name
  if (sdf->HasElement("sensor_topic"))
    this->sensor_topic = sdf->GetElement("sensor_topic")->Get<string>();
  else
    std::cout << RED << "ERROR: sensor_topic not found" << RESET << std::endl;

  }
}


bool MoveModel::GetModelPointer()
{
  ROS_INFO_COND(this->debug_msgs, YELLOW "TRYING TO GET MOBILE MODEL: %s" RESET, this->mobile_model_name.c_str());
  this->mobile_model = this->world->ModelByName(this->mobile_model_name);

  if(!this->mobile_model)
    return false;
  else
  {
    ROS_INFO(BLUE "HANDLE TO MODEL OBTAINED CORRECTLY" RESET);
    this->handle_to_model = true;
    return true;
  }
}





/**
 * @brief Get a pointer to an SDF file.
 * @param sdfPath Absolute path to the model file.
 * @return Return an sdf::SDFPtr to the file.
 */
sdf::SDFPtr MoveModel::GetSDFfile(fs::path sdfPath)
{

  sdf::SDFPtr sdf_File (new sdf::SDF());
  sdf::init(sdf_File);
  sdf::readFile(sdfPath, sdf_File);

  return sdf_File;
}



bool MoveModel::MobileModelReady()
{
  ROS_INFO_COND(this->debug_msgs, "CHECKING IF MOBILE MODEL IS READY");
  if(!this->world->ModelByName(this->mobile_model_name))
    return false;
  else
  {
    ROS_INFO(GREEN "MOBILE MODEL IS READY" RESET);
    return true;
  }
}



void MoveModel::MoveMobileModel()
{
  ROS_INFO_COND(this->debug_msgs, YELLOW "MOVING MODEL..." RESET);

  ignition::math::Pose3d pose = utils::ComputeRandomPose(this->RANDMODE);
  this->world->SetPaused(true);
  this->mobile_model->SetWorldPose(pose);
  this->world->SetPaused(false);
  ROS_INFO_COND(this->debug_msgs, YELLOW "MODEL MOVED" RESET);

}



void MoveModel::SetupROS()
{
  //ROS
  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized()) {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->ros_node = new ros::NodeHandle("arvc_gazebo_ros_save_cloud");

  ros::SubscribeOptions ros_so =
    ros::SubscribeOptions::create<sensor_msgs::PointCloud2>(
        this->sensor_topic, 1,
        boost::bind(&MoveModel::PointCloudCallback, this, _1),
        ros::VoidPtr(), &this->ros_cbqueue);
  
  this->ros_sub = this->ros_node->subscribe(ros_so);
  this->callback_queue_thread = boost::thread(boost::bind(&MoveModel::QueueThread, this));
}



void MoveModel::SavePointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  // ROS_INFO_COND(this->debug_msgs, "SAVING POINTCLOUD...");
  pcl::PCDWriter writer;
  std::stringstream ss;
  ss.str("");
  ss << this->pcd_dir.string() << "/" << std::setfill('0') << std::setw(5)  << this->env_count << ".pcd";
  ROS_INFO_COND(this->debug_msgs, "SAVING POINTCLOUD IN %s", ss.str().c_str());

  if(cloud->points.size() != cloud->width)
  {
    int cloud_size = cloud->points.size();
    cloud->width = cloud_size;
    cloud->height = 1;
  }
  
  if(!cloud->empty())
    writer.write<pcl::PointXYZI>(ss.str(), *cloud, this->pc_binary);
}



void MoveModel::CheckOutputDirs()
{
  this->pcd_dir = this->output_dir / "pcd";

  if(!fs::exists(this->output_dir))
    fs::create_directory(this->output_dir);

  if(!fs::exists(this->pcd_dir))
    fs::create_directory(this->pcd_dir);


  ROS_INFO_COND(this->debug_msgs, BLUE "PointClouds Output Directory:" RESET);
  std::cout << this->pcd_dir << std::endl;

}



void MoveModel::PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input, pcl_pc2);

  pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

  *pcl_cloud = *temp_cloud;
}


/**
 * @brief Check that pose dont lies inside truss structure
 * @return Return true if pose is valid
 */
bool MoveModel::ValidPose(ignition::math::Pose3d pose)
{
  using namespace ignition::math;

  Vector3d position = pose.Pos();

  for (AxisAlignedBox bbx : this->links_bbx)
  {
    if(bbx.Contains(position))
    {
      ROS_INFO_COND(this->debug_msgs, RED "INVALID POSE COMPUTED, RECOMPUTING..." RESET);
      return false;
    }
  }
  
  return true;
}




void MoveModel::QueueThread()
{
  static const double timeout = 0.01;
  while (this->ros_node->ok())
  {
    this->ros_cbqueue.callAvailable(ros::WallDuration(timeout));
  }
}


};

// Register this plugin with the simulator
// GZ_REGISTER_WORLD_PLUGIN(MoveModel)
}
