#include "arvc_gazebo_ros_plugins/arvc_gazebo_ros_model_move_v2.hpp"



namespace gazebo {

GZ_REGISTER_WORLD_PLUGIN(MoveModel)


MoveModel::MoveModel() {
  ROS_INFO(RED "CONSTRUCTOR" RESET);
  this->pcl_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
  this->callback_count = 0;
}


void MoveModel::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{

  std::cout << RED << "LOADING PLUGIN..." << RESET << std::endl;

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


void MoveModel::GenerateDataset()
{

  while (!this->fixed_model){
    this->fixed_model = this->world->ModelByName(this->fixed_model_name);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  ROS_INFO_COND(this->debug_msgs, YELLOW "FIXED MODEL FOUND");

  while (!this->sensor_model){
    this->sensor_model = this->world->ModelByName(this->sensor_name);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  ROS_INFO_COND(this->debug_msgs, YELLOW "SENSOR MODEL FOUND");

  ROS_INFO_COND(this->debug_msgs, YELLOW "WAITING FOR POINTCLOUD...");
  while (this->callback_count < 2) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  ROS_INFO_COND(this->debug_msgs, GREEN "POINTCLOUD RECEIVED" RESET);

  physics::Link_V links = this->fixed_model->GetLinks();
  for(physics::LinkPtr link : links)
    this->links_bbx.push_back(link->CollisionBoundingBox());

  int estado = 0;
  int current_num = 0;
  this->env_count = 0;

  while (this->env_count < this->NUM_ENV)
  {
    switch (estado)
    {
    case 0:
        ROS_INFO(GREEN "STARTING TO MOVE THE MODEL..." RESET);
        current_num = utils::ResumeEnvCount(this->pcd_dir);
        if(current_num > 0)
          this->env_count = current_num + 1;
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        estado = 1;
      break;

    case 1:
      ROS_INFO( YELLOW "ENVIROMENT %d" RESET, this->env_count);
      this->MoveMobileModel();

      if(this->paused) {
        ROS_INFO(YELLOW "PAUSED: Press enter to continue ..." RESET);
        std::getchar();
      }

      estado = 2;
      break;
    
    case 2:
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));

      if (this->save_pcd)
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

    this->output_dir        = this->config["common"]["out_dir"].as<std::string>();
    this->NUM_ENV           = this->config["common"]["num_env"].as<int>();
    this->save_pcd          = this->config["common"]["save_pcd"].as<bool>();
    this->pcd_binary        = this->config["common"]["pcd_binary"].as<bool>();
    this->paused            = this->config["common"]["paused"].as<bool>();
    this->debug_msgs        = this->config["common"]["debug_msgs"].as<bool>();

    this->fixed_model_name  = this->config["collision"]["model"].as<std::string>();

    this->sensor_name       = this->config["sensor"]["name"].as<std::string>();
    this->sensor_topic      = this->config["sensor"]["topic"].as<std::string>();
    this->sensor_offset     = this->config["sensor"]["collision_offset"].as<uint>();

    this->truss_offset      = this->config["poses"]["offset"].as<float>();
    this->RANDMODE          = this->config["poses"]["rand_mode"].as<std::string>();
  }
  else {
    std::cout << RED << "Param yaml_config inside plugin declaration" << RESET << std::endl;
  }
}



void MoveModel::MoveMobileModel()
{
  ROS_INFO_COND(this->debug_msgs, YELLOW "MOVING MODEL..." RESET);

  ignition::math::AxisAlignedBox truss_bbx = this->fixed_model->BoundingBox();
  ignition::math::Vector3d truss_min = truss_bbx.Min();
  ignition::math::Vector3d truss_max = truss_bbx.Max();
  float offset = 2.0;

  ignition::math::Pose3d pose;
  pose = utils::ComputeRandomPose(this->RANDMODE, truss_min, truss_max, offset);
  
  this->world->SetPaused(true);
  this->sensor_model->SetWorldPose(pose);
  this->world->SetPaused(false);

  ignition::math::AxisAlignedBox bbx = this->sensor_model->BoundingBox();



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



  pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_labeled (new pcl::PointCloud<pcl::PointXYZL>);
  pcl::copyPointCloud(*cloud, *cloud_labeled);

  for (int i = 0; i < cloud_labeled->points.size(); i++)
    cloud_labeled->points[i].label = cloud->points[i].intensity;

  if (!cloud_labeled->empty())
    writer.write<pcl::PointXYZL>(ss.str(), *cloud_labeled, this->pcd_binary);


  // OLD WAY
/*   if(cloud->points.size() != cloud->width)
  {
    int cloud_size = cloud->points.size();
    cloud->width = cloud_size;
    cloud->height = 1;
  }
  
  if(!cloud->empty())
    writer.write<pcl::PointXYZI>(ss.str(), *cloud, this->pcd_binary);
 */

}



void MoveModel::CheckOutputDirs()
{
  this->pcd_dir = this->output_dir / "pcd";

  if(!fs::exists(this->output_dir))
    fs::create_directories(this->output_dir);

  if(!fs::exists(this->pcd_dir))
    fs::create_directories(this->pcd_dir);


  ROS_INFO_COND(this->debug_msgs, BLUE "PointClouds Output Directory:" RESET);
  std::cout << this->pcd_dir << std::endl;

}



void MoveModel::PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input, pcl_pc2);

  pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

  *this->pcl_cloud = *temp_cloud;
  this->callback_count++;
}


/**
 * @brief Check that pose dont lies inside truss structure
 * @return Return true if pose is valid
 */
bool MoveModel::ValidPose(ignition::math::AxisAlignedBox _sensor_bbx)
{
  using namespace ignition::math;

  for (AxisAlignedBox bbx : this->links_bbx) {

    if(bbx.Intersects(_sensor_bbx)) {
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


}; // namespace gazebo
