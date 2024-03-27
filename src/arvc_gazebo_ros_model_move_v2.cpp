#include "arvc_gazebo_ros_plugins/arvc_gazebo_ros_model_move_v2.hpp"


using namespace std;

#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"  
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"

namespace gazebo {

class MoveModel : public WorldPlugin {

////////////////////////////////////////////////////////////////////////////////
  public: MoveModel() {
    ROS_INFO(RED "CONSTRUCTOR" RESET);
    this->pcl_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
    this->handle_to_cam = false;
    this->handle_to_model = false;
    this->env_count = 0;
  }


////////////////////////////////////////////////////////////////////////////////
  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
  {
    this->world = _parent;
    this->world->SetPhysicsEnabled(false);


    this->ParseArgs(_sdf);

    this->InsertCamera();

    this->SetupROS();

    this->CheckOutputDirs();
    
    this->updateConnection =  event::Events::ConnectWorldUpdateBegin(
                              std::bind(&MoveModel::OnUpdate, this));

    this->generator_thread = boost::thread(boost::bind(&MoveModel::GenerateDataset, this));

    ROS_INFO(GREEN "ARVC GAZEBO MoveModel PLUGIN LOADED" RESET);
  }


////////////////////////////////////////////////////////////////////////////////
  /**
   * @brief Se ejecuta una única vez inmediatamente tras la función Load()
   */
  public: void Init()
  { 
    // gazebo::common::Console::SetQuiet(true);
    this->fixed_model = this->world->ModelByName(this->fixed_model_name);

    // Get all links of the structure
    physics::Link_V links = this->fixed_model->GetLinks();
    for(physics::LinkPtr link : links)
      this->links_bbx.push_back(link->CollisionBoundingBox());


  }


////////////////////////////////////////////////////////////////////////////////
  /**
   * @brief Hilo que se ejecuta cada vez que se avanza un paso en la simulación
   */
  private: void OnUpdate()
  { 
    if(!this->handle_to_model)
      this->GetModelPointer();

    if(!this->handle_to_cam)
      this->GetCameraPointer();

    if(!this->fixed_model)
      this->world->ModelByName(this->fixed_model_name);

  }


////////////////////////////////////////////////////////////////////////////////
  private: void GenerateDataset()
  {
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
        this->TakeScreenShot();
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


////////////////////////////////////////////////////////////////////////////////
  public: void ParseArgs(sdf::ElementPtr sdf)
  {
    std::cout << BLUE << "PARSING ARGUMENTS... " << RESET << std::endl;

    this->GetYamlConfig();

    // PARSE ARGUMENTS
    if (!sdf->HasElement("out_dir")) {
      this->output_dir = this->config["plugin"]["out_dir"].as<std::string>();
    } else {
      this->output_dir = sdf->GetElement("out_dir")->Get<std::string>();
    }
    std::cout << BLUE << "OUTPUT DIR: " << RESET << '\n' << this->output_dir.c_str() << std::endl;

    if (!sdf->HasElement("NUM_ENV")) {
      this->NUM_ENV = this->config["plugin"]["num_env"].as<int>();
    } else {
      this->NUM_ENV = sdf->GetElement("NUM_ENV")->Get<int>();
    }
    std::cout << BLUE << "NUM_ENV: " << RESET << '\n' << this->NUM_ENV << std::endl;

    if (!sdf->HasElement("positive_dist")) {
      this->pos_dist = this->config["plugin"]["positive_dist"].as<ignition::math::Vector3d>();
    } else {
      this->pos_dist = sdf->GetElement("positive_dist")->Get<ignition::math::Vector3d>();
    }

    if (!sdf->HasElement("negative_dist")) {
      this->neg_dist = this->config["plugin"]["negative_dist"].as<ignition::math::Vector3d>();
    } else {
      this->neg_dist = sdf->GetElement("negative_dist")->Get<ignition::math::Vector3d>();
    }

    if (!sdf->HasElement("pc_binary")) {
      this->pc_binary = this->config["plugin"]["pc_binary"].as<bool>();
    } else {
      this->pc_binary = sdf->GetElement("pc_binary")->Get<bool>();
    }
    std::cout << BLUE << "BINARY CLOUD: " << RESET << '\n' << this->pc_binary << std::endl;


    if (!sdf->HasElement("rand_mode")) {
      this->RANDMODE = this->config["plugin"]["rand_mode"].as<std::string>();
    } else {
      this->RANDMODE = sdf->GetElement("rand_mode")->Get<std::string>();
    }
    std::cout << BLUE << "RAND_MODE: " << RESET << '\n' << this->RANDMODE << std::endl;


    if (!sdf->HasElement("debug_msgs")) {
      this->debug_msgs = this->config["plugin"]["debug_msgs"].as<bool>();
    } else {
      this->debug_msgs = sdf->GetElement("debug_msgs")->Get<bool>();
    }
    std::cout << BLUE << "DEBUG: " << RESET << '\n' << this->debug_msgs << std::endl;

    // PAUSES THE PROGRAM UNTIL USER PRESS ENTER
    if (!sdf->HasElement("paused")) {
      this->paused = this->config["plugin"]["paused"].as<bool>();
    } else {
      this->paused = sdf->GetElement("paused")->Get<bool>();
    }
      std::cout << YELLOW << "PAUSED MODE: " << RESET << "\n " << this->paused << std::endl;

    // Gets the model name
    if (!sdf->HasElement("mobile_model_name")) {
      this->mobile_model_name = this->config["plugin"]["mobile_model_name"].as<string>();
    } else {
      this->mobile_model_name = sdf->GetElement("mobile_model_name")->Get<string>();
    }
      std::cout << YELLOW << "MOBILE MODEL NAME: " << RESET << "\n " << this->mobile_model_name << std::endl;

    // Gets the model name
    if (!sdf->HasElement("fixed_model_name")) {
      this->fixed_model_name = this->config["plugin"]["fixed_model_name"].as<string>();
    } else {
      this->fixed_model_name = sdf->GetElement("fixed_model_name")->Get<string>();
    }
      std::cout << YELLOW << "FIXED MODEL NAME: " << RESET << "\n " << this->fixed_model_name << std::endl;

    // Gets the model name
    if (!sdf->HasElement("sensor_topic")) {
      this->sensor_topic = this->config["plugin"]["sensor_topic"].as<string>();
    } else {
      this->sensor_topic = sdf->GetElement("sensor_topic")->Get<string>();
    }
      std::cout << YELLOW << "SENSOR TOPIC: " << RESET << "\n " << this->sensor_topic << std::endl;

    if (!sdf->HasElement("cam_name")) {
      this->cam_name = this->config["plugin"]["cam_name"].as<string>();
    } else {
      this->cam_name = sdf->GetElement("cam_name")->Get<string>();
    }
      std::cout << YELLOW << "CAMERA MODEL NAME: " << RESET << "\n " << this->cam_name << std::endl;
  }


  /////////////////////////////////
  private: void GetYamlConfig()
  {
    fs::path package_path(ros::package::getPath("arvc_dataset_generator"));
    fs::path config_path = package_path / "config/test_dg_config.yaml";

    std::cout << YELLOW << "YAML CONFIG PATH: " << RESET << "\n " << config_path.string().c_str() << std::endl;

    this->config = YAML::LoadFile(config_path.string());
  }

////////////////////////////////////////////////////////////////////////////////
  private: bool GetCameraPointer()
  {
    ROS_INFO_COND(this->debug_msgs, YELLOW "TRYING TO GET CAMERA: %s" RESET, this->cam_name.c_str());
    sensors::SensorPtr sensor = sensors::get_sensor(this->cam_name);

    if(!sensor)
      return false;
    else 
    {
      ROS_INFO(BLUE "HANDLE TO CAM OBTAINED CORRECTLY" RESET);
      this->camera = std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
      this->camera_model = this->world->ModelByName(this->cam_name);
      this->handle_to_cam = true;

      return true;
    }
  }


////////////////////////////////////////////////////////////////////////////////
  private: bool GetModelPointer()
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




////////////////////////////////////////////////////////////////////////////////
  /**
   * @brief Get a pointer to an SDF file.
   * @param sdfPath Absolute path to the model file.
   * @return Return an sdf::SDFPtr to the file.
   */
  private: sdf::SDFPtr GetSDFfile(fs::path sdfPath)
  {

    sdf::SDFPtr sdf_File (new sdf::SDF());
    sdf::init(sdf_File);
    sdf::readFile(sdfPath, sdf_File);

    return sdf_File;
  }


////////////////////////////////////////////////////////////////////////////////
  private: bool MobileModelReady()
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


////////////////////////////////////////////////////////////////////////////////
  private: void MoveMobileModel()
  {
    ROS_INFO_COND(this->debug_msgs, YELLOW "MOVING MODEL..." RESET);

    ignition::math::Pose3d pose = this->ComputeRandomPose();
    this->world->SetPaused(true);
    this->mobile_model->SetWorldPose(pose);
    this->world->SetPaused(false);
    ROS_INFO_COND(this->debug_msgs, YELLOW "MODEL MOVED" RESET);

  }


////////////////////////////////////////////////////////////////////////////////
  private: void SetupROS()
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


////////////////////////////////////////////////////////////////////////////////
  private: void SavePointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
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


////////////////////////////////////////////////////////////////////////////////
  private: void CheckOutputDirs()
  {
    this->pcd_dir = this->output_dir / "pcd";
    this->images_dir = this->output_dir / "images";

    if(!fs::exists(this->output_dir))
      fs::create_directory(this->output_dir);

    if(!fs::exists(this->pcd_dir))
      fs::create_directory(this->pcd_dir);


    ROS_INFO_COND(this->debug_msgs, BLUE "PointClouds Output Directory:" RESET);
    std::cout << this->pcd_dir << std::endl;

  }


////////////////////////////////////////////////////////////////////////////////
  private: void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
  {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);

    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

    *pcl_cloud = *temp_cloud;
  }


////////////////////////////////////////////////////////////////////////////////
  /**
   * @brief Compute random pose X Y Z R P Y
   * @return Return the random pose
   */
  private: ignition::math::Pose3d ComputeRandomPose()
  {
    using namespace ignition::math;
    Pose3d pose;
    Vector3d position;
    Vector3d rotation;

    do{
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
        ROS_ERROR("WRONG RANDMODE, POSSIBLE OPTIONS ARE: uniform, normal");

      pose.Set(position, rotation);
    }while (!this->ValidPose(pose));
    
    return pose;
  }


////////////////////////////////////////////////////////////////////////////////
  /**
   * @brief Check that pose dont lies inside truss structure
   * @return Return true if pose is valid
   */
  private: bool ValidPose(ignition::math::Pose3d pose)
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


////////////////////////////////////////////////////////////////////////////////
  /**
   * @brief
   * @param 
   * @return
  */
  private: bool collisionBbx(ignition::math::AxisAlignedBox box1, ignition::math::AxisAlignedBox box2)
  {
    
  }

////////////////////////////////////////////////////////////////////////////////
  private: void QueueThread()
  {
    static const double timeout = 0.01;
    while (this->ros_node->ok())
    {
      this->ros_cbqueue.callAvailable(ros::WallDuration(timeout));
    }
  }


  // VARIABLES
  
  // GAZEBO
  private: physics::WorldPtr world;
  private: physics::ModelPtr mobile_model;
  private: physics::ModelPtr fixed_model;
  private: event::ConnectionPtr updateConnection;

  // SENSORS    
  private: physics::ModelPtr sensor_model;
  private: physics::ModelPtr camera_model;
  private: sensors::CameraSensorPtr camera;
  private: ignition::math::Pose3d camera_pose;



  // CONFIGURATION
  private: YAML::Node config;
  private: std::string RANDMODE;
  private: int NUM_ENV;
  // private: int NUM_MODELS;
  private: std::filesystem::path env_dir;
  private: std::filesystem::path models_dir;
  private: std::filesystem::path output_dir;
  private: std::filesystem::path pcd_dir;
  private: std::filesystem::path images_dir;
  private: std::filesystem::path cam_path;
  private: std::string world_name;
  private: std::string mobile_model_name;
  private: std::string fixed_model_name;
  private: std::string cam_name;
  private: std::string sensor_name;
  private: std::string sensor_topic;
  private: ignition::math::Vector3d pos_offset;
  private: ignition::math::Vector3d neg_offset;
  private: ignition::math::Vector3d pos_dist;
  private: ignition::math::Vector3d neg_dist;
  private: ignition::math::Vector3d min_scale;
  private: ignition::math::Vector3d max_scale;
  private: bool pc_binary;


  // ROS
  private: ros::NodeHandle* ros_node;
  private: ros::Subscriber ros_sub;
  private: ros::SubscribeOptions ros_so;
  private: ros::CallbackQueue ros_cbqueue;
  private: boost::thread callback_queue_thread;


  // PCL
  private: pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud;


  // HELPERS
  private: bool debug_msgs;
  private: bool ousterReady;
  private: bool handle_to_cam;
  private: bool handle_to_model;
  private: bool take_screenshot;
  private: int env_count;
  private: int laser_retro;
  private: bool paused;
  private: boost::thread generator_thread;
  private: std::vector<ignition::math::AxisAlignedBox> links_bbx;

};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(MoveModel)
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