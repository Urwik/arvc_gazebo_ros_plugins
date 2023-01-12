#include <filesystem>
#include <iostream>
#include <algorithm>
#include <math.h>

//MULTITHREADING
// #include <thread>
// #include <mutex>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rosgraph_msgs/Log.h>


#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/gui/gui.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include <Eigen/Eigen>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

#include <sdf/sdf.hh>

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
class MoveModel : public WorldPlugin
{
  public: MoveModel(){
    ROS_INFO(RED "CONSTRUCTOR" RESET);
    this->pcl_cloud.reset(new PointCloud);
    this->handle_to_cam = false;
    this->handle_to_model = false;
  }

  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
  {
    this->world = _parent;

    this->world->SetPhysicsEnabled(false);

    this->ParseArgs(_sdf);

    this->CheckOutputDirs();

    this->InsertCamera();

    this->SetupROS();
    
    boost::thread arvc_thread(boost::bind(&MoveModel::GenerateDataset, this));

    ROS_INFO(GREEN "ARVC GAZEBO MoveModel PLUGIN LOADED" RESET);
  }

  /**
   * @brief Se ejecuta una única vez inmediatamente tras la función Load()
   */
  public: void Init()
  { 
    this->updateConnection =  event::Events::ConnectWorldUpdateBegin(
                              std::bind(&MoveModel::OnUpdate, this));
  }

  /**
   * @brief Hilo que se ejecuta cada vez que se avanza un paso en la simulación
   */
  private: void OnUpdate()
  { 
    if(!this->handle_to_model){
      if(!this->target_model)
            this->target_model = this->world->ModelByName("os_128");
      else{
        ROS_INFO(BLUE "HANDLE TO MODEL OBTAINED CORRECTLY" RESET);
        this->handle_to_model = true;
      }
    }

    if(!this->handle_to_cam)
    {
      if(this->GetCamera())
      {
        ROS_INFO(BLUE "HANDLE TO CAM OBTAINED CORRECTLY" RESET);
        this->handle_to_cam = true;
      }
    }
  }

  private: void GenerateDataset()
  {
    int estado = 0;
    int env_count_ = 0;

    while (this->env_count < this->num_env)
    {
      switch (estado)
      {
      case 0:
        if(this->CheckOusterReady()){
          ROS_INFO(GREEN "STARTING TO MOVE THE MODEL..." RESET);
          estado = 1;
        }
        break;

      case 1:
        ROS_INFO( YELLOW "ENVIROMENT %d" RESET, this->env_count);
        this->MoveTargetModel();
        estado = 2;
        break;
      
      case 2:

        this->TakeScreenShot();
        this->SavePointCloud(this->pcl_cloud);
        estado = 3;
        break;
        
      case 3:

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
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

  public: void ParseArgs(sdf::ElementPtr sdf)
  {
    std::cout << BLUE << "PARSING ARGUMENTS... " << RESET << std::endl;

    // PARSE ARGUMENTS
    if (!sdf->HasElement("out_dir")) {
      this->output_dir = "/media/arvc/data/datasets/ARVC_GZF";
    } else {
      this->output_dir = sdf->GetElement("out_dir")->Get<std::string>();
      std::cout << BLUE << "OUTPUT DIR: " << RESET << '\n' << this->output_dir.c_str() << std::endl;
    }

    if (!sdf->HasElement("num_env")) {
      this->num_env = 100;
    } else {
      this->num_env = sdf->GetElement("num_env")->Get<int>();
    }

    if (!sdf->HasElement("positive_dist")) {
      this->pos_dist = ignition::math::Vector3d(10.0, 10.0, 5.0);
    } else {
      this->pos_dist = sdf->GetElement("positive_dist")->Get<ignition::math::Vector3d>();
    }

    if (!sdf->HasElement("negative_dist")) {
      this->neg_dist = ignition::math::Vector3d(-10.0, -10.0, 0.0);
    } else {
      this->neg_dist = sdf->GetElement("negative_dist")->Get<ignition::math::Vector3d>();
    }

    if (!sdf->HasElement("pc_binary")) {
      this->pc_binary = true;
    } else {
      this->pc_binary = sdf->GetElement("pc_binary")->Get<bool>();
    }

    if (!sdf->HasElement("rand_mode")) {
      this->randMode = "uniform";
    } else {
      this->randMode = sdf->GetElement("rand_mode")->Get<std::string>();
    }

    if (!sdf->HasElement("arvc_debug")) {
      this->arvc_debug = false;
    } else {
      this->arvc_debug = sdf->GetElement("arvc_debug")->Get<bool>();
    }
  }

  private: bool GetCamera()
  {
    this->camera_sensor = sensors::get_sensor("arvc_cam");

    if(!this->camera_sensor)
      return false;
    else 
    {
      this->arvc_cam = std::dynamic_pointer_cast<sensors::CameraSensor>(this->camera_sensor);
      return true;
    }
  }

  private: void InsertCamera()
  {
    sdf::SDFPtr camera_sdf = this->GetSDFfile("/home/arvc/workSpaces/arvc_ws/src/arvc_dataset_generator/models/camera_test/model.sdf");
    this->world->InsertModelSDF(*camera_sdf);
  }

  private: void TakeScreenShot()
  {
    // ROS_INFO_COND(this->arvc_debug, "TAKING SCREENSHOT");
    std::stringstream ss;
    ss.str("");
    ss << this->images_dir.string() << "/" << std::setfill('0') << std::setw(5)  << this->env_count << ".jpg";
    bool img_saved = this->arvc_cam->SaveFrame(ss.str().c_str());
    ROS_INFO_COND(this->arvc_debug, "IMAGE SAVED IN: %s", ss.str().c_str());

  }

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

  private: bool CheckOusterReady()
  {
    ROS_INFO_COND(this->arvc_debug, "CHECKING IF OUSTER IS READY");
    if(!this->world->ModelByName("os_128"))
      return false;
    else
    {
      ROS_INFO(GREEN "OUSTER IS READY" RESET);
      return true;
    }
  }

  private: void MoveTargetModel()
  {
    ROS_INFO_COND(this->arvc_debug, YELLOW "MOVING MODEL..." RESET);

    ignition::math::Pose3d pose = this->ComputeRandomPose();
    this->world->SetPaused(true);
    this->target_model->SetWorldPose(pose);
    this->world->SetPaused(false);
  }


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
          "/os1/pointCloud", 1,
          boost::bind(&MoveModel::PointCloudCallback, this, _1),
          ros::VoidPtr(), &this->ros_cbqueue);
    
    this->ros_sub = this->ros_node->subscribe(ros_so);
    this->callback_queue_thread = boost::thread(boost::bind(&MoveModel::QueueThread, this));
  }

  private: void SavePointCloud(PointCloud::Ptr cloud)
  {
    // ROS_INFO_COND(this->arvc_debug, "SAVING POINTCLOUD...");
    pcl::PCDWriter writer;
    std::stringstream ss;
    ss.str("");
    ss << this->pcd_dir.string() << "/" << std::setfill('0') << std::setw(5)  << this->env_count << ".pcd";
    ROS_INFO_COND(this->arvc_debug, "SAVING POINTCLOUD IN %s", ss.str().c_str());

    if(cloud->points.size() != cloud->width)
    {
      int cloud_size = cloud->points.size();
      cloud->width = cloud_size;
      cloud->height = 1;
    }
    
    if(!cloud->empty())
      writer.write<PointT>(ss.str(), *cloud, this->pc_binary);
  }

  private: void CheckOutputDirs()
  {
    this->pcd_dir = this->output_dir / "pcd";
    this->images_dir = this->output_dir / "images";

    if(!fs::exists(this->pcd_dir))
      fs::create_directory(this->pcd_dir);

    if(!fs::exists(this->images_dir))
      fs::create_directory(this->images_dir);

    ROS_INFO_COND(this->arvc_debug, BLUE "PointClouds Output Directory:" RESET);
    std::cout << this->pcd_dir << std::endl;

    ROS_INFO_COND(this->arvc_debug, BLUE "Images Output Directory:" RESET);
    std::cout << this->images_dir << std::endl;
    
  }

  private: void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
  {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);

    PointCloud::Ptr temp_cloud (new PointCloud);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

    *pcl_cloud = *temp_cloud;
  }

  /**
   * @brief Set random pose to a model.
   * @param modelElement sdf::ElementPtr to the model element.
   */
  private: void SetModelPose(sdf::ElementPtr modelElement)
  {
    sdf::ElementPtr poseElement = modelElement->GetElement("pose");
    ignition::math::Pose3d pose = this->ComputeRandomPose();
    std::string pose_str = this->Pose2string(pose);
    
    poseElement->Set<ignition::math::Pose3d>(pose);
  }

  /**
   * @brief Set random pose to a model.
   * @param modelElement sdf::ElementPtr to the model element.
   */
  private: void SetModelPosition(sdf::ElementPtr modelElement)
  {
    using namespace ignition::math;
    sdf::ElementPtr poseElement = modelElement->GetElement("pose");
    Pose3d orig_pose = poseElement->Get<Pose3d>();
    Pose3d new_pose_;
    Vector3d position = this->ComputeEnvRandPosition();
    new_pose_.Set(position, Vector3d(0.0,0.0,0.0));
    
    poseElement->Set<Pose3d>(new_pose_);
  }

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

    rotation.X() = Rand::DblUniform(0, 2*M_PI);
    rotation.Y() = Rand::DblUniform(0, 2*M_PI);
    rotation.Z() = Rand::DblUniform(0, 2*M_PI);

    if (this->randMode == "uniform")
    {
      position.X() = Rand::DblUniform(this->neg_dist.X(), this->pos_dist.X()); 
      position.Y() = Rand::DblUniform(this->neg_dist.Y(), this->pos_dist.Y()); 
      position.Z() = Rand::DblUniform(this->neg_dist.Z(), this->pos_dist.Z()); 
    }
    else if (this->randMode == "normal")
    {
      position.X() = Rand::DblNormal(0,this->pos_dist.X()); 
      position.Y() = Rand::DblNormal(0,this->pos_dist.Y()); 
      position.Z() = Rand::DblNormal(0,this->pos_dist.Z()); 
    }
    else
      ROS_ERROR("WRONG randMode, POSSIBLE OPTIONS ARE: uniform, normal");

    // position = this->ApplyOffset(position);
    pose.Set(position, rotation);

    return pose;
  }

  /**
   * @brief Compute random pose X Y Z R P Y
   * @return Return the random pose
   */
  private: ignition::math::Pose3d ComputeWorldRandomPose()
  {
    using namespace ignition::math;
    Pose3d pose;
    Vector3d position;
    Vector3d rotation;

    rotation.X() = 0;
    rotation.Y() = 0;
    rotation.Z() = Rand::DblUniform(0, 2*M_PI);

    if (this->randMode == "uniform")
    {
      position.X() = Rand::DblUniform(-2, 2); 
      position.Y() = Rand::DblUniform(-2, 2); 
      position.Z() = Rand::DblUniform(-0.5, 0.5); 
    }
    else if (this->randMode == "normal")
    {
      position.X() = Rand::DblNormal(0, 1); 
      position.Y() = Rand::DblNormal(0, 1); 
      position.Z() = Rand::DblNormal(0, 0.5); 
    }
    else
      ROS_ERROR("WRONG randMode, POSSIBLE OPTIONS ARE: uniform, normal");

    pose.Set(position, rotation);

    return pose;
  }

  /**
   * @brief Compute random pose X Y Z
   */
  private: ignition::math::Vector3d ComputeEnvRandPosition()
  {
    using namespace ignition::math;
    Vector3d position;
    Vector3d offset_(10, 10, 0);

    if (this->randMode == "uniform")
    {
      position.X() = Rand::DblUniform(-30, 30);
      position.Y() = Rand::DblUniform(-30, 30);
      position.Z() = Rand::DblUniform(0, 0.5);
    }
    else if (this->randMode == "normal")
    {
      position.X() = Rand::DblNormal(0, 15); 
      position.Y() = Rand::DblNormal(0, 15);
      position.Z() = Rand::DblNormal(0, 0.25);
    }

    return this->ApplyOffset(position, offset_);
  }


  /**
   * @brief Apply offset to the passed coordinate.
   * 
   */
  private: ignition::math::Vector3d ApplyOffset(ignition::math::Vector3d input)
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

  private: ignition::math::Vector3d ApplyOffset(ignition::math::Vector3d input, ignition::math::Vector3d offset_)
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

  /**
   * @brief Transforms ignition::math::Pose3d to a string
   * @param pose ignition::math::Pose3d Pose (X,Y,Z,R,P,Y) in double
   * @return
   */
  private: std::string Pose2string(ignition::math::Pose3d pose)
  {
    std::stringstream ss;
    ss.str("");
    ss << pose.X()    << " " << pose.Y()     << " " << pose.Z() << " "
       << pose.Roll() << " " << pose.Pitch() << " " << pose.Yaw();

    return ss.str();
  }


  private: void QueueThread()
  {
    static const double timeout = 0.01;
    while (this->ros_node->ok())
    {
      this->ros_cbqueue.callAvailable(ros::WallDuration(timeout));
    }
  }


  public:
    physics::WorldPtr world;
    physics::ModelPtr target_model;
    transport::NodePtr gz_node;
    transport::PublisherPtr gz_pub;
    transport::SubscriberPtr gz_sub;
    ignition::math::Vector3d pos_offset,
                             neg_offset,
                             pos_dist,
                             neg_dist,
                             min_scale,
                             max_scale;
    std::string randMode;
    int laser_retro = 1;
    int env_count = 0;
    int numModels;
    int num_env;
    sensors::SensorPtr camera_sensor;
    sensors::CameraSensorPtr arvc_cam;
    rendering::CameraPtr cam;


    //ROS
    ros::NodeHandle* ros_node;
    ros::Subscriber ros_sub;
    ros::SubscribeOptions ros_so;
    ros::CallbackQueue ros_cbqueue;
    boost::thread callback_queue_thread;

    //THREADS
    // boost::thread arvc_thread;
    boost::mutex arvc_mutex;

    //PCL
    fs::path env_dir;
    fs::path models_dir;
    fs::path output_dir;
    fs::path pcd_dir;
    fs::path images_dir;


    PointCloud::Ptr pcl_cloud;

    
  private: 
    event::ConnectionPtr updateConnection, deleteConnection, postRenderConnection;
    event::ConnectionPtr cameraUpdateConnection;
    bool ousterReady = false;
    bool handle_to_cam;
    bool handle_to_model;
    bool take_screenshot = false;
    std::string world_name;
    bool arvc_debug = false;
    bool pc_binary = true;


};
// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(MoveModel)
}