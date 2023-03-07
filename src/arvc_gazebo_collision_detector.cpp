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
class CollisionDetector : public WorldPlugin
{
  public: CollisionDetector(){

  }

  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
  {
    this->world = _parent;

    this->insertModels();

    boost::thread generator_thread(boost::bind(&CollisionDetector::checkCollisions, this));

    // std::thread chech_collision_thread(std::bind(&CollisionDetector::checkCollisions, this));
  }

  public: void Init()
  {
  }


  //////////////////////////////////////////////////////////////////////////////
  private: void insertModels()
  {
    using namespace ignition::math;


    fs::path model_path_1 = "/home/arvc/workSpaces/arvc_ws/src/arvc_dataset_generator/models/unit_box_1/model.sdf";
    sdf::SDFPtr sdf_ptr_1 = this->GetSdfPtr(model_path_1);
    sdf::ElementPtr model_element_ptr_1 = this->GetModelElementPtr(sdf_ptr_1);

    fs::path model_path_2 = "/home/arvc/workSpaces/arvc_ws/src/arvc_dataset_generator/models/unit_box_2/model.sdf";
    sdf::SDFPtr sdf_ptr_2 = this->GetSdfPtr(model_path_2);
    sdf::ElementPtr model_element_ptr_2 = this->GetModelElementPtr(sdf_ptr_2);

    Pose3d pose_1(0, 0, 0.5, 0, 0, 0);
    Pose3d pose_2(0, 0, 2, 0, 0, 0);

    this->SetModelPose(model_element_ptr_1, pose_1);
    this->world->InsertModelSDF(*sdf_ptr_1);

    this->SetModelPose(model_element_ptr_2, pose_2);
    this->world->InsertModelSDF(*sdf_ptr_2);
  }

  private: void checkCollisions()
  {
    using namespace ignition::math;

    while(true){


        physics::ModelPtr model_1 = this->world->ModelByName("unit_box_1");
        physics::ModelPtr model_2 = this->world->ModelByName("unit_box_2");

        if(model_1)
        {
          if(model_2)
          {
            AxisAlignedBox bbx_1 = model_1->CollisionBoundingBox();
            AxisAlignedBox bbx_2 = model_2->CollisionBoundingBox();

            if(bbx_1.Intersects(bbx_2))
              std::cout << "Objects are in collision" << std::endl;
            else
              std::cout << "Objects are not in collision" << std::endl;
          }
        }
      
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

  //////////////////////////////////////////////////////////////////////////////
  private: sdf::SDFPtr GetSdfPtr(fs::path path) 
  {
    sdf::SDFPtr sdf_file_ptr (new sdf::SDF());
    sdf::init(sdf_file_ptr);
    sdf::readFile(path, sdf_file_ptr);

    return sdf_file_ptr;
  }


  private: sdf::ElementPtr GetModelElementPtr(sdf::SDFPtr sdfPtr)
  {
    sdf::ElementPtr modelElement = sdfPtr->Root()->GetElement("model");

    return modelElement;
  }


  /////////////////////////////////
  private: void SetModelPose(sdf::ElementPtr modelElement, ignition::math::Pose3d pose)
  {
    using namespace ignition::math;

    sdf::ElementPtr poseElement = modelElement->GetElement("pose");
    poseElement->Set<Pose3d>(pose);
  }


  public:
    physics::WorldPtr world;
    physics::Model_V models;
  private: 
    event::ConnectionPtr updateConnection;

};
// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(CollisionDetector)
}