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
    this->models = this->world->Models();

    // std::thread chech_collision_thread(std::bind(&CollisionDetector::checkCollisions, this));
  }

  public: void Init()
  {
    this->checkCollisions();
  }

  //////////////////////////////////////////////////////////////////////////////
  private: void checkCollisions()
  {
    namespace im = ignition::math;

    std::vector<im::AxisAlignedBox> models_bbx;

    for (size_t i = 0; i < this->models.size(); i++)
      models_bbx.push_back(this->models[i]->CollisionBoundingBox());


    im::Vector3d inlier(0,0,0.5);
    im::Vector3d outlier(0,0,10);


    for (im::AxisAlignedBox bbx : models_bbx)
    {
      std::cout << "Contains Inlier: " << bbx.Contains(inlier) << std::endl;
      std::cout << "Contains Outlier: " << bbx.Contains(outlier) << std::endl;
    }
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