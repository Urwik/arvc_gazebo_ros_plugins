#include <functional>
// #include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include <ros/ros.h>
#include <ros/advertise_options.h>
#include <tf/tf.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <geometry_msgs/TransformStamped.h>
#include <QPushButton>

#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"  
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"

namespace gazebo
{
  class TfPublisher : public ModelPlugin
  {

    /// \brief Constructor
    public: TfPublisher();

    /// \brief Destructor
    public: ~TfPublisher();

    /// \brief Load the plugin
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);

    /// \brief Initialize the plugin
    public: void Init(); 
    
    /// \brief Initialize a ROS node
    public: void RosSetup();

    public: void buttonClicked();

    /// \brief Publish tf between model and gazebo world frame
    private: 
      void PubThread();
      

    // Pointer to the model
    public:
      std::string parent_name, child_name;
      physics::ModelPtr parent_model, child_model;
      ignition::math::Pose3d parent_pose, child_pose;


    private:  
      tf2::Transform tf2_transform;

      std::unique_ptr<ros::NodeHandle> nh;
      tf2_ros::TransformBroadcaster tf_broadcaster;
      std::thread ros_publication_thread;
  };
  
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(TfPublisher)
}