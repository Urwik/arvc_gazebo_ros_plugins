#include <functional>
// #include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include <ros/ros.h>
#include <ros/advertise_options.h>
#include <tf/tf.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/TransformStamped.h>


namespace gazebo
{
  class PubWorldTF : public ModelPlugin
  {

    /// \brief Constructor
    public: PubWorldTF();

    /// \brief Destructor
    public: ~PubWorldTF();

    /// \brief Load the plugin
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
    
    /// \brief Update world Connection
    public: void OnUpdate();
    
    /// \brief Publish tf between model and gazebo world frame
    private: void PubTF();
      

    // Pointer to the model
    private:  physics::ModelPtr model;
              ignition::math::Pose3d world_pose;
              ros::NodeHandle* nh_;
              ros::Publisher pub_;
              boost::shared_ptr<tf2_ros::TransformBroadcaster> br2;
              tf::Transform tf;
              event::ConnectionPtr updateConnection;
              std::string frameName;
  };
}