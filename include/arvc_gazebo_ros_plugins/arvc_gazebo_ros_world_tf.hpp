#pragma once

#include <thread>
#include <functional>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>
#include <ros/advertise_options.h>
#include <tf/tf.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/TransformStamped.h>

#include "console_utils.hpp"


namespace gazebo
{
  class PubWorldTF : public ModelPlugin
  {

    public: 
    /// \brief Constructor
    PubWorldTF();

    /// \brief Destructor
    ~PubWorldTF();


    private:
    
    /// \brief Load the plugin
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    
    /// \brief Update world Connection
    void getConfig(sdf::ElementPtr _sdf);
    
    /// \brief Publish tf between model and gazebo world frame
    void PubThread();

    void setupROS();
      

    // Pointer to the model
    physics::ModelPtr model;
    std::string frameName;
    int hz;

    std::thread pub_thread;

    ros::NodeHandle *ros_node;
    tf2_ros::TransformBroadcaster tf_broadcaster;

    ignition::math::Pose3d world_pose;

    utils::Console console;
  };
}