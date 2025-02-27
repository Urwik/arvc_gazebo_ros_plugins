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

namespace gazebo
{
  class JointMove : public ModelPlugin
  {

    public: 
    /// \brief Constructor
    JointMove();

    /// \brief Destructor
    ~JointMove();


    private:
    
    /// \brief Load the plugin
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    void moveJoint();
    
    // Pointer to the model
    physics::ModelPtr model;
    std::string jointName;
    boost::thread movement_thread;

  };
}