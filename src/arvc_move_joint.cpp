
#include "arvc_gazebo_ros_plugins/arvc_move_joint.hpp"

namespace gazebo
{
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(JointMove)


  ////////////////////////////////////////////////////////////////////////////////
  // Constructor
  JointMove::JointMove() {
  }


  ////////////////////////////////////////////////////////////////////////////////
  // Destructor
  JointMove::~JointMove() {
  }


  //////////////////////////////////////////////////////////////////////////////
  void JointMove::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    this->model = _model;
    this->movement_thread = boost::thread(boost::bind(&JointMove::moveJoint, this));

  }


  void JointMove::moveJoint()
  {
    this->jointName = "linkA_link0";
    gazebo::physics::JointPtr joint = this->model->GetJoint(jointName);
    if (!joint)
    {
      std::cerr << "Joint " << jointName << " not found" << std::endl;
      return;
    }
    else
    {
      std::cout << "Joint " << jointName << " found" << std::endl;
    }

    joint->SetParam("fmax",  0, 100.0);
    
    while (true)
    {

        // Set joint position
        joint->SetPosition(0, 1.57/2.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        
        joint->SetPosition(0, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // Set joint velocity

        // joint->SetParam("vel",  0, 1.0);       
        // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        
        // joint->SetParam("vel",  0, -1.0);
        // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
  } 

}
