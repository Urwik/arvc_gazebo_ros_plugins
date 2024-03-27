
#include "arvc_gazebo_ros_plugins/arvc_gazebo_ros_tf_publisher.h"
#include "csv.hpp"
#include <QApplication>
#include <QPushButton>

namespace gazebo
{
  ////////////////////////////////////////////////////////////////////////////////
  // Constructor
  TfPublisher::TfPublisher()
  {
    this->parent_model.reset();
    this->child_model.reset();
    this->nh.reset();
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Destructor
  TfPublisher::~TfPublisher()
  {
    this->parent_model.reset();
    this->child_model.reset();
    this->nh.reset();
  }


  //////////////////////////////////////////////////////////////////////////////
  void TfPublisher::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
  {
    ROS_INFO( GREEN "----- LOADING" YELLOW  " TF_PUBLISHER " GREEN "PLUGIN -----" RESET);

    // Store the pointer to the model
    this->child_model = _model;
  }


  void TfPublisher::Init()
  {
    this->RosSetup();
    this->ros_publication_thread = std::thread(&TfPublisher::PubThread, this);
    ROS_INFO( GREEN "----- TF PLUGIN INITIALIZED CORRECTLY -----" RESET);
  }


  void TfPublisher::RosSetup()
  {    
    if (!ros::isInitialized()) {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    this->nh.reset(new ros::NodeHandle("gazebo_client"));
  }


  void TfPublisher::buttonClicked(){
    this->child_pose = this->child_model->WorldPose();
    std::cout << "X,Y,Z:" << std::endl;
    std::cout << this->child_pose.X() << "," << this->child_pose.Y() << "," << this->child_pose.Z() << std::endl;
    std::cout << "Roll,Pitch,Yaw:" << std::endl;
    std::cout << this->child_pose.Roll() << "," << this->child_pose.Pitch() << "," << this->child_pose.Yaw() << std::endl;
  }

  void TfPublisher::PubThread(){

    int seq = 0;
    QApplication app(seq, nullptr);
    QPushButton button("Save Pose");
    QObject::connect(button, &QPushButton::clicked, this, &TfPublisher::buttonClicked);
    
    button.show();
    app.exec();
  }

}
