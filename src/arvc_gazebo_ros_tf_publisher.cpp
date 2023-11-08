
#include "arvc_gazebo_ros_plugins/arvc_gazebo_ros_tf_publisher.h"




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
    this->nh.reset(new ros::NodeHandle("gazebo_client"));
  }


  //////////////////////////////////////////////////////////////////////////////
  void TfPublisher::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
  {
    ROS_INFO( GREEN "----- LOADING" YELLOW  " TF_PUBLISHER " GREEN "PLUGIN -----" RESET);

    // Store the pointer to the model
    this->child_model = _model;
  }


  void TfPublisher::RosSetup()
  {    
    if (!ros::isInitialized()) {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
  }



  void TfPublisher::Init()
  {
    this->RosSetup();
    this->ros_publication_thread = std::thread(&TfPublisher::PubThread, this);

    ROS_INFO( GREEN "----- TF PLUGIN INITIALIZED CORRECTLY -----" RESET);
  }

  //////////////////////////////////////////////////////////////////////////////
  void TfPublisher::PubThread(){

    while (true)
    {
      ROS_INFO(GREEN "TF PUBLISHER THREAD STARTED" RESET);
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    
    // int seq = 0;
/*     while(true)
    {
      tf::Vector3 pose;
      tf::Quaternion quat;

      geometry_msgs::TransformStamped transform;

      transform.header.frame_id = "gz_world";
      transform.header.seq = seq;
      transform.header.stamp = ros::Time::now();

      transform.child_frame_id = this->frameName;

      transform.transform.translation.x = this->model->WorldPose().Pos().X();
      transform.transform.translation.y = this->model->WorldPose().Pos().Y();
      transform.transform.translation.z = this->model->WorldPose().Pos().Z();

      transform.transform.rotation.w = this->model->WorldPose().Rot().W();
      transform.transform.rotation.x = this->model->WorldPose().Rot().X();
      transform.transform.rotation.y = this->model->WorldPose().Rot().Y();
      transform.transform.rotation.z = this->model->WorldPose().Rot().Z();      

      this->_tf_broadcaster->sendTransform(transform);
      seq++;

      ros::spinOnce();
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    } */
  }

}
