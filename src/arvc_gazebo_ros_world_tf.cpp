
#include <arvc_gazebo_ros_plugins/arvc_gazebo_ros_world_tf.h>


namespace gazebo
{
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(PubWorldTF)

  ////////////////////////////////////////////////////////////////////////////////
  // Constructor
  PubWorldTF::PubWorldTF()
  {
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Destructor
  PubWorldTF::~PubWorldTF()
  {
  }


  void PubWorldTF::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
  {
    this->model = _parent;

    if (!this->model)
    {
      ROS_WARN("No model found");
    }
    
    if (_sdf->HasElement("frameName")) {
      frameName = _sdf->GetElement("frameName")->Get<std::string>();
    }

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&PubWorldTF::OnUpdate, this));

    if (!ros::isInitialized()) {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    br2.reset(new tf2_ros::TransformBroadcaster());

    ROS_INFO("PUB TF BETWEEN %s - gz_world", frameName.c_str());
    ROS_INFO("----- TF PLUGIN LOADED CORRECTLY -----");
  }

  void PubWorldTF::OnUpdate()
  {
    try
    {
      PubTF();
    }
    catch(const std::exception& e)
    {
      std::cerr << e.what() << '\n';
    }
  }


  void PubWorldTF::PubTF(){
    tf::Vector3 pose;
    tf::Quaternion quat;

    geometry_msgs::TransformStamped transform;

    transform.header.frame_id = "gz_world";
    transform.header.stamp = ros::Time::now();

    // transform.child_frame_id = frameName.c_str();
    transform.child_frame_id = "os1_sensor";


    transform.transform.translation.x = this->model->WorldPose().Pos().X();
    transform.transform.translation.y = this->model->WorldPose().Pos().Y();
    transform.transform.translation.z = this->model->WorldPose().Pos().Z();

    transform.transform.rotation.w = this->model->WorldPose().Rot().W();
    transform.transform.rotation.x = this->model->WorldPose().Rot().X();
    transform.transform.rotation.y = this->model->WorldPose().Rot().Y();
    transform.transform.rotation.z = this->model->WorldPose().Rot().Z();      

    this->br2->sendTransform(transform);
  }

}
