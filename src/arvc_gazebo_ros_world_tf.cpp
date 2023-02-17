
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


  //////////////////////////////////////////////////////////////////////////////
  void PubWorldTF::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
  {
    this->model = _parent;

    if (_sdf->HasElement("target_frame")) {
      this->frameName = _sdf->GetElement("target_frame")->Get<std::string>();
      std::cout << "DETECTA QUE HAY UN FRAMENAME" << std::endl;
    }
    else
      this->frameName = "base_link";


    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&PubWorldTF::OnUpdate, this));

    if (!ros::isInitialized()) {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    
    this->nh_.reset(new ros::NodeHandle("gazebo_client"));

    br2.reset(new tf2_ros::TransformBroadcaster());

    boost::thread ros_pub_thread(boost::bind(&PubWorldTF::PubTF, this));


    ROS_INFO("PUB TF BETWEEN %s - gz_world", this->frameName.c_str());
    ROS_INFO("----- TF PLUGIN LOADED CORRECTLY -----");
  }


  //////////////////////////////////////////////////////////////////////////////
  void PubWorldTF::OnUpdate()
  {
  }


  //////////////////////////////////////////////////////////////////////////////
  void PubWorldTF::PubTF(){

    int seq = 0;
    while(true)
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

      this->br2->sendTransform(transform);
      seq++;

      ros::spinOnce();
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
  }

}
