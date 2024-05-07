
#include "arvc_gazebo_ros_plugins/arvc_gazebo_ros_world_tf.hpp"

namespace gazebo
{
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(PubWorldTF)


  ////////////////////////////////////////////////////////////////////////////////
  // Constructor
  PubWorldTF::PubWorldTF() {
  }


  ////////////////////////////////////////////////////////////////////////////////
  // Destructor
  PubWorldTF::~PubWorldTF() {
  }


  //////////////////////////////////////////////////////////////////////////////
  void PubWorldTF::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    this->model = _model;

    this->getConfig(_sdf);

    // this->setupROS();

    this->pub_thread = std::thread(std::bind(&PubWorldTF::PubThread, this));

    this->console.info("----- TF PLUGIN LOADED CORRECTLY -----");
  }

  void PubWorldTF::getConfig(sdf::ElementPtr _sdf) {    // Parse args from SDF
    
    if (_sdf)
      this->console.info("SDF Pointer correct loaded", GREEN);
    else
      this->console.info("SDF Pointer incorrect loaded", RED);

    this->console.info(" --- Getting parameters from SDF -------------");


    if (_sdf->HasElement("debug"))
    {
      this->console.enable = _sdf->GetElement("debug")->Get<bool>();
      this->console.info("\tGetting debug flag from SDF: " + std::to_string(this->console.enable), RESET);
    }
    else {
      this->console.info("Debug flag not found in SDF, using default value: true", ORANGE);
      this->console.enable = true;
    }

    
    
    if (_sdf->HasElement("target_frame")){
      this->frameName = _sdf->GetElement("target_frame")->Get<std::string>();
      this->console.info("\tTarget frame loaded: " + this->frameName, RESET);
    }
    else
    {
      this->console.info("Target frame not found in SDF, using default value: base_link", ORANGE);
      this->frameName = "base_link";
    }
    
    if (_sdf->HasElement("hz"))
    {
      this->hz = _sdf->GetElement("hz")->Get<int>();
      this->console.info("\tHz loaded: " + std::to_string(this->hz), RESET);
    }
    else {
      this->console.info("Hz not found in SDF, using default value: 100", ORANGE);
      this->hz = 100;
    }
  }


  //////////////////////////////////////////////////////////////////////////////
  void PubWorldTF::PubThread(){

    std::mutex mtx;

    while (!this->model) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    int seq = 0;
    while(true)
    {
      geometry_msgs::TransformStamped transform;

      transform.header.frame_id = "gz_world";
      transform.header.seq = seq;
      transform.header.stamp = ros::Time::now();

      transform.child_frame_id = this->frameName;

      mtx.lock();
      this->world_pose = this->model->WorldPose();
      mtx.unlock();
      this->console.debug("Got world pose", GREEN);

      transform.transform.translation.x = this->world_pose.Pos().X();
      transform.transform.translation.y = this->world_pose.Pos().Y();
      transform.transform.translation.z = this->world_pose.Pos().Z();

      transform.transform.rotation.w = this->world_pose.Rot().W();
      transform.transform.rotation.x = this->world_pose.Rot().X();
      transform.transform.rotation.y = this->world_pose.Rot().Y();
      transform.transform.rotation.z = this->world_pose.Rot().Z();      

      this->tf_broadcaster.sendTransform(transform);
      // ros::spinOnce();
      std::this_thread::sleep_for(std::chrono::milliseconds(1000/this->hz));

      seq++;
    }
  }

  void PubWorldTF::setupROS()
  {
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                       << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    this->ros_node = new ros::NodeHandle(this->frameName + "_gz_world_tf");
  }

}
