#include <iostream>
#include <thread>
#include <mutex>

// #include <boost/thread.hpp>
// #include <boost/thread/mutex.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>


using namespace std;

#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"  
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"



namespace gazebo
{
class CollisionDetector : public ModelPlugin
{
  public: 
  
  CollisionDetector(){
    std::cout << BLUE << "Collision Detector Plugin Constructor" << RESET << std::endl;
  }

  void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
  {

    this->model = _model;

    // boost::thread collision_thread(boost::bind(&CollisionDetector::checkCollisions, this));
    this->collision_thread = std::thread(std::bind(&CollisionDetector::checkCollisions, this));

    std::cout << GREEN << "Collision Detector Plugin Loaded" << RESET << std::endl;

  }

  //////////////////////////////////////////////////////////////////////////////
  private: 
  
  void checkCollisions()
  {
    using namespace ignition::math;

    std::cout << "Waiting for model to be initialized" << std::endl;
    while (!this->model) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    std::cout << "Waiting for world to be initialized" << std::endl;
    while (!this->world) {
      this->world = this->model->GetWorld();
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    bool collision = false;
    while(true) {
      this->models = this->world->Models();
      AxisAlignedBox bbx_1 = this->model->CollisionBoundingBox();
      
      for(auto tmp_model : this->models) {
        AxisAlignedBox bbx_2 = tmp_model->CollisionBoundingBox();

        if (this->model->GetName() != tmp_model->GetName()) {
          if(bbx_1.Intersects(bbx_2)) {
            collision = true;
            continue;
          }
        }
      }

      if (collision) {
        std::cout << RED << "Model is in collision" << RESET << std::endl;
        collision = false;
      } else {
        std::cout << GREEN << "Model is not in collision" << RESET << std::endl;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  private:
    physics::WorldPtr world;
    physics::ModelPtr model;
    physics::Model_V models;

    std::thread collision_thread;

};
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(CollisionDetector)
}