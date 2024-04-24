#pragma once

#include <ignition/math.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include <gazebo/physics/physics.hh>
#include <filesystem>

namespace fs = std::filesystem;

namespace utils
{

ignition::math::Pose3d ComputeRandomPose(std::string _mode, ignition::math::Vector3d _origin, float _range) {
    using namespace ignition::math;
    Pose3d pose;
    Vector3d position;
    Vector3d rotation;

    if (_mode == "uniform") {
        position.X() = Rand::DblUniform(-_range, _range); 
        position.Y() = Rand::DblUniform(-_range, _range); 
        position.Z() = Rand::DblUniform(-_range, _range); 
        
        rotation.X() = Rand::DblUniform(0, 2*M_PI);
        rotation.Y() = Rand::DblUniform(0, 2*M_PI);
        rotation.Z() = Rand::DblUniform(0, 2*M_PI);
    }
    else if (_mode == "normal") {
        position.X() = Rand::DblNormal(0,_range/3); 
        position.Y() = Rand::DblNormal(0,_range/3); 
        position.Z() = Rand::DblNormal(5,_range/3); 

        rotation.X() = Rand::DblNormal(0, 2*M_PI/3);
        rotation.Y() = Rand::DblNormal(0, 2*M_PI/3);
        rotation.Z() = Rand::DblNormal(0, 2*M_PI/3);
    }
    else
    {
        ROS_ERROR("WRONG RANDOM MODE, POSSIBLE OPTIONS ARE: uniform, normal");
    }

    position = position + _origin;
    // position = this->ApplyOffset(position);
    pose.Set(position, rotation);

    return pose;
}

ignition::math::Pose3d ComputeRandomPose(std::string _mode, ignition::math::Vector3d _min, ignition::math::Vector3d _max, float _offset = 0.0) {
    using namespace ignition::math;
    Pose3d pose;
    Vector3d position;
    Vector3d rotation;

    auto now = std::chrono::system_clock::now();
    auto epoch = now.time_since_epoch();
    auto value = std::chrono::duration_cast<std::chrono::seconds>(epoch);
    long duration = value.count();
    int seed = static_cast<int>(duration);

    Rand::Seed(seed);

    if (_mode == "uniform") {
        position.X() = Rand::DblUniform(_min.X(), _max.X()); 
        position.Y() = Rand::DblUniform(_min.Y(), _max.Y()); 
        position.Z() = Rand::DblUniform(_min.Z() + _offset, _max.Z()); 
        
        rotation.X() = Rand::DblUniform(0, 2*M_PI);
        rotation.Y() = Rand::DblUniform(0, 2*M_PI);
        rotation.Z() = Rand::DblUniform(0, 2*M_PI);
    }
    else if (_mode == "normal") {
        position.X() = Rand::DblNormal(0, (_max.X()-_min.X()) / 3); 
        position.Y() = Rand::DblNormal(0, (_max.Y()-_min.Y()) / 3); 
        position.Z() = Rand::DblNormal(0, (_max.Z()-_min.Z()) / 3); 

        rotation.X() = Rand::DblNormal(0, 2*M_PI/3);
        rotation.Y() = Rand::DblNormal(0, 2*M_PI/3);
        rotation.Z() = Rand::DblNormal(0, 2*M_PI/3);
    }
    else
    {
        ROS_ERROR("WRONG RANDOM MODE, POSSIBLE OPTIONS ARE: uniform, normal");
    }
    // position = this->ApplyOffset(position);
    pose.Set(position, rotation);

    return pose;
}

  /// @brief Get last saved cloud count and continue from that number
  int ResumeEnvCount(fs::path _pcd_dir) {

    bool first_entry = true;
    int last_num = 0;

    if (!fs::is_empty(_pcd_dir)) {
      
      for (const fs::directory_entry entry : fs::directory_iterator(_pcd_dir)) {
        if(entry.path().extension() == ".pcd") {
          try{
            int actual_num = std::stoi(entry.path().stem());

            if(actual_num > last_num)
              last_num = actual_num;
          }

          catch(const std::exception& e){
            ROS_WARN("CAN'T READ FILE: %s", entry.path().string().c_str());
          }
        }
      }

      ROS_INFO( "Starting in Env: %d", last_num);
      return last_num;
    }
    else {
      ROS_INFO( "Starting in Env: %d", last_num);
      return last_num;
    }
  }



} // namespace utils
