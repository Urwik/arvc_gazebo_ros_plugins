/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <functional>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/gui/qt.h>
#include <gazebo/physics/physics.hh>
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/GuiPlugin.hh"
#include "gazebo/common/common.hh"
#include <cmath>
#include <chrono>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <yaml-cpp/yaml.h>

namespace gazebo
{
  class SystemGUI : public SystemPlugin
  {
    /////////////////////////////////////////////
    /// \brief Destructor
    public: virtual ~SystemGUI()
    {
    }

    /////////////////////////////////////////////
    /// \brief Called after the plugin has been constructed.
    public: void Load(int /*_argc*/, char ** /*_argv*/)
    {
    //    this->preRenderConnection = event::Events::ConnectPreRender(
    //         std::bind(&SystemGUI::Update, this));

        // // Initialize camera movement parameters
        // this->cameraOrbitCenter = ignition::math::Vector3d(0, 0, 1.7); // Center point to orbit around
        // this->cameraOrbitRadius = 8.0; // Orbit radius in meters
        // this->cameraOrbitHeight = 3.0; // Height above orbit center
        // this->cameraAngularSpeed = 0.1; // Angular speed in rad/s
        // this->cameraClockwise = true; // Orbit direction
        // this->cameraMovementEnabled = false; // Start disabled, enable when camera is found
        // this->startTime = std::chrono::steady_clock::now();

        // // Initialize model following parameters
        // this->followModelName = "ouster_OS1"; // Default model to follow (change as needed)
        // this->followModelEnabled = false; // Start disabled
        // this->followOffset = ignition::math::Vector3d(3.0, 0.0, 2.0); // 3m behind, 2m up
        // this->lookAtFollowedModel = true; // Camera looks at the model

        // this->myThread = boost::thread(&SystemGUI::runningThread, this);
        
        // std::cout << "SystemGUI plugin loaded with camera orbit parameters:\n";
        // std::cout << "  - Orbit center: (" << this->cameraOrbitCenter.X() << ", " 
        //           << this->cameraOrbitCenter.Y() << ", " << this->cameraOrbitCenter.Z() << ")\n";
        // std::cout << "  - Orbit radius: " << this->cameraOrbitRadius << " m\n";
        // std::cout << "  - Angular speed: " << this->cameraAngularSpeed << " rad/s\n";
        // std::cout << "  - Follow model: " << this->followModelName << "\n";
        // std::cout << "  - Follow offset: (" << this->followOffset.X() << ", " 
        //           << this->followOffset.Y() << ", " << this->followOffset.Z() << ")\n";
    }

    /////////////////////////////////////////////
    // \brief Called once after Load
    private: void Init()
    {
        gzmsg << "POIOrbit plugin initialized." << std::endl;
        this->config = YAML::LoadFile(CONFIG_FILE_PATH); // CONFIG_FILE_PATH should be in CMakeLists.txt
        this->config = this->config["camera"];
        
        // Load orbit configuration
        this->cameraOrbitRadius = this->config["orbit_radius"].as<double>();
        this->cameraOrbitHeight = this->config["orbit_height"].as<double>();
        this->cameraAngularSpeed = this->config["angular_speed"].as<double>();
        this->cameraClockwise = (this->config["orbit_direction"].as<std::string>() == "clockwise");
        
        // Load orbit center
        this->cameraOrbitCenter = ignition::math::Vector3d(
            this->config["orbit_center"]["x"].as<double>(),
            this->config["orbit_center"]["y"].as<double>(),
            this->config["orbit_center"]["z"].as<double>()
        );

                // Initialize model following parameters
        this->followModelName = this->config["follow_model"].as<std::string>(); // Default model to follow
        this->followModelEnabled = this->config["enable_follow"].as<bool>(); // Enable model following
        this->followOffset = ignition::math::Vector3d(
            this->config["follow_offset"]["x"].as<double>(),
            this->config["follow_offset"]["y"].as<double>(),
            this->config["follow_offset"]["z"].as<double>()
        ); // 3m behind, 2m up
        this->lookAtFollowedModel = this->config["look_at_model"].as<bool>(); // Camera looks at the model


        std::cout << "SystemGUI plugin initialized with camera orbit parameters:" << std::endl;
        std::cout << "  - Orbit center: (" << this->cameraOrbitCenter.X() << ", " 
                  << this->cameraOrbitCenter.Y() << ", " << this->cameraOrbitCenter.Z() << ")" << std::endl;
        std::cout << "  - Orbit radius: " << this->cameraOrbitRadius << " m" << std::endl;
        std::cout << "  - Angular speed: " << this->cameraAngularSpeed << " rad/s" << std::endl;
        std::cout << "  - Follow model: " << this->followModelName << std::endl;
        std::cout << "  - Follow offset: (" << this->followOffset.X() << ", " 
                  << this->followOffset.Y() << ", " << this->followOffset.Z() << ")" << std::endl;  
        std::cout << "  - Look at followed model: " << (this->lookAtFollowedModel ? "Yes" : "No") << std::endl;
        // Start the thread to monitor camera and perform movements


        this->myThread = boost::thread(&SystemGUI::runningThread, this);

    }

    /////////////////////////////////////////////
    /// \brief Called every PreRender event. See the Load function.
private:
    void runningThread()
    {
        boost::this_thread::sleep(boost::posix_time::milliseconds(60000));

        while (!this->userCam)
        {
            // Get a pointer to the active user camera
            this->userCam = gui::get_active_camera();

            if (this->userCam)
            {
                // Enable saving frames
                this->userCam->EnableSaveFrame(true);

                // Specify the path to save frames into
                this->userCam->SetSaveFramePathname("/tmp/gazebo_frames");

                std::cout << "User camera initialized: " << this->userCam->Name() << "\n";
                
                // Enable camera movement once camera is found
                this->cameraMovementEnabled = true;
                this->followModelEnabled = true; // Enable model following instead of orbit
                this->startTime = std::chrono::steady_clock::now();
                std::cout << "Camera orbit movement enabled!\n";
                std::cout << "Camera model following enabled for model: " << this->followModelName << "\n";
            }
            else
            {
                boost::this_thread::sleep(boost::posix_time::milliseconds(100));
            }
        }

        while (true)
        {
            if (this->userCam)
            {
                // // Choose between orbit mode and follow mode
                // if (this->followModelEnabled)
                // {

                //     // Follow a specific model
                //     this->FollowModelWithCamera(this->followModelName, this->followOffset, this->lookAtFollowedModel);
                // }
                // else
                // {
                    // Orbit around a fixed point
                    this->MoveCameraInOrbit();
                // }
            }
            
            // Wait before next update
            boost::this_thread::sleep(boost::posix_time::milliseconds(25)); // 40 FPS
            // this->Update();
        }
    }


    /////////////////////////////////////////////
    /// \brief Set camera pose to match a specific model's pose
    /// \param modelName Name of the model to follow
    /// \param offset Optional position offset from model (default: 0,0,0)
    /// \param lookAtModel If true, camera looks at model; if false, camera looks in the same direction as model's X-axis
    private: void SetCameraPoseAsModel(const std::string& modelName, 
                                      const ignition::math::Vector3d& offset = ignition::math::Vector3d::Zero,
                                      bool lookAtModel = false)
    {
        if (!this->userCam)
        {
            std::cout << "Warning: User camera not initialized yet!\n";
            return;
        }

        physics::WorldPtr world = physics::get_world("default");
        if (!world)
        {
            std::cout << "Warning: Could not get world pointer!\n";
            return;
        }

        // Get the model
        physics::ModelPtr model = world->ModelByName(modelName);
        if (!model)
        {
            std::cout << "Warning: Model '" << modelName << "' not found!\n";
            return;
        }

        // Get model's world pose
        ignition::math::Pose3d modelPose = model->WorldPose();
        
        // Calculate camera position (model position + offset)
        ignition::math::Vector3d cameraPosition = modelPose.Pos() + offset;
        
        ignition::math::Quaterniond cameraOrientation;
        
        if (lookAtModel)
        {
            // Camera looks toward the model from the offset position
            ignition::math::Vector3d lookDirection = modelPose.Pos() - cameraPosition;
            lookDirection.Normalize();
            
            // Calculate yaw and pitch angles to look at model
            double yaw = atan2(lookDirection.Y(), lookDirection.X());
            double pitch = atan2(-lookDirection.Z(), 
                                sqrt(lookDirection.X() * lookDirection.X() + 
                                    lookDirection.Y() * lookDirection.Y()));
            
            // Create camera orientation (roll = 0)
            cameraOrientation.Euler(0, pitch, yaw);
        }
        else
        {
            // Camera looks in the same direction as the model's X-axis
            // Get the model's X-axis direction in world coordinates
            ignition::math::Vector3d modelXAxis = modelPose.Rot().RotateVector(ignition::math::Vector3d(1, 0, 0));
            modelXAxis.Normalize();
            
            // Calculate yaw and pitch angles to align camera's view with model's X-axis
            double yaw = atan2(modelXAxis.Y(), modelXAxis.X());
            double pitch = atan2(-modelXAxis.Z(), 
                                sqrt(modelXAxis.X() * modelXAxis.X() + 
                                    modelXAxis.Y() * modelXAxis.Y()));
            
            // Create camera orientation (roll = 0) aligned with model's X-axis
            cameraOrientation.Euler(0, pitch, yaw);
        }
        
        // Set the new camera pose
        ignition::math::Pose3d newCameraPose(cameraPosition, cameraOrientation);
        
        // Thread-safe camera pose setting
        boost::mutex::scoped_lock lock(this->mutex);
        this->userCam->SetWorldPose(newCameraPose);
        lock.unlock();
        
        // Debug output
        std::cout << "Camera pose set to model '" << modelName << "' position: (" 
                  << cameraPosition.X() << ", " << cameraPosition.Y() << ", " 
                  << cameraPosition.Z() << ")\n";
    }

    /////////////////////////////////////////////
    /// \brief Follow a model continuously with camera (call this in a loop)
    /// \param modelName Name of the model to follow
    /// \param offset Position offset from model
    /// \param lookAtModel If true, camera looks at model; if false, camera looks in the same direction as model's X-axis
    private: void FollowModelWithCamera(const std::string& modelName,
                                       const ignition::math::Vector3d& offset = ignition::math::Vector3d(2.0, 0.0, 1.0),
                                       bool lookAtModel = true)
    {
        // Continuously update camera to follow the model
        this->SetCameraPoseAsModel(modelName, offset, lookAtModel);
    }

    /////////////////////////////////////////////
    /// \brief Move camera in orbit around the center point
    private: void MoveCameraInOrbit()
    {
        // Calculate elapsed time since start
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - this->startTime);
        double timeInSeconds = elapsed.count() / 1000.0;
        
        // Calculate orbit angle based on time and angular speed
        double angle = timeInSeconds * this->cameraAngularSpeed;
        
        // Apply clockwise/counterclockwise direction
        if (this->cameraClockwise)
        {
            angle = -angle; // Negate for clockwise rotation
        }

        // Calculate new camera position on the orbit
        ignition::math::Vector3d newPosition(
            this->cameraOrbitCenter.X() + this->cameraOrbitRadius * cos(angle),
            this->cameraOrbitCenter.Y() + this->cameraOrbitRadius * sin(angle),
            this->cameraOrbitCenter.Z() + this->cameraOrbitHeight
        );
        
        // Calculate orientation so camera looks toward orbit center
        ignition::math::Vector3d lookDirection = this->cameraOrbitCenter - newPosition;
        lookDirection.Normalize();
        
        // Calculate yaw and pitch angles
        double yaw = atan2(lookDirection.Y(), lookDirection.X());
        double pitch = atan2(-lookDirection.Z(), 
                            sqrt(lookDirection.X() * lookDirection.X() + 
                                lookDirection.Y() * lookDirection.Y()));
        
        // Create camera orientation (roll = 0)
        ignition::math::Quaterniond cameraOrientation;
        cameraOrientation.Euler(0, pitch, yaw);
        
        // Set the new camera pose
        ignition::math::Pose3d newPose(newPosition, cameraOrientation);
        
        // Thread-safe camera pose setting
        boost::mutex::scoped_lock lock(this->mutex);
        this->userCam->SetWorldPose(newPose);
        lock.unlock();
        
        // Debug output (reduced frequency)
        static int counter = 0;
        if (++counter % 100 == 0) // Print every 100 iterations (every 5 seconds at 20 FPS)
        {
            std::cout << "Camera orbiting - Angle: " << (angle * 180.0 / M_PI) 
                      << " degrees, Position: (" << newPosition.X() << ", " 
                      << newPosition.Y() << ", " << newPosition.Z() << ")\n";
        }
    }

/*     ///////////////////////////////////////////
    / \brief Called every PreRender event. See the Load function.
    private: void Update()
    {
    //   if (!this->userCam)
    //   {
    //     // Get a pointer to the active user camera
    //     this->userCam = gui::get_active_camera();

    //     // Enable saving frames
    //     this->userCam->EnableSaveFrame(true);

    //     // Specify the path to save frames into
    //     this->userCam->SetSaveFramePathname("/tmp/gazebo_frames");

    //     if (this->userCam)
    //     {
    //       std::cout << "User camera initialized: "
    //                 << this->userCam->Name() << "\n";
          
    //       return;
    //     }
    //   }

    //   // Get scene pointer
    //   rendering::ScenePtr scene = rendering::get_scene();

    //   // Wait until the scene is initialized.
    //   if (!scene || !scene->Initialized())
    //     return;

    //   // Look for a specific visual by name.
    //   if (scene->GetVisual("ground_plane"))
    //     std::cout << "Has ground plane visual\n";
    
        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    } */

    /// Pointer the user camera.
    private: rendering::UserCameraPtr userCam;

    /// All the event connections.
    private: event::ConnectionPtr preRenderConnection;
    private: boost::thread myThread;
    private: boost::mutex mutex;
    
    // Camera movement parameters
    private: ignition::math::Vector3d cameraOrbitCenter;
    private: double cameraOrbitRadius;
    private: double cameraOrbitHeight;
    private: double cameraAngularSpeed;
    private: bool cameraClockwise;
    private: bool cameraMovementEnabled;
    private: std::chrono::steady_clock::time_point startTime;
    
    // Model following parameters
    private: std::string followModelName;
    private: bool followModelEnabled;
    private: ignition::math::Vector3d followOffset;
    private: bool lookAtFollowedModel;
    private: YAML::Node config; // Configuration loaded from file

  };

  // Register this plugin with the simulator
  GZ_REGISTER_SYSTEM_PLUGIN(SystemGUI)
}