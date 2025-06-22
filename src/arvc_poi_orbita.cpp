#include <ignition/math/Pose3.hh>
#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/rendering/rendering.hh"

#include <cmath>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <iomanip>
#include <sstream>

#include <yaml-cpp/yaml.h>
#include <arvc_utils/console.hpp>

namespace gazebo
{
    class POIOrbit : public WorldPlugin
    {

    private:
        physics::WorldPtr world;
        physics::ModelPtr model;
        std::string modelName;
        event::ConnectionPtr updateConnection;
        YAML::Node config;
        utils::Console custom_console;
        bool modelsFound;
        
        // Configuration parameters
        ignition::math::Vector3d orbitCenter;
        double orbitRadius;
        double orbitHeight;
        double angularSpeed;
        bool clockwise;
        bool modelInitialized;
        boost::thread run_thread;
        bool threadRunning;
        boost::mutex poseMutex;
        

    public:
        void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
        {
            this->world = _parent;
            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&POIOrbit::OnUpdate, this));

            // this->usr_camera = gui::get_active_camera();
        }

    public:
        void Init() override
        {
            // This function is called after the world has been loaded.
            gzmsg << "POIOrbit plugin initialized." << std::endl;
            this->config = YAML::LoadFile(CONFIG_FILE_PATH); // CONFIG_FILE_PATH should be in CMakeLists.txt
            this->modelName = this->config["model_name"].as<std::string>();
            
            // Load orbit configuration
            this->orbitRadius = this->config["orbit_radius"].as<double>();
            this->orbitHeight = this->config["orbit_height"].as<double>();
            this->angularSpeed = this->config["angular_speed"].as<double>();
            this->clockwise = (this->config["orbit_direction"].as<std::string>() == "clockwise");
            
            // Load orbit center
            this->orbitCenter = ignition::math::Vector3d(
                this->config["orbit_center"]["x"].as<double>(),
                this->config["orbit_center"]["y"].as<double>(),
                this->config["orbit_center"]["z"].as<double>()
            );
            
            this->modelInitialized = false;
            this->modelsFound = false;
            this->threadRunning = false;
            
            this->custom_console.debug("POIOrbit plugin initialized with model: " + this->modelName, utils::Console::GREEN);
            this->custom_console.debug("Orbit radius: " + std::to_string(this->orbitRadius), utils::Console::BLUE);
            this->custom_console.debug("Angular speed: " + std::to_string(this->angularSpeed), utils::Console::BLUE);
            std::string direction = this->clockwise ? "clockwise" : "counterclockwise";
            this->custom_console.debug("Orbit direction: " + direction, utils::Console::BLUE);


        }

    public:
        void Reset() override
        {
            // This function is called when the world is reset.
            gzmsg << "POIOrbit plugin reset." << std::endl;
            
            // Stop the thread if running
            if (this->threadRunning)
            {
                this->threadRunning = false;
                if (this->run_thread.joinable())
                {
                    this->run_thread.join();
                }
            }
        }

    private:
        void RunOrbitThread()
        {
            this->custom_console.debug("Orbit thread started.", utils::Console::GREEN);
            
            while (this->threadRunning)
            {

                if (!this->modelsFound)
                {
                    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
                    continue;
                }

                // Set initial sensor pose if not initialized
                if (!this->modelInitialized)
                {
                    // Calculate initial position on the orbit
                    double initialAngle = 0.0; // Start at 0 degrees
                    ignition::math::Vector3d initialPosition(
                        this->orbitCenter.X() + this->orbitRadius * cos(initialAngle),
                        this->orbitCenter.Y() + this->orbitRadius * sin(initialAngle),
                        this->orbitCenter.Z() + this->orbitHeight
                    );
                    
                    ignition::math::Pose3d initialPose;
                    initialPose.Set(
                        initialPosition,
                        ignition::math::Quaterniond(0, 0, 0) // No rotation initially
                    );
                    
                    boost::mutex::scoped_lock lock(this->poseMutex);
                    this->model->SetWorldPose(initialPose);
                    lock.unlock();
                    
                    this->modelInitialized = true;
                    
                    this->custom_console.debug("Model initialized at position: " + 
                        std::to_string(initialPose.Pos().X()) + ", " +
                        std::to_string(initialPose.Pos().Y()) + ", " +
                        std::to_string(initialPose.Pos().Z()), utils::Console::BLUE);
                    
                    continue;
                }

                // Calculate orbit angle based on simulation time and angular speed
                double currentTime = this->world->SimTime().Double();
                double angle = currentTime * this->angularSpeed;
                
                // Apply clockwise/counterclockwise direction
                if (this->clockwise)
                {
                    angle = -angle; // Negate for clockwise rotation
                }

                // Calculate new position on the orbit
                ignition::math::Vector3d newPosition(
                    this->orbitCenter.X() + this->orbitRadius * cos(angle),
                    this->orbitCenter.Y() + this->orbitRadius * sin(angle),
                    this->orbitCenter.Z() + this->orbitHeight
                );
                
                // Calculate orientation so X-axis points toward orbit center
                ignition::math::Vector3d xDirection = this->orbitCenter - newPosition;
                xDirection.Normalize();
                
                // Create a coordinate frame with X pointing to orbit center
                // Z-axis pointing up (world Z)
                ignition::math::Vector3d zDirection(0, 0, 1);
                
                // Y-axis is cross product of Z and X (right-hand rule)
                ignition::math::Vector3d yDirection = zDirection.Cross(xDirection);
                yDirection.Normalize();
                
                // Recalculate Z-axis to ensure orthogonality
                zDirection = xDirection.Cross(yDirection);
                zDirection.Normalize();
                
                // Create rotation matrix and convert to quaternion
                ignition::math::Matrix3d rotMatrix(
                    xDirection.X(), yDirection.X(), zDirection.X(),
                    xDirection.Y(), yDirection.Y(), zDirection.Y(),
                    xDirection.Z(), yDirection.Z(), zDirection.Z()
                );
                ignition::math::Quaterniond orientation(rotMatrix);

                // Set the new pose
                ignition::math::Pose3d newPose(newPosition, orientation);

                // Thread-safe pose setting
                boost::mutex::scoped_lock lock(this->poseMutex);
                this->model->SetWorldPose(newPose);
                lock.unlock();
                
                // Debug output (reduced frequency)
                static int counter = 0;
                if (++counter % 100 == 0) // Print every 100 iterations
                {
                    this->custom_console.debug("Orbiting - Angle: " + std::to_string(angle * 180.0 / M_PI) + " degrees", utils::Console::YELLOW);
                }
                
                // Sleep to control update rate
                boost::this_thread::sleep(boost::posix_time::milliseconds(10));
            }
            
            this->custom_console.debug("Orbit thread stopped.", utils::Console::RED);
        }

    public:
        void OnUpdate()
        {
            if (!this->modelsFound)
            {
                // This function is called every simulation iteration.
                if (!this->model)
                {
                    this->model = this->world->ModelByName(this->modelName);
                    if (!this->model)
                    {
                        // gzerr << "Model " << this->modelName <<  " not found!" << std::endl;
                        return;
                    }
                    else 
                    {
                        // Model already found, no need to search again
                        this->custom_console.debug("--> Sensor " + this->modelName + " found.", utils::Console::GREEN);
                    }
                }
                
                this->modelsFound = true;
                
                // Start the orbit thread once models are found
                if (!this->threadRunning)
                {
                    this->threadRunning = true;
                    this->run_thread = boost::thread(&POIOrbit::RunOrbitThread, this);
                    this->custom_console.debug("Orbit thread launched.", utils::Console::BLUE);
                }
            }
        }
    };

    // Register this plugin with the simulator
    GZ_REGISTER_WORLD_PLUGIN(POIOrbit)
}