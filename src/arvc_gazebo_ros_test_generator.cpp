#include "arvc_gazebo_ros_plugins/arvc_gazebo_ros_test_generator.hpp"
#include "sdf_utils.hpp"
#include "train_utils.hpp"
#include "test_utils.hpp"
#include "yaml_conversions.hpp"
#include "timer_utils.hpp"

using namespace std;

namespace gazebo
{
    // Register this plugin with the simulator
    GZ_REGISTER_WORLD_PLUGIN(TestDatasetGenerator)
    /////////////////////////////////

    TestDatasetGenerator::TestDatasetGenerator()
    {
        // std::cout << "Delay to enable the attach gdb vscode debug" << std::endl;
        // bool gdb_attached = false;
        // while (!gdb_attached) {
        //   sleep(1);
        // }

        cout << RED << "Running Plugin Constructor..." << RESET << endl;
        this->cloud_I.reset(new PointCloudI);
        this->cloud_L.reset(new PointCloudL);

        this->env_count = 0;
        this->laser_retro = 1;
    }

    /////////////////////////////////
    TestDatasetGenerator::~TestDatasetGenerator()
    {
        if (generator_thread.joinable())
        {
            generator_thread.join();
        }
    }

    //////////////////////////////////////////////////////////////////////////////
    void TestDatasetGenerator::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
    {
        this->world = _parent;

        this->getConfig(_sdf);

        this->SetupROS();

        this->CheckOutputDirs();

        this->generator_thread = boost::thread(boost::bind(&TestDatasetGenerator::GenerateDataset, this));

        this->console.info("ARVC GAZEBO DATASET GENERATOR PLUGIN LOADED", GREEN);
    }


    //////////////////////////////////////////////////////////////////////////////
    // MAIN FUNCTION
    void TestDatasetGenerator::GenerateDataset()
    {

        this->world->Reset();
        this->world->SetPhysicsEnabled(this->config["physics"].as<bool>());
        bool physics_enabled = this->world->PhysicsEnabled();
        this->console.debug("Physics enabled: " + std::to_string(physics_enabled));

        std::vector<std::string> env_models;
        std::vector<std::string> all_models;

        gazebo::common::Console::SetQuiet(true);

        bool first_run = true;
        int items_to_generate = this->config["generator"]["items_to_generate"].as<int>();
        int env_change_iteration = this->config["environment"]["change_iteration"].as<int>();
        bool env_move = this->config["environment"]["move"].as<bool>();
        int env_change_counter = 0;

        this->world->SetPaused(true);

        while (this->env_count < items_to_generate)
        {

            if (first_run)
            {
                this->console.info("FIRST ENVIRONMENT GENERATION STARTING TO SPAWN MODELS...");
                this->ResumeEnvCount();
                this->insertSensorModel();
                this->getStructureModel();

                // Environment
                env_models = this->SpawnRandomEnviroment();
                env_models = this->removeModelsInCollision(env_models, this->structure_model->GetName());

                do
                {
                    this->moveSensorRandomly();
                } while (this->checkSensorCollisionWithStructure());

                env_change_counter++;

                this->world->SetPaused(false);

                if (this->config["generator"]["paused"].as<bool>())
                {
                    this->console.info("## PAUSED ##: Press enter to continue ...", YELLOW);
                    std::getchar();
                }

                if (this->config["generator"]["data"]["save"].as<bool>())
                    this->SavePointCloud();

                first_run = false;
            }

            else
            {
                this->console.info("GENERATING RANDOM ENVIROMENT..." + std::to_string(this->env_count));

                if (env_change_iteration != 0)
                {
                    if (env_change_counter == env_change_iteration)
                    {
                        this->console.debug("Enviroment change iteration reached");
                        this->removeModelsByName(env_models);
                        env_models = this->SpawnRandomEnviroment();
                        env_models = this->removeModelsInCollision(env_models, this->structure_model->GetName());

                        env_change_counter = 0;
                        this->console.debug("New Enviroment spawned");
                    }
                    else
                    {
                        if (env_move)
                        {
                            this->moveEnvironmentRandomly(env_models);
                            env_models = this->removeModelsInCollision(env_models, this->structure_model->GetName());
                        }
                    }
                }

                do
                {
                    this->moveSensorRandomly();
                } while (this->checkSensorCollisionWithStructure());

                this->world->SetPaused(false);
                std::this_thread::sleep_for(std::chrono::milliseconds(50));

                if (this->config["generator"]["paused"].as<bool>())
                {
                    this->console.info("## PAUSED ##: Press enter to continue ...", YELLOW);
                    std::getchar();
                }

                if (this->config["generator"]["data"]["save"].as<bool>())
                    this->SavePointCloud();

                env_change_counter++;
                this->env_count++;
            }
        }
        this->console.info("FINISHED GENERATING DATASET", GREEN);
    }

    void TestDatasetGenerator::getConfig(sdf::ElementPtr _sdf)
    {
        std::cout << BLUE << "PARSING ARGUMENTS... " << RESET << std::endl;

        if (_sdf->HasElement("yaml_config"))
        {
            std::string yaml_config = _sdf->GetElement("yaml_config")->Get<std::string>();
            this->config = YAML::LoadFile(yaml_config);
        }
        else
        {
            std::cout << RED << "Param yaml_config inside plugin declaration" << RESET << std::endl;
        }

        this->console.enable = this->config["debug"].as<bool>();
    }

    void TestDatasetGenerator::getStructureModel()
    {
        this->console.debug("GETTING STRUCTURE MODEL...");

        physics::Model_V model_vector = this->world->Models();
        std::string structure_name;
        std::vector<std::string> tokens;
        for (physics::ModelPtr model : model_vector)
        {
            this->console.debug("Is this model the structure?: " + model->GetName());
            std::string tmp_name = model->GetName();
            try {
                tokens = utils::splitString(tmp_name, '_');
                tmp_name = tokens[0];
            }
            catch(const std::exception& e) {
                std::cerr << e.what() << '\n';
            }

            if (tmp_name == "orthogonal" || tmp_name == "crossed") {
                this->console.debug("Structure model found: " + model->GetName(), GREEN);
                structure_name = model->GetName();
                std::string structure_offset = tokens[2].substr(0, 4);
                this->config["structure"]["z_offset"] = std::stod(structure_offset);
                break;
            }
        }

        while (!this->structure_model)
        {
            this->structure_model = this->world->ModelByName(structure_name);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        this->console.debug("STRUCTURE MODEL ready", GREEN);
    }

    void TestDatasetGenerator::insertSensorModel()
    {

        this->console.debug("INSERTING SENSOR MODEL...");
        sdf::SDFPtr sensor_sdf = utils::getSDFfile(this->config["sensor"]["model_path"].as<std::string>());
        sdf::ElementPtr sensor_element = sensor_sdf->Root()->GetElement("model");

        std::string sensor_name = this->config["sensor"]["name"].as<std::string>();
        utils::setModelName(sensor_element, sensor_name);

        sdf::ElementPtr cylinder_elem = sensor_element->GetElement("link")->GetElement("collision")->GetElement("geometry")->GetElement("cylinder");
        float current_radius = cylinder_elem->GetElement("radius")->Get<float>();
        float current_length = cylinder_elem->GetElement("length")->Get<float>();

        float new_radius = this->config["sensor"]["collision_offset"].as<float>() + current_radius;
        float new_length = this->config["sensor"]["collision_offset"].as<float>() + current_length;

        cylinder_elem->GetElement("radius")->Set(new_radius);
        cylinder_elem->GetElement("length")->Set(new_length);

        boost::mutex mtx;
        mtx.lock();
        this->world->InsertModelSDF(*sensor_sdf);
        // Wait for the sensor to be ready
        this->console.debug("Waiting for sensor model to be ready...");
        this->sensor_model = this->world->ModelByName(sensor_name);
        mtx.unlock();

        while (!this->sensor_model)
        {
            // mtx.lock();
            this->sensor_model = this->world->ModelByName(sensor_name);
            // mtx.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        this->console.debug("Sensor MODEL ready", GREEN);
    }

    void TestDatasetGenerator::moveSensorRandomly()
    {
        this->world->SetPaused(true);

        this->console.debug("MOVING SENSOR RANDOMLY...");

        im::Vector3d max_pos = this->structure_model->CollisionBoundingBox().Max();
        im::Vector3d min_pos = this->structure_model->CollisionBoundingBox().Min();

        min_pos.Z() = min_pos.Z() + this->config["structure"]["z_offset"].as<double>();

        im::Vector3d position = utils::computeRandomPosition(min_pos, max_pos);
        im::Vector3d rotation = utils::computeRandomRotation();

        im::Pose3d orig_pose = this->sensor_model->WorldPose();
        im::Pose3d new_pose;
        new_pose.Set(position, rotation);
        this->sensor_model->SetWorldPose(new_pose);

        this->world->SetPaused(false);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    void TestDatasetGenerator::removeModelsByName(std::vector<std::string> models)
    {
        this->world->SetPaused(true);

        this->console.debug("DELETING MODELS...");

        if (models.size() == 0)
        {
            this->console.debug("No models to delete");
            return;
        }

        for (const std::string &model_name : models)
        {
            this->console.debug("DELETING MODEL: " + model_name);
            this->world->RemoveModel(model_name);
            while (this->world->ModelByName(model_name))
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        this->world->SetPaused(false);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    std::vector<std::string> TestDatasetGenerator::SpawnRandomEnviroment()
    {
        this->world->SetPaused(true);

        this->console.debug("SPAWNING RANDOM ENVIROMENT...");

        int item_count = this->config["environment"]["item_count"].as<int>();
        fs::path model_path = this->config["environment"]["model_path"].as<fs::path>();
        im::Vector2d length = this->config["environment"]["length"].as<im::Vector2d>();
        im::Vector2d width = this->config["environment"]["width"].as<im::Vector2d>();
        im::Vector2d height = this->config["environment"]["height"].as<im::Vector2d>();
        im::Vector3d max_pos = this->config["environment"]["position"]["max"].as<im::Vector3d>();
        im::Vector3d min_pos = this->config["environment"]["position"]["min"].as<im::Vector3d>();
        im::Vector2d scale_val = this->config["environment"]["scale"].as<im::Vector2d>();

        std::vector<std::string> model_names;

        for (int i = 0; i < item_count; i++)
        {
            fs::path orig_model_sdf = model_path / "model.sdf";
            fs::path temp_model_sdf = utils::copySDFfile(orig_model_sdf);
            this->console.debug("Crea una copia del SDF: " + temp_model_sdf.string());

            sdf::SDFPtr temp_sdfFile = utils::getSDFfile(temp_model_sdf);

            if (!temp_sdfFile)
                this->console.debug("Error al cargar el SDF: " + temp_model_sdf.string());
            else
                this->console.debug("Obtiene un puntero al SDF: " + temp_sdfFile->Root()->GetName());

            sdf::ElementPtr model_element = temp_sdfFile->Root()->GetElement("model");
            this->console.debug("Obtiene un puntero al modelo: " + model_element->GetName());

            std::string model_name = model_element->GetName() + "_" + std::to_string(i);
            utils::setModelName(model_element, model_name);
            this->console.debug("Setea el nombre del modelo: " + model_name);

            im::Vector3d position = utils::computeRandomPosition(min_pos, max_pos);
            this->console.debug("Genera una posicion aleatoria: ");
            // im::Vector3d scale = utils::computeRandomScale(length, width, height); // DIFFERENT SCALE FOR EACH AXIS
            im::Vector3d scale = utils::computeRandomScale(scale_val); // SAME SCALE FOR ALL AXIS

            this->console.debug("Genera una escala aleatoria: ");

            utils::setModelPosition(model_element, position);
            this->console.debug("Setea la pose del modelo: ");
            utils::setMeshScale(model_element, scale);
            this->console.debug("Setea la escala del modelo: ");

            this->world->InsertModelSDF(*temp_sdfFile);
            // Wait for model to be inserted
            while (!this->world->ModelByName(model_name))
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            this->console.debug("Inserta el modelo en el mundo: ");

            model_names.push_back(model_name);
        }

        this->world->SetPaused(false);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        this->console.debug("ENVIRONMENT SPAWNED CORRECTLY");
        return model_names;
    }

    void TestDatasetGenerator::moveEnvironmentRandomly(std::vector<std::string> model_names)
    {

        this->world->SetPaused(true);
        this->console.debug("Moving Enviroment randomly");

        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        im::Vector3d max_pos = this->config["environment"]["position"]["max"].as<im::Vector3d>();
        im::Vector3d min_pos = this->config["environment"]["position"]["min"].as<im::Vector3d>();

        for (std::string tmp_model_name : model_names)
        {
            physics::ModelPtr tmp_model;

            do
            {
                tmp_model = this->world->ModelByName(tmp_model_name);
            } while (!tmp_model);

            im::Vector3d position = utils::computeRandomPosition(min_pos, max_pos);

            im::Pose3d tmp_pose = tmp_model->WorldPose();
            tmp_pose.Pos() = position;

            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            tmp_model->SetWorldPose(tmp_pose);
        }

        this->world->SetPaused(false);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    //---- CHECK FUNCTIONS -----------------------------------------------------//
    void TestDatasetGenerator::CheckOutputDirs()
    {
        this->console.debug("CHECKING OUTPUT DIRECTORIES...");
        this->pcd_dir = this->config["generator"]["data"]["out_dir"].as<std::string>() + "/pcd";

        if (!fs::exists(this->pcd_dir))
        {
            this->console.debug("Creating output directories");
            fs::create_directories(this->pcd_dir);
        }
    }

    void TestDatasetGenerator::ResumeEnvCount()
    {
        this->console.debug("RESUMING ENVIRONMENT COUNT...");
        fs::directory_entry last_entry;
        int last_num = 0;

        if (!fs::is_empty(this->pcd_dir))
        {

            for (const fs::directory_entry entry : fs::directory_iterator(this->pcd_dir))
            {
                if (entry.path().extension() == ".pcd")
                {
                    try
                    {
                        int actual_num = std::stoi(entry.path().stem());

                        if (actual_num > last_num)
                            last_num = actual_num;
                    }

                    catch (const std::exception &e)
                    {
                        this->console.info("Error the pcd file has no correct name: " + entry.path().string());
                    }
                }
            }

            this->env_count = last_num + 1;
            this->console.info("Starting in Env: " + std::to_string(this->env_count));
        }
        else
        {
            this->env_count = 0;
            this->console.debug("No previous data found, starting from: " + std::to_string(this->env_count));
        }
    }

    bool TestDatasetGenerator::checkSensorCollisionWithStructure()
    {
        this->console.debug("-- CHECKING SENSOR COLLISION WITH STRUCTURE", YELLOW); 
        ignition::math::AxisAlignedBox sensor_bbx = this->sensor_model->CollisionBoundingBox();

        physics::Link_V links = this->structure_model->GetLinks();
        for (physics::LinkPtr link : links) {
            ignition::math::AxisAlignedBox link_bbx = link->CollisionBoundingBox();
            if (sensor_bbx.Intersects(link_bbx))
                return true;    
        }

        return false;
    }

    bool TestDatasetGenerator::checkCollisions(std::string model_name_a, std::string model_name_b)
    {
        this->console.debug("-- CHECKING COLLISIONS", YELLOW);
        this->console.debug("model_a: " + model_name_a, RESET);
        this->console.debug("model_b: " + model_name_b, RESET);
        physics::ModelPtr model_a;
        physics::ModelPtr model_b;
        im::AxisAlignedBox model_a_bbx;
        im::AxisAlignedBox model_b_bbx;

        do
        {
            model_a = this->world->ModelByName(model_name_a);
            boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
        } while (!model_a);
        this->console.debug("\tGot model a");

        do
        {
            model_b = this->world->ModelByName(model_name_b);
            boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
        } while (!model_b);
        this->console.debug("\tGot model b");

        // boost::this_thread::sleep_for(boost::chrono::milliseconds(1000));

        std::mutex mtx;
        mtx.lock();
        this->console.debug("Getting models bounding boxes:");
        model_a_bbx = model_a->CollisionBoundingBox();
        this->console.debug("\tGot model a bounding box", RESET);
        model_b_bbx = model_b->CollisionBoundingBox();
        this->console.debug("\tGot model b bounding box", RESET);
        mtx.unlock();

        return model_a_bbx.Intersects(model_b_bbx);
    }

    std::vector<std::string> TestDatasetGenerator::removeModelsInCollision(std::vector<std::string> original_models, std::string model_fixed)
    {

        this->world->SetPaused(true);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        this->console.debug("REMOVING MODELS IN COLLISION...");

        std::vector<std::string> removed_models;
        std::vector<std::string> new_models;

        for (const std::string &model_name : original_models)
        {
            if (this->checkCollisions(model_name, model_fixed))
            {
                this->console.debug("Collision detected with: " + model_name, RED);
                removed_models.push_back(model_name);
                this->world->RemoveModel(model_name);
                while (this->world->ModelByName(model_name))
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                this->console.debug("Model removed: " + model_name);
            }
        }

        new_models = utils::removeFromVector(original_models, removed_models);

        this->world->SetPaused(false);

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        return new_models;
    }

    //---- POINTCLOUD -----------------------------------------------------//
    void TestDatasetGenerator::SavePointCloud()
    {
        this->console.debug("SAVING POINTCLOUD...");

        pcl::PCDWriter writer;
        std::stringstream ss;
        ss.str("");
        ss << this->pcd_dir.string() << "/" << std::setfill('0') << std::setw(5) << this->env_count << ".pcd";

        this->console.info("Saving point cloud in: " + ss.str(), GREEN);

        int ms_delay = this->config["generator"]["iteration_delay"].as<int>();

        this->world->SetPaused(false);
        std::this_thread::sleep_for(std::chrono::milliseconds(ms_delay));

        this->console.debug("Waitting for generated data ready");
        while (this->cloud_L->empty())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        if (!this->cloud_L->empty())
        {
            if (this->cloud_L->points.size() != this->cloud_L->width)
            {
                int cloud_size = this->cloud_L->points.size();
                this->cloud_L->width = cloud_size;
                this->cloud_L->height = 1;
            }
            writer.write<PointL>(ss.str(), *this->cloud_L, this->config["generator"]["data"]["binary"].as<bool>());
            this->console.info("Point cloud saved", GREEN);
        }
        else
        {
            this->console.info("Cloud is empty", RED);
        }

        this->world->SetPaused(true);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    //---- ROS -----------------------------------------------------//
    void TestDatasetGenerator::SetupROS()
    {
        // Make sure the ROS node for Gazebo has already been initialized
        if (!ros::isInitialized())
        {
            ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                             << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
            return;
        }

        this->ros_node = new ros::NodeHandle("arvc_gazebo_ros_dataset_generator");
        std::string topic_name = this->config["sensor"]["topic"].as<std::string>();

        this->console.debug("Subscribing to topic: " + topic_name);
        ros::SubscribeOptions ros_so =
            ros::SubscribeOptions::create<sensor_msgs::PointCloud2>(
                topic_name, 1, boost::bind(&TestDatasetGenerator::PointCloudCallback, this, _1),
                ros::VoidPtr(), &this->ros_cbqueue);

        this->ros_sub = this->ros_node->subscribe(ros_so);
        this->callback_queue_thread = boost::thread(boost::bind(&TestDatasetGenerator::QueueThread, this));
    }

    void TestDatasetGenerator::QueueThread()
    {
        static const double timeout = 0.01;
        while (this->ros_node->ok())
        {
            this->ros_cbqueue.callAvailable(ros::WallDuration(timeout));
        }
    }

    void TestDatasetGenerator::PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &input)
    {
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*input, pcl_pc2);

        PointCloudI::Ptr temp_cloud(new PointCloudI);
        pcl::fromPCLPointCloud2(pcl_pc2, *this->cloud_I);

        pcl::copyPointCloud(*this->cloud_I, *this->cloud_L);

        for (size_t i = 0; i < this->cloud_I->points.size(); i++)
            this->cloud_L->points[i].label = this->cloud_I->points[i].intensity;
    }

}