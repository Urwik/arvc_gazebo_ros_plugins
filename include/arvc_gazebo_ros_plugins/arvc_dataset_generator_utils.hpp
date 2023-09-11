#include <iostream>
#include <eigen3/Eigen/Dense>


#include <filesystem>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

using namespace std;

#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"  
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"


namespace arvc{

namespace plugin{
    class model_base
    {
    public:
        string name;
        fs::path path;
        int num_models;
        string rand_mode;
        Eigen::Vector3f min_scale;
        Eigen::Vector3f max_scale;
        Eigen::Vector3f negative_offset;
        Eigen::Vector3f positive_offset;
        Eigen::Vector3f positive_dist;
        Eigen::Vector3f negative_dist;
        Eigen::Vector3f rotation_range;
        
    };


    class configuration
    {
        public:
        configuration(){
        }

        configuration(filesystem::path config_path){
            parse_config(config_path);
            environment env(this->num_env_models);
            labeled labeled(this->num_lbld_models);
        }

        class simulation
        {
            public:
            simulation(){
            // SIMULATION
                this->paused = true;
                this->debug_msgs = true;
                this->log_msgs = true;
                this->iteration_delay = 100;
                this->data_capture_delay = 1000;
            }

            ~simulation(){
                this->paused = false;
                this->debug_msgs = false;
                this->log_msgs = false;
                this->iteration_delay = 0;
                this->data_capture_delay = 0;
            }

            bool paused;
            bool debug_msgs;
            bool log_msgs;
            int iteration_delay; // ms
            int data_capture_delay; // ms
        };

        class environment
        {
            public:
            environment(){}

            environment(int arg){
                model_base model[arg];
            }

            int num_env_models;   
            string world_name;
            bool insert_models;
            bool dynamic_models;
            model_base model[5];
        };


        class labeled
        {
            public:
            labeled(){}

            labeled(int arg){
                model_base model[arg];   
            }

            ~labeled(){
                this->num_lbld_models = 0;
                this->insert_models = false;
                this->dynamic_models = false;
            }

            int num_lbld_models;
            bool insert_models;
            bool dynamic_models;
            model_base model[5];   
        };


        class sensor
        {
            public:
            sensor(){
                this->enable = true;
                this->name = "simulated_sensor";
                this->topic = "/sim_sensor/data";
                this->path = fs::path("/media/arvc/data/datasets/New_Dataset");
            }

            bool enable;
            string name;
            string topic;
            fs::path path;
        };


        class camera
        {
            public:
            camera(){
                this->enable = true;
                this->name = "camera";
                this->topic = "/sim_camera";
                this->path = fs::path("/media/arvc/data/datasets/New_Dataset");
            }

            bool enable;
            string name;
            string topic;
            fs::path path;
        };


        class output_data
        {
            public:
            output_data(){
                this->enable = true;
                this->out_dir = fs::path("/media/arvc/data/datasets/New_Dataset");
                this->quantity = 1000;
                this->pc_binary = true;
            }

            ~output_data(){
                this->enable = false;
                this->out_dir = "";
                this->quantity = 0;
                this->pc_binary = false;
            }

            bool enable;
            fs::path out_dir;
            int quantity;
            bool pc_binary;
        };

        
        YAML::Node config;

        configuration::simulation simulation;
        configuration::sensor sensor;
        configuration::camera camera;
        configuration::output_data out_data;
        configuration::environment env;
        configuration::labeled labeled;

        int num_env_models;
        int num_lbld_models;


        virtual void parse_config(filesystem::path config_path)
        {
            namespace fs = std::filesystem;
            


            this->config = YAML::LoadFile(config_path.string().c_str());

            // SIMULATION
            this->simulation.paused = config["simulation"]["paused"].as<bool>();
            this->simulation.debug_msgs = config["simulation"]["debug_msgs"].as<bool>();
            this->simulation.log_msgs = config["simulation"]["log_msgs"].as<bool>();

            // SENSORS
            this->sensor.enable = config["sensors"]["enable"].as<bool>();
            this->sensor.name = config["sensors"]["name"].as<string>();
            this->sensor.topic = config["sensors"]["topic"].as<string>();
            this->sensor.path = config["sensors"]["path"].as<string>();

            // CAMERAS
            this->camera.enable = config["cameras"]["enable"].as<bool>();
            this->camera.name = config["cameras"]["name"].as<string>();
            this->camera.topic = config["cameras"]["topic"].as<string>();
            this->camera.path = config["cameras"]["path"].as<string>();

            // OUTPUT DATA
            this->out_data.enable = config["output_data"]["enable"].as<bool>();
            this->out_data.out_dir = config["output_data"]["out_dir"].as<string>();
            this->out_data.quantity = config["output_data"]["quantity"].as<int>();
            this->out_data.pc_binary = config["output_data"]["pc_binary"].as<bool>();

            // ENVIRONMENT
            this->env.world_name = config["environment"]["world_name"].as<string>();
            this->env.insert_models = config["environment"]["insert_models"].as<bool>();
            this->env.dynamic_models = config["environment"]["dynamic_models"].as<bool>();
            this->env.num_env_models = sizeof(this->env.model) / sizeof(configuration::environment::model);

            for (int i = 0 ; i < this->env.num_env_models  ; i++)
            {
                this->env.model[i].name = config["environment"]["model"][i]["name"].as<string>();
                this->env.model[i].path = config["environment"]["model"][i]["path"].as<string>();
                this->env.model[i].num_models = config["environment"]["model"][i]["num_models"].as<int>();
                this->env.model[i].rand_mode = config["environment"]["model"][i]["rand_mode"].as<string>();
                this->env.model[i].min_scale = config["environment"]["model"][i]["min_scale"].as<Eigen::Vector3f>();
                this->env.model[i].max_scale = config["environment"]["model"][i]["max_scale"].as<Eigen::Vector3f>();
                this->env.model[i].negative_offset = config["environment"]["model"][i]["negative_offset"].as<Eigen::Vector3f>();
                this->env.model[i].positive_offset = config["environment"]["model"][i]["positive_offset"].as<Eigen::Vector3f>();
                this->env.model[i].positive_dist = config["environment"]["model"][i]["positive_dist"].as<Eigen::Vector3f>();
                this->env.model[i].negative_dist = config["environment"]["model"][i]["negative_dist"].as<Eigen::Vector3f>();
                this->env.model[i].rotation_range = config["environment"]["model"][i]["rotation_range"].as<Eigen::Vector3f>();
            }
                
            // LABELED
            this->labeled.insert_models = config["labeled"]["insert_models"].as<bool>();
            this->labeled.dynamic_models = config["labeled"]["dynamic_models"].as<bool>();
            this->labeled.num_lbld_models = sizeof(this->labeled.model) / sizeof(configuration::labeled::model);

            for (int i = 0 ; i <  this->labeled.num_lbld_models ; i++)
            {
                this->labeled.model[i].name = config["labeled"]["model"][i]["name"].as<string>();
                this->labeled.model[i].path = config["labeled"]["model"][i]["path"].as<string>();
                this->labeled.model[i].num_models = config["labeled"]["model"][i]["num_models"].as<int>();
                this->labeled.model[i].rand_mode = config["labeled"]["model"][i]["rand_mode"].as<string>();
                this->labeled.model[i].min_scale = config["labeled"]["model"][i]["min_scale"].as<Eigen::Vector3f>();
                this->labeled.model[i].max_scale = config["labeled"]["model"][i]["max_scale"].as<Eigen::Vector3f>();
                this->labeled.model[i].negative_offset = config["labeled"]["model"][i]["negative_offset"].as<Eigen::Vector3f>();
                this->labeled.model[i].positive_offset = config["labeled"]["model"][i]["positive_offset"].as<Eigen::Vector3f>();
                this->labeled.model[i].positive_dist = config["labeled"]["model"][i]["positive_dist"].as<Eigen::Vector3f>();
                this->labeled.model[i].negative_dist = config["labeled"]["model"][i]["negative_dist"].as<Eigen::Vector3f>();
                this->labeled.model[i].rotation_range = config["labeled"]["model"][i]["rotation_range"].as<Eigen::Vector3f>();
            }
        }

    };

}
}