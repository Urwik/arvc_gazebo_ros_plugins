#pragma once
#include <iostream>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

#include <filesystem>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

using namespace std;
namespace fs = std::filesystem;
namespace im = ignition::math;

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
        model_base()
        {
            this->name = "";
            this->type = "";
            this->path = fs::path("");
            this->num_models = 0;
            this->rand_mode = "";
            this->min_scale = im::Vector3d(0,0,0);
            this->max_scale = im::Vector3d(0,0,0);
            this->negative_offset = im::Vector3d(0,0,0);
            this->positive_offset = im::Vector3d(0,0,0);
            this->positive_dist = im::Vector3d(0,0,0);
            this->negative_dist = im::Vector3d(0,0,0);
            this->rotation_range = im::Vector3d(0,0,0);

        }
        ~model_base()
        {
            this->name = "";
            this->type = "";
            this->path = fs::path("");
            this->num_models = 0;
            this->rand_mode = "";
            this->min_scale = im::Vector3d(0,0,0);
            this->max_scale = im::Vector3d(0,0,0);
            this->negative_offset = im::Vector3d(0,0,0);
            this->positive_offset = im::Vector3d(0,0,0);
            this->positive_dist = im::Vector3d(0,0,0);
            this->negative_dist = im::Vector3d(0,0,0);
            this->rotation_range = im::Vector3d(0,0,0);
        }

        string name;
        string type;
        fs::path path;
        int num_models;
        string rand_mode;
        im::Vector3d min_scale;
        im::Vector3d max_scale;
        im::Vector3d negative_offset;
        im::Vector3d positive_offset;
        im::Vector3d positive_dist;
        im::Vector3d negative_dist;
        im::Vector3d rotation_range;
    };

    class configuration
    {
        public:
        configuration(){
        }

        configuration(filesystem::path config_path){
            parse_config(config_path);
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

            friend std::ostream& operator<<(std::ostream& os, const configuration::simulation& obj)
            {
                os << GREEN << "---- SIMULATION ----" << RESET << '\n';
                os << "Paused: " <<  boolalpha  << obj.paused << '\n';
                os << "Debug: " <<  boolalpha  << obj.debug_msgs << '\n';
                os << "Log: "   <<  boolalpha  << obj.log_msgs << '\n';
                return os;
            }
            
            bool paused;
            bool debug_msgs;
            bool log_msgs;
            int iteration_delay; // ms
            int data_capture_delay; // ms
        };

        class sensor
        {
            public:
            sensor(){
                this->enable = true;
                this->name = "";
                this->topic = "";
                this->path = fs::path("");
                this->offset = im::Vector3d(0,0,0);
            }

            ~sensor(){
                this->enable = false;
                this->name = "";
                this->topic = "";
                this->path = "";
                this->offset = im::Vector3d(0,0,0);
            }

            friend std::ostream& operator<<(std::ostream& os, const sensor& obj)
            {
                os << GREEN << "---- SENSOR ----" << RESET << '\n';
                os << "Enable: " <<  boolalpha  << obj.enable << '\n';
                os << "Name: " << obj.name << '\n';
                os << "Topic: " << obj.topic << '\n';
                os << "Path: "  << obj.path << '\n';
                return os;
            }

            bool enable;
            string name;
            string topic;
            fs::path path;
            im::Vector3d offset;
        };

        class camera
        {
            public:
            camera(){
                this->enable = true;
                this->name = "";
                this->topic = "/";
                this->path = fs::path("");
            }

            ~camera(){
                this->enable = true;
                this->name = "";
                this->topic = "/";
                this->path = fs::path("");
            }

            friend std::ostream& operator<<(std::ostream& os, const camera& obj)
            {
                os << GREEN << "---- CAMERA ----" << RESET << '\n';
                os << "Enable: " <<  boolalpha  << obj.enable << '\n';
                os << "Name: " << obj.name << '\n';
                os << "Topic: " << obj.topic << '\n';
                os << "Path: "  << obj.path << '\n';
                return os;
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

            friend std::ostream& operator<<(std::ostream& os, const output_data& obj)
            {
                os << GREEN << "---- OUT DATA ----" << RESET << '\n';
                os << "Enable: " <<  boolalpha  << obj.enable << '\n';
                os << "Output Dir: " << obj.out_dir << '\n';
                os << "Quantity: " << obj.quantity << '\n';
                os << "PC Binary: "  <<  boolalpha << obj.pc_binary << '\n';
                return os;
            }

            bool enable;
            fs::path out_dir;
            int quantity;
            bool pc_binary;
        };

        class environment
        {
            public:
            environment(){
                this->num_env_models = 0;
                this->world_model = "";
                this->world_name = "";
                this->insert_models = false;
                this->dynamic_models = false;
                this->model = new model_base[1];
            }

            environment(int arg){
                this->num_env_models = arg;
                this->world_model = "";
                this->world_name = "";
                this->insert_models = NULL;
                this->dynamic_models = NULL;
                this->model = new model_base[arg];
            }

            ~environment(){
                this->num_env_models = 0;
                this->world_model = "";
                this->world_name = "";
                this->insert_models = NULL;
                this->dynamic_models = NULL;
            }

            int num_env_models;   
            string world_name;
            string world_model;
            bool insert_models;
            bool dynamic_models;
            model_base *model;

            friend std::ostream& operator<<(std::ostream& os, const environment& obj)
            {
                os << GREEN << "---- ENVIRONMENT ----" << RESET << '\n';
                os << "Num Environment Models: " << obj.num_env_models << '\n';
                os << "World Name: "  << obj.world_name << '\n';
                os << "World Model: "  << obj.world_model << '\n';
                os << "Insert_models: "  << obj.insert_models << '\n';
                os << "Dynamic_models: "  << obj.dynamic_models << '\n';
                os << "Models: "  << '\n';
                for (int i = 0 ; i < obj.num_env_models ; i++)
                {
                    os << '\t' << "Model_" << i << ": "  << obj.model[i].name << '\n';
                    os << "\t\t" << "Path: " << obj.model[i].path << '\n';
                    os << "\t\t" << "Num Models: "  << obj.model[i].num_models << '\n';
                    os << "\t\t" << "Rand Mode: "  << obj.model[i].rand_mode << '\n';
                    os << "\t\t" << "Min Scale: "  << obj.model[i].min_scale << '\n';
                    os << "\t\t" << "Max Scale: "  << obj.model[i].max_scale << '\n';
                    os << "\t\t" << "Negative Offset: "  << obj.model[i].negative_offset << '\n';
                    os << "\t\t" << "Positive Offset: "  << obj.model[i].positive_offset << '\n';
                    os << "\t\t" << "Positive Dist: "  << obj.model[i].positive_dist << '\n';
                    os << "\t\t" << "Negative Dist: "  << obj.model[i].negative_dist << '\n';
                    os << "\t\t" << "Rotation Range: "  << obj.model[i].rotation_range << '\n';
                }
                return os;
            }
        };


        class labeled
        {
            public:
            labeled(){
                this->num_lbld_models = 0;
                this->insert_models = false;
                this->dynamic_models = false;
                this->model = new model_base[1];
            }

            labeled(int arg){
                this->num_lbld_models = arg;
                this->insert_models = false;
                this->dynamic_models = false;
                this->model = new model_base[arg];
            }

            ~labeled(){
                this->num_lbld_models = 0;
                this->insert_models = false;
                this->dynamic_models = false;
            }
            
            friend std::ostream& operator<<(std::ostream& os, const labeled& obj)
            {
                os << YELLOW << "---- LABELED ----" << RESET << '\n';
                os << "Num labeled Models: " << obj.num_lbld_models << '\n';
                os << "Insert_models: "  << obj.insert_models << '\n';
                os << "Dynamic_models: "  << obj.dynamic_models << '\n';
                os << "Models: "  << '\n';

                for (int i = 0 ; i < obj.num_lbld_models ; i++)
                {
                    os << '\t' << "Model_" << i << ": "  << obj.model[i].name << '\n';
                    os << "\t\t" << "Path: " << obj.model[i].path << '\n';
                    os << "\t\t" << "Num Models: "  << obj.model[i].num_models << '\n';
                    os << "\t\t" << "Rand Mode: "  << obj.model[i].rand_mode << '\n';
                    os << "\t\t" << "Min Scale: "  << obj.model[i].min_scale << '\n';
                    os << "\t\t" << "Max Scale: "  << obj.model[i].max_scale << '\n';
                    os << "\t\t" << "Negative Offset: "  << obj.model[i].negative_offset << '\n';
                    os << "\t\t" << "Positive Offset: "  << obj.model[i].positive_offset << '\n';
                    os << "\t\t" << "Positive Dist: "  << obj.model[i].positive_dist << '\n';
                    os << "\t\t" << "Negative Dist: "  << obj.model[i].negative_dist << '\n';
                    os << "\t\t" << "Rotation Range: "  << obj.model[i].rotation_range << '\n';
                }
                return os;
            }


            int num_lbld_models;
            bool insert_models;
            bool dynamic_models;
            model_base *model;   
        };




        
        YAML::Node config;

        configuration::simulation simulation;
        configuration::sensor sensor;
        configuration::camera camera;
        configuration::output_data out_data;
        configuration::environment env;
        configuration::labeled lab_mod;

        int num_env_models;
        int num_lbld_models;


        virtual void parse_config(filesystem::path config_path)
        {
            cout << RED << config_path << RESET << endl;
            this->config = YAML::LoadFile(config_path.string().c_str());

            const YAML::Node& model_list = config["environment"]["model"];
            const YAML::Node& lbld_model_list = config["labeled_models"]["model"];

            this->num_env_models = model_list.size();
            this->num_lbld_models = lbld_model_list.size();

            this->env = environment(this->num_env_models);
            this->lab_mod = labeled(this->num_lbld_models);

            // SIMULATION
            this->simulation.paused = config["simulation"]["paused"].as<bool>();
            this->simulation.debug_msgs = config["simulation"]["debug_msgs"].as<bool>();
            this->simulation.log_msgs = config["simulation"]["log_msgs"].as<bool>();

            // SENSORS
            this->sensor.enable = config["sensor"]["enable"].as<bool>();
            this->sensor.name = config["sensor"]["name"].as<string>();
            this->sensor.topic = config["sensor"]["topic"].as<string>();
            this->sensor.path = config["sensor"]["path"].as<string>();
            this->sensor.offset = config["sensor"]["offset"].as<im::Vector3d>();

            // CAMERAS
            this->camera.enable = config["camera"]["enable"].as<bool>();
            this->camera.name = config["camera"]["name"].as<string>();
            this->camera.topic = config["camera"]["topic"].as<string>();
            this->camera.path = config["camera"]["path"].as<string>();

            // OUTPUT DATA
            this->out_data.enable = config["data"]["enable"].as<bool>();
            this->out_data.out_dir = config["data"]["out_dir"].as<string>();
            this->out_data.quantity = config["data"]["quantity"].as<int>();
            this->out_data.pc_binary = config["data"]["pc_binary"].as<bool>();

            // ENVIRONMENT
            this->env.world_model = config["environment"]["world_model"].as<string>();
            this->env.world_name = config["environment"]["world_name"].as<string>();
            this->env.insert_models = config["environment"]["insert_models"].as<bool>();
            this->env.dynamic_models = config["environment"]["dynamic_models"].as<bool>();

            for (int i = 0 ; i < this->env.num_env_models  ; i++)
            {
                this->env.model[i].name = config["environment"]["model"][i]["name"].as<string>();
                this->env.model[i].type = config["environment"]["model"][i]["type"].as<string>();
                this->env.model[i].path = config["environment"]["model"][i]["path"].as<string>();
                this->env.model[i].num_models = config["environment"]["model"][i]["num_models"].as<int>();
                this->env.model[i].rand_mode = config["environment"]["model"][i]["rand_mode"].as<string>();
                this->env.model[i].min_scale = config["environment"]["model"][i]["min_scale"].as<im::Vector3d>();
                this->env.model[i].max_scale = config["environment"]["model"][i]["max_scale"].as<im::Vector3d>();
                this->env.model[i].negative_offset = config["environment"]["model"][i]["negative_offset"].as<im::Vector3d>();
                this->env.model[i].positive_offset = config["environment"]["model"][i]["positive_offset"].as<im::Vector3d>();
                this->env.model[i].positive_dist = config["environment"]["model"][i]["positive_dist"].as<im::Vector3d>();
                this->env.model[i].negative_dist = config["environment"]["model"][i]["negative_dist"].as<im::Vector3d>();
                this->env.model[i].rotation_range = config["environment"]["model"][i]["rotation_range"].as<im::Vector3d>();
            }
                
            // LABELED
            this->lab_mod.insert_models = config["labeled_models"]["insert_models"].as<bool>();
            this->lab_mod.dynamic_models = config["labeled_models"]["dynamic_models"].as<bool>();

            for (int i = 0 ; i <  this->lab_mod.num_lbld_models ; i++)
            {
                this->lab_mod.model[i].name = config["labeled_models"]["model"][i]["name"].as<string>();
                this->lab_mod.model[i].type = config["labeled_models"]["model"][i]["type"].as<string>();
                this->lab_mod.model[i].path = config["labeled_models"]["model"][i]["path"].as<string>();
                this->lab_mod.model[i].num_models = config["labeled_models"]["model"][i]["num_models"].as<int>();
                this->lab_mod.model[i].rand_mode = config["labeled_models"]["model"][i]["rand_mode"].as<string>();
                this->lab_mod.model[i].min_scale = config["labeled_models"]["model"][i]["min_scale"].as<im::Vector3d>();
                this->lab_mod.model[i].max_scale = config["labeled_models"]["model"][i]["max_scale"].as<im::Vector3d>();
                this->lab_mod.model[i].negative_offset = config["labeled_models"]["model"][i]["negative_offset"].as<im::Vector3d>();
                this->lab_mod.model[i].positive_offset = config["labeled_models"]["model"][i]["positive_offset"].as<im::Vector3d>();
                this->lab_mod.model[i].positive_dist = config["labeled_models"]["model"][i]["positive_dist"].as<im::Vector3d>();
                this->lab_mod.model[i].negative_dist = config["labeled_models"]["model"][i]["negative_dist"].as<im::Vector3d>();
                this->lab_mod.model[i].rotation_range = config["labeled_models"]["model"][i]["rotation_range"].as<im::Vector3d>();
            }
        }

    };

}
}

namespace YAML 
{
  template<>
  struct convert<ignition::math::Vector3d> 
  {
    static Node encode(const ignition::math::Vector3d& v3d) 
    {
      Node node;
      node.push_back(v3d.X());
      node.push_back(v3d.Y());
      node.push_back(v3d.Z());
      return node;
    }

    static bool decode(const Node& node, ignition::math::Vector3d& v3d) 
    {
      if(!node.IsSequence() || node.size() != 3) {
        return false;
      }

      double x = node[0].as<double>();
      double y = node[1].as<double>();
      double z = node[2].as<double>();

      v3d.Set(x, y, z);

      return true;
    }
  };
}