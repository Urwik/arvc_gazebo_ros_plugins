#include <iostream>
#include <filesystem>
#include <ignition/math.hh>

#include "arvc_gazebo_ros_plugins/arvc_dataset_generator_utils.hpp"

namespace fs = std::filesystem;
using namespace std;
namespace im = ignition::math;

int main(int argc, char** argv)
{
    string package_path =  ros::package::getPath("arvc_dataset_generator");
    fs::path path( package_path + "/config/dataset_generator_config.yaml");

    arvc::plugin::configuration config(path);

    for (size_t i = 0; i < 100; i++)
    {
        cout << im::Rand::DblNormal(0, 1) << endl; 
    }
    


    // cout << type(config.env.model).name() << endl;
    // cout << config.sensor << endl;
    // cout << config.camera << endl;
    // cout << config.out_data << endl;
    // cout << config.env << endl;
    // cout << config.lab_mod << endl;

    // cout << config.

    return 0;
}