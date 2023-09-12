#include <iostream>
#include <filesystem>

#include "arvc_gazebo_ros_plugins/arvc_dataset_generator_utils.hpp"

namespace fs = std::filesystem;
using namespace std;

int main(int argc, char** argv)
{
    fs::path path("/home/arvc/workSpaces/arvc_ws/src/arvc_dataset_generator/config/dataset_generator_config.yaml");

    arvc::plugin::configuration config(path);

    cout << config.simulation << endl;
    cout << config.sensor << endl;
    cout << config.camera << endl;
    cout << config.out_data << endl;
    cout << config.env << endl;
    cout << config.lab_mod << endl;
    return 0;
}