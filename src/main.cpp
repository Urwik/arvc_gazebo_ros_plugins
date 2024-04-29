#include <iostream>
#include <filesystem>
#include <ignition/math.hh>


namespace fs = std::filesystem;
using namespace std;
namespace im = ignition::math;

int main(int argc, char** argv)
{
    string package_path =  ros::package::getPath("arvc_dataset_generator");
    fs::path path( package_path + "/config/dataset_generator_config.yaml");


    for (size_t i = 0; i < 100; i++)
    {
        cout << im::Rand::DblNormal(0, 1) << endl; 
    }


    return 0;
}