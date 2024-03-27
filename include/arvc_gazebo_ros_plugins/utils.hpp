#pragma once

#include <ignition/math.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include <gazebo/physics/physics.hh>
#include <filesystem>

namespace fs = std::filesystem;

namespace utils
{
    ignition::math::Pose3d ComputeRandomPose(std::string _mode);
    ignition::math::Pose3d ComputeWorldRandomPose(std::string _mode);
    ignition::math::Vector3d ComputeEnvRandPosition();
    ignition::math::Vector3d ComputeRandRotation();
    ignition::math::Vector3d ComputeRandomScale();
    ignition::math::Vector3d ComputeRandomScale(ignition::math::Vector3d min_scale_, ignition::math::Vector3d max_scale_);
    bool ReachPositionOffset(ignition::math::Pose3d pose);
    bool collisionBbx(ignition::math::AxisAlignedBox box1, ignition::math::AxisAlignedBox box2);
    ignition::math::Vector3d ApplyOffset(ignition::math::Vector3d input);
    ignition::math::Vector3d ApplyOffset(ignition::math::Vector3d input, ignition::math::Vector3d offset_);
    void ApplyRotation(gazebo::physics::ModelPtr model_ptr, ignition::math::Vector3d rotation);
    int GetNumOfItems(fs::path path);
    Eigen::VectorXf SetModelWeights(fs::path path);
};

// HELPER FUNCTIONS CALCULATIONS
/////////////////////////////////
ignition::math::Pose3d utils::ComputeRandomPose(std::string _mode) {
    using namespace ignition::math;
    Pose3d pose;
    Vector3d position;
    Vector3d rotation;

    if (_mode == "uniform") {
        position.X() = Rand::DblUniform(this->neg_dist.X(), this->pos_dist.X()); 
        position.Y() = Rand::DblUniform(this->neg_dist.Y(), this->pos_dist.Y()); 
        position.Z() = Rand::DblUniform(this->neg_dist.Z(), this->pos_dist.Z()); 
        
        rotation.X() = Rand::DblUniform(0, this->rot_range[0]* (2*M_PI/360));
        rotation.Y() = Rand::DblUniform(0, this->rot_range[1]* (2*M_PI/360));
        rotation.Z() = Rand::DblUniform(0, this->rot_range[2]* (2*M_PI/360));
    }
    else if (_mode == "normal") {
        position.X() = Rand::DblNormal(0,this->pos_dist.X()); 
        position.Y() = Rand::DblNormal(0,this->pos_dist.Y()); 
        position.Z() = Rand::DblNormal(0,this->pos_dist.Z()); 

        rotation.X() = Rand::DblNormal(0, (this->rot_range[0]*(2*M_PI/360))/3);
        rotation.Y() = Rand::DblNormal(0, (this->rot_range[1]*(2*M_PI/360))/3);
        rotation.Z() = Rand::DblNormal(0, (this->rot_range[2]*(2*M_PI/360))/3);
    }
    else
    {
        ROS_ERROR("WRONG RANDOM MODE, POSSIBLE OPTIONS ARE: uniform, normal");
    }

    // position = this->ApplyOffset(position);
    pose.Set(position, rotation);

    return pose;
}

/////////////////////////////////
ignition::math::Pose3d utils::ComputeWorldRandomPose(std::string _mode) {
    using namespace ignition::math;
    Pose3d pose;
    Vector3d position;
    Vector3d rotation;

    rotation.X() = 0;
    rotation.Y() = 0;
    rotation.Z() = Rand::DblUniform(0, 2*M_PI);

    if (_mode == "uniform")
    {
        position.X() = Rand::DblUniform(-2, 2); 
        position.Y() = Rand::DblUniform(-2, 2); 
        position.Z() = Rand::DblUniform(-0.5, 0.5); 
    }
    else if (_mode == "normal")
    {
        position.X() = Rand::DblNormal(0, 1); 
        position.Y() = Rand::DblNormal(0, 1); 
        position.Z() = Rand::DblNormal(0, 0.5); 
    }
    else
        ROS_ERROR("WRONG RANDOM MODE, POSSIBLE OPTIONS ARE: uniform, normal");

    pose.Set(position, rotation);

    return pose;
}

/////////////////////////////////
ignition::math::Vector3d utils::ComputeEnvRandPosition()
{
    using namespace ignition::math;
    Vector3d position;
    Vector3d offset_(10, 10, 0);

    if (this->RANDMODE == "uniform")
    {
        position.X() = Rand::DblUniform(-30, 30);
        position.Y() = Rand::DblUniform(-30, 30);
        position.Z() = Rand::DblUniform(0, 0.5);
    }
    else if (this->RANDMODE == "normal")
    {
        position.X() = Rand::DblNormal(0, 15); 
        position.Y() = Rand::DblNormal(0, 15);
        position.Z() = Rand::DblNormal(0, 0.25);
    }
    else
        ROS_ERROR("WRONG RANDOM MODE, POSSIBLE OPTIONS ARE: uniform, normal");

    return this->ApplyOffset(position, offset_);
}

/////////////////////////////////
ignition::math::Vector3d utils::ComputeRandRotation()
{
using namespace ignition::math;
Vector3d rotation;
Vector3d offset_(10, 10, 0);

rotation.X() = Rand::DblUniform(0, 2*M_PI);
rotation.Y() = Rand::DblUniform(0, 2*M_PI);
rotation.Z() = Rand::DblUniform(0, 2*M_PI);

return rotation;
}

/////////////////////////////////
ignition::math::Vector3d utils::ComputeRandomScale()
{
ignition::math::Vector3d scale;
scale.X() = ignition::math::Rand::DblUniform(this->min_scale.X(), this->max_scale.X());
scale.Y() = ignition::math::Rand::DblUniform(this->min_scale.Y(), this->max_scale.Y());
scale.Z() = ignition::math::Rand::DblUniform(this->min_scale.Z(), this->max_scale.Z());

return scale;
}

/////////////////////////////////
ignition::math::Vector3d utils::ComputeRandomScale(ignition::math::Vector3d min_scale_, ignition::math::Vector3d max_scale_)
{
ignition::math::Vector3d scale;
scale.X() = ignition::math::Rand::DblUniform(min_scale_.X(), max_scale_.X());
scale.Y() = ignition::math::Rand::DblUniform(min_scale_.Y(), max_scale_.Y());
scale.Z() = ignition::math::Rand::DblUniform(min_scale_.Z(), max_scale_.Z());

return scale;
}


//////////////////////////////////////////////////////////////////////////////
bool utils::ReachPositionOffset(ignition::math::Pose3d pose)
{
float distance = this->sensor_model->WorldPose().Pos().Distance(pose.Pos());

if (distance > this->sensor_offset)
    return true;
else
    return false;
}


//////////////////////////////////////////////////////////////////////////////
/**
 * @brief
 * @param 
 * @return
 */
bool collisionBbx(ignition::math::AxisAlignedBox box1, ignition::math::AxisAlignedBox box2)
{
if (box1.Intersects(box2) || box2.Intersects(box1))
    return true;
else
    return false;
}

/////////////////////////////////
ignition::math::Vector3d utils::ApplyOffset(ignition::math::Vector3d input)
{
ignition::math::Vector3d output;

for (size_t i = 0; i < 3; i++)
{
    if(input[i] < 0)
    output[i] = input[i] + this->neg_offset[i];
    else
    output[i] = input[i] + this->pos_offset[i];
}

return output;
}

/////////////////////////////////
ignition::math::Vector3d utils::ApplyOffset(ignition::math::Vector3d input, ignition::math::Vector3d offset_)
{
ignition::math::Vector3d output;

for (size_t i = 0; i < 3; i++)
{
    if(input[i] < 0)
    output[i] = input[i] - offset_[i];
    else
    output[i] = input[i] + offset_[i];
}

return output;
}

/**
 * @brief Aplly rotation to a model
 * @param model_ptr Pointer to a model in gazebo
 * @param rotation Rotation vector R P Y
 */
void utils::ApplyRotation(gazebo::physics::ModelPtr model_ptr, ignition::math::Vector3d rotation)
{
ignition::math::Pose3d pose;

pose = model_ptr->WorldPose();
this->world->SetPaused(true);
pose.Rot().Euler(rotation);
model_ptr->SetWorldPose(pose);
this->world->SetPaused(false);
}

/////////////////////////////////
int utils::GetNumOfItems(fs::path path) {
    fs::directory_iterator dir_iter(path);
    return int(std::distance(dir_iter, fs::directory_iterator{}));
}

/////////////////////////////////
Eigen::VectorXf utils::SetModelWeights(fs::path path) 
{
using namespace ignition::math;
const int num_models = this->GetNumOfItems(path);
Eigen::VectorXf model_weights(num_models);

for (size_t i = 0; i < num_models; i++)
    model_weights(i) = Rand::IntUniform(0,100);

model_weights = model_weights / model_weights.sum();

return model_weights;
}