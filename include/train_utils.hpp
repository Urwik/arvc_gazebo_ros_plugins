#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace im = ignition::math;

namespace utils
{

    void setModelPose(gazebo::physics::ModelPtr model, im::Pose3d pose)
    {
        model->SetWorldPose(pose);
    }

    im::Vector3d computeRandomScale(im::Vector2d length, im::Vector2d width, im::Vector2d height, std::string rand_mode="uniform")
    {
        im::Vector3d scale;

        if (rand_mode == "uniform")
        {
            scale.X() = im::Rand::DblUniform(width.X(), width.Y());
            scale.Y() = im::Rand::DblUniform(height.X(), height.Y());
            scale.Z() = im::Rand::DblUniform(length.X(), length.Y());
        }
        else if (rand_mode == "normal")
        {
            scale.Y() = im::Rand::DblNormal(0, width.Y() / 3); // 3 is a factor to adjust standard deviation
            scale.Z() = im::Rand::DblNormal(0, height.Y() / 3);
            scale.X() = im::Rand::DblNormal(0, length.Y() / 3);
        }
        else
            std::cout << "WRONG RANDOM MODE, POSSIBLE OPTIONS ARE: uniform, normal" << std::endl;

        return scale;
    }

    im::Vector3d computeRandomScale(im::Vector2d _scale, std::string rand_mode="uniform")
    {
        im::Vector3d scale;
        double scale_value = 1;

        if (rand_mode == "uniform")
            scale_value = im::Rand::DblUniform(_scale.X(), _scale.Y());
        else if (rand_mode == "normal")
            scale_value = im::Rand::DblNormal(0, _scale.Y() / 3); // 3 is a factor to adjust standard deviation
        else
            std::cout << "WRONG RANDOM MODE, POSSIBLE OPTIONS ARE: uniform, normal" << std::endl;

        scale.X() = scale_value;
        scale.Y() = scale_value;
        scale.Z() = scale_value;
        
        return scale;
    }

    im::Vector3d computeRandomPosition(im::Vector3d min, im::Vector3d max, std::string rand_mode = "uniform")
    {
        im::Vector3d position;

        if (rand_mode == "uniform")
        {
            position.X() = im::Rand::DblUniform(min.X(), max.X());
            position.Y() = im::Rand::DblUniform(min.Y(), max.Y());
            position.Z() = im::Rand::DblUniform(min.Z(), max.Z());
        }
        else if (rand_mode == "normal")
        {
            position.Y() = im::Rand::DblNormal(0, max.X() / 3); // 3 is a factor to reduce the standard deviation
            position.Z() = im::Rand::DblNormal(0, max.Y() / 3);
            position.X() = im::Rand::DblNormal(0, max.Z() / 3);
        }
        else
            std::cout << "WRONG RANDOM MODE, POSSIBLE OPTIONS ARE: uniform, normal" << std::endl;

        return position;
    }

    im::Vector3d computeRandomRotation(im::Vector3d rotation_range = im::Vector3d(360,360,360), std::string rand_mode = "uniform")
    {
        im::Vector3d rotation;

        if (rand_mode == "uniform")
        {
            rotation.X() = im::Rand::DblUniform(0, rotation_range.X() * (2 * M_PI / 360));
            rotation.Y() = im::Rand::DblUniform(0, rotation_range.Y() * (2 * M_PI / 360));
            rotation.Z() = im::Rand::DblUniform(0, rotation_range.Z() * (2 * M_PI / 360));
        }
        else if (rand_mode == "normal")
        {
            rotation.Y() = im::Rand::DblNormal(0, rotation_range.X() * (2 * M_PI / 360) / 3); // 3 is a factor to reduce the standard deviation
            rotation.Z() = im::Rand::DblNormal(0, rotation_range.Y() * (2 * M_PI / 360) / 3);
            rotation.X() = im::Rand::DblNormal(0, rotation_range.Z() * (2 * M_PI / 360) / 3);
        }
        else
            std::cout << "WRONG RANDOM MODE, POSSIBLE OPTIONS ARE: uniform, normal" << std::endl;

        return rotation;
    }

    im::Pose3d computeRandomPose(im::Vector3d trans_min, im::Vector3d trans_max, im::Vector3d rotation_range = im::Vector3d(360,360,360), std::string rand_mode = "uniform")
    {
        im::Pose3d pose;
        im::Vector3d position;
        im::Vector3d rotation;

        position = computeRandomPosition(trans_min, trans_max, rand_mode);
        rotation = computeRandomRotation(rotation_range, rand_mode);

        pose.Set(position, rotation);

        return pose;
    }



    std::vector<std::string> removeFromVector(std::vector<std::string> original_models, std::vector<std::string> model_to_remove) {

        for (auto model_name : model_to_remove) {
        auto it = std::find(original_models.begin(), original_models.end(), model_name);
        if (it != original_models.end()) {
            original_models.erase(it);
        }
        }
        return original_models;
    }


    // //////////////////////////////////////////////////////////////////////////////
    // /// @brief Moves groud model randomly
    // void MoveGroundModel()
    // {
    //     ROS_INFO_COND(this->config.simulation.debug_msgs, "MOVING GROUND MODEL");
    //     physics::ModelPtr world_model = this->world->ModelByName(this->config.env.world_name);
    //     im::Pose3d pose = this->ComputeWorldRandomPose();

    //     // To use SetWorldPose is recommendable pause the world
    //     this->world->SetPaused(true);
    //     world_model->SetWorldPose(pose);
    //     this->world->SetPaused(false);
    // }

} // namespace utils