#pragma once

#include <sdf/sdf.hh>
#include <gazebo/gazebo.hh>
#include <filesystem>

namespace fs = std::filesystem;
namespace im = ignition::math;

namespace utils
{

    sdf::SDFPtr getSDFfile(fs::path sdfPath)
    {
        if (!fs::exists(sdfPath))
        {
            std::cerr << "File does not exist: " << sdfPath << std::endl;
            return nullptr;
        }

        sdf::SDFPtr sdf_file(new sdf::SDF());
        sdf::init(sdf_file);
        sdf::readFile(sdfPath, sdf_file);

        return sdf_file;
    }

    fs::path getTemporarySDFfile(fs::path orig_path)
    {
        if (!fs::exists(orig_path))
        {
            std::cerr << "File does not exist: " << orig_path << std::endl;
            return "";
        }

        fs::path new_model_path = orig_path.parent_path() / "temp_model.sdf";
        fs::path new_config_path = orig_path.parent_path() / "temp_model.config";
        fs::path orig_config_path = orig_path.parent_path() / "model.config";

        fs::copy_file(orig_path, new_model_path, fs::copy_options::overwrite_existing);
        fs::copy_file(orig_config_path, new_config_path, fs::copy_options::overwrite_existing);

        return new_model_path;
    }

    fs::path copySDFfile(fs::path orig_path)
    {
        if (!fs::exists(orig_path))
        {
            std::cerr << "File does not exist: " << orig_path << std::endl;
            return "";
        }

        fs::path new_model_path = orig_path.parent_path() / "temp_model.sdf";
        fs::path new_config_path = orig_path.parent_path() / "temp_model.config";
        fs::path orig_config_path = orig_path.parent_path() / "model.config";

        fs::copy_file(orig_path, new_model_path, fs::copy_options::overwrite_existing);
        fs::copy_file(orig_config_path, new_config_path, fs::copy_options::overwrite_existing);

        return new_model_path;
    }


    void setModelName(sdf::ElementPtr modelElement, std::string _model_name)
    {
        modelElement->GetAttribute("name")->Set(_model_name);
    }

    /////////////////////////////////
    void setModelPose(sdf::ElementPtr modelElement, im::Pose3d pose)
    {
        sdf::ElementPtr pose_element = modelElement->GetElement("pose");
        pose_element->Set<im::Pose3d>(pose);
    }


    /////////////////////////////////
    void setModelPosition(sdf::ElementPtr modelElement, im::Vector3d position)
    {
        sdf::ElementPtr poseElement = modelElement->GetElement("pose");

        im::Pose3d orig_pose = poseElement->Get<im::Pose3d>();
        im::Pose3d new_pose_;
        new_pose_.Set(position, orig_pose.Rot().Euler());

        poseElement->Set<im::Pose3d>(new_pose_);
    }

    void setModelRotation(sdf::ElementPtr modelElement, im::Vector3d rotation)
    {

        sdf::ElementPtr poseElement = modelElement->GetElement("pose");
        im::Pose3d orig_pose = poseElement->Get<im::Pose3d>();
        im::Pose3d new_pose_;

        new_pose_.Set(orig_pose.Pos(), rotation);

        poseElement->Set<im::Pose3d>(new_pose_);
    }


    void setModelScale(sdf::ElementPtr model, im::Vector3d scale)
    {

        sdf::ElementPtr linkElement = model->GetElement("link");
        sdf::ElementPtr visualElement = linkElement->GetElement("visual");

        while (visualElement)
        {
            sdf::ElementPtr sizeElement = visualElement->GetElement("geometry")->GetElement("box")->GetElement("size");
            sdf::ElementPtr poseElement = visualElement->GetElement("pose");

            im::Vector3d size = sizeElement->Get<im::Vector3d>();
            size = size * scale;

            im::Pose3d pose = poseElement->Get<im::Pose3d>();
            pose.Pos() = pose.Pos() * scale;

            sizeElement->Set<im::Vector3d>(size);
            poseElement->Set<im::Pose3d>(pose);

            visualElement = visualElement->GetNextElement("visual");
        }

        // Set Scale to Collision element
        sdf::ElementPtr collisionElement = linkElement->GetElement("collision");
        sdf::ElementPtr sizeElement = collisionElement->GetElement("geometry")->GetElement("box")->GetElement("size");
        sizeElement->Set<im::Vector3d>(scale);
    }

    /////////////////////////////////
    void setMeshScale(sdf::ElementPtr model, im::Vector3d scale)
    {

        sdf::ElementPtr linkElement = model->GetElement("link");
        sdf::ElementPtr visualElement = linkElement->GetElement("visual");
        sdf::ElementPtr collisionElement = linkElement->GetElement("collision");
        sdf::ElementPtr scaleElement;

        while (visualElement)
        {
            if (!visualElement->GetElement("geometry")->GetElement("mesh")->GetElement("scale"))
            {
                scaleElement = visualElement->GetElement("geometry")
                                   ->GetElement("mesh")
                                   ->AddElement("scale");
            }
            else
            {
                scaleElement = visualElement->GetElement("geometry")
                                   ->GetElement("mesh")
                                   ->GetElement("scale");
            }
            scaleElement->Set<im::Vector3d>(scale);
            visualElement = visualElement->GetNextElement("visual");
        }


        // while (collisionElement)
        // {
        //     sdf::ElementPtr sizeElement = collisionElement->GetElement("geometry")->GetElement("box")->GetElement("size");
        //     sdf::ElementPtr poseElement = collisionElement->GetElement("pose");          

        //     im::Pose3d pose = poseElement->Get<im::Pose3d>();
        //     pose.Pos() = pose.Pos() * scale;

        //     im::Vector3d size = sizeElement->Get<im::Vector3d>();
        //     size = size * scale;

        //     sizeElement->Set<im::Vector3d>(size);
        //     poseElement->Set<im::Pose3d>(pose);

        //     collisionElement = collisionElement->GetNextElement("collision");
        // }

    }

    /////////////////////////////////
    void setLaserRetroForVisualElement(sdf::ElementPtr model, const int start_value)
    {
        sdf::ElementPtr linkElement = model->GetElement("link");
        sdf::ElementPtr visualElement = linkElement->GetElement("visual");

        while (visualElement)
        {
            sdf::ElementPtr retroElement = visualElement->GetElement("laser_retro");
            retroElement->Set<int>(start_value);

            visualElement = visualElement->GetNextElement("visual");
        }
    }

        /////////////////////////////////
    int setIncrementalLaserRetroForVisualElement(sdf::ElementPtr model, const int start_value)
    {
        sdf::ElementPtr linkElement = model->GetElement("link");
        sdf::ElementPtr visualElement = linkElement->GetElement("visual");
        int retro_value = start_value;

        while (visualElement)
        {
            sdf::ElementPtr retroElement = visualElement->GetElement("laser_retro");
            retroElement->Set<int>(retro_value);

            retro_value++;
            visualElement = visualElement->GetNextElement("visual");
        }

        return retro_value;
    }
} // namespace utils