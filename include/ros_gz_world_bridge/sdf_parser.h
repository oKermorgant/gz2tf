#ifndef ROS_GZ_WORLD_BRIDGE_SDF_PARSER_H
#define ROS_GZ_WORLD_BRIDGE_SDF_PARSER_H

#include <sdf/Root.hh>
#include <sdf/World.hh>
#include <sdf/Link.hh>
#include <ros_gz_world_bridge/urdf_export.h>
#include <ros_gz_world_bridge/sdf_paths.h>
#include <sdformat_urdf/sdformat_urdf.hpp>
#include <optional>

#ifdef GZ_FORTRESS
#include <ignition/math/Pose3.hh>
namespace gz = ignition;
#else
#include <gz/math/Pose3.hh>
#endif


namespace ros_gz_world_bridge
{

struct PlacedModel
{
  urdf::ModelInterfaceConstSharedPtr urdf;
  std::optional<gz::math::Pose3d> pose;
};


using LinkMap = std::unordered_map<std::string, std::string>;
using DynamicModel = std::tuple<urdf::ModelInterfaceConstSharedPtr, LinkMap, std::string>;
using Geometry = std::pair<urdf::Pose, std::string>;

template <class LinkElem>
bool is_Same(const Geometry &geom, std::shared_ptr<LinkElem> &urdf)
{
  if(static_cast<urdf::Mesh*>(urdf->geometry.get())->filename != geom.second)
    return false;
  const auto &p1{urdf->origin.position};
  const auto &p2{geom.first.position};

  if(p1.x != p2.x || p1.y != p2.y || p1.z != p2.z)
    return false;

  const auto &r1{urdf->origin.rotation};
  const auto &r2{geom.first.rotation};
  return r1.x==r2.x && r1.y==r2.y && r1.z==r2.z && r1.w==r2.w;
}

template <class LinkElem>
void resolveMeshes(std::vector<std::shared_ptr<LinkElem>> &elems,
                   const std::filesystem::path &sdf_root)
{
  std::vector<Geometry> previous;
  std::vector<std::shared_ptr<LinkElem>> to_remove;

  for(auto &elem: elems)
  {
    const auto geom{elem->geometry};
    if(geom->type == urdf::Geometry::MESH)
    {
      if(std::find_if(previous.begin(), previous.end(),
                      [&](auto prev){return is_Same(prev, elem);}) != previous.end())
      {
        to_remove.push_back(elem);
        continue;
      }

      auto mesh = static_cast<urdf::Mesh*>(geom.get());
      previous.push_back({elem->origin, mesh->filename});

      mesh->filename = sdf_paths::resolveURI(mesh->filename, sdf_root);
    }
  }

  for(auto &elem: to_remove)
  {
    auto where{std::find(elems.begin(), elems.end(), elem)};
    if(where != elems.end())
      elems.erase(where);
  }
}



std::string snake_case(std::string name)
{
  for(auto &c: name)
  {
    if(c == ' ') c = '_';
    else c = std::tolower(c);
  }
  return name;
};

PlacedModel convertModel(sdf::Model sdf_model)
{
  PlacedModel converted;
  if(sdf_model.Static())
    converted.pose = sdf_model.RawPose();

  // remove absolute pose that annoy sdformat_urdf
  if(sdf_model.RawPose() != gz::math::Pose3d{})
    sdf_model.SetRawPose({});

  // add a dummy link for composite models
  if(sdf_model.LinkCount() == 0)
  {
    sdf::Link link;
    link.SetName(sdf_model.Name() + "_base");
    sdf_model.AddLink(link);
  }

  sdf::Errors errors;
  converted.urdf = sdformat_urdf::convert_model(sdf_model, errors);
  if(errors.size())
  {
    std::cerr << "When converting " << sdf_model.Name() << ": ";
    for(auto &err: errors)
      std::cerr << "\n  " << err.Message();
    std::cerr << '\n';
    return converted;
  }

  // adapt meshes that are relative to this model
  const auto sdf_root{std::filesystem::path(sdf_paths::resolveURI(sdf_model.Uri())).parent_path()};
  for(auto [_,link]: converted.urdf->links_)
  {
    resolveMeshes(link->visual_array, sdf_root);
    resolveMeshes(link->collision_array, sdf_root);
  }
  return converted;
}

DynamicModel regroupModels(const std::vector<PlacedModel> &models, const std::string &world_name)
{
  if(models.empty())
    return {};

  auto urdf{std::make_shared<urdf::ModelInterface>()};

  // floating joints between world and model roots
  // the bridge will publish the corresponding transforms
  const auto addMovingJoint = [&](const std::string &child)
  {
    auto joint = std::make_shared<urdf::Joint>();
    joint->name = child;
    joint->parent_link_name = "world";
    joint->child_link_name = child;
    joint->type = joint->CONTINUOUS;
    joint->axis.z = 1;
    urdf->joints_[child] = joint;
    return child;
  };

  const auto addFixedJoint = [&](const std::string &child, gz::math::Pose3d pose)
  {
    auto joint = std::make_shared<urdf::Joint>();
    joint->name = child;
    joint->parent_link_name = "world";
    joint->child_link_name = child;
    joint->type = joint->FIXED;
    auto &tf{joint->parent_to_joint_origin_transform};
    tf.position.x = pose.Pos().X();
    tf.position.y = pose.Pos().Y();
    tf.position.z = pose.Pos().Z();
    tf.rotation.x = pose.Rot().X();
    tf.rotation.y = pose.Rot().Y();
    tf.rotation.z = pose.Rot().Z();
    tf.rotation.w = pose.Rot().W();
    urdf->joints_[child] = joint;
  };


  // add root world link
  auto world_link = std::make_shared<urdf::Link>();
  world_link->name = "world";
  urdf->links_["world"] = urdf->root_link_ = world_link;

  urdf->name_ = world_name;
  LinkMap links;

  for(const auto &model: models)
  {
    // change model name to root link one
    const auto prefix{snake_case(model.urdf->getName()) + "_"};
    const auto world_joint{prefix + model.urdf->root_link_->name};
    if(model.pose.has_value())
      addFixedJoint(world_joint, model.pose.value());
    else
      links[model.urdf->getName()] = addMovingJoint(world_joint);

    // transfert all materials
    for(auto [name,mat]: model.urdf->materials_)
    {
      auto new_mat = urdf->materials_[prefix+name] = mat;
      new_mat->name = prefix+name;
    }

    // transfert all links
    for(auto [name,link]: model.urdf->links_)
    {
      auto new_link = urdf->links_[prefix+name] = link;
      new_link->name = prefix+name;

      for(auto visual: link->visual_array)
      {
        if(!visual->material_name.empty())
          visual->material_name.insert(0,prefix);

        if(visual->material && !visual->material->name.empty())
          visual->material->name.insert(0, prefix);
      }
    }

    // transfert all joints
    for(auto [name, joint]: model.urdf->joints_)
    {
      const auto parent{prefix + joint->parent_link_name};
      const auto child{prefix + joint->child_link_name};

      auto converted = urdf->joints_[prefix+name] = joint;
      converted->parent_link_name = parent;
      converted->child_link_name = child;
    }
  }
  return {urdf, links, world_name};
}


DynamicModel parseWorldSDF(const std::string &world_xml,
                           const std::vector<std::string> &ignored,
                           bool use_static)
{
  auto sdf_dom = std::make_shared<sdf::Root>();
  auto errors = sdf_dom->LoadSdfString(world_xml);
  if(!errors.empty())
    return {};

  if(sdf_dom->WorldCount() != 1)
  {
    std::cerr << "Cannot parse world " << std::endl;
    return {};
  }

  const auto world{sdf_dom->WorldByIndex(0)};
  const auto model_count{world->ModelCount()};

  std::vector<PlacedModel> models;
  models.reserve(sdf_dom->WorldByIndex(0)->ModelCount());
  for(size_t m = 0; m < model_count; ++m)
  {
    auto mod{*world->ModelByIndex(m)};
    const auto name{mod.Name()};
    if(std::any_of(ignored.begin(), ignored.end(), [&name](const auto &ignored)
    {return snake_case(name).find(ignored) != name.npos;}))
      continue;
    auto converted{convertModel(*world->ModelByIndex(m))};
    if(converted.urdf)
    {
      if(!use_static)
        converted.pose.reset();
      models.push_back(converted);
    }
  }

  std::cout << "Could convert " << models.size() << " / " << model_count << " models" << std::endl;

  for(const auto &[urdf,pose]: models)
  {
    std::cout << " - " << urdf->getName();
    if(pose.has_value())
      std::cout << " (static @ " << pose.value() << ")";
    std::cout << "\n   base link: " << urdf->getRoot()->name;
    std::cout << "\n   joints: " << urdf->joints_.size();
    std::cout << std::endl;
  }

  return regroupModels(models, world->Name());
}

}


#endif // ROS_GZ_WORLD_BRIDGE_SDF_PARSER_H
