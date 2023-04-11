#ifndef URDF_EXPORT_H
#define URDF_EXPORT_H

#include <urdf/model.h>
#include <tinyxml.h>

namespace my_urdf_export
{

std::string values2str(unsigned int count, const double *values, double (*conv)(double)=nullptr)
{
  std::stringstream ss;
  for (unsigned int i = 0 ; i < count ; i++)
  {
    if (i > 0)
      ss << " ";
    ss << (conv ? conv(values[i]) : values[i]);
  }
  return ss.str();
}
std::string values2str(urdf::Vector3 vec)
{
  double xyz[3];
  xyz[0] = vec.x;
  xyz[1] = vec.y;
  xyz[2] = vec.z;
  return values2str(3, xyz);
}
std::string values2str(urdf::Rotation rot)
{
  double rpy[3];
  rot.getRPY(rpy[0], rpy[1], rpy[2]);
  return values2str(3, rpy);
}
std::string values2str(urdf::Color c)
{
  double rgba[4];
  rgba[0] = std::min(c.r, 1.f);
  rgba[1] = std::min(c.g, 1.f);
  rgba[2] = std::min(c.b, 1.f);
  rgba[3] = std::min(c.a, 1.f);
  return values2str(4, rgba);
}
std::string values2str(double d)
{
  return values2str(1, &d);
}

bool exportPose(urdf::Pose &pose, TiXmlElement* xml)
{
  TiXmlElement *origin = new TiXmlElement("origin");
  std::string pose_xyz_str = values2str(pose.position);
  std::string pose_rpy_str = values2str(pose.rotation);
  origin->SetAttribute("xyz", pose_xyz_str);
  origin->SetAttribute("rpy", pose_rpy_str);
  xml->LinkEndChild(origin);
  return true;
}

bool exportMaterial(urdf::Material &material, TiXmlElement *robot, TiXmlElement *visual = nullptr)
{
  // export basic name to visual
  if(visual)
  {
    TiXmlElement *material_xml = new TiXmlElement("material");
    material_xml->SetAttribute("name", material.name);
    visual->LinkEndChild(material_xml);
  }

  auto material_xml = new TiXmlElement("material");
  material_xml->SetAttribute("name", material.name);

  // export complete definition to root
  if (!material.texture_filename.empty())
  {
    TiXmlElement* texture = new TiXmlElement("texture");
    texture->SetAttribute("filename", material.texture_filename);
    material_xml->LinkEndChild(texture);
  }

  TiXmlElement* color = new TiXmlElement("color");
  color->SetAttribute("rgba", values2str(material.color));
  material_xml->LinkEndChild(color);
  robot->LinkEndChild(material_xml);
  return true;
}

bool exportSphere(urdf::Sphere &s, TiXmlElement *xml)
{
  // e.g. add <sphere radius="1"/>
  TiXmlElement *sphere_xml = new TiXmlElement("sphere");
  sphere_xml->SetAttribute("radius", values2str(s.radius));
  xml->LinkEndChild(sphere_xml);
  return true;
}

bool exportBox(urdf::Box &b, TiXmlElement *xml)
{
  // e.g. add <box size="1 1 1"/>
  TiXmlElement *box_xml = new TiXmlElement("box");
  box_xml->SetAttribute("size", values2str(b.dim));
  xml->LinkEndChild(box_xml);
  return true;
}

bool exportCylinder(urdf::Cylinder &y, TiXmlElement *xml)
{
  // e.g. add <cylinder radius="1"/>
  TiXmlElement *cylinder_xml = new TiXmlElement("cylinder");
  cylinder_xml->SetAttribute("radius", values2str(y.radius));
  cylinder_xml->SetAttribute("length", values2str(y.length));
  xml->LinkEndChild(cylinder_xml);
  return true;
}

bool exportMesh(urdf::Mesh &m, TiXmlElement *xml)
{
  // e.g. add <mesh filename="my_file" scale="1 1 1"/>
  TiXmlElement *mesh_xml = new TiXmlElement("mesh");
  if (!m.filename.empty())
    mesh_xml->SetAttribute("filename", m.filename);
  mesh_xml->SetAttribute("scale", values2str(m.scale));
  xml->LinkEndChild(mesh_xml);
  return true;
}

bool exportGeometry(urdf::GeometrySharedPtr &geom, TiXmlElement *xml)
{
  TiXmlElement *geometry_xml = new TiXmlElement("geometry");
  if (urdf::dynamic_pointer_cast<urdf::Sphere>(geom))
  {
    exportSphere((*(urdf::dynamic_pointer_cast<urdf::Sphere>(geom).get())), geometry_xml);
  }
  else if (urdf::dynamic_pointer_cast<urdf::Box>(geom))
  {
    exportBox((*(urdf::dynamic_pointer_cast<urdf::Box>(geom).get())), geometry_xml);
  }
  else if (urdf::dynamic_pointer_cast<urdf::Cylinder>(geom))
  {
    exportCylinder((*(urdf::dynamic_pointer_cast<urdf::Cylinder>(geom).get())), geometry_xml);
  }
  else if (urdf::dynamic_pointer_cast<urdf::Mesh>(geom))
  {
    exportMesh((*(urdf::dynamic_pointer_cast<urdf::Mesh>(geom).get())), geometry_xml);
  }
  else
  {
    urdf::Sphere *s = new urdf::Sphere();
    s->radius = 0.03;
    geom.reset(s);
    exportSphere((*(urdf::dynamic_pointer_cast<urdf::Sphere>(geom).get())), geometry_xml);
  }

  xml->LinkEndChild(geometry_xml);
  return true;
}

bool exportInertial(urdf::Inertial &i, TiXmlElement *xml)
{
  // adds <inertial>
  //        <mass value="1"/>
  //        <pose xyz="0 0 0" rpy="0 0 0"/>
  //        <inertia ixx="1" ixy="0" />
  //      </inertial>
  TiXmlElement *inertial_xml = new TiXmlElement("inertial");

  TiXmlElement *mass_xml = new TiXmlElement("mass");
  mass_xml->SetAttribute("value", values2str(i.mass));
  inertial_xml->LinkEndChild(mass_xml);

  exportPose(i.origin, inertial_xml);

  TiXmlElement *inertia_xml = new TiXmlElement("inertia");
  inertia_xml->SetAttribute("ixx", values2str(i.ixx));
  inertia_xml->SetAttribute("ixy", values2str(i.ixy));
  inertia_xml->SetAttribute("ixz", values2str(i.ixz));
  inertia_xml->SetAttribute("iyy", values2str(i.iyy));
  inertia_xml->SetAttribute("iyz", values2str(i.iyz));
  inertia_xml->SetAttribute("izz", values2str(i.izz));
  inertial_xml->LinkEndChild(inertia_xml);

  xml->LinkEndChild(inertial_xml);

  return true;
}

bool exportVisual(urdf::Visual &vis, TiXmlElement *link, TiXmlElement *robot)
{
  // <visual group="default">
  //   <origin rpy="0 0 0" xyz="0 0 0"/>
  //   <geometry>
  //     <mesh filename="mesh.dae"/>
  //   </geometry>
  //   <material name="Grey"/>
  // </visual>
  TiXmlElement * visual_xml = new TiXmlElement("visual");

  exportPose(vis.origin, visual_xml);

  exportGeometry(vis.geometry, visual_xml);

  if(vis.material)
    exportMaterial(*vis.material, robot, visual_xml);

  link->LinkEndChild(visual_xml);

  return true;
}

bool exportCollision(urdf::Collision &col, TiXmlElement* link)
{
  // <collision group="default">
  //   <origin rpy="0 0 0" xyz="0 0 0"/>
  //   <geometry>
  //     <mesh filename="mesh.dae"/>
  //   </geometry>
  //   <material name="Grey"/>
  // </collision>
  TiXmlElement * collision_xml = new TiXmlElement("collision");

  exportPose(col.origin, collision_xml);

  exportGeometry(col.geometry, collision_xml);

  link->LinkEndChild(collision_xml);

  return true;
}

bool exportLink(urdf::Link &link, TiXmlElement* xml)
{
  TiXmlElement * link_xml = new TiXmlElement("link");
  link_xml->SetAttribute("name", link.name);

  if (link.inertial)
    exportInertial(*link.inertial, link_xml);
  for (std::size_t i = 0 ; i < link.visual_array.size() ; ++i)
    exportVisual(*link.visual_array[i], link_xml, xml);
  for (std::size_t i = 0 ; i < link.collision_array.size() ; ++i)
    exportCollision(*link.collision_array[i], link_xml);

  xml->LinkEndChild(link_xml);

  return true;
}


bool exportJointDynamics(urdf::JointDynamics &jd, TiXmlElement* xml)
{
  TiXmlElement *dynamics_xml = new TiXmlElement("dynamics");
  dynamics_xml->SetAttribute("damping", values2str(jd.damping) );
  dynamics_xml->SetAttribute("friction", values2str(jd.friction) );
  xml->LinkEndChild(dynamics_xml);
  return true;
}

bool exportJointLimits(urdf::JointLimits &jl, TiXmlElement* xml)
{
  TiXmlElement *limit_xml = new TiXmlElement("limit");
  limit_xml->SetAttribute("effort", values2str(jl.effort) );
  limit_xml->SetAttribute("velocity", values2str(jl.velocity) );
  limit_xml->SetAttribute("lower", values2str(jl.lower) );
  limit_xml->SetAttribute("upper", values2str(jl.upper) );
  xml->LinkEndChild(limit_xml);
  return true;
}

bool exportJointSafety(urdf::JointSafety &js, TiXmlElement* xml)
{
  TiXmlElement *safety_xml = new TiXmlElement("safety_controller");
  safety_xml->SetAttribute("k_position", values2str(js.k_position) );
  safety_xml->SetAttribute("k_velocity", values2str(js.k_velocity) );
  safety_xml->SetAttribute("soft_lower_limit", values2str(js.soft_lower_limit) );
  safety_xml->SetAttribute("soft_upper_limit", values2str(js.soft_upper_limit) );
  xml->LinkEndChild(safety_xml);
  return true;
}

bool exportJointCalibration(urdf::JointCalibration &jc, TiXmlElement* xml)
{
  if (jc.falling || jc.rising)
  {
    TiXmlElement *calibration_xml = new TiXmlElement("calibration");
    if (jc.falling)
      calibration_xml->SetAttribute("falling", values2str(*jc.falling) );
    if (jc.rising)
      calibration_xml->SetAttribute("rising", values2str(*jc.rising) );
    //calibration_xml->SetAttribute("reference_position", values2str(jc.reference_position) );
    xml->LinkEndChild(calibration_xml);
  }
  return true;
}

bool exportJointMimic(urdf::JointMimic &jm, TiXmlElement* xml)
{
  if (!jm.joint_name.empty())
  {
    TiXmlElement *mimic_xml = new TiXmlElement("mimic");
    mimic_xml->SetAttribute("offset", values2str(jm.offset) );
    mimic_xml->SetAttribute("multiplier", values2str(jm.multiplier) );
    mimic_xml->SetAttribute("joint", jm.joint_name );
    xml->LinkEndChild(mimic_xml);
  }
  return true;
}

bool exportJoint(urdf::Joint &joint, TiXmlElement* xml)
{
  TiXmlElement * joint_xml = new TiXmlElement("joint");
  joint_xml->SetAttribute("name", joint.name);
  if (joint.type == urdf::Joint::PLANAR)
    joint_xml->SetAttribute("type", "planar");
  else if (joint.type == urdf::Joint::FLOATING)
    joint_xml->SetAttribute("type", "floating");
  else if (joint.type == urdf::Joint::REVOLUTE)
    joint_xml->SetAttribute("type", "revolute");
  else if (joint.type == urdf::Joint::CONTINUOUS)
    joint_xml->SetAttribute("type", "continuous");
  else if (joint.type == urdf::Joint::PRISMATIC)
    joint_xml->SetAttribute("type", "prismatic");
  else if (joint.type == urdf::Joint::FIXED)
    joint_xml->SetAttribute("type", "fixed");

  // origin
  exportPose(joint.parent_to_joint_origin_transform, joint_xml);

  // axis
  TiXmlElement * axis_xml = new TiXmlElement("axis");
  axis_xml->SetAttribute("xyz", values2str(joint.axis));
  joint_xml->LinkEndChild(axis_xml);

  // parent
  TiXmlElement * parent_xml = new TiXmlElement("parent");
  parent_xml->SetAttribute("link", joint.parent_link_name);
  joint_xml->LinkEndChild(parent_xml);

  // child
  TiXmlElement * child_xml = new TiXmlElement("child");
  child_xml->SetAttribute("link", joint.child_link_name);
  joint_xml->LinkEndChild(child_xml);

  if (joint.dynamics)
    exportJointDynamics(*(joint.dynamics), joint_xml);
  if (joint.limits)
    exportJointLimits(*(joint.limits), joint_xml);
  if (joint.safety)
    exportJointSafety(*(joint.safety), joint_xml);
  if (joint.calibration)
    exportJointCalibration(*(joint.calibration), joint_xml);
  if (joint.mimic)
    exportJointMimic(*(joint.mimic), joint_xml);

  xml->LinkEndChild(joint_xml);
  return true;
}



TiXmlDocument*  exportURDF(const urdf::ModelInterface &model)
{
  TiXmlDocument *doc = new TiXmlDocument();

  TiXmlElement *robot = new TiXmlElement("robot");
  robot->SetAttribute("name", model.name_);
  doc->LinkEndChild(robot);


  for(auto [_,mat]: model.materials_)
    exportMaterial(*mat, robot);

  for(auto [_,link]: model.links_)
    exportLink(*link, robot);

  for(auto [_,joint]: model.joints_)
    exportJoint(*joint, robot);

  return doc;
}




}

#endif // URDF_EXPORT_H
