#include <gz2tf/urdf_export.h>
#include <iostream>

using namespace tinyxml2;


tinyxml2::XMLElement* createChild(
    XMLElement *parent,
    const std::string &tag,
    const std::map<std::string, std::string> &attributes = {})
{
  auto elem{parent->InsertNewChildElement(tag.c_str())};
  for(auto &[key,val]: attributes)
  {
    if(!val.empty())
      elem->SetAttribute(key.c_str(), val.c_str());
  }
  return elem;
}

bool isFinite(double value)
{
  return std::to_string(value).find("inf") == std::string::npos;
}

tinyxml2::XMLElement* createChild(
    XMLElement *parent,
    const std::string &tag,
    const std::map<std::string, double> &attributes)
{
  auto elem{parent->InsertNewChildElement(tag.c_str())};
  for(auto &[key,val]: attributes)
    elem->SetAttribute(key.c_str(), std::to_string(val).c_str());
  return elem;
}

// converters
std::string values2str(unsigned int count, const double *values, double (*conv)(double) = {})
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

/*std::string values2str(double d)
{
  return values2str(1, &d);
}*/


// exporters
bool exportPose(urdf::Pose &pose, tinyxml2::XMLElement* xml)
{
  createChild(xml, "origin",
              {{"xyz", values2str(pose.position)},
               {"rpy", values2str(pose.rotation)}});
  return true;
}

bool exportMaterial(urdf::Material &material, tinyxml2::XMLElement *robot, tinyxml2::XMLElement *visual = {})
{
  // export basic name to visual
  if(visual)
    createChild(visual, "material", {{"name", material.name}});

  auto material_xml{createChild(robot, "material",
                                {{"name", material.name}})};

  // export complete definition to root
  if (!material.texture_filename.empty())
  {
    createChild(material_xml, "texture",
                {{"filename", material.texture_filename}});
  }

  createChild(material_xml, "color",
              {{"rgba", values2str(material.color)}});
  return true;
}

bool exportSphere(urdf::Sphere &s, tinyxml2::XMLElement *xml)
{
  // e.g. add <sphere radius="1"/>
  createChild(xml, "sphere",
              {{"radius", s.radius}});
  return true;
}

bool exportBox(urdf::Box &b, tinyxml2::XMLElement *xml)
{
  // e.g. add <box size="1 1 1"/>
  createChild(xml, "box",
              {{"size", values2str(b.dim)}});
  return true;
}

bool exportCylinder(urdf::Cylinder &y, tinyxml2::XMLElement *xml)
{
  // e.g. add <cylinder radius="1"/>
  createChild(xml, "cylinder",
              {{"radius", y.radius},
               {"length", y.length}});
  return true;
}

bool exportMesh(urdf::Mesh &m, tinyxml2::XMLElement *xml)
{
  // e.g. add <mesh filename="my_file" scale="1 1 1"/>

  if(!m.filename.empty() && m.filename[0] == '/')
    m.filename = "file://" + m.filename;

  createChild(xml, "mesh",
              {{"filename", m.filename},
               {"scale", values2str(m.scale)}});
  return true;
}

bool exportGeometry(urdf::GeometrySharedPtr &geom, tinyxml2::XMLElement *xml)
{
  auto geometry_xml{createChild(xml, "geometry")};
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
  return true;
}

bool exportInertial(urdf::Inertial &i, tinyxml2::XMLElement *xml)
{
  // adds <inertial>
  //        <mass value="1"/>
  //        <pose xyz="0 0 0" rpy="0 0 0"/>
  //        <inertia ixx="1" ixy="0" />
  //      </inertial>

  auto inertial_xml{createChild(xml, "inertial")};
  createChild(inertial_xml, "mass",
              {{"value", i.mass}});

  exportPose(i.origin, inertial_xml);

  createChild(inertial_xml, "inertia",
              {{"ixx", i.ixx},
               {"ixy", i.ixy},
               {"ixz", i.ixz},
               {"iyy", i.iyy},
               {"iyz", i.iyz},
               {"izz", i.izz}});

  return true;
}

bool exportVisual(urdf::Visual &vis, tinyxml2::XMLElement *link, tinyxml2::XMLElement *robot)
{
  // <visual group="default">
  //   <origin rpy="0 0 0" xyz="0 0 0"/>
  //   <geometry>
  //     <mesh filename="mesh.dae"/>
  //   </geometry>
  //   <material name="Grey"/>
  // </visual>
  auto visual_xml{createChild(link, "visual")};

  exportPose(vis.origin, visual_xml);
  exportGeometry(vis.geometry, visual_xml);

  if(vis.material)
    exportMaterial(*vis.material, robot, visual_xml);

  return true;
}

bool exportCollision(urdf::Collision &col, tinyxml2::XMLElement* link)
{
  // <collision group="default">
  //   <origin rpy="0 0 0" xyz="0 0 0"/>
  //   <geometry>
  //     <mesh filename="mesh.dae"/>
  //   </geometry>
  //   <material name="Grey"/>
  // </collision>
  auto collision_xml{createChild(link, "collision")};;

  exportPose(col.origin, collision_xml);
  exportGeometry(col.geometry, collision_xml);

  return true;
}

bool exportLink(urdf::Link &link, tinyxml2::XMLElement* xml)
{
  auto link_xml{createChild(xml, "link",
                            {{"name", link.name}})};

  if (link.inertial)
    exportInertial(*link.inertial, link_xml);
  for (std::size_t i = 0 ; i < link.visual_array.size() ; ++i)
    exportVisual(*link.visual_array[i], link_xml, xml);
  for (std::size_t i = 0 ; i < link.collision_array.size() ; ++i)
    exportCollision(*link.collision_array[i], link_xml);

  return true;
}


bool exportJointDynamics(urdf::JointDynamics &jd, tinyxml2::XMLElement* xml)
{
  createChild(xml, "dynamics",
              {{"damping", jd.damping},
               {"friction", jd.friction}});
  return true;
}

bool exportJointLimits(urdf::JointLimits &jl, tinyxml2::XMLElement* xml)
{
  std::map<std::string, double> limits{{"effort", jl.effort},{"velocity", jl.velocity}};

  if(isFinite(jl.lower) && isFinite(jl.upper))
  {
    limits["lower"] = jl.lower;
    limits["upper"] = jl.upper;
  }

  createChild(xml, "limit",limits);
  return true;
}

bool exportJointSafety(urdf::JointSafety &js, tinyxml2::XMLElement* xml)
{
  createChild(xml, "safety_controller",
              {{"k_position", js.k_position},
               {"k_velocity", js.k_velocity},
               {"soft_lower_limit", js.soft_lower_limit},
               {"soft_upper_limit", js.soft_upper_limit}});
  return true;
}

bool exportJointCalibration(urdf::JointCalibration &jc, tinyxml2::XMLElement* xml)
{
  if (jc.falling || jc.rising)
  {
    auto calibration_xml{createChild(xml, "calibration")};
    if (jc.falling)
      calibration_xml->SetAttribute("falling", std::to_string(*jc.falling).c_str());
    if (jc.rising)
      calibration_xml->SetAttribute("rising", std::to_string(*jc.rising).c_str());
    //calibration_xml->SetAttribute("reference_position", values2str(jc.reference_position) );
  }
  return true;
}

bool exportJointMimic(urdf::JointMimic &jm, tinyxml2::XMLElement* xml)
{
  if (!jm.joint_name.empty())
  {
    createChild(xml, "mimic",
                {{"offset", std::to_string(jm.offset)},
                 {"multiplier", std::to_string(jm.multiplier)},
                 {"joint", jm.joint_name}});
  }
  return true;
}

bool exportJoint(urdf::Joint &joint, tinyxml2::XMLElement* xml)
{
  auto joint_xml{createChild(xml, "joint", {{"name", joint.name}})};

  auto has_limits{true};
  if(joint.limits)
    has_limits == isFinite(joint.limits->lower) && isFinite(joint.limits->lower);

  if (joint.type == urdf::Joint::PLANAR)
    joint_xml->SetAttribute("type", "planar");
  else if (joint.type == urdf::Joint::FLOATING)
    joint_xml->SetAttribute("type", "floating");
  else if (joint.type == urdf::Joint::REVOLUTE && has_limits)
    joint_xml->SetAttribute("type", "revolute");
  else if (joint.type == urdf::Joint::CONTINUOUS || !has_limits)
    joint_xml->SetAttribute("type", "continuous");
  else if (joint.type == urdf::Joint::PRISMATIC)
    joint_xml->SetAttribute("type", "prismatic");
  else if (joint.type == urdf::Joint::FIXED)
    joint_xml->SetAttribute("type", "fixed");

  // origin
  exportPose(joint.parent_to_joint_origin_transform, joint_xml);

  // axis
  createChild(joint_xml, "axis", {{"xyz", values2str(joint.axis)}});

  // parent
  createChild(joint_xml, "parent", {{"link", joint.parent_link_name}});

  // child
  createChild(joint_xml, "child", {{"link", joint.child_link_name}});

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

  return true;
}


std::unique_ptr<XMLDocument> gz2tf::exportURDF(const urdf::ModelInterface &model)
{
  auto doc{std::make_unique<tinyxml2::XMLDocument>()};
  auto robot{doc->NewElement("robot")};
  robot->SetAttribute("name", model.name_.c_str());
  doc->LinkEndChild(robot);

  for(auto [_,mat]: model.materials_)
    exportMaterial(*mat, robot);

  for(auto [_,link]: model.links_)
    exportLink(*link, robot);

  for(auto [_,joint]: model.joints_)
    exportJoint(*joint, robot);

  return doc;
}



