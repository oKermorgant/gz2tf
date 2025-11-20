#ifndef GZ2TF_URDF_EXPORT_H
#define GZ2TF_URDF_EXPORT_H

#include <urdf/model.h>
#include <tinyxml2.h>

namespace gz2tf
{
std::unique_ptr<tinyxml2::XMLDocument> exportURDF(const urdf::ModelInterface &model);
}

#endif // GZ2TF_URDF_EXPORT_H
