#ifndef GZ2TF_WORLD_BRIDGE_SDF_PATHS_H
#define GZ2TF_WORLD_BRIDGE_SDF_PATHS_H

#include <string>
#include <string.h>
#include <filesystem>
#include <unordered_map>
#include <fstream>
#include <vector>
#include <algorithm>
#include <iostream>

namespace gz2tf
{

namespace sdf_paths
{

namespace fs = std::filesystem;
using Paths = std::vector<fs::path>;

std::vector<std::string> split(std::string s, const char* delim = ":")
{
  std::vector<std::string> subs;
  char* sub = strtok(s.data(), delim);
  while(sub)
  {
    // ignore digit paths
    char *non_digits;
    strtof(sub, &non_digits);
    if(*non_digits)
      subs.push_back(sub);
    sub = strtok(nullptr, delim);
  }
  return subs;
}

Paths paths()
{
  Paths gz_paths;

  gz_paths.push_back(GZ_SIM_SYS_PATH);

  for(auto env: {
      "IGN_GAZEBO_RESOURCE_PATH",
      "GZ_SIM_RESOURCE_PATH",
      "GAZEBO_MODEL_PATH",
      "SDF_PATH"})
  {
    const auto paths{std::getenv(env)};
    if(!paths)
      continue;

    for(const auto &path: split(paths, ":"))
    {
      if(fs::exists(path))
        gz_paths.push_back(path);
    }
  }
  return gz_paths;
}

std::string resolveURI(const std::string &uri, const fs::path &base = "")
{
  if(fs::path(uri).is_absolute())
    return uri;

  static const auto paths{sdf_paths::paths()};

  if(fs::path(uri).extension() == ".sdf" || fs::path(uri).extension() == ".world")
  {
    for(auto &path: paths)
    {
      if(fs::exists(path / uri))
        return path / uri;
    }
  }

  // identify uri type
  const auto sep{uri.find("://")};

  // relative file
  if(sep == uri.npos && fs::path(uri).has_extension())
    return base / uri;

  const auto getFile = [&](const fs::path &model, const fs::path &file)
  {
    if(file.has_extension())
      return model / file;
    return model / "model.sdf";
  };

  const auto relPath = [](const std::vector<std::string> &elems, size_t start)
  {
    if(start >= elems.size())
      return fs::path{};
    fs::path rel{elems[start]};
    for(size_t sub = start+1; sub < elems.size(); ++sub)
      rel /= elems[sub];
    return rel;
  };

  // split into elements
  const auto type{uri.substr(0, sep)};
  const auto elems{split(sep == uri.npos ? uri : uri.substr(sep+3, uri.npos), "/")};

  if(type == "http" || type == "https")
  {
    static const fs::path fuel_root{std::string(std::getenv("HOME")) + "/" + GZ_SIM_HOME_PATH};
    const fs::path fuel{fuel_root / elems[0] / elems[1] / elems[2] / elems[3]};

    if(!fs::exists(fuel))
      return "";

    for(const auto &model: fs::directory_iterator(fuel))
    {
      if(model.is_directory() && fs::exists(model.path() / "model.config"))
        return getFile(model.path(), relPath(elems, 4));
    }
  }
  else
  {
    // find this model
    for(auto path: paths)
    {
      if(fs::exists(path / elems[0] / "model.config"))
        return getFile(path / elems[0], relPath(elems, 1));
    }
  }

  std::cerr << "Could not resolve " << uri << std::endl;
  return "";
}

std::string readWorld(const std::string &path)
{
  const auto abs_world{sdf_paths::resolveURI(path)};
  if(abs_world.empty())
    return {};
  std::ifstream in(abs_world.c_str());
  std::stringstream content;
  content << in.rdbuf();
  return content.str();
}
}


}




#endif