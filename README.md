# ROS <-> GZ bridge for Gazebo worlds

The `world_bridge` node parses a Gazebo world file and publishes the corresponding poses.

The whole world is converted to a single URDF published on `/gz_world_bridge/robot_description`.

Poses of the models are retrieved from Gazebo topics. They are published on the ROS 2 side either as `tf` or as `PoseStamped`.
Only the transforms between the world and the root frame of the models is bridged.
The bridge does not consider internal, non fixed joints. It is up to another bridge to deal with such values (e.g. JointState).

Several parameters allow tuning the behavior:

- `namespace` (default `gz_world_bridge`): namespace of the robot description and pose topic
- `file`: the path to the world file (relative to Gazebo resource path, if needed)
- `world`: the content of a world (SDF xml), if `file` is not set
- `only_static` (default `False`): will only include the static models in the overall `robot_description`. World pose of the root link of non-static models are still published and can act e.g. as ground truth.
- `use_tf` (default `False`): publish models poses on `/tf`. If `False`, publish the world poses on `/<namespace>/poses` as `PoseStamped`
- `ignored` (list of string, default empty): models of the world that should be ignored

If both `file` and `world` are empty, the node will build the world description on-the-fly through a parser included in the `simple_launch` package.
