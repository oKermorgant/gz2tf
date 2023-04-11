# Republish pose on tf

This packages contains a single node that subscribes to a pose topic.

It will simply republish the pose as a transform on `/tf`. If the parent frame is not defined then `world` will be used.

It can be used to simulate perfect localization (on `/tf`) from a ground truth topic published by a simulator.

Supported messages are:
- `geometry_msgs/Pose`
- `geometry_msgs/PoseStamped`
- `geometry_msgs/Transform`
- `geometry_msgs/TransformStamped`
- `nav_msgs/Odometry`
- `sensor_msgs/Imu` (assumes null translation)


## Parameters

- `topic`: topic to subscribe to (defaults to `pose_gt`)
- `child_frame`: child frame to be used in tf publisher (defaults to `base_link`)
- `parent_frame`: parent frame to be used in tf publisher (defaults to `world`)

The frame parameters are only used if the corresponding information is not available in the message itself:
- `Pose` and `Transform` do not convey any frame
- `PoseStamped` and `Imu` only convey `child_frame` in the header
- `TransformStamped` and `Odometry` convey both `child_frame` explicitely and `parent_frame` in the header
