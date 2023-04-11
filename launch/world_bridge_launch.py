from simple_launch import SimpleLauncher


def generate_launch_description():
    
    sl = SimpleLauncher()

    sl.declare_arg('rviz',True)

    # bridge params
    sl.declare_arg('file','some.world', description = 'File to Gz world')
    sl.declare_arg('world','', description = 'Content of Gz world (optional)')
    sl.declare_arg('use_static',True, description = 'Publish <static> models as tf_static')
    sl.declare_arg('use_tf',False, description = 'Bridge poses as /tf instead of separate pose topic')

    sl.node('ros_gz_world_bridge','world_bridge', parameters = sl.arg_map('file','world','use_static','use_tf'))

    if sl.arg('rviz'):
        sl.rviz(sl.find('ros_gz_world_bridge', 'world_bridge.rviz'))

    return sl.launch_description()
