import launch
import launch_ros.actions

def generate_launch_description():
    # 创建一个LaunchDescription对象
    ld = launch.LaunchDescription()

    # 创建第一个节点的启动描述
    client_node = launch_ros.actions.Node(
        package='mocap-ros2',
        executable='client_node',
        name='mocap_client_node',
        output='screen'
    )

    # 创建第二个节点的启动描述
    transform_node = launch_ros.actions.Node(
        package='frame_transform',
        executable='mocap_to_visual_odometry',
        name='mocap_to_visual_odometry_node',
        output='screen'
    )

    # 将这两个节点添加到LaunchDescription中
    ld.add_action(client_node)
    ld.add_action(transform_node)

    return ld
