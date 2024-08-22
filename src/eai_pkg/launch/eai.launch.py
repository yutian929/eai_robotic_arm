# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node

# 定义函数名称为：generate_launch_description
def generate_launch_description():
    realsense_node = Node(
        package="eai_pkg",
        executable="realsense_node"
        )

    layout_node = Node(
        package="eai_pkg",
        executable="layout_node"
        )
    
    ai_node = Node(
        package="eai_pkg",
        executable="ai_node"
        )

    center_node = Node(
        package="eai_pkg",
        executable="center_node"
        )

    arm_node = Node(
        package="eai_pkg",
        executable="arm_node"
        )

    user_cmd_node = Node(
        package="eai_pkg",
        executable="user_cmd_node"
        )
    
    asrpro_node = Node(
        package="eai_pkg",
        executable="asrpro_node"
        )

    speaker_node = Node(
        package="eai_pkg",
        executable="speaker_node"
        )

    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription([realsense_node, layout_node, arm_node, user_cmd_node, asrpro_node, speaker_node, center_node])
    # 返回让ROS2根据launch描述执行节点
    return launch_description


# [asrpro_node-5] Traceback (most recent call last):
# [asrpro_node-5]   File "/home/eai/ros2_ws/eai4_ws/install/eai_pkg/lib/eai_pkg/asrpro_node", line 33, in <module>
# [asrpro_node-5]     sys.exit(load_entry_point('eai-pkg==0.0.0', 'console_scripts', 'asrpro_node')())
# [asrpro_node-5]   File "/home/eai/ros2_ws/eai4_ws/install/eai_pkg/lib/eai_pkg/asrpro_node", line 25, in importlib_load_entry_point
# [asrpro_node-5]     return next(matches).load()
# [asrpro_node-5] StopIteration
# [ERROR] [asrpro_node-5]: process has died [pid 214263, exit code 1, cmd '/home/eai/ros2_ws/eai4_ws/install/eai_pkg/lib/eai_pkg/asrpro_node --ros-args'].
