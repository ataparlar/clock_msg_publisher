
import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
import yaml

def generate_launch_description():

    # with open(vehicle_cmd_gate_param_path, "r") as f:
    #     vehicle_cmd_gate_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    bag_file_param = DeclareLaunchArgument(
        "bag_file" #, default_value=TextSubstitution(text="0")
    )
    # topic_param = DeclareLaunchArgument(
    #     "topic" #, default_value=TextSubstitution(text="0")
    # )

    clock_pub_params = {
        "bag_file": bag_file_param,
        # "topic": topic_param
    }

    return LaunchDescription([
        Node(
            package='clock_msg_publisher',
            executable='clock_msg_pub',
            name='clock_msg_publisher',
            parameters=[clock_pub_params],
            output='screen'
        )

    ])