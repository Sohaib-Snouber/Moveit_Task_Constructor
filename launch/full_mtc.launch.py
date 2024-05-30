from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("robot").to_dict()

    # MTC Demo node
    pick_place_full = Node(
        package="mtc_tutorial",
        executable="full_mtc",
        output="screen",
        parameters=[
            moveit_config,
        ],
    )

    return LaunchDescription([pick_place_full])