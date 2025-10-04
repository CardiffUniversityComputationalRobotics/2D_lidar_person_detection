from launch_ros.actions import Node
from launch import LaunchDescription

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Contains all of the nodes to be launched"""

    dr_spaam_ros_dir = FindPackageShare(package="dr_spaam_ros").find("dr_spaam_ros")

    # ===========================================
    # ! NODES
    # ===========================================
    dr_spaam_ros_node = Node(
        package="dr_spaam_ros",
        executable="dr_spaam_ros_node",
        name="dr_spaam_ros_node",
        output="screen",
        parameters=[
            dr_spaam_ros_dir + "/config/dr_spaam_ros.yaml",
            dr_spaam_ros_dir + "/config/topics.yaml",
        ],
    )

    # ===========================================
    # ! LAUNCH DESCRIPTION DECLARATION
    # ===========================================
    ld = LaunchDescription()

    # ===========================================
    # ! ADDING NODES
    # ===========================================
    ld.add_action(dr_spaam_ros_node)

    return ld
