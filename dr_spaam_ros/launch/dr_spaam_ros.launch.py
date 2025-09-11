from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    """Contains all of the nodes to be launched"""

    # ===========================================
    # ! NODES
    # ===========================================
    dr_spaam_ros_node = (
        Node(
            package="dr_spaam_ros",
            executable="dr_spaam_ros_node",
            name="dr_spaam_ros_node",
            output="screen",
        ),
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
