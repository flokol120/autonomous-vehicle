
import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    physics = DeclareLaunchArgument(
            "physics",
            default_value = "bullet",
            description = "one of 'bullet', 'dart', 'ode', 'simbody'")

    world = DeclareLaunchArgument(
            "world",
            default_value = get_package_share_directory("gazebo_simulation") + "/resource/world/world.world",
            description = "path to the world file")

    model_path_env = SetEnvironmentVariable(name="GAZEBO_MODEL_PATH",
                                            value=get_package_share_directory("gazebo_simulation") + "/resource")

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("gazebo_ros"), "launch"), "/gazebo.launch.py"]),
             )

    gazebo_simulation_path = os.path.join(
        get_package_share_directory("gazebo_simulation"))

    xacro_file = f"{gazebo_simulation_path}/resource/models/car/car.urdf.xacro"

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {"robot_description": doc.toxml()}

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params]
    )

    spawn_entity = Node(package="gazebo_ros", executable="spawn_entity.py",
                        arguments=["-topic", "robot_description",
                                   "-entity", "car",
                                   "-x", "-4", "-y", "-8", "-z", "0.07"],
                        output="screen")

    load_joint_state_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "start",
             "joint_state_broadcaster"],
        output="screen")

    load_joint_velocity_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "start", "velocity_controller"],
        output="screen")

    load_joint_steering_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "start", "steering_controller"],
        output="screen")

    load_imu_sensor_broadcaster = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "start", "imu_sensor_broadcaster"],
        output="screen")

    ackermann_controller = Node(package="car_controller", executable="ackermann_controller",
                                output="screen")
    joy_node = Node(package="joy", executable="joy_node",
                                output="screen")

    return LaunchDescription([
        model_path_env,
        physics,
        world,
        gazebo,
        ackermann_controller,
        joy_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_velocity_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_steering_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_imu_sensor_broadcaster],
            )
        ),
        node_robot_state_publisher,
        spawn_entity,
    ])
