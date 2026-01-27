from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="g1_description",
            description="Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="g1_29dof_lock_waist_rev_1_0.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_config", 
            default_value="rl_arm_controller.yaml",
            description="Controller configuration file. Use rl_arm_controller.yaml for \
                ros2 launch g1_description sdk.launch.py        27-joint whole-body RL control.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rqt_cm", 
            default_value="true",
            description="Start rqt_controller_manager automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "network_interface",
            default_value='lo',
            description="network_interface for Unitree SDK2",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "enable_lowlevel_write",
            default_value="true",
            description="Enable low-level command writing, useful in debugging or testing scenarios. \
                        If set to true, the robot will receive low-level commands from the controller.",
        )
    )

    # Initialize Arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    controller_config = LaunchConfiguration("controller_config")
    use_rviz = LaunchConfiguration("use_rviz")
    use_rqt_cm = LaunchConfiguration("use_rqt_cm")
    prefix = LaunchConfiguration("prefix")
    network_interface = LaunchConfiguration("network_interface")
    enable_lowlevel_write = LaunchConfiguration("enable_lowlevel_write")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ", 
            "prefix:=", 
            prefix,
            " ", 
            "network_interface:=",
            network_interface,
        ]
    )
    robot_description = {"robot_description": robot_description_content}


    controller_params = {
        "network_interface": network_interface,
        "enable_lowlevel_write": enable_lowlevel_write
    }

    controller_config_path = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "config",
            controller_config
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz2", "g1.rviz"]
    )

    control_node = Node(
        package="legged_ros2_control",
        executable="g1_node",
        parameters=[controller_config_path, robot_description, controller_params],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(use_rviz),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster", "--controller-manager", "/controller_manager"],
    )

    static_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["static_controller", "-c", "/controller_manager", "--inactive"],
    )

    rl_controller_legs_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "rl_controller_legs",
            "-c",
            "/controller_manager",
            "--inactive",
        ],
    )

    rl_controller_whole_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "rl_controller_whole",
            "-c",
            "/controller_manager",
            "--inactive",
        ],
    )

    # # NOTE: Arm trajectory controllers are only available when using rl_arm_controller.yaml
    # # with a legs-only policy (e.g., policy_12.onnx with matching observation config)
    # left_arm_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["left_arm_controller", "-c", "/controller_manager"],
    # )
    # right_arm_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["right_arm_controller", "-c", "/controller_manager"],
    # )

    rqt_controller_manager = Node(
        package="rqt_controller_manager",
        executable="rqt_controller_manager",
        condition=IfCondition(use_rqt_cm),
    )

    # rqt_robot_steering = Node(
    #     package="rqt_robot_steering",
    #     executable="rqt_robot_steering",
    # )

    # Chain spawners sequentially to avoid CycloneDDS participant exhaustion
    # static_controller -> joint_state_broadcaster -> imu_sensor_broadcaster
    # -> rl_controller_legs -> rl_controller_whole
    
    delay_joint_state_after_static = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=static_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    
    delay_imu_after_joint_state = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[imu_sensor_broadcaster_spawner],
        )
    )
    
    delay_rl_whole_after_imu = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=imu_sensor_broadcaster_spawner,
            on_exit=[rl_controller_whole_spawner],
        )
    )

    # delay_rl_whole_after_legs = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=rl_controller_legs_spawner,
    #         on_exit=[rl_controller_whole_spawner],
    #     )
    # )
    
    # delay_left_arm_after_rl = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=rl_controller_whole_spawner,
    #         on_exit=[left_arm_controller_spawner],
    #     )
    # )
    # delay_right_arm_after_rl = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=rl_controller_whole_spawner,
    #         on_exit=[right_arm_controller_spawner],
    #     )
    # )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        static_controller_spawner,
        delay_joint_state_after_static,
        delay_imu_after_joint_state,
        delay_rl_whole_after_imu,
        # delay_rl_whole_after_legs,
        # delay_left_arm_after_rl,
        # delay_right_arm_after_rl,
        delay_rviz_after_joint_state_broadcaster_spawner,
        # rqt_controller_manager,
        # rqt_robot_steering
    ]

    # Configure CycloneDDS with the specified network interface
    # Use 'lo' for simulation, 'enp129s0' (or your interface) for real robot
    def setup_cyclonedds(context):
        net_if = LaunchConfiguration('network_interface').perform(context)
        return [
            SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
            SetEnvironmentVariable(
                'CYCLONEDDS_URI',
                f'<CycloneDDS><Domain><General><Interfaces>'
                f'<NetworkInterface name="{net_if}" priority="default" multicast="default" />'
                f'</Interfaces></General></Domain></CycloneDDS>'
            ),
        ]
    
    return LaunchDescription(
        declared_arguments + 
        # [OpaqueFunction(function=setup_cyclonedds)] + 
        nodes
    )




