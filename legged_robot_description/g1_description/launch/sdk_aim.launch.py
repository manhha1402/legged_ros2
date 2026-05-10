import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    use_sim_str = LaunchConfiguration("use_sim").perform(context).lower()
    use_sim = (use_sim_str == "true")

    enable_lowlevel_write_str = LaunchConfiguration("enable_lowlevel_write").perform(context).lower()
    enable_lowlevel_write_bool = (enable_lowlevel_write_str == "true")

    prefix = LaunchConfiguration("prefix").perform(context)
    network_interface = LaunchConfiguration("network_interface").perform(context)
    domain_id = LaunchConfiguration("domain_id").perform(context)
    description_package = LaunchConfiguration("description_package").perform(context)
    description_file = LaunchConfiguration("description_file").perform(context)
    rl_policy = LaunchConfiguration("rl_policy").perform(context)
    controller_config = LaunchConfiguration("controller_config").perform(context)
    use_rviz = LaunchConfiguration("use_rviz")
    use_rqt_cm = LaunchConfiguration("use_rqt_cm")

    robot_description_content = Command([
        FindExecutable(name="xacro"), " ",
        PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
        " prefix:=", prefix,
        " use_sim:=", "true" if use_sim else "false",
        " network_interface:=", network_interface,
        " domain_id:=", domain_id,
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    rl_policy_path_param = {
        "rl_policy_path": os.path.join(
            FindPackageShare(description_package).perform(context),
            "config", "rl", rl_policy
        )
    }
    controller_config_path = PathJoinSubstitution([FindPackageShare(description_package), "config", controller_config])
    rviz_config_file = PathJoinSubstitution([FindPackageShare(description_package), "rviz2", "g1.rviz"])

    if use_sim:
        control_node_pkg = "controller_manager"
        control_node_exe = "ros2_control_node"
    else:
        control_node_pkg = "legged_ros2_control"
        control_node_exe = "g1_node"

    sim_joy_nodes = []
    if use_sim:
        joy_node = Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            parameters=[{"dev": "/dev/input/js0"}]  
        )
        joy_teleop_node = Node(
            package="g1_description", 
            executable="sim_joy_teleop.py",  
            output="screen"
        )
        sim_joy_nodes = [joy_node, joy_teleop_node]


    control_node = Node(
        package=control_node_pkg,
        executable=control_node_exe,
        parameters=[
            controller_config_path,
            robot_description,
            rl_policy_path_param,
            {
                "use_sim": use_sim,
                "use_gains": not use_sim,
                "network_interface": network_interface,
                "domain_id": int(domain_id),
                "enable_lowlevel_write": enable_lowlevel_write_bool,
            },
        ],
        remappings=[("~/robot_description", "/robot_description")],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster", "--controller-manager", "/controller_manager"],
    )

    static_args = ["static_controller", "--controller-manager", "/controller_manager"]
    if use_sim:
        static_args.append("--inactive")

    # static_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=static_args,
    # )

    rl_args = ["rl_controller_whole", "--controller-manager", "/controller_manager"]

    if use_sim:
        rl_args.append("--inactive")
    
    left_arm_args = ["left_arm_controller", "-c", "/controller_manager"]
    right_arm_args = ["right_arm_controller", "-c", "/controller_manager"]
    if not use_sim:
        left_arm_args.append("--inactive")
        right_arm_args.append("--inactive")

    left_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=left_arm_args,
    )
    right_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=right_arm_args,
    )

    rl_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=rl_args,
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    rqt_controller_manager = Node(
        package="rqt_controller_manager",
        executable="rqt_controller_manager",
        condition=IfCondition(use_rqt_cm),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(use_rviz),
    )

    # # --- Event Handlers ---
    # delay_joint_state_after_static = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=static_controller_spawner,
    #         on_exit=[joint_state_broadcaster_spawner],
    #     )
    # )
    
    delay_imu_after_joint_state = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[imu_sensor_broadcaster_spawner],
        )
    )
    
    delay_arm_after_imu = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=imu_sensor_broadcaster_spawner,
            on_exit=[left_arm_controller_spawner,right_arm_controller_spawner],
        )
    )

    delay_whole_after_aims = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=right_arm_controller_spawner,
            on_exit=[rl_controller_spawner],
        )
    )

    # Add gripper controller spawners
    left_inspire_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_inspire_gripper_controller", "--controller-manager", "/controller_manager"],
    )
    right_inspire_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_inspire_gripper_controller", "--controller-manager", "/controller_manager"],
    )
    delay_rviz_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    return [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_imu_after_joint_state,
        delay_arm_after_imu,
        delay_whole_after_aims,
        delay_rviz_after_jsb,
        rqt_controller_manager,
        left_inspire_gripper_controller_spawner,
        right_inspire_gripper_controller_spawner,
    ] + sim_joy_nodes


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument("use_sim", default_value="true"))
    declared_arguments.append(DeclareLaunchArgument("description_package", default_value="g1_description"))
    declared_arguments.append(
        DeclareLaunchArgument("description_file", default_value="g1_29dof_lock_waist_rev_1_0.urdf.xacro"))
    declared_arguments.append(DeclareLaunchArgument("rl_policy", default_value="policy_29.onnx"))
    declared_arguments.append(DeclareLaunchArgument("controller_config", default_value="rl_arm_controller.yaml"))
    declared_arguments.append(DeclareLaunchArgument("use_rviz", default_value="false"))
    declared_arguments.append(DeclareLaunchArgument("use_rqt_cm", default_value="true"))
    declared_arguments.append(DeclareLaunchArgument("prefix", default_value='""'))
    declared_arguments.append(DeclareLaunchArgument("network_interface", default_value="lo"))
    declared_arguments.append(
        DeclareLaunchArgument(
            "domain_id",
            default_value="0",
            description="Unitree DDS domain id; must match unitree_mujoco simulate/config.yaml or `./unitree_mujoco -i`",
        )
    )
    declared_arguments.append(DeclareLaunchArgument("enable_lowlevel_write", default_value="true"))

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])