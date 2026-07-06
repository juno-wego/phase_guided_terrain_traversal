import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _equals(name: str, value: str) -> IfCondition:
    return IfCondition(
        PythonExpression(["'", LaunchConfiguration(name), "' == '", value, "'"])
    )


def generate_launch_description():
    point_lio_share = get_package_share_directory("point_lio")
    elevation_mapping_share = get_package_share_directory("elevation_mapping")
    grid_map_filter_share = get_package_share_directory("grid_map_filter_node")
    go2_description_share = get_package_share_directory("go2_description")

    point_lio_config = os.path.join(point_lio_share, "config", "unilidar_l1.yaml")
    elevation_mapping_config_dir = os.path.join(elevation_mapping_share, "config")
    go2_description_path = os.path.join(go2_description_share, "urdf", "go2_description.urdf")
    grid_map_filter_config = os.path.join(
        grid_map_filter_share,
        "config",
        "filters_demo.yaml",
    )
    elevation_mapping_rviz = os.path.join(
        elevation_mapping_share,
        "rviz2",
        "go2_elevation_mapping.rviz",
    )
    aurora_ld_library_path = ":".join([
        "/home/wego/dddmr_navigation/src/go2/aurora_ros/aurora_remote_public/lib/linux_x86_64",
        os.environ.get("LD_LIBRARY_PATH", ""),
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "localization_source",
            default_value="aurora",
            description="Which odometry source to use: 'aurora' or 'pointlio'.",
        ),
        DeclareLaunchArgument(
            "use_go2_driver",
            default_value="true",
            description="Launch the Go2 DDS bridge that publishes /go2/lidar_points and /go2/imu.",
        ),
        DeclareLaunchArgument(
            "go2_interface",
            default_value="enp88s0",
            description="Network interface used by go2_driver to connect to the robot.",
        ),
        DeclareLaunchArgument(
            "use_transform_sensors",
            default_value="false",
            description="Run the legacy utlidar transform node before feeding elevation mapping.",
        ),
        DeclareLaunchArgument(
            "lid_topic",
            default_value="/utlidar/transformed_cloud",
            description="PointCloud2 topic consumed by Point-LIO.",
        ),
        DeclareLaunchArgument(
            "imu_topic",
            default_value="/go2/imu",
            description="IMU topic consumed by Point-LIO.",
        ),
        DeclareLaunchArgument(
            "elevation_cloud_topic",
            default_value="/go2/lidar_points",
            description="PointCloud2 topic consumed by elevation mapping in aurora mode.",
        ),
        DeclareLaunchArgument(
            "ignore_points_inside_min_x",
            default_value="0.0",
            description="Remove points inside a base-frame exclusion box: min x.",
        ),
        DeclareLaunchArgument(
            "ignore_points_inside_max_x",
            default_value="0.0",
            description="Remove points inside a base-frame exclusion box: max x.",
        ),
        DeclareLaunchArgument(
            "ignore_points_inside_min_y",
            default_value="0.0",
            description="Remove points inside a base-frame exclusion box: min y.",
        ),
        DeclareLaunchArgument(
            "ignore_points_inside_max_y",
            default_value="0.0",
            description="Remove points inside a base-frame exclusion box: max y.",
        ),
        DeclareLaunchArgument(
            "ignore_points_inside_min_z",
            default_value="0.0",
            description="Remove points inside a base-frame exclusion box: min z.",
        ),
        DeclareLaunchArgument(
            "ignore_points_inside_max_z",
            default_value="0.0",
            description="Remove points inside a base-frame exclusion box: max z.",
        ),
        DeclareLaunchArgument(
            "robot_pose_topic",
            default_value="/slamware_ros_sdk_server_node/odom",
            description="Odometry topic consumed by elevation mapping in aurora mode.",
        ),
        DeclareLaunchArgument(
            "aurora_ip",
            default_value="192.168.11.1",
            description="IP address of the Aurora device.",
        ),
        DeclareLaunchArgument(
            "aurora_map_frame",
            default_value="aurora_odom",
            description="World frame published by Aurora.",
        ),
        DeclareLaunchArgument(
            "aurora_base_frame",
            default_value="aurora_base",
            description="Robot frame published by Aurora.",
        ),
        DeclareLaunchArgument(
            "aurora_to_base_x",
            default_value="-0.9",
            description="Static transform X offset from Aurora base frame to base_footprint.",
        ),
        DeclareLaunchArgument(
            "aurora_to_base_y",
            default_value="-0.1",
            description="Static transform Y offset from Aurora base frame to base_footprint.",
        ),
        DeclareLaunchArgument(
            "aurora_to_base_z",
            default_value="-0.4",
            description="Static transform Z offset from Aurora base frame to base_footprint.",
        ),
        DeclareLaunchArgument(
            "aurora_to_base_roll",
            default_value="0",
            description="Static transform roll from Aurora base frame to base_footprint.",
        ),
        DeclareLaunchArgument(
            "aurora_to_base_pitch",
            default_value="0",
            description="Static transform pitch from Aurora base frame to base_footprint.",
        ),
        DeclareLaunchArgument(
            "aurora_to_base_yaw",
            default_value="0",
            description="Static transform yaw from Aurora base frame to base_footprint.",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="Launch RViz with the elevation mapping view.",
        ),
        DeclareLaunchArgument(
            "heightmap_output",
            default_value="/elevation_heightmap",
            description="Output PointCloud2 topic for the policy heightmap.",
        ),
        DeclareLaunchArgument(
            "heightmap_input",
            default_value="/elevation_map",
            description="GridMap topic sampled by the policy heightmap exporter.",
        ),
        DeclareLaunchArgument(
            "heightmap_layer",
            default_value="elevation",
            description="GridMap layer sampled by the policy heightmap exporter.",
        ),
        DeclareLaunchArgument(
            "use_grid_map_filter",
            default_value="false",
            description="Run the optional grid-map filter node that publishes /filtered_elevation_map.",
        ),
        DeclareLaunchArgument(
            "heightmap_rows",
            default_value="11",
            description="Number of forward samples in the exported heightmap.",
        ),
        DeclareLaunchArgument(
            "heightmap_cols",
            default_value="9",
            description="Number of lateral samples in the exported heightmap.",
        ),
        DeclareLaunchArgument(
            "heightmap_dx",
            default_value="0.1",
            description="Forward spacing between heightmap samples in meters.",
        ),
        DeclareLaunchArgument(
            "heightmap_dy",
            default_value="0.1",
            description="Lateral spacing between heightmap samples in meters.",
        ),
        DeclareLaunchArgument(
            "heightmap_forward_offset",
            default_value="0.0",
            description="Forward shift of the exported heightmap center in robot frame meters.",
        ),
        DeclareLaunchArgument(
            "heightmap_lateral_offset",
            default_value="0.0",
            description="Lateral shift of the exported heightmap center in robot frame meters.",
        ),
        DeclareLaunchArgument(
            "map_length_x",
            default_value="3.0",
            description="Elevation map length in X in meters.",
        ),
        DeclareLaunchArgument(
            "map_length_y",
            default_value="3.0",
            description="Elevation map length in Y in meters.",
        ),
        DeclareLaunchArgument(
            "map_resolution",
            default_value="0.02",
            description="Elevation map resolution in meters.",
        ),
        DeclareLaunchArgument(
            "map_frame",
            default_value="start",
            description="Global frame used by elevation mapping, RViz, and heightmap export.",
        ),
        DeclareLaunchArgument(
            "track_point_frame",
            default_value="base_footprint",
            description="Frame whose XY position the elevation map follows. Set to the map frame for a fixed global map.",
        ),
        DeclareLaunchArgument(
            "visibility_cleanup_enabled",
            default_value="false",
            description="Enable elevation-map visibility cleanup.",
        ),
        DeclareLaunchArgument(
            "visibility_cleanup_rate",
            default_value="0.0",
            description="Visibility cleanup rate in Hz. Zero disables the cleanup timer.",
        ),
        DeclareLaunchArgument(
            "enable_continuous_cleanup",
            default_value="false",
            description="Clear the map before each new scan update.",
        ),
        DeclareLaunchArgument(
            "scanning_duration",
            default_value="1.0",
            description="How long a scan remains protected from cleanup and multi-height rejection.",
        ),
        DeclareLaunchArgument(
            "lio_world_frame",
            default_value="map",
            description="Parent frame published by Point-LIO odometry.",
        ),
        DeclareLaunchArgument(
            "lio_body_frame",
            default_value="base_link",
            description="Child frame published by Point-LIO odometry.",
        ),
        DeclareLaunchArgument(
            "base_frame",
            default_value="base_footprint",
            description="Robot base frame used by elevation mapping and heightmap export.",
        ),
        Node(
            package="go2_base",
            executable="go2_driver",
            name="go2_driver",
            output="screen",
            arguments=[LaunchConfiguration("go2_interface")],
            remappings=[
                ("odom", "/go2/wheel_odom"),
                ("/tf", "/go2/tf"),
            ],
            condition=IfCondition(LaunchConfiguration("use_go2_driver")),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": open(go2_description_path, "r", encoding="utf-8").read(),
            }],
            condition=_equals("localization_source", "aurora"),
        ),
        Node(
            package="transform_sensors",
            executable="transform_everything",
            name="sensor_transformer",
            output="screen",
            remappings=[
                ("/utlidar/cloud", "/go2/lidar_points"),
                ("/utlidar/imu", "/go2/imu"),
            ],
            condition=IfCondition(LaunchConfiguration("use_transform_sensors")),
        ),
        Node(
            package="point_lio",
            executable="pointlio_mapping",
            name="laserMapping",
            output="screen",
            condition=_equals("localization_source", "pointlio"),
            parameters=[
                point_lio_config,
                {
                    "odom_header_frame_id": LaunchConfiguration("lio_world_frame"),
                    "odom_child_frame_id": LaunchConfiguration("lio_body_frame"),
                    "common.lid_topic": LaunchConfiguration("lid_topic"),
                    "common.imu_topic": LaunchConfiguration("imu_topic"),
                    "use_imu_as_input": False,
                    "prop_at_freq_of_imu": True,
                    "check_satu": True,
                    "init_map_size": 10,
                    "point_filter_num": 1,
                    "space_down_sample": True,
                    "filter_size_surf": 0.1,
                    "filter_size_map": 0.1,
                    "cube_side_length": 1000.0,
                    "runtime_pos_log_enable": False,
                    "odometry.publish_odometry_without_downsample": False,
                },
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="start_to_map_tf",
            output="screen",
            arguments=["0", "0", "0", "0", "0", "0", "start", "map"],
            condition=_equals("localization_source", "pointlio"),
        ),
        Node(
            package="slamware_ros_sdk",
            executable="slamware_ros_sdk_server_node",
            name="slamware_ros_sdk_server_node",
            output="screen",
            condition=_equals("localization_source", "aurora"),
            additional_env={"LD_LIBRARY_PATH": aurora_ld_library_path},
            parameters=[{
                "ip_address": LaunchConfiguration("aurora_ip"),
                "angle_compensate": True,
                "map_frame": LaunchConfiguration("aurora_map_frame"),
                "robot_frame": LaunchConfiguration("aurora_base_frame"),
                "odom_frame": LaunchConfiguration("aurora_map_frame"),
                "laser_frame": "aurora_laser",
                "imu_frame": "aurora_imu_link",
                "camera_left": "aurora_camera_left",
                "camera_right": "aurora_camera_right",
                "robot_pose_pub_period": 0.01,
                "odometry_pub_period": 0.01,
                "map_pub_period": 0.2,
                "point_cloud_pub_period": 0.1,
                "imu_raw_data_period": 0.005,
                "ladar_data_clockwise": True,
                "robot_pose_topic": "/aurora_robot_pose",
                "odom_topic": "/slamware_ros_sdk_server_node/odom",
                "point_cloud_topic_name": "/slamware_ros_sdk_server_node/point_cloud",
            }],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="start_to_aurora_tf",
            output="screen",
            arguments=[
                "0",
                "0",
                "0",
                "0",
                "0",
                "0",
                "start",
                LaunchConfiguration("aurora_map_frame"),
            ],
            condition=_equals("localization_source", "aurora"),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="aurora_base_to_go2_base_footprint",
            output="screen",
            arguments=[
                "--x", LaunchConfiguration("aurora_to_base_x"),
                "--y", LaunchConfiguration("aurora_to_base_y"),
                "--z", LaunchConfiguration("aurora_to_base_z"),
                "--roll", LaunchConfiguration("aurora_to_base_roll"),
                "--pitch", LaunchConfiguration("aurora_to_base_pitch"),
                "--yaw", LaunchConfiguration("aurora_to_base_yaw"),
                "--frame-id", LaunchConfiguration("aurora_base_frame"),
                "--child-frame-id", "base_footprint",
            ],
            condition=_equals("localization_source", "aurora"),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="aurora_laser_to_base_tf",
            output="screen",
            arguments=[
                "0",
                "0",
                "0.0315",
                "0",
                "0",
                "0",
                "1",
                LaunchConfiguration("aurora_base_frame"),
                "aurora_laser",
            ],
            condition=_equals("localization_source", "aurora"),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="aurora_left_cam_to_base_tf",
            output="screen",
            arguments=[
                "0.0418",
                "0.03",
                "0",
                "-0.5",
                "0.5",
                "-0.5",
                "0.5",
                LaunchConfiguration("aurora_base_frame"),
                "aurora_camera_left",
            ],
            condition=_equals("localization_source", "aurora"),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="aurora_right_cam_to_left_cam_tf",
            output="screen",
            arguments=[
                "0.06",
                "0",
                "0",
                "0",
                "0",
                "0",
                "1",
                "aurora_camera_left",
                "aurora_camera_right",
            ],
            condition=_equals("localization_source", "aurora"),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="aurora_imu_to_left_cam_tf",
            output="screen",
            arguments=[
                "0.03",
                "0",
                "0",
                "0",
                "0",
                "-0.7071068",
                "0.7071068",
                "aurora_camera_left",
                "aurora_imu_link",
            ],
            condition=_equals("localization_source", "aurora"),
        ),
        Node(
            package="elevation_mapping",
            executable="elevation_mapping",
            name="elevation_mapping",
            output="screen",
            condition=_equals("localization_source", "pointlio"),
            parameters=[
                os.path.join(elevation_mapping_config_dir, "robots", "ground_truth_demo.yaml"),
                os.path.join(elevation_mapping_config_dir, "elevation_maps", "long_range.yaml"),
                os.path.join(elevation_mapping_config_dir, "sensor_processors", "perfect.yaml"),
                {
                    "ground_truth_cloud.topic": "/cloud_registered",
                    "robot_pose_with_covariance_topic": "/aft_mapped_to_init",
                    "map_frame_id": LaunchConfiguration("map_frame"),
                    "robot_base_frame_id": "base_link",
                    "ground_truth_cloud.sensor_processor.ignore_points_inside_min_x": ParameterValue(
                        LaunchConfiguration("ignore_points_inside_min_x"),
                        value_type=float,
                    ),
                    "ground_truth_cloud.sensor_processor.ignore_points_inside_max_x": ParameterValue(
                        LaunchConfiguration("ignore_points_inside_max_x"),
                        value_type=float,
                    ),
                    "ground_truth_cloud.sensor_processor.ignore_points_inside_min_y": ParameterValue(
                        LaunchConfiguration("ignore_points_inside_min_y"),
                        value_type=float,
                    ),
                    "ground_truth_cloud.sensor_processor.ignore_points_inside_max_y": ParameterValue(
                        LaunchConfiguration("ignore_points_inside_max_y"),
                        value_type=float,
                    ),
                    "ground_truth_cloud.sensor_processor.ignore_points_inside_min_z": ParameterValue(
                        LaunchConfiguration("ignore_points_inside_min_z"),
                        value_type=float,
                    ),
                    "ground_truth_cloud.sensor_processor.ignore_points_inside_max_z": ParameterValue(
                        LaunchConfiguration("ignore_points_inside_max_z"),
                        value_type=float,
                    ),
                    "track_point_frame_id": LaunchConfiguration("track_point_frame"),
                    "target_frame_init_submap": LaunchConfiguration("base_frame"),
                    "length_in_x": ParameterValue(
                        LaunchConfiguration("map_length_x"),
                        value_type=float,
                    ),
                    "length_in_y": ParameterValue(
                        LaunchConfiguration("map_length_y"),
                        value_type=float,
                    ),
                    "resolution": ParameterValue(
                        LaunchConfiguration("map_resolution"),
                        value_type=float,
                    ),
                    "time_tolerance": 0.2,
                    "min_update_rate": 10.0,
                    "fused_map_publishing_rate": 15.0,
                    "publish_heightmap": False,
                    "relocate_rate": 10.0,
                    "visibility_cleanup_rate": ParameterValue(
                        LaunchConfiguration("visibility_cleanup_rate"),
                        value_type=float,
                    ),
                    "enable_visibility_cleanup": ParameterValue(
                        LaunchConfiguration("visibility_cleanup_enabled"),
                        value_type=bool,
                    ),
                    "enable_continuous_cleanup": ParameterValue(
                        LaunchConfiguration("enable_continuous_cleanup"),
                        value_type=bool,
                    ),
                    "scanning_duration": ParameterValue(
                        LaunchConfiguration("scanning_duration"),
                        value_type=float,
                    ),
                },
            ],
        ),
        Node(
            package="elevation_mapping",
            executable="elevation_mapping",
            name="elevation_mapping",
            output="screen",
            condition=_equals("localization_source", "aurora"),
            parameters=[
                os.path.join(elevation_mapping_config_dir, "robots", "ground_truth_demo.yaml"),
                os.path.join(elevation_mapping_config_dir, "elevation_maps", "long_range.yaml"),
                os.path.join(elevation_mapping_config_dir, "sensor_processors", "perfect.yaml"),
                {
                    "ground_truth_cloud.topic": LaunchConfiguration("elevation_cloud_topic"),
                    "robot_pose_with_covariance_topic": LaunchConfiguration("robot_pose_topic"),
                    "map_frame_id": LaunchConfiguration("map_frame"),
                    "robot_base_frame_id": "base_link",
                    "ground_truth_cloud.sensor_processor.ignore_points_inside_min_x": ParameterValue(
                        LaunchConfiguration("ignore_points_inside_min_x"),
                        value_type=float,
                    ),
                    "ground_truth_cloud.sensor_processor.ignore_points_inside_max_x": ParameterValue(
                        LaunchConfiguration("ignore_points_inside_max_x"),
                        value_type=float,
                    ),
                    "ground_truth_cloud.sensor_processor.ignore_points_inside_min_y": ParameterValue(
                        LaunchConfiguration("ignore_points_inside_min_y"),
                        value_type=float,
                    ),
                    "ground_truth_cloud.sensor_processor.ignore_points_inside_max_y": ParameterValue(
                        LaunchConfiguration("ignore_points_inside_max_y"),
                        value_type=float,
                    ),
                    "ground_truth_cloud.sensor_processor.ignore_points_inside_min_z": ParameterValue(
                        LaunchConfiguration("ignore_points_inside_min_z"),
                        value_type=float,
                    ),
                    "ground_truth_cloud.sensor_processor.ignore_points_inside_max_z": ParameterValue(
                        LaunchConfiguration("ignore_points_inside_max_z"),
                        value_type=float,
                    ),
                    "track_point_frame_id": LaunchConfiguration("track_point_frame"),
                    "target_frame_init_submap": LaunchConfiguration("base_frame"),
                    "length_in_x": ParameterValue(
                        LaunchConfiguration("map_length_x"),
                        value_type=float,
                    ),
                    "length_in_y": ParameterValue(
                        LaunchConfiguration("map_length_y"),
                        value_type=float,
                    ),
                    "resolution": ParameterValue(
                        LaunchConfiguration("map_resolution"),
                        value_type=float,
                    ),
                    "time_tolerance": 0.2,
                    "min_update_rate": 10.0,
                    "fused_map_publishing_rate": 15.0,
                    "publish_heightmap": False,
                    "relocate_rate": 10.0,
                    "visibility_cleanup_rate": ParameterValue(
                        LaunchConfiguration("visibility_cleanup_rate"),
                        value_type=float,
                    ),
                    "enable_visibility_cleanup": ParameterValue(
                        LaunchConfiguration("visibility_cleanup_enabled"),
                        value_type=bool,
                    ),
                    "enable_continuous_cleanup": ParameterValue(
                        LaunchConfiguration("enable_continuous_cleanup"),
                        value_type=bool,
                    ),
                    "scanning_duration": ParameterValue(
                        LaunchConfiguration("scanning_duration"),
                        value_type=float,
                    ),
                },
            ],
        ),
        Node(
            package="grid_map_filter_node",
            executable="filters_demo_node",
            name="grid_map_filters_node",
            output="screen",
            parameters=[grid_map_filter_config],
            condition=IfCondition(LaunchConfiguration("use_grid_map_filter")),
        ),
        Node(
            package="heightmap_node",
            executable="heightmap_node",
            name="heightmap_node",
            output="screen",
            parameters=[{
                "input": LaunchConfiguration("heightmap_input"),
                "output": LaunchConfiguration("heightmap_output"),
                "layer": LaunchConfiguration("heightmap_layer"),
                "map_frame": LaunchConfiguration("map_frame"),
                "base_frame": LaunchConfiguration("base_frame"),
                "num_heightscans": ParameterValue(
                    LaunchConfiguration("heightmap_rows"),
                    value_type=int,
                ),
                "num_widthscans": ParameterValue(
                    LaunchConfiguration("heightmap_cols"),
                    value_type=int,
                ),
                "dist_x": ParameterValue(
                    LaunchConfiguration("heightmap_dx"),
                    value_type=float,
                ),
                "dist_y": ParameterValue(
                    LaunchConfiguration("heightmap_dy"),
                    value_type=float,
                ),
                "forward_offset": ParameterValue(
                    LaunchConfiguration("heightmap_forward_offset"),
                    value_type=float,
                ),
                "lateral_offset": ParameterValue(
                    LaunchConfiguration("heightmap_lateral_offset"),
                    value_type=float,
                ),
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz",
            output="screen",
            arguments=["--display-config", elevation_mapping_rviz],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])
