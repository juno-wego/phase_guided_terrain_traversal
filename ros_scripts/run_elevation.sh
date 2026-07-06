#!/bin/bash

set -eo pipefail

cd "$(dirname "$0")/../ros_ws"
source /opt/ros/humble/setup.bash

if [ -f /home/wego/dddmr_navigation/install/setup.bash ]; then
  source /home/wego/dddmr_navigation/install/setup.bash
fi

pkill -f 'ros2 launch wego_pgtt|go2_driver|pointlio_mapping|elevation_mapping|transform_everything|heightmap_node|filters_demo_node|slamware_ros_sdk_server_node|start_to_map_tf|odom_to_base_link_tf|start_to_aurora_tf|aurora_base_to_base_link_tf|base_link_to_body_tf|aurora_laser_to_base_tf|aurora_left_cam_to_base_tf|aurora_right_cam_to_left_cam_tf|aurora_imu_to_left_cam_tf|rviz2' || true
sleep 1

colcon build --symlink-install --packages-select \
  transform_sensors \
  point_lio \
  elevation_mapping \
  heightmap_node \
  grid_map_filter_node \
  wego_pgtt

source install/local_setup.bash

exec ros2 launch wego_pgtt go2_elevation_mapping.launch.py \
  localization_source:=aurora \
  use_go2_driver:=true \
  use_transform_sensors:=false \
  elevation_cloud_topic:=/go2/lidar_points \
  ignore_points_inside_min_x:=-0.40 \
  ignore_points_inside_max_x:=0.25 \
  ignore_points_inside_min_y:=-0.30 \
  ignore_points_inside_max_y:=0.30 \
  ignore_points_inside_min_z:=-0.75 \
  ignore_points_inside_max_z:=0.20 \
  use_grid_map_filter:=false \
  heightmap_input:=/elevation_map \
  heightmap_layer:=elevation \
  heightmap_dx:=0.16 \
  map_length_x:=4.0 \
  map_length_y:=4.0 \
  map_resolution:=0.02 \
  visibility_cleanup_enabled:=false \
  visibility_cleanup_rate:=0.0 \
  enable_continuous_cleanup:=false \
  scanning_duration:=1.0 \
  rviz:=true
