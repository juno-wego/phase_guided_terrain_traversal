# wego_pgtt

Go2 실기에서 PGTT 정책 모드를 `서비스 한 번`으로 켜고, `서비스 한 번 더`로 끄는 패키지다.

정책을 끄면 `go2_driver -> /go2_motion_cmd -> sports mode` 경로를 우선 써서 리모컨 제어 복귀를 시도한다.

## 빠른 시작

터미널 1:

```bash
cd /home/wego/phase_guided_terrain_traversal/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch wego_pgtt go2_stair_policy_mode.launch.py
```

터미널 2:

```bash
source /opt/ros/humble/setup.bash
source /home/wego/phase_guided_terrain_traversal/ros_ws/install/setup.bash
ros2 service call /toggle_pgtt_mode std_srvs/srv/Trigger "{}"
```

위 명령을 한 번 더 호출하면 정책 모드가 종료된다.

```bash
ros2 service call /toggle_pgtt_mode std_srvs/srv/Trigger "{}"
```

## 자주 쓰는 명령

기본 실행:

```bash
ros2 launch wego_pgtt go2_stair_policy_mode.launch.py
```

기본 실행은 `리모컨 스틱 입력`을 정책 명령으로 사용한다.

정책 파일 바꿔서 실행:

```bash
ros2 launch wego_pgtt go2_stair_policy_mode.launch.py \
  policy_file:=/home/wego/phase_guided_terrain_traversal/policies/policy_go2_pgtt_level20_run0
```

`/cmd_vel` 입력으로 정책을 돌리고 싶으면:

```bash
ros2 launch wego_pgtt go2_stair_policy_mode.launch.py \
  command_source:=cmd_vel
```

## 바꾸는 인자만 정리

기본 인자 확인:

```bash
ros2 launch wego_pgtt go2_stair_policy_mode.launch.py --show-args
```

실제로 자주 바꾸는 값:

```text
policy_file
command_source
auto_start_go2_driver
go2_driver_setup
```

기본값:

```text
policy_file=/home/wego/phase_guided_terrain_traversal/policies/policy_go2_pgtt_level20_run0
command_source=controller
auto_start_go2_driver=true
go2_driver_setup=/home/wego/dddmr_navigation/install/setup.bash
```

`command_source` 의미:

```text
controller : Unitree 리모컨 스틱 값을 정책 명령으로 사용
cmd_vel    : ROS 2 /cmd_vel 값을 정책 명령으로 사용
```

복귀 관련 인자:

```text
auto_start_go2_driver : 종료 시 /go2_motion_cmd 가 없으면 go2_driver를 자동으로 띄움
go2_driver_setup      : dddmr_navigation install/setup.bash 경로
```

## 내부적으로 하는 일

서비스 서버는 내부적으로 아래 명령 형태로 정책을 실행한다.

```bash
python3 /home/wego/phase_guided_terrain_traversal/deploy/deploy_real.py \
  --robot go2 \
  --method pgtt \
  --policy-file /home/wego/phase_guided_terrain_traversal/policies/policy_go2_pgtt_level20_run0 \
  --command_type controller \
  --auto-arm \
  --no-prompt
```

직접 정책만 따로 테스트할 때:

```bash
cd /home/wego/phase_guided_terrain_traversal
python3 deploy/deploy_real.py \
  --robot go2 \
  --method pgtt \
  --policy-file /home/wego/phase_guided_terrain_traversal/policies/policy_go2_pgtt_level20_run0 \
  --command_type controller \
  --auto-arm \
  --no-prompt
```

`/cmd_vel` 모드 예시:

```bash
python3 deploy/deploy_real.py \
  --robot go2 \
  --method pgtt \
  --policy-file /home/wego/phase_guided_terrain_traversal/policies/policy_go2_pgtt_level20_run0 \
  --command_type cmd_vel \
  --cmd-vel-topic /cmd_vel \
  --auto-arm \
  --no-prompt
```

정책 실행 중 속도 명령을 `/cmd_vel`로 넣을 때:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.15, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10
```

## 종료 후 복귀

정책 모드를 끄면 아래 순서로 리모컨 제어 복귀를 시도한다.

```text
1. /go2_motion_cmd 서비스 확인
2. 없으면 go2_driver 자동 실행
3. SETSPORTSMODE
4. RECOVERY_STAND
5. STOP_MOVE
6. 그래도 실패하면 direct SDK fallback
```

go2_driver 자동 실행을 끄고 싶으면:

```bash
ros2 launch wego_pgtt go2_stair_policy_mode.launch.py \
  auto_start_go2_driver:=false
```

런치 자체를 종료할 때:

```bash
Ctrl+C
```

## Elevation Mapping

Go2 라이다 elevation map + heightmap 실행:

```bash
cd /home/wego/phase_guided_terrain_traversal/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch wego_pgtt go2_elevation_mapping.launch.py
```

RViz까지 같이 띄우려면:

```bash
ros2 launch wego_pgtt go2_elevation_mapping.launch.py rviz:=true
```

heightmap 출력 토픽을 바꾸려면:

```bash
ros2 launch wego_pgtt go2_elevation_mapping.launch.py \
  heightmap_output:=/elevation_heightmap
```

heightmap 크기와 샘플 간격을 바꾸려면:

```bash
ros2 launch wego_pgtt go2_elevation_mapping.launch.py \
  heightmap_rows:=11 \
  heightmap_cols:=9 \
  heightmap_dx:=0.1 \
  heightmap_dy:=0.1
```

현재 레포 기준 실행 체인:

```text
/utlidar/cloud, /utlidar/imu
-> transform_sensors
-> /utlidar/transformed_cloud, /utlidar/transformed_imu
-> point_lio
-> /cloud_registered, /aft_mapped_to_init
-> elevation_mapping
-> /elevation_map
-> heightmap_node
-> /elevation_heightmap
```

주의:

```text
- /utlidar/cloud, /utlidar/imu 는 다른 드라이버가 먼저 올리고 있어야 함
- heightmap_node 입력은 현재 코드 기준으로 /elevation_map, layer=elevation 사용
```

## 확인용 명령

노드 확인:

```bash
ros2 node list | grep wego_pgtt
```

서비스 확인:

```bash
ros2 service list | grep toggle_pgtt_mode
```

서비스 타입 확인:

```bash
ros2 service type /toggle_pgtt_mode
```
