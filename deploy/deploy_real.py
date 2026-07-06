import sys
import os
import signal
import time
import threading

for _sdk_path in [
    "/home/wego/go2_python_ws/install/unitree_sdk2py/lib/python3.10/site-packages",
    "/home/wego/go2_python_ws/src/unitree_sdk2_python",
    "/home/wego/unitree_sdk2_python",
]:
    if os.path.isdir(_sdk_path) and _sdk_path not in sys.path:
        sys.path.insert(0, _sdk_path)

import numpy as np
import torch 
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_

from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
from  unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_

from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.go2.sport.sport_client import SportClient
# import configs.unitree_legged_const as go2
from deploy.common.rotation_helper import get_gravity_orientation, transform_imu_data
from deploy.common.remote_controller import RemoteController, KeyMap
import deploy.policy_net as pn
# from sensor_msgs_py import point_cloud2
# from sensor_msgs.msg import PointCloud2
import copy
import array
from collections import namedtuple
import sys
from typing import Iterable, List, NamedTuple, Optional

import numpy as np
try:
    from numpy.lib.recfunctions import (structured_to_unstructured, unstructured_to_structured)
except ImportError:
    pass

_DATATYPES = {
    1: np.dtype(np.int8),
    2: np.dtype(np.uint8),
    3: np.dtype(np.int16),
    4: np.dtype(np.uint16),
    5: np.dtype(np.int32),
    6: np.dtype(np.uint32),
    7: np.dtype(np.float32),
    8: np.dtype(np.float64),
}
_DUMMY_FIELD_PREFIX = 'unnamed_field'


def dtype_from_fields(fields, point_step=None):
    field_names, field_offsets, field_datatypes = [], [], []
    for i, field in enumerate(fields):
        datatype = _DATATYPES[field.datatype]
        name = field.name if field.name else f'{_DUMMY_FIELD_PREFIX}_{i}'
        assert field.count > 0
        for a in range(field.count):
            subfield_name = f'{name}_{a}' if field.count > 1 else name
            field_names.append(subfield_name)
            field_offsets.append(field.offset + a * datatype.itemsize)
            field_datatypes.append(datatype.str)
    dtype_dict = {'names': field_names, 'formats': field_datatypes, 'offsets': field_offsets}
    if point_step is not None:
        dtype_dict['itemsize'] = point_step
    return np.dtype(dtype_dict)

import argparse

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
except ImportError:
    rclpy = None
    Node = None
    Twist = None

num_heightscans=11
num_widthscans=9

_parser = argparse.ArgumentParser(description="Deploy real robot")
_parser.add_argument('--robot', type=str, default='go2', help='Robot name: go2 or anymal')
_parser.add_argument('--method', type=str, default='pgtt', help='Method: pgtt, baseline, or wild')
_parser.add_argument('--level', type=str, default='level03', help='Terrain level: level03, level07, etc.')
_parser.add_argument('--run', type=int, default=0, help='Run number')
_parser.add_argument('--policy-file', type=str, default=None, help='Explicit policy file path')
_parser.add_argument('--command_type', type=str, default='hybrid', help='Command source: hybrid, controller, fixed, or cmd_vel')
_parser.add_argument('--vx', type=float, default=0.2, help='Forward speed when using fixed command')
_parser.add_argument('--vy', type=float, default=0.0, help='Lateral speed when using fixed command')
_parser.add_argument('--yaw', type=float, default=0.0, help='Yaw rate when using fixed command')
_parser.add_argument('--cmd-vel-topic', type=str, default='/cmd_vel', help='ROS 2 cmd_vel topic when using cmd_vel commands')
_parser.add_argument('--network', type=str, default=None, help='Network interface for ChannelFactoryInitialize')
_parser.add_argument('--auto-arm', action='store_true', help='Start the policy automatically after the stand-up sequence')
_parser.add_argument('--no-prompt', action='store_true', help='Skip the interactive safety prompt')
_parser.add_argument('--debug-inputs', action='store_true', help='Print policy input diagnostics periodically')
_parser.add_argument('--debug-print-hz', type=float, default=1.0, help='Diagnostic print rate when --debug-inputs is enabled')
_args, _ = _parser.parse_known_args()

mode = _args.method
command_type = _args.command_type
filename = _args.policy_file or f"policies/policy_{_args.robot}_{_args.method}_{_args.level}_run{_args.run}"
cmd_fixed = np.array([_args.vx, _args.vy, _args.yaw])
controller_deadband = 0.05

PHASES=np.array([0.,np.pi,np.pi,0.])
ctrl_dt=0.02
pd_dt=0.005
_policy_decimation = round(ctrl_dt / pd_dt)
freq=2.
reorder=[3,4,5,0,1,2,9,10,11,6,7,8]
PosStopF = 2.146e9
VelStopF = 16000.0
cmd_scale=np.array([0.5,0.5,0.8])


_GAIT_MODULE = None


def _get_gait_module():
    global _GAIT_MODULE
    if _GAIT_MODULE is None:
        import robots.gait as gait  # Only needed for the wild policy.
        _GAIT_MODULE = gait
    return _GAIT_MODULE


if Node is not None and Twist is not None:
    class CmdVelSubscriber(Node):
        def __init__(self, topic_name: str):
            super().__init__('pgtt_cmd_vel_listener')
            self._lock = threading.Lock()
            self._latest_cmd = np.zeros(3, dtype=float)
            self._last_msg_time = 0.0
            self._timeout_sec = 0.3
            self.create_subscription(Twist, topic_name, self.cmd_vel_callback, 10)

        def cmd_vel_callback(self, msg: Twist) -> None:
            with self._lock:
                self._latest_cmd[:] = [msg.linear.x, msg.linear.y, msg.angular.z]
                self._last_msg_time = time.monotonic()

        def get_velocity(self) -> np.ndarray:
            with self._lock:
                if self._last_msg_time <= 0.0:
                    return np.zeros(3, dtype=float)
                if time.monotonic() - self._last_msg_time > self._timeout_sec:
                    return np.zeros(3, dtype=float)
                return self._latest_cmd.copy()

        def get_status(self):
            with self._lock:
                if self._last_msg_time <= 0.0:
                    return False, None
                return True, time.monotonic() - self._last_msg_time
else:
    CmdVelSubscriber = None

def read_points(
        cloud,
        field_names: Optional[List[str]] = None,
        skip_nans: bool = False,
        uvs: Optional[Iterable] = None,
        reshape_organized_cloud: bool = False) -> np.ndarray:
    """
    Wrapper for ros2 point cloud and unitree point cloud. 
    Copied directly from point cloud library.
    """

    required_attrs = ['width', 'height', 'fields', 'point_step', 'data', 'is_bigendian', 'is_dense']
    for attr in required_attrs:
        assert hasattr(cloud, attr), f"Missing attribute: {attr}"

    if isinstance(cloud.data, list):
        buffer = bytes(cloud.data)
    else:
        buffer = cloud.data

    points = np.ndarray(
        shape=(cloud.width * cloud.height, ),
        dtype=dtype_from_fields(cloud.fields, point_step=cloud.point_step),
        buffer=buffer)


    if field_names is not None:
        assert all(field_name in points.dtype.names for field_name in field_names), \
            'Requests field is not in the fields of the PointCloud!'

        points = points[list(field_names)]


    if bool(sys.byteorder != 'little') != bool(cloud.is_bigendian):
        points = points.byteswap(inplace=True)

 
    if skip_nans and not cloud.is_dense:
        
        not_nan_mask = np.ones(len(points), dtype=bool)
        for field_name in points.dtype.names:
        
            not_nan_mask = np.logical_and(
                not_nan_mask, ~np.isnan(points[field_name]))
        
        points = points[not_nan_mask]

    
    if uvs is not None:
    
        if not isinstance(uvs, np.ndarray):
            uvs = np.fromiter(uvs, int)
    
        points = points[uvs]

    
    if reshape_organized_cloud and cloud.height > 1:
        points = points.reshape(cloud.width, cloud.height)

    return points
import cv2
import numpy as np


class Custom():

    def __init__(self):
        self.Kp = 60.0 #60.0 #20 #40
        self.Kd = 3. #2.0   #0.5  #1 
        self.time_consume = 0
        self.rate_count = 0
        self.sin_count = 0
        self.motiontime = 0

        self.low_cmd = unitree_go_msg_dds__LowCmd_()  
        self.low_state = None  
        self._targetPos_1 = [0.0, 1.36, -2.65, 0.0, 1.36, -2.65,
                             -0.2, 1.36, -2.65, 0.2, 1.36, -2.65]
        # self._targetPos_2 = [0.0, 0.67, -1.3, 0.0, 0.67, -1.3,
        #                     0.0, 0.67, -1.3, 0.0, 0.67, -1.3]

        self._targetPos_2= [0 ,0.9 ,-1.8 ,0 ,0.9, -1.8, 0 ,0.9, -1.8, 0 ,0.9, -1.8]
        self._targetPos_3= self._targetPos_2
        self._targetPos_3=self._targetPos_1
        # self._targetPos_3 = [0.0, 1.36, -2.65, 0.0, 1.36, -2.65,
        #                      -0.2, 1.36, -2.65, 0.2, 1.36, -2.65]

        self.startPos = [0.0] * 12
        self.duration_1 = int(2/pd_dt)
        self.duration_2 = int(0.5/pd_dt)
        self.duration_3 = int(5/pd_dt)
        self.duration_4 = int(1000/pd_dt)
        self.duration_5 = int(2/pd_dt)
        self.duration_6 = int(5/pd_dt)

        self.percent_1 = 0
        self.percent_2 = 0
        self.percent_3 = 0
        self.percent_4 = 0
        self.percent_5 = 0 
        self.percent_6 = 0 

        self.firstRun = True
        self.done = False

        # thread handling
        self.lowCmdWriteThreadPtr = None

        self.crc = CRC()
        
        self.counter=0
        self.qj=np.zeros(12)
        self.dqj=np.zeros(12)
        self.cmd=np.zeros(3)
        # self.obs=np.zeros(120)
        self.remote_controller = RemoteController()
        self.action=np.zeros(12)
        self.pd_counter=0
        self.policy_target=np.array(self._targetPos_2)
        self.default_pos=np.array(self._targetPos_2)
        self.action_scale=0.5
        self.policy_network=pn.policy_net(policy_file=filename)
        self.phase=PHASES
        self.dt=ctrl_dt
        # self.lin_vel=np.zeros(3)
        self.heightmap = np.full((num_heightscans, num_widthscans), 0.0)
        self.sit_requested = False
        self.exit_after_sit = False
        self.handover_requested = False
        self.policy_armed = _args.auto_arm
        self._arm_msg_printed = False
        self._prev_a_pressed = False
        self._prev_b_pressed = False
        self.cmd_vel_node = None
        self.ros_enabled = False
        self.low_level_entered = False
        self.handover_completed = False
        self.active_command_source = "idle"
        self.debug_inputs = _args.debug_inputs
        self.debug_print_period = 0.0 if _args.debug_print_hz <= 0.0 else 1.0 / _args.debug_print_hz
        self.last_debug_print_time = 0.0
        self.last_heightmap_time = 0.0
        self.last_heightmap_width = num_widthscans
        self.last_heightmap_height = num_heightscans

        if command_type in {"cmd_vel", "hybrid"}:
            self.init_cmd_vel_subscriber()

    def Init(self):
        self.InitLowCmd()

        # create publisher #
        self.lowcmd_publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher.Init()

        # create subscriber # 
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateMessageHandler, 10)


        # self.lin_vel_subscriber = ChannelSubscriber("rt/aft_mapped_to_init", Odometry_)
        # self.lin_vel_subscriber.Init(self.LinVelMessageHandler, 10)


        self.heightmap_subscriber  = ChannelSubscriber("rt/elevation_heightmap", PointCloud2_)
        self.heightmap_subscriber.Init(self.HeightMapMessageHandler, 10)


        self.sc = SportClient()  
        self.sc.SetTimeout(5.0)
        self.sc.Init()

        self.msc = MotionSwitcherClient()
        self.msc.SetTimeout(5.0)
        self.msc.Init()

    def init_cmd_vel_subscriber(self):
        if rclpy is None or CmdVelSubscriber is None:
            raise RuntimeError(
                "command_type=cmd_vel requires ROS 2 Python packages. "
                "Source the ROS 2 environment before running deploy_real.py."
            )

        if not rclpy.ok():
            rclpy.init(args=None)

        self.cmd_vel_node = CmdVelSubscriber(_args.cmd_vel_topic)
        self.ros_enabled = True
        print(f"Listening for velocity commands on '{_args.cmd_vel_topic}'.")

    def get_controller_command(self) -> np.ndarray:
        return np.array([
            cmd_scale[0] * self.remote_controller.ly,
            cmd_scale[1] * self.remote_controller.lx * -1,
            cmd_scale[2] * self.remote_controller.rx * -1,
        ], dtype=float)

    def update_command(self):
        if command_type == "controller":
            self.cmd = self.get_controller_command()
            self.active_command_source = "controller"
        elif command_type == "cmd_vel":
            if self.cmd_vel_node is None:
                self.cmd = np.zeros(3)
                self.active_command_source = "idle"
                return
            rclpy.spin_once(self.cmd_vel_node, timeout_sec=0.0)
            self.cmd = self.cmd_vel_node.get_velocity()
            self.active_command_source = "cmd_vel" if np.linalg.norm(self.cmd) > 1e-4 else "idle"
        elif command_type == "hybrid":
            controller_cmd = self.get_controller_command()
            controller_active = np.max(np.abs(controller_cmd)) > controller_deadband

            cmd_vel_cmd = np.zeros(3)
            if self.cmd_vel_node is not None:
                rclpy.spin_once(self.cmd_vel_node, timeout_sec=0.0)
                cmd_vel_cmd = self.cmd_vel_node.get_velocity()
            cmd_vel_active = np.linalg.norm(cmd_vel_cmd) > 1e-4

            if controller_active:
                self.cmd = controller_cmd
                self.active_command_source = "controller"
            elif cmd_vel_active:
                self.cmd = cmd_vel_cmd
                self.active_command_source = "cmd_vel"
            else:
                self.cmd = np.zeros(3)
                self.active_command_source = "idle"
        else:
            self.cmd = cmd_fixed.copy()
            self.active_command_source = "fixed"

    def shutdown(self):
        # The low-level recurrent thread is still active until process exit.
        # Let process teardown own the ROS cleanup to avoid destroying the node
        # while run_policy() may still call spin_once().
        self.ros_enabled = False

    def Start(self):
        self.lowCmdWriteThreadPtr = RecurrentThread(
            interval=pd_dt, target=self.LowCmdWrite, name="writebasiccmd"
        )
        self.lowCmdWriteThreadPtr.Start()

    def EnterLowLevelMode(self):
        while self.low_state is None:
            time.sleep(0.01)

        status, result = self.msc.CheckMode()
        while result['name']:
            self.msc.ReleaseMode()
            time.sleep(0.1)
            status, result = self.msc.CheckMode()
        self.low_level_entered = True

    # Private methods
    def InitLowCmd(self):
        self.low_cmd.head[0]=0xFE
        self.low_cmd.head[1]=0xEF
        self.low_cmd.level_flag = 0xFF
        self.low_cmd.gpio = 0
        for i in range(20):
            self.low_cmd.motor_cmd[i].mode = 0x01  # (PMSM) mode
            self.low_cmd.motor_cmd[i].q= PosStopF
            self.low_cmd.motor_cmd[i].kp = 0
            self.low_cmd.motor_cmd[i].dq = VelStopF
            self.low_cmd.motor_cmd[i].kd = 0
            self.low_cmd.motor_cmd[i].tau = 0

    def LowStateMessageHandler(self, msg: LowState_):
        self.low_state = msg
        self.remote_controller.set(self.low_state.wireless_remote)
        for i in range(12):
            index=reorder[i]
            self.qj[index] = self.low_state.motor_state[i].q
            self.dqj[index] = self.low_state.motor_state[i].dq

    # def LinVelMessageHandler(self, msg: Odometry_):
    #     linear = msg.twist.twist.linear        
    #     self.lin_vel[0]=linear.x
    #     self.lin_vel[1]=linear.y
    #     self.lin_vel[2]=linear.z
        # print(linear)
        # print("FR_0 motor state: ", msg.motor_state[go2.LegID["FR_0"]])
        # print("IMU state: ", msg.imu_state)
        # print("Battery state: voltage: ", msg.power_v, "current: ", msg.power_a)
    def HeightMapMessageHandler(self,msg:PointCloud2_):
        
        width = msg.width
        height = msg.height
        self.last_heightmap_time = time.monotonic()
        self.last_heightmap_width = width
        self.last_heightmap_height = height

        for i, point in enumerate(read_points(msg, field_names=("x", "y", "z"), skip_nans=False)):
            _, _, z = point
            row = i // width
            col = i % width
            self.heightmap[row, col] = z

    def maybe_print_input_debug(self, obs: np.ndarray, z_values: np.ndarray, z_normal: np.ndarray) -> None:
        if not self.debug_inputs:
            return

        now = time.monotonic()
        if self.debug_print_period > 0.0 and (now - self.last_debug_print_time) < self.debug_print_period:
            return
        self.last_debug_print_time = now

        center_row = self.heightmap.shape[0] // 2
        center_col = self.heightmap.shape[1] // 2
        front_row = self.heightmap.shape[0] - 1

        if self.last_heightmap_time > 0.0:
            heightmap_age_text = f"{now - self.last_heightmap_time:.2f}s"
        else:
            heightmap_age_text = "never"

        if command_type == "cmd_vel" and self.cmd_vel_node is not None:
            has_cmd, cmd_age = self.cmd_vel_node.get_status()
            cmd_age_text = f"{cmd_age:.2f}s" if has_cmd and cmd_age is not None else "never"
        elif command_type == "hybrid" and self.cmd_vel_node is not None:
            has_cmd, cmd_age = self.cmd_vel_node.get_status()
            cmd_age_text = f"{cmd_age:.2f}s" if has_cmd and cmd_age is not None else "never"
        else:
            cmd_age_text = "n/a"

        controller_cmd = self.get_controller_command()
        print(
            "[policy-input] "
            f"src={command_type}/{self.active_command_source} "
            f"cmd=[{self.cmd[0]:+.3f} {self.cmd[1]:+.3f} {self.cmd[2]:+.3f}] "
            f"controller=[{controller_cmd[0]:+.3f} {controller_cmd[1]:+.3f} {controller_cmd[2]:+.3f}] "
            f"cmd_age={cmd_age_text} "
            f"heightmap_age={heightmap_age_text} "
            f"heightmap_shape={self.last_heightmap_height}x{self.last_heightmap_width} "
            f"z_min={np.min(z_values):+.3f} "
            f"z_max={np.max(z_values):+.3f} "
            f"z_center={self.heightmap[center_row, center_col]:+.3f} "
            f"z_front_center={self.heightmap[front_row, center_col]:+.3f} "
            f"z_norm_max={np.max(z_normal):+.3f} "
            f"obs_dim={obs.size}"
        )
        
    def LowCmdWrite(self):
        if self.low_state is None:
            return
        if self.firstRun:
            for i in range(12):
                self.startPos[i] = self.low_state.motor_state[i].q
            self.firstRun = False

        a_pressed = self.remote_controller.button[KeyMap.A] == 1
        b_pressed = self.remote_controller.button[KeyMap.B] == 1
        a_rising = a_pressed and not self._prev_a_pressed
        b_rising = b_pressed and not self._prev_b_pressed
        self._prev_a_pressed = a_pressed
        self._prev_b_pressed = b_pressed

        if a_rising and not self.sit_requested:
            self.request_sit(exit_after_sit=False)

        if self.sit_requested and self.percent_6 >= 1.0 and not self.exit_after_sit and b_rising:
            self.reset_for_standup()

        self.percent_1 += 1.0 / self.duration_1
        self.percent_1 = min(self.percent_1, 1)
        if self.percent_1 < 1 and not self.sit_requested:
            for i in range(12):
                self.low_cmd.motor_cmd[i].q = (1 - self.percent_1) * self.startPos[i] + self.percent_1 * self._targetPos_1[i]
                self.low_cmd.motor_cmd[i].dq = 0
                self.low_cmd.motor_cmd[i].kp = self.Kp
                self.low_cmd.motor_cmd[i].kd = self.Kd
                self.low_cmd.motor_cmd[i].tau = 0

        if (self.percent_1 == 1) and (self.percent_2 <= 1) and not self.sit_requested:
            self.percent_2 += 1.0 / self.duration_2
            self.percent_2 = min(self.percent_2, 1)
            for i in range(12):
                self.low_cmd.motor_cmd[i].q = (1 - self.percent_2) * self._targetPos_1[i] + self.percent_2 * self._targetPos_2[i]
                self.low_cmd.motor_cmd[i].dq = 0
                self.low_cmd.motor_cmd[i].kp = self.Kp
                self.low_cmd.motor_cmd[i].kd = self.Kd
                self.low_cmd.motor_cmd[i].tau = 0

        if (self.percent_1 == 1) and (self.percent_2 == 1) and (self.percent_3 < 1) and not self.sit_requested:
            self.percent_3 += 1.0 / self.duration_3
            self.percent_3 = min(self.percent_3, 1)
            for i in range(12):
                self.low_cmd.motor_cmd[i].q = self._targetPos_2[i] 
                self.low_cmd.motor_cmd[i].dq = 0
                self.low_cmd.motor_cmd[i].kp = self.Kp
                self.low_cmd.motor_cmd[i].kd = self.Kd
                self.low_cmd.motor_cmd[i].tau = 0

        if (self.percent_1 == 1) and (self.percent_2 == 1) and (self.percent_3 == 1) and not self.policy_armed and not self.sit_requested:
            if b_rising:
                self.policy_armed = True
                print("Policy armed! Starting execution...")
            else:
                if not self._arm_msg_printed:
                    print("Robot standing. Press B to start policy execution. Press A to sit down.")
                    self._arm_msg_printed = True
                for i in range(12):
                    self.low_cmd.motor_cmd[i].q = self._targetPos_2[i]
                    self.low_cmd.motor_cmd[i].dq = 0
                    self.low_cmd.motor_cmd[i].kp = self.Kp
                    self.low_cmd.motor_cmd[i].kd = self.Kd
                    self.low_cmd.motor_cmd[i].tau = 0

        if (self.percent_1 == 1) and (self.percent_2 == 1) and (self.percent_3 == 1) and self.policy_armed and (self.percent_4 < 1) and not self.sit_requested:
            self.percent_4 += 1.0 / self.duration_4
            self.percent_4 = min(self.percent_4, 1)
            if self.pd_counter % _policy_decimation == 0:
                self.policy_target = self.run_policy()
            self.pd_counter += 1
            for i in range(12):
                self.low_cmd.motor_cmd[i].q = self.policy_target[i]
                self.low_cmd.motor_cmd[i].dq = 0
                self.low_cmd.motor_cmd[i].kp = self.Kp
                self.low_cmd.motor_cmd[i].kd = self.Kd
                self.low_cmd.motor_cmd[i].tau = 0
        if (((self.percent_1 == 1) and (self.percent_2 == 1) and (self.percent_3 == 1) and (self.percent_4 == 1) and (self.percent_5 < 1)) or self.sit_requested):

            if self.percent_5 <= 1e-10:
                for i in range(12):
                    self.startPos[i] = self.low_state.motor_state[i].q
            self.percent_5 += 1.0 / self.duration_5
            self.percent_5 = min(self.percent_5, 1)
            if self.percent_5 == 1:
                self.percent_6 += 1.0 / self.duration_6
                self.percent_6 = min(self.percent_6, 1)
                for i in range(12):
                    self.low_cmd.motor_cmd[i].q = self._targetPos_3[i]
                    self.low_cmd.motor_cmd[i].dq = 0
                    self.low_cmd.motor_cmd[i].kp = self.Kp
                    self.low_cmd.motor_cmd[i].kd = self.Kd
                    self.low_cmd.motor_cmd[i].tau = 0
            else:
                for i in range(12):
                    self.low_cmd.motor_cmd[i].q = (1 - self.percent_5) * self.startPos[i] + self.percent_5 * self._targetPos_3[i]
                    self.low_cmd.motor_cmd[i].dq = 0
                    self.low_cmd.motor_cmd[i].kp = self.Kp
                    self.low_cmd.motor_cmd[i].kd = self.Kd
                    self.low_cmd.motor_cmd[i].tau = 0

    
        
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_publisher.Write(self.low_cmd)

    def request_sit(self, exit_after_sit: bool):
        self.policy_armed = False
        self.sit_requested = True
        self.exit_after_sit = exit_after_sit
        self._arm_msg_printed = False
        print("Sit requested. Press B after sitting to stand up again.")

    def request_handover(self):
        self.policy_armed = False
        self.sit_requested = False
        self.exit_after_sit = False
        self.handover_requested = True
        self.handover_completed = False
        self._arm_msg_printed = False
        print("Policy stop requested. Handing control back from a standing pose.")

    def reset_for_standup(self):
        for i in range(12):
            self.startPos[i] = self.low_state.motor_state[i].q
        self.percent_1 = 0
        self.percent_2 = 0
        self.percent_3 = 0
        self.percent_4 = 0
        self.percent_5 = 0
        self.percent_6 = 0
        self.pd_counter = 0
        self.action = np.zeros(12)
        self.policy_target = np.array(self._targetPos_2)
        self.phase = PHASES.copy()
        self.sit_requested = False
        self.exit_after_sit = False
        self.handover_requested = False
        self.handover_completed = False
        self.policy_armed = False
        self._arm_msg_printed = False
        print("Standing up again. Press B once more to restart the policy.")

    def run_policy(self):
        self.counter += 1
        
        quat = self.low_state.imu_state.quaternion
        
        ang_vel = np.array([self.low_state.imu_state.gyroscope], dtype=np.float32).reshape(3)
        
        gravity_orientation = get_gravity_orientation(quat)
        qj_obs = self.qj.copy()
        dqj_obs = self.dqj.copy()
        gait_freq=freq


        self.phase+=2*np.pi*gait_freq*self.dt
        self.phase=np.fmod(self.phase,2*np.pi)
        
        sin = np.sin(self.phase)
        cos = np.cos(self.phase)
        phase_obs = np.concatenate([cos, sin])
        z_values=self.heightmap.ravel()
        z_normal=z_values-np.min(z_values)
        z_normal*=1.5
        self.update_command()
        num_actions = 12
        if mode=="baseline":
            obs = np.hstack([
                # linvel,#3
                ang_vel,#3
                gravity_orientation,#3
                qj_obs-self.default_pos,#12
                dqj_obs,#12
                # phase_obs,#8
                z_normal,#N*M
                # gait_freq,#1
                self.action,#12
                self.cmd,#3
                ])        # self.obs[:3]
        else:
            obs = np.hstack([
                    # linvel,#3
                    ang_vel,#3
                    gravity_orientation,#3
                    qj_obs-self.default_pos,#12
                    dqj_obs,#12
                    phase_obs,#8
                    z_normal,#N*M
                    gait_freq,#1
                    self.action,#12
                    self.cmd,#3
            ])        # self.obs[:3]

        self.maybe_print_input_debug(obs, z_values, z_normal)
    
        obs_tensor=torch.tensor(np.asarray(obs).copy(), dtype=torch.float32).reshape((1,-1))
        self.action = self.policy_network(obs_tensor).detach().numpy().squeeze()
        if mode=="wild":
                # print(self.phase)
                gait = _get_gait_module()
                oscilator_angles = gait.joint_trajectory(self.phase,-0.2,-0.3)
                target_dof_pos = oscilator_angles + self.action * self.action_scale
        else:
                target_dof_pos = self.default_pos + self.action * self.action_scale
        return target_dof_pos
    
    
if __name__ == '__main__':
    if not _args.no_prompt:
        print("WARNING: Please ensure there are no obstacles around the robot while running this example.")
        input("Press Enter to continue...")
    else:
        print("WARNING: Running without the interactive safety prompt.")

    if _args.network is not None:
        ChannelFactoryInitialize(0, _args.network)
    else:
        ChannelFactoryInitialize(0)

    custom = Custom()
    custom.Init()
    custom.Start()
    custom.EnterLowLevelMode()

    shutdown_requested = {"value": False}

    def request_shutdown(signum, frame):
        if shutdown_requested["value"]:
            return
        shutdown_requested["value"] = True
        print(f"Received signal {signum}. Stopping policy and returning to the default mode.")
        custom.request_handover()

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)

    exit_code = 0
    try:
        while True:
            if custom.handover_requested:
                print("Control handover requested. Exiting policy process for external restore.")
                exit_code = 2
                break
            if custom.exit_after_sit and custom.percent_6 >= 1.0:
                time.sleep(1)
                print("Done!")
                break
            time.sleep(0.05)
    finally:
        custom.shutdown()

    sys.exit(exit_code)
