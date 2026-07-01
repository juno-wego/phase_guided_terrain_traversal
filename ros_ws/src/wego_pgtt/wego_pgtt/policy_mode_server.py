import os
import signal
import subprocess
import sys
import time
import importlib
import shutil
import shlex
from typing import Optional, Union

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class PolicyModeServer(Node):
    def __init__(self) -> None:
        super().__init__("wego_pgtt_policy_mode_server")

        self.declare_parameter("service_name", "toggle_pgtt_mode")
        self.declare_parameter("repo_root", "")
        self.declare_parameter("deploy_script", "")
        self.declare_parameter("policy_file", "")
        self.declare_parameter("network_interface", "")
        self.declare_parameter("python_executable", sys.executable)
        self.declare_parameter("robot", "go2")
        self.declare_parameter("method", "pgtt")
        self.declare_parameter("command_source", "controller")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("motion_service_name", "/go2_motion_cmd")
        self.declare_parameter("go2_driver_setup", "/home/wego/dddmr_navigation/install/setup.bash")
        self.declare_parameter("auto_start_go2_driver", True)
        self.declare_parameter("auto_arm", True)
        self.declare_parameter("restore_to_classic", True)

        self.service_name = self.get_parameter("service_name").get_parameter_value().string_value
        self.repo_root = self.get_parameter("repo_root").get_parameter_value().string_value
        self.deploy_script = self.get_parameter("deploy_script").get_parameter_value().string_value
        self.policy_file = self.get_parameter("policy_file").get_parameter_value().string_value
        self.network_interface = self.get_parameter("network_interface").get_parameter_value().string_value
        self.python_executable = self.get_parameter("python_executable").get_parameter_value().string_value
        self.robot = self.get_parameter("robot").get_parameter_value().string_value
        self.method = self.get_parameter("method").get_parameter_value().string_value
        self.command_source = self.get_parameter("command_source").get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        self.motion_service_name = self.get_parameter("motion_service_name").get_parameter_value().string_value
        self.go2_driver_setup = self.get_parameter("go2_driver_setup").get_parameter_value().string_value
        self.auto_start_go2_driver = self.get_parameter("auto_start_go2_driver").get_parameter_value().bool_value
        self.auto_arm = self.get_parameter("auto_arm").get_parameter_value().bool_value
        self.restore_to_classic = self.get_parameter("restore_to_classic").get_parameter_value().bool_value

        self.process: Optional[subprocess.Popen] = None
        self.motion_service_process: Optional[subprocess.Popen] = None
        self.needs_restore = False
        self.channel_initialized = False

        self.toggle_service = self.create_service(Trigger, self.service_name, self.handle_toggle)
        self.poll_timer = self.create_timer(1.0, self.poll_policy_process)

        self.get_logger().info(
            f"Policy mode server ready on service '{self.service_name}'. "
            "First call starts PGTT mode, next call returns to classic mode."
        )

    def handle_toggle(self, request, response):
        del request
        if self.process is not None and self.process.poll() is None:
            self.stop_policy_process()
            response.success = True
            response.message = "Stopping PGTT policy mode and returning to classic mode."
            return response

        try:
            self.start_policy_process()
        except Exception as exc:
            response.success = False
            response.message = f"Failed to start PGTT policy mode: {exc}"
            self.get_logger().error(response.message)
            return response

        response.success = True
        response.message = "PGTT policy mode started."
        return response

    def start_policy_process(self) -> None:
        if not self.repo_root:
            raise RuntimeError("Parameter 'repo_root' must not be empty.")

        deploy_script = self.deploy_script or os.path.join(self.repo_root, "deploy", "deploy_real.py")
        if not os.path.isfile(deploy_script):
            raise FileNotFoundError(f"deploy_real.py not found: {deploy_script}")

        if not self.policy_file:
            raise RuntimeError("Parameter 'policy_file' must point to a valid policy file.")

        policy_file = self.policy_file
        if not os.path.isabs(policy_file):
            policy_file = os.path.join(self.repo_root, policy_file)
        if not os.path.isfile(policy_file):
            raise FileNotFoundError(f"Policy file not found: {policy_file}")

        env = os.environ.copy()
        env["PYTHONPATH"] = self.build_pythonpath(env)
        command_source = self.normalize_command_source(self.command_source)

        cmd = [
            self.python_executable,
            deploy_script,
            "--robot",
            self.robot,
            "--method",
            self.method,
            "--policy-file",
            policy_file,
            "--command_type",
            command_source,
            "--no-prompt",
        ]
        if command_source == "cmd_vel":
            cmd.extend(["--cmd-vel-topic", self.cmd_vel_topic])
        if self.network_interface:
            cmd.extend(["--network", self.network_interface])
        if self.auto_arm:
            cmd.append("--auto-arm")

        self.get_logger().info(f"Starting PGTT policy process with policy '{policy_file}'.")
        self.process = subprocess.Popen(cmd, cwd=self.repo_root, env=env)
        self.needs_restore = True

    def normalize_command_source(self, command_source: str) -> str:
        valid_sources = {"controller", "cmd_vel", "fixed"}
        if command_source not in valid_sources:
            raise RuntimeError(
                f"Unsupported command_source '{command_source}'. "
                f"Expected one of: {sorted(valid_sources)}."
            )
        return command_source

    def stop_policy_process(self) -> None:
        if self.process is None or self.process.poll() is not None:
            return
        self.get_logger().info("Stopping PGTT policy process.")
        self.process.send_signal(signal.SIGINT)
        deadline = time.time() + 8.0
        while self.process.poll() is None and time.time() < deadline:
            time.sleep(0.05)

    def poll_policy_process(self) -> None:
        if self.process is None:
            return

        return_code = self.process.poll()
        if return_code is None:
            return

        self.get_logger().info(f"PGTT policy process exited with code {return_code}.")
        self.process = None

        if self.needs_restore and self.restore_to_classic and return_code != 0:
            self.restore_classic_mode()
        self.needs_restore = False
        self.stop_motion_service_process()

    def build_pythonpath(self, env) -> str:
        entries = []
        for candidate in [
            self.repo_root,
            "/home/wego/go2_python_ws/src/unitree_sdk2_python",
            "/home/wego/unitree_sdk2_python",
            "/home/wego/go2_python_ws/install/unitree_sdk2py/lib/python3.10/site-packages",
            env.get("PYTHONPATH", ""),
        ]:
            if not candidate:
                continue
            if os.pathsep in candidate:
                for part in candidate.split(os.pathsep):
                    if part and part not in entries:
                        entries.append(part)
                continue
            if candidate not in entries:
                entries.append(candidate)
        return os.pathsep.join(entries)

    def restore_classic_mode(self) -> None:
        self.get_logger().info("Restoring Go2 classic mode.")
        if self.restore_classic_mode_via_ros_service():
            return

        try:
            self.ensure_unitree_sdk_importable()
            self.init_unitree_channel()
            from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
            from unitree_sdk2py.go2.sport.sport_client import SportClient
        except Exception as exc:
            self.get_logger().error(f"Failed to import Unitree SDK for mode restore: {exc}")
            return

        try:
            sport = SportClient()
            sport.SetTimeout(5.0)
            sport.Init()

            switcher = MotionSwitcherClient()
            switcher.SetTimeout(5.0)
            switcher.Init()

            self.log_mode_state("CheckMode(before restore)", switcher.CheckMode())
            self.log_sdk_result("SelectMode(advanced)", switcher.SelectMode("advanced"))
            time.sleep(0.5)
            self.log_sdk_result("RecoveryStand", sport.RecoveryStand())
            time.sleep(0.5)
            self.log_sdk_result("StopMove", sport.StopMove())
            time.sleep(0.2)
            self.log_sdk_result("ClassicWalk(False)", sport.ClassicWalk(False))
            time.sleep(0.2)
            self.log_sdk_result("SwitchJoystick(False)", sport.SwitchJoystick(False))
            time.sleep(0.2)
            self.log_mode_state("CheckMode(after restore)", switcher.CheckMode())
        except Exception as exc:
            self.get_logger().error(f"Failed while restoring classic mode: {exc}")

    def restore_classic_mode_via_ros_service(self) -> bool:
        ros2_path = shutil.which("ros2")
        if ros2_path is None:
            self.get_logger().warning("ros2 CLI not found. Falling back to direct SDK restore.")
            return False

        service_name = self.motion_service_name.strip() or "/go2_motion_cmd"
        selected_name = self.find_motion_service_name(service_name)
        if selected_name is None and self.auto_start_go2_driver:
            self.start_go2_driver_process()
            selected_name = self.wait_for_motion_service(service_name, timeout_sec=12.0)
        if selected_name is None:
            self.get_logger().warning(
                f"Motion service {service_name!r} not available. Falling back to direct SDK restore."
            )
            return False

        tasks = [
            ("SETSPORTSMODE", 13),
            ("RECOVERY_STAND", 5),
            ("STOP_MOVE", 2),
        ]
        for label, task in tasks:
            cmd = ["ros2", "service", "call", selected_name, "go2_msgs/srv/MotionCmd", f"{{task: {task}}}"]
            try:
                result = self.run_go2_ros_command(cmd, timeout=8.0)
            except Exception as exc:
                self.get_logger().warning(f"{label} via ROS service failed to execute: {exc}")
                return False

            stdout = (result.stdout or "").strip()
            stderr = (result.stderr or "").strip()
            self.get_logger().info(
                f"{label} via {selected_name} exited {result.returncode}. stdout={stdout!r} stderr={stderr!r}"
            )
            if result.returncode != 0 or "respond: 0" not in stdout:
                self.get_logger().warning(
                    f"{label} via ROS service did not succeed cleanly. Falling back to direct SDK restore."
                )
                return False
            time.sleep(0.2)

        self.get_logger().info(f"Go2 mode restored via ROS service {selected_name}.")
        return True

    def find_motion_service_name(self, requested_name: str) -> Optional[str]:
        try:
            listed = self.run_go2_ros_command(["ros2", "service", "list"], timeout=5.0)
        except Exception as exc:
            self.get_logger().warning(f"Failed to query ROS services for restore: {exc}")
            return None

        available_services = set(line.strip() for line in listed.stdout.splitlines() if line.strip())
        candidate_names = [requested_name]
        if requested_name.startswith("/"):
            candidate_names.append(requested_name[1:])
        else:
            candidate_names.append("/" + requested_name)
        return next((name for name in candidate_names if name in available_services), None)

    def wait_for_motion_service(self, requested_name: str, timeout_sec: float) -> Optional[str]:
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            selected_name = self.find_motion_service_name(requested_name)
            if selected_name is not None:
                return selected_name
            time.sleep(0.25)
        return None

    def start_go2_driver_process(self) -> None:
        if self.motion_service_process is not None and self.motion_service_process.poll() is None:
            return

        if not self.go2_driver_setup or not os.path.isfile(self.go2_driver_setup):
            self.get_logger().warning(
                f"go2_driver setup file not found: {self.go2_driver_setup!r}. Cannot auto-start driver."
            )
            return

        interface = self.network_interface or os.environ.get("GO2_IFACE", "enp88s0")
        shell_cmd = (
            "source /opt/ros/humble/setup.bash && "
            f"source {shlex.quote(self.go2_driver_setup)} && "
            f"ros2 run go2_base go2_driver {shlex.quote(interface)}"
        )
        self.get_logger().info(f"Auto-starting go2_driver for motion restore on interface '{interface}'.")
        self.motion_service_process = subprocess.Popen(
            ["bash", "-lc", shell_cmd],
            cwd=self.repo_root or None,
            env=os.environ.copy(),
        )

    def run_go2_ros_command(self, cmd_parts: list[str], timeout: float) -> subprocess.CompletedProcess:
        if not self.go2_driver_setup or not os.path.isfile(self.go2_driver_setup):
            raise FileNotFoundError(f"go2_driver setup file not found: {self.go2_driver_setup!r}")

        shell_cmd = (
            "source /opt/ros/humble/setup.bash && "
            f"source {shlex.quote(self.go2_driver_setup)} && "
            + " ".join(shlex.quote(part) for part in cmd_parts)
        )
        return subprocess.run(
            ["bash", "-lc", shell_cmd],
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout,
            env=os.environ.copy(),
        )

    def log_sdk_result(self, name: str, result: Union[int, tuple]) -> None:
        if isinstance(result, tuple):
            code = result[0]
        else:
            code = result
        self.get_logger().info(f"{name} returned {code}.")

    def log_mode_state(self, name: str, result: tuple) -> None:
        code, data = result
        self.get_logger().info(f"{name} returned {code}, data={data}.")

    def init_unitree_channel(self) -> None:
        if self.channel_initialized:
            return

        self.ensure_unitree_sdk_importable()
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize

        if self.network_interface:
            ChannelFactoryInitialize(0, self.network_interface)
        else:
            ChannelFactoryInitialize(0)
        self.channel_initialized = True

    def ensure_unitree_sdk_importable(self) -> None:
        candidates = [
            "/home/wego/go2_python_ws/src/unitree_sdk2_python",
            "/home/wego/unitree_sdk2_python",
            "/home/wego/go2_python_ws/install/unitree_sdk2py/lib/python3.10/site-packages",
        ]
        for candidate in reversed(candidates):
            if os.path.isdir(candidate) and candidate not in sys.path:
                sys.path.insert(0, candidate)

        unitree_module = sys.modules.get("unitree_sdk2py")
        if unitree_module is not None:
            module_file = getattr(unitree_module, "__file__", "") or ""
            if not any(path in module_file for path in candidates):
                for name in list(sys.modules):
                    if name == "unitree_sdk2py" or name.startswith("unitree_sdk2py."):
                        sys.modules.pop(name, None)

        try:
            importlib.import_module("unitree_sdk2py.comm.motion_switcher.motion_switcher_client")
            importlib.import_module("unitree_sdk2py.go2.sport.sport_client")
        except Exception as exc:
            raise ImportError(
                "Could not import unitree_sdk2py SDK modules from the expected paths."
            ) from exc

    def shutdown(self) -> None:
        if self.process is None or self.process.poll() is not None:
            return

        self.get_logger().info("Shutdown requested. Stopping PGTT policy process first.")
        self.stop_policy_process()
        deadline = time.time() + 15.0
        while self.process.poll() is None and time.time() < deadline:
            time.sleep(0.25)

        if self.process.poll() is None:
            self.get_logger().warning("PGTT policy process did not stop in time. Killing it.")
            self.process.kill()
            self.process.wait(timeout=5.0)

        return_code = self.process.poll()
        if self.restore_to_classic and return_code != 0:
            self.restore_classic_mode()
        self.process = None
        self.needs_restore = False
        self.stop_motion_service_process()

    def stop_motion_service_process(self) -> None:
        if self.motion_service_process is None:
            return
        if self.motion_service_process.poll() is None:
            self.motion_service_process.terminate()
            try:
                self.motion_service_process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self.motion_service_process.kill()
                self.motion_service_process.wait(timeout=5.0)
        self.motion_service_process = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PolicyModeServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
