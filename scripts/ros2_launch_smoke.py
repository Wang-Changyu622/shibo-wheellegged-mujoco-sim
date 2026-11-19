#!/usr/bin/env python3
"""Build the ROS package, launch its installed node and receive a real DDS clock.

Run in the project's ROS-compatible Python environment. The child process is
stopped with SIGINT and must shut down cleanly. This script never targets a
pre-existing process and writes build logs outside tracked source directories.
"""
from __future__ import annotations

import argparse
import hashlib
from importlib.metadata import version
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[1]


def find_ros_prefix(explicit: Path | None) -> Path:
    if explicit is not None:
        candidates = [explicit]
    else:
        candidates = [Path(sys.base_prefix)]
        if executable := shutil.which("ros2"):
            candidates.append(Path(executable).resolve().parents[1])
        candidates.extend(Path(entry) for entry in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep) if entry)
    for prefix in candidates:
        if (prefix / "bin" / "ros2").is_file():
            return prefix.resolve()
    raise RuntimeError("ROS 2 prefix not found; activate its environment or provide --ros-prefix")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "artifacts" / "ros2")
    parser.add_argument("--ros-prefix", type=Path)
    parser.add_argument("--profile", choices=("reference", "hardware_2s"), default="hardware_2s")
    parser.add_argument("--viewer", action="store_true", help="also exercise a real native MuJoCo window")
    args = parser.parse_args()
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    report_name = "viewer_launch" if args.viewer else "launch"
    report_path = output / f"{report_name}.json"
    report_path.write_text(json.dumps({"passed": False, "status": "running"}) + "\n")
    child = node = None
    ros_initialized = False
    try:
        prefix = find_ros_prefix(args.ros_prefix)
        python = Path(sys.executable)
        colcon = python.parent / "colcon"
        if not colcon.is_file():
            raise RuntimeError("colcon must be installed in the active Python environment")
        install = output / "colcon-install"
        package_prefix = install / "wheellegged_ros"
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = "75" if args.viewer else "74"
        env["ROS_LOCALHOST_ONLY"] = "1"
        env["PATH"] = os.pathsep.join((str(python.parent), str(prefix / "bin"), env.get("PATH", "")))
        env["AMENT_PREFIX_PATH"] = os.pathsep.join((str(package_prefix), str(prefix)))
        installed_python = package_prefix / "lib" / f"python{sys.version_info.major}.{sys.version_info.minor}" / "site-packages"
        env["PYTHONPATH"] = os.pathsep.join((str(installed_python), str(ROOT / "src"), env.get("PYTHONPATH", "")))
        build_command = [str(colcon), "--log-base", str(output / "colcon-log"), "build",
                         "--base-paths", "ros2", "--build-base", str(output / "colcon-build"),
                         "--install-base", str(install), "--event-handlers", "console_direct+"]
        with (output / "build.log").open("w") as log:
            build = subprocess.run(build_command, cwd=ROOT, env=env, stdout=log,
                                   stderr=subprocess.STDOUT, timeout=120, check=False)
        if build.returncode != 0:
            raise RuntimeError(f"colcon build exited {build.returncode}; see {output / 'build.log'}")
        invalid_profile = subprocess.run(
            [str(python), "-m", "wheellegged_ros.sim_node", "--ros-args", "-p", "profile:=unknown_profile"],
            cwd=ROOT, env=env, capture_output=True, text=True, timeout=15, check=False)
        if invalid_profile.returncode == 0 or "Unknown hardware profile" not in invalid_profile.stderr:
            raise RuntimeError("the installed node did not reject an unknown hardware profile")

        # A separate rclpy process subscribes to the installed launch result,
        # proving that the launched node is actually advancing the simulation.
        os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]
        os.environ["ROS_LOCALHOST_ONLY"] = env["ROS_LOCALHOST_ONLY"]
        import rclpy
        from rclpy.node import Node
        from rosgraph_msgs.msg import Clock
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry

        rclpy.init()
        ros_initialized = True
        node = Node("wheellegged_launch_smoke_probe")
        clocks = []
        subscription = node.create_subscription(
            Clock, "/clock", lambda message: clocks.append(message.clock.sec + message.clock.nanosec * 1e-9), 10)
        positions = []
        odom_subscription = node.create_subscription(
            Odometry, "/odom", lambda message: positions.append(message.pose.pose.position.x), 10)
        publisher = node.create_publisher(Twist, "/cmd_vel", 10)
        command_count = 0
        launch_command = [str(python), str(prefix / "bin" / "ros2"), "launch",
                          "wheellegged_ros", "simulation.launch.py", "terrain:=flat",
                          f"profile:={args.profile}", f"viewer:={str(args.viewer).lower()}"]
        minimum_time = 1.0 if args.viewer else .25
        with (output / f"{report_name}.log").open("w") as log:
            child = subprocess.Popen(launch_command, cwd=ROOT, env=env, stdout=log,
                                     stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline and child.poll() is None:
                rclpy.spin_once(node, timeout_sec=.05)
                if args.viewer and clocks and .1 < clocks[-1] < .8:
                    command = Twist()
                    command.linear.x = .05
                    publisher.publish(command)
                    command_count += 1
                if len(clocks) >= 20 and clocks[-1] >= minimum_time:
                    break
            assert subscription and odom_subscription
            if child.poll() is not None:
                raise RuntimeError("ros2 launch exited before publishing a running simulation clock")
            if len(clocks) < 20 or clocks[-1] < minimum_time:
                raise RuntimeError("launched node did not publish advancing /clock within timeout")
            if any(later <= earlier for earlier, later in zip(clocks, clocks[1:])):
                raise RuntimeError("launched /clock was not strictly increasing")
            child.send_signal(signal.SIGINT)
            return_code = child.wait(timeout=10)
        if return_code != 0:
            raise RuntimeError(f"ros2 launch failed clean SIGINT shutdown: {return_code}")
        launch_log = (output / f"{report_name}.log").read_text()
        # ros2 launch itself may return zero even when its child crashed.
        if ("[ERROR]" in launch_log or "process has died" in launch_log
                or "process has finished cleanly" not in launch_log):
            raise RuntimeError("the launched simulator process did not shut down cleanly; inspect the launch log")
        if "dt=0.0005s" not in launch_log:
            raise RuntimeError("launched node did not report the final 0.5 ms physics step")
        if f"profile={args.profile}" not in launch_log:
            raise RuntimeError("launched node did not report the requested hardware profile")
        if args.viewer:
            if "MuJoCo passive viewer ready" not in launch_log:
                raise RuntimeError("passive viewer did not finish creating its native window")
            if command_count == 0 or len(positions) < 20 or max(positions) - min(positions) < .002:
                raise RuntimeError("viewer simulation did not respond to external ROS motion commands")
        source_paths = [ROOT / path for path in (
            "src/wheellegged/model.py", "src/wheellegged/control.py", "src/wheellegged/lqr.py",
            "src/wheellegged/state.py", "src/wheellegged/jump.py", "scripts/ros2_launch_smoke.py",
            "src/wheellegged/hardware.py", "src/wheellegged/mass_properties.py",
            "src/wheellegged/viewer.py",
            "config/hardware_parameters.json", "models/cad_mass_properties.json",
            "ros2/wheellegged_ros/wheellegged_ros/sim_node.py", "ros2/wheellegged_ros/setup.py",
            "ros2/wheellegged_ros/package.xml", "ros2/wheellegged_ros/launch/simulation.launch.py")]
        report = {
            "passed": True, "scope": "colcon build plus real ros2 launch and cross-process DDS /clock",
            "process_return_code": return_code, "physics_dt_seconds": .0005,
            "profile": args.profile, "native_viewer_enabled": args.viewer,
            "invalid_profile_rejected": True,
            "colcon_build": "passed", "startup": True, "sigint_clean_shutdown": True,
            "received_clock_messages": len(clocks), "last_simulation_clock_seconds": clocks[-1],
            "clock_strictly_increasing": True, "ros_domain_id": env["ROS_DOMAIN_ID"],
            "external_velocity_commands_sent": command_count,
            "received_odom_messages": len(positions),
            "body_x_range_m": max(positions) - min(positions) if positions else None,
            "python_version": sys.version.split()[0],
            "tool_versions": {package: version(package) for package in ("colcon-core", "setuptools", "mujoco")},
            "source_sha256": {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest() for path in source_paths},
        }
        report_path.write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report, indent=2))
    except Exception as error:
        report_path.write_text(json.dumps({"passed": False, "error": f"{type(error).__name__}: {error}"}, indent=2) + "\n")
        raise
    finally:
        if child is not None and child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait(timeout=5)
        if node is not None:
            node.destroy_node()
        if ros_initialized:
            import rclpy
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
