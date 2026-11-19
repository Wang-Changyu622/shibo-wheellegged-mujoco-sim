#!/usr/bin/env python3
"""Exercise actual ROS 2 DDS transport and MuJoCo; no simulator mocks.

Run with a Python environment that contains ROS 2 and both project packages.
This checks the adapter contract, not terrain or jump-control performance.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import time

os.environ.setdefault("ROS_DOMAIN_ID", "73")
os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")

import numpy as np
import mujoco
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                      qos_profile_sensor_data)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Empty
from tf2_msgs.msg import TFMessage

from wheellegged_ros.sim_node import SimulationNode
from wheellegged.hardware import DEFAULT_PROFILE, PROFILE_NAMES


def body_origin_velocity(model, data) -> np.ndarray:
    """Independent oracle: body-origin Jacobians in world axes, then rotate.

    Do not use the adapter's mj_objectVelocity API here: BODY and XBODY have
    different origins/axes when CAD inertial frames are not identity.
    """
    linear, angular = np.zeros((3, model.nv)), np.zeros((3, model.nv))
    mujoco.mj_jacBody(model, data, linear, angular, model.body("chassis").id)
    world_to_body = data.body("chassis").xmat.reshape(3, 3).T
    return np.r_[world_to_body @ angular @ data.qvel,
                 world_to_body @ linear @ data.qvel]


def check_rotated_body_frame(simulator, executor, received) -> dict:
    """Publish a tilted, moving test state through DDS after the dynamic run.

    This is a deliberate kinematic test fixture, not a controller maneuver.
    Restore the simulation state afterwards; no physics is integrated here.
    """
    simulator._timer.cancel()
    data, model = simulator.data, simulator.model
    saved_qpos, saved_qvel, saved_time = data.qpos.copy(), data.qvel.copy(), data.time
    try:
        quaternion = np.empty(4)
        mujoco.mju_euler2Quat(quaternion, np.array([.2, -.3, .7]), "xyz")
        data.joint("floating_base").qpos[3:7] = quaternion
        data.joint("floating_base").qvel[:] = [.1, .2, .3, .4, .5, .6]
        data.time = saved_time + .1
        mujoco.mj_forward(model, data)
        expected = body_origin_velocity(model, data)
        # The free-joint angular velocity is body-local, while its translational
        # velocity is world-oriented. This checks the Jacobian oracle itself.
        rotation = data.body("chassis").xmat.reshape(3, 3)
        np.testing.assert_allclose(expected, np.r_[[.4, .5, .6], rotation.T @ [.1, .2, .3]], atol=1e-12)
        inertial_velocity = np.empty(6)
        mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY,
                                model.body("chassis").id, inertial_velocity, 1)
        separation = float(np.linalg.norm(expected - inertial_velocity))
        assert separation > .01, "frame fixture does not distinguish body and inertia axes"
        simulator.publish_state()
        stamp = round(data.time * 1_000_000_000)
        deadline = time.monotonic() + 2
        matching = []
        while time.monotonic() < deadline and not matching:
            executor.spin_once(timeout_sec=.01)
            matching = [message for message in received["odom"]
                        if stamp_nanoseconds(message.header.stamp) == stamp]
        assert matching, "rotated frame fixture did not arrive through DDS"
        actual = np.r_[vector_values(matching[-1].twist.twist.angular),
                       vector_values(matching[-1].twist.twist.linear)]
        np.testing.assert_allclose(actual, expected, atol=1e-12)
        return {"passed": True, "oracle": "body-origin Jacobian and free-joint kinematics",
                "expected_twist_angular_then_linear": expected.tolist(),
                "received_twist_angular_then_linear": actual.tolist(),
                "inertial_frame_twist_difference_norm": separation}
    finally:
        data.qpos[:], data.qvel[:], data.time = saved_qpos, saved_qvel, saved_time
        mujoco.mj_forward(model, data)


def stamp_seconds(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def stamp_nanoseconds(stamp) -> int:
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def vector_values(vector) -> np.ndarray:
    return np.array([vector.x, vector.y, vector.z])


def quaternion_values(quaternion) -> np.ndarray:
    return np.array([quaternion.x, quaternion.y, quaternion.z, quaternion.w])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("artifacts/ros2/smoke.json"))
    parser.add_argument("--profile", choices=PROFILE_NAMES, default=DEFAULT_PROFILE)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps({"passed": False, "status": "running"}) + "\n")
    rclpy.init()
    simulator = probe = executor = None
    try:
        simulator = SimulationNode(parameter_overrides=[
            Parameter("realtime_factor", value=2.0),
            Parameter("use_sim_time", value=True),
            Parameter("profile", value=args.profile),
        ])
        probe = Node("wheellegged_ros_smoke_probe")
        received = {name: [] for name in ("clock", "joints", "imu", "odom", "tf", "tf_static")}
        subscriptions = [
            probe.create_subscription(kind, topic, received[name].append, qos)
            for name, kind, topic, qos in (
                ("clock", Clock, "/clock", 10),
                ("joints", JointState, "/joint_states", qos_profile_sensor_data),
                ("imu", Imu, "/imu", qos_profile_sensor_data),
                ("odom", Odometry, "/odom", 10),
                ("tf", TFMessage, "/tf", 10),
                ("tf_static", TFMessage, "/tf_static", QoSProfile(
                    depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    reliability=ReliabilityPolicy.RELIABLE)),
            )
        ]
        # Capture the exact physical state at each real publisher invocation.
        # The original publisher and DDS transport are still exercised; this
        # validates field mapping, not just quaternion norm or array lengths.
        expected_states = {}
        original_publish_state = simulator.publish_state

        def record_and_publish_state():
            data, model = simulator.data, simulator.model
            base_velocity = body_origin_velocity(model, data)
            expected_states[round(data.time * 1_000_000_000)] = {
                "body_position": data.body("chassis").xpos.copy(),
                "body_quaternion": data.body("chassis").xquat[[1, 2, 3, 0]].copy(),
                "body_velocity": base_velocity.copy(),
                "imu_quaternion": data.sensor("imu_orientation").data[[1, 2, 3, 0]].copy(),
                "imu_gyro": data.sensor("imu_gyro").data.copy(),
                "imu_acceleration": data.sensor("imu_accelerometer").data.copy(),
            }
            original_publish_state()

        simulator.publish_state = record_and_publish_state
        velocity_publisher = probe.create_publisher(Twist, "/cmd_vel", 10)
        jump_publisher = probe.create_publisher(Empty, "/jump", 10)
        leg_publisher = probe.create_publisher(JointState, "/leg_command", 10)
        leg_events = []
        leg_subscription = next(subscription for subscription in simulator.subscriptions
                                if subscription.topic_name == "/leg_command")
        original_leg_callback = leg_subscription.callback

        def record_leg_callback(message):
            before = simulator.command
            before_time = simulator._last_command_time
            qpos, qvel, sim_time = (simulator.data.qpos.copy(), simulator.data.qvel.copy(),
                                    simulator.data.time)
            original_leg_callback(message)
            leg_events.append({
                "names": list(message.name),
                "command_unchanged": before == simulator.command,
                "targets": list(simulator.command.hip_targets),
                "watchdog_not_refreshed": before_time == simulator._last_command_time,
                "physics_untouched": bool(np.array_equal(qpos, simulator.data.qpos)
                    and np.array_equal(qvel, simulator.data.qvel) and sim_time == simulator.data.time),
            })

        leg_subscription.callback = record_leg_callback
        invalid_commands = []
        velocity_subscription = next(
            subscription for subscription in simulator.subscriptions
            if subscription.topic_name == "/cmd_vel")
        original_velocity_callback = velocity_subscription.callback

        def record_velocity_callback(message):
            before = simulator.command
            last_command_time = simulator._last_command_time
            original_velocity_callback(message)
            if not all(map(math.isfinite, (message.linear.x, message.angular.z))):
                invalid_commands.append({
                    "command_unchanged": simulator.command == before,
                    "watchdog_not_refreshed": simulator._last_command_time == last_command_time,
                })

        velocity_subscription.callback = record_velocity_callback
        forwarded_jumps = []
        original_update = simulator.controller.update

        def record_and_update(command):
            if command.jump:
                forwarded_jumps.append(float(simulator.data.time))
            return original_update(command)

        simulator.controller.update = record_and_update
        executor = SingleThreadedExecutor()
        executor.add_node(simulator)
        executor.add_node(probe)
        deadline = time.monotonic() + 20.0
        next_command = 0.2
        saw_forward = saw_reverse = False
        jump_sent = False
        invalid_sent = set()
        leg_sent = set()
        leg_schedule = (
            (.50, ["R_hip", "L_hip"], [-.20, -.24]),  # resolve by name, not array order
            (.70, ["L_hip", "R_hip"], [float("nan"), -.2]),
            (.75, ["L_hip", "L_hip"], [-.2, -.2]),
            (.80, ["L_hip", "R_hip"], [-.2]),
            (.85, ["L_hip", "unknown"], [-.2, -.2]),
        )
        while simulator.data.time < 2.8 and time.monotonic() < deadline:
            now = simulator.data.time
            for send_time, names, positions in leg_schedule:
                if now >= send_time and send_time not in leg_sent:
                    leg_publisher.publish(JointState(name=names, position=positions))
                    leg_sent.add(send_time)
            if 0.2 <= now < 1.6 and now >= next_command:
                message = Twist()
                message.linear.x = 0.05 if now < 0.9 else -0.04
                velocity_publisher.publish(message)
                next_command = now + 0.05
            for label, send_time in (("nan_velocity", 1.8), ("inf_yaw", 2.2)):
                if now >= send_time and label not in invalid_sent:
                    invalid = Twist()
                    if label == "nan_velocity":
                        invalid.linear.x = float("nan")
                    else:
                        invalid.angular.z = float("inf")
                    velocity_publisher.publish(invalid)
                    invalid_sent.add(label)
            saw_forward |= simulator.command.velocity > 0
            saw_reverse |= simulator.command.velocity < 0
            if now >= 2.65 and not jump_sent:
                jump_publisher.publish(Empty())
                jump_sent = True
            executor.spin_once(timeout_sec=0.01)
        assert simulator.data.time >= 2.8, "simulation clock did not advance"
        assert subscriptions  # Keep subscription objects alive for the test duration.
        assert saw_forward and saw_reverse, "/cmd_vel did not reach the controller in both directions"
        assert simulator.command.velocity == simulator.command.yaw_rate == 0.0, "command watchdog failed"
        assert len(invalid_commands) == 2, "invalid /cmd_vel messages did not reach the subscription"
        assert all(all(result.values()) for result in invalid_commands), "invalid command modified state or watchdog"
        assert len(forwarded_jumps) == 1, "/jump must forward exactly one controller event"
        assert len(leg_events) == len(leg_schedule), "/leg_command messages were not delivered"
        assert not leg_events[0]["command_unchanged"]
        assert all(event["command_unchanged"] for event in leg_events[1:]), "invalid leg target accepted"
        assert all(event["physics_untouched"] and event["watchdog_not_refreshed"]
                   for event in leg_events), "leg callback changed physics or velocity timeout"
        np.testing.assert_allclose(simulator.command.hip_targets, [-.24, -.20], atol=1e-12)
        counts = {name: len(messages) for name, messages in received.items()}
        assert all(counts[name] >= 10 for name in received if name != "tf_static"), counts
        assert counts["tf_static"] >= 1, "transient-local IMU transform was not delivered"
        clocks = [stamp_seconds(message.clock) for message in received["clock"]]
        assert np.all(np.diff(clocks) > 0), "non-monotonic /clock"
        assert abs(clocks[-1] - simulator.data.time) < 0.1
        for message in received["imu"]:
            assert message.header.frame_id == "imu_link"
            quaternion = message.orientation
            norm = sum(v * v for v in (quaternion.w, quaternion.x, quaternion.y, quaternion.z))
            assert abs(norm - 1.0) < 1e-6, "invalid ROS xyzw quaternion"
            assert all(math.isfinite(value) for value in (
                message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z,
                message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z,
            ))
            expected = expected_states[stamp_nanoseconds(message.header.stamp)]
            np.testing.assert_allclose(quaternion_values(message.orientation), expected["imu_quaternion"], atol=1e-12)
            np.testing.assert_allclose(vector_values(message.angular_velocity), expected["imu_gyro"], atol=1e-12)
            np.testing.assert_allclose(vector_values(message.linear_acceleration), expected["imu_acceleration"], atol=1e-12)
        for message in received["odom"]:
            assert (message.header.frame_id, message.child_frame_id) == ("odom", "base_link")
            expected = expected_states[stamp_nanoseconds(message.header.stamp)]
            np.testing.assert_allclose(vector_values(message.pose.pose.position), expected["body_position"], atol=1e-12)
            np.testing.assert_allclose(quaternion_values(message.pose.pose.orientation), expected["body_quaternion"], atol=1e-12)
            np.testing.assert_allclose(vector_values(message.twist.twist.angular), expected["body_velocity"][:3], atol=1e-12)
            np.testing.assert_allclose(vector_values(message.twist.twist.linear), expected["body_velocity"][3:], atol=1e-12)
        for message in received["tf"]:
            assert len(message.transforms) == 1
            transform = message.transforms[0]
            assert (transform.header.frame_id, transform.child_frame_id) == ("odom", "base_link")
            expected = expected_states[stamp_nanoseconds(transform.header.stamp)]
            np.testing.assert_allclose(vector_values(transform.transform.translation), expected["body_position"], atol=1e-12)
            np.testing.assert_allclose(quaternion_values(transform.transform.rotation), expected["body_quaternion"], atol=1e-12)
        for message in received["tf_static"]:
            assert len(message.transforms) == 1
            transform = message.transforms[0]
            assert (transform.header.frame_id, transform.child_frame_id) == ("base_link", "imu_link")
            np.testing.assert_allclose(vector_values(transform.transform.translation), simulator.model.site("imu").pos, atol=1e-12)
            np.testing.assert_allclose(quaternion_values(transform.transform.rotation),
                                       simulator.model.site("imu").quat[[1, 2, 3, 0]], atol=1e-12)
        expected_joint_names = {
            f"{side}_{joint}" for side in ("L", "R")
            for joint in ("hip", "knee", "wheel_joint", "rod_pivot")
        }
        for joints in received["joints"]:
            assert set(joints.name) == expected_joint_names, "missing, duplicate, or spurious joints"
            assert len(joints.name) == len(joints.position) == len(joints.velocity) == len(joints.effort) == 8
            assert np.isfinite([joints.position, joints.velocity, joints.effort]).all()
        wheel_trace = [joints.position[joints.name.index("L_wheel_joint")]
                       for joints in received["joints"]]
        assert np.ptp(wheel_trace) > 0.01, "joint telemetry is stuck at zero or stale"
        assert np.isfinite(simulator.data.qpos).all() and np.isfinite(simulator.data.qvel).all()
        rotated_body_frame = check_rotated_body_frame(simulator, executor, received)
        report = {
            "passed": True, "scope": "real DDS adapter integration; not a control-performance test",
            "simulated_seconds": float(simulator.data.time), "received_messages": counts,
            "forward_command_received": saw_forward, "reverse_command_received": saw_reverse,
            "command_timeout_stopped_reference": True, "jump_events_forwarded": len(forwarded_jumps),
            "quaternion_and_frames_valid": True, "monotonic_clock": True,
            "stamped_mujoco_fields_match": True, "static_imu_transform_matches_model": True,
            "rotated_body_frame": rotated_body_frame,
            "profile": args.profile, "robot_mass_kg": float(simulator.model.body_mass.sum()),
            "physics_dt_seconds": float(simulator.model.opt.timestep),
            "mujoco_version": mujoco.__version__,
            "joint_count": len(expected_joint_names), "joint_readings_dynamic": True,
            "invalid_command_checks": invalid_commands,
            "leg_command_events": leg_events,
            "leg_targets_preserved_after_velocity_commands_and_timeout": True,
            "ros_domain_id": os.environ["ROS_DOMAIN_ID"],
            "source_sha256": {
                str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                for path in (
                    Path("src/wheellegged/model.py"), Path("src/wheellegged/control.py"),
                    Path("src/wheellegged/lqr.py"),
                    Path("src/wheellegged/state.py"), Path("src/wheellegged/jump.py"),
                    Path("src/wheellegged/hardware.py"), Path("src/wheellegged/mass_properties.py"),
                    Path("config/hardware_parameters.json"), Path("models/cad_mass_properties.json"),
                    Path("ros2/wheellegged_ros/wheellegged_ros/sim_node.py"),
                    Path("scripts/ros2_smoke.py"),
                )
            },
        }
        args.output.write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report, indent=2))
    except Exception as error:
        args.output.write_text(json.dumps({
            "passed": False, "error": f"{type(error).__name__}: {error}",
        }, indent=2) + "\n")
        raise
    finally:
        if executor is not None:
            executor.shutdown()
        if probe is not None:
            probe.destroy_node()
        if simulator is not None:
            simulator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
