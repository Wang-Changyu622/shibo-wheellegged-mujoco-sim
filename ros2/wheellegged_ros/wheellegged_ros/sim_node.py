"""Single-owner MuJoCo loop with ROS command input and stamped telemetry.

The simulation timer uses a steady wall clock, while messages use MuJoCo time.
This distinction prevents a use_sim_time deadlock before the first /clock message.
ROS callbacks never write robot positions or bypass the core controller.
"""
from __future__ import annotations

from contextlib import ExitStack
from dataclasses import replace
import math
from typing import Any

import mujoco
import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Empty
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from wheellegged.control import BalanceController, Command, balanced_data
from wheellegged.hardware import DEFAULT_PROFILE, PROFILE_NAMES
from wheellegged.model import load_model


def simulation_stamp(seconds: float) -> Time:
    whole, nanos = divmod(int(round(seconds * 1_000_000_000)), 1_000_000_000)
    return Time(sec=whole, nanosec=nanos)


def assign_quaternion(message: Any, wxyz: np.ndarray) -> None:
    message.x, message.y, message.z, message.w = (float(wxyz[i]) for i in (1, 2, 3, 0))


def assign_vector(message: Any, xyz: np.ndarray) -> None:
    message.x, message.y, message.z = map(float, xyz)


class SimulationNode(Node):
    """Publish ideal simulator state, explicitly without an estimator/noise model."""

    def __init__(self, **kwargs: Any):
        super().__init__("wheellegged_sim", **kwargs)
        self._viewer = None
        self._viewer_timer = None
        self._viewer_context = ExitStack()
        defaults = {
            "terrain": "flat", "profile": DEFAULT_PROFILE,
            "viewer": False,
            "physics_dt": 0.0005, "realtime_factor": 1.0,
            "publish_rate": 100.0, "command_timeout": 0.5,
            "hip_angle": -0.20, "steps_per_callback": 4,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        params = {key: self.get_parameter(key).value for key in defaults}
        for key in ("physics_dt", "realtime_factor", "publish_rate", "command_timeout"):
            if not math.isfinite(params[key]) or params[key] <= 0:
                raise ValueError(f"{key} must be finite and positive")
        if params["steps_per_callback"] < 1:
            raise ValueError("steps_per_callback must be positive")
        if params["profile"] not in PROFILE_NAMES:
            raise ValueError(f"Unknown hardware profile {params['profile']!r}; choose {PROFILE_NAMES}")
        self.model = load_model(params["terrain"], params["physics_dt"], profile=params["profile"])
        self.data = balanced_data(self.model, hip=params["hip_angle"])
        self.controller = BalanceController(self.model, self.data)
        self.command = Command(hip_angle=params["hip_angle"])
        self._last_command_time = -math.inf
        self._timeout = float(params["command_timeout"])
        self._jump_pending = False
        self._publish_period = 1.0 / params["publish_rate"]
        self._next_publish = self._publish_period
        self._steps = int(params["steps_per_callback"])
        self._base_id = self.model.body("chassis").id
        self._joint_names = [
            self.model.joint(i).name for i in range(self.model.njnt)
            if int(self.model.jnt_type[i]) in (
                int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE))
        ]
        self.create_subscription(Twist, "/cmd_vel", self._on_velocity, 10)
        self.create_subscription(Empty, "/jump", self._on_jump, 10)
        self.create_subscription(JointState, "/leg_command", self._on_legs, 10)
        self._joint_pub = self.create_publisher(JointState, "/joint_states", qos_profile_sensor_data)
        self._imu_pub = self.create_publisher(Imu, "/imu", qos_profile_sensor_data)
        self._odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self._clock_pub = self.create_publisher(ClockMessage, "/clock", 10)
        self._tf = TransformBroadcaster(self)
        self._static_tf = StaticTransformBroadcaster(self)
        self._publish_imu_transform()
        self._wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._timer = self.create_timer(
            self.model.opt.timestep * self._steps / params["realtime_factor"],
            self._advance, clock=self._wall_clock,
        )
        self.publish_state()
        if params["viewer"]:
            # The ROS timer remains the only owner of physics/control. The
            # passive viewer consumes that same state and never runs a second loop.
            from wheellegged.viewer import native_viewer

            self._viewer = self._viewer_context.enter_context(native_viewer(self.model, self.data))
            with self._viewer.lock():
                self._viewer.cam.distance = .45
                self._viewer.cam.azimuth = 135
                self._viewer.cam.elevation = -20
            self._viewer_timer = self.create_timer(
                1.0 / 60.0, self._sync_viewer, clock=self._wall_clock)
            self.get_logger().info("MuJoCo passive viewer ready; ROS /cmd_vel, /leg_command and /jump")
        self.get_logger().info(
            f"MuJoCo {mujoco.__version__}: {params['terrain']}, profile={params['profile']}, "
            f"dt={self.model.opt.timestep:g}s, command timeout={self._timeout:g}s"
        )

    def _sync_viewer(self) -> None:
        if self._viewer is None or not self._viewer.is_running():
            if self._viewer_timer is not None:
                self._viewer_timer.cancel()
            return
        with self._viewer.lock():
            self._viewer.cam.lookat[:] = [*self.data.body("chassis").xpos[:2], .045]
        self._viewer.set_texts([(
            mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_TOPLEFT,
            "ROS 2 input\nProfile\nSim time\nTarget speed / yaw\nHip target L / R",
            f"/cmd_vel, /leg_command, /jump\n{self.get_parameter('profile').value}\n"
            f"{self.data.time:.2f} s\n{self.command.velocity:+.3f} m/s / "
            f"{self.command.yaw_rate:+.3f} rad/s\n"
            f"{self.command.hip_targets[0]:+.3f} / {self.command.hip_targets[1]:+.3f} rad",
        )])
        self._viewer.sync(state_only=True)

    def destroy_node(self) -> bool:
        if self._viewer_timer is not None:
            self._viewer_timer.cancel()
        if self._viewer is not None:
            self._viewer_context.close()
            self._viewer = None
        return super().destroy_node()

    def _on_velocity(self, message: Twist) -> None:
        if not all(map(math.isfinite, (message.linear.x, message.angular.z))):
            self.get_logger().warning("Ignored non-finite /cmd_vel")
            return
        self.command = replace(self.command, velocity=message.linear.x, yaw_rate=message.angular.z)
        self._last_command_time = self.data.time

    def _on_legs(self, message: JointState) -> None:
        # Named pairs avoid an undocumented left/right array convention. Leg
        # setpoints persist across velocity watchdog expiry and jump recovery.
        if (len(message.name) != 2 or set(message.name) != {"L_hip", "R_hip"}
                or len(message.position) != 2
                or not all(map(math.isfinite, message.position))):
            self.get_logger().warning("Ignored /leg_command: require finite L_hip and R_hip positions")
            return
        targets = dict(zip(message.name, message.position))
        self.command = replace(self.command, left_hip_angle=targets["L_hip"],
                               right_hip_angle=targets["R_hip"])

    def _on_jump(self, _: Empty) -> None:
        # A message requests one event; the core controller owns all jump phases.
        self._jump_pending = True

    def _advance(self) -> None:
        for _ in range(self._steps):
            if self.data.time - self._last_command_time > self._timeout:
                self.command = replace(self.command, velocity=0., yaw_rate=0.)
            request = replace(self.command, jump=self._jump_pending)
            self._jump_pending = False
            self.data.ctrl[:] = self.controller.update(request)
            mujoco.mj_step(self.model, self.data)
            # mj_step integrates qpos after computing sensor values. Refresh every
            # step so both the next controller observation and telemetry are current.
            mujoco.mj_forward(self.model, self.data)
        if self.data.time + 1e-12 >= self._next_publish:
            self.publish_state()
            self._next_publish += self._publish_period
            if self._next_publish <= self.data.time:
                self._next_publish = self.data.time + self._publish_period

    def _publish_imu_transform(self) -> None:
        site = self.model.site("imu")
        if int(site.bodyid[0]) != self._base_id:
            raise ValueError("ROS adapter expects the IMU site directly on chassis")
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = "base_link", "imu_link"
        assign_vector(transform.transform.translation, site.pos)
        assign_quaternion(transform.transform.rotation, site.quat)
        self._static_tf.sendTransform(transform)

    def publish_state(self) -> None:
        stamp = simulation_stamp(self.data.time)
        self._clock_pub.publish(ClockMessage(clock=stamp))
        joint_state = JointState()
        joint_state.header.stamp, joint_state.header.frame_id = stamp, "base_link"
        joint_state.name = self._joint_names
        joint_state.position = [float(self.data.joint(name).qpos[0]) for name in self._joint_names]
        joint_state.velocity = [float(self.data.joint(name).qvel[0]) for name in self._joint_names]
        joint_state.effort = [float(self.data.qfrc_actuator[self.model.joint(name).dofadr[0]])
                              for name in self._joint_names]
        self._joint_pub.publish(joint_state)
        imu = Imu()
        imu.header.stamp, imu.header.frame_id = stamp, "imu_link"
        assign_quaternion(imu.orientation, self.data.sensor("imu_orientation").data)
        assign_vector(imu.angular_velocity, self.data.sensor("imu_gyro").data)
        assign_vector(imu.linear_acceleration, self.data.sensor("imu_accelerometer").data)
        # All-zero covariance means unknown. These are ideal simulator sensors,
        # not estimated hardware accuracy; no invented covariance is reported.
        self._imu_pub.publish(imu)
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, "odom", "base_link"
        assign_vector(odom.pose.pose.position, self.data.body("chassis").xpos)
        assign_quaternion(odom.pose.pose.orientation, self.data.body("chassis").xquat)
        velocity = np.empty(6)
        # XBODY is the body origin/axes used by base_link. BODY would return
        # velocity at the inertial COM expressed in its principal-inertia axes.
        mujoco.mj_objectVelocity(
            self.model, self.data, mujoco.mjtObj.mjOBJ_XBODY, self._base_id, velocity, 1)
        assign_vector(odom.twist.twist.angular, velocity[:3])
        assign_vector(odom.twist.twist.linear, velocity[3:])
        self._odom_pub.publish(odom)
        transform = TransformStamped()
        transform.header, transform.child_frame_id = odom.header, odom.child_frame_id
        assign_vector(transform.transform.translation, self.data.body("chassis").xpos)
        assign_quaternion(transform.transform.rotation, self.data.body("chassis").xquat)
        self._tf.sendTransform(transform)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = SimulationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
