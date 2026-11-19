"""Voltage wheel feedback and bounded position-servo coordination."""
from dataclasses import dataclass
import math

import mujoco
import numpy as np

from .lqr import design_lqr
from .jump import JumpSupervisor
from .model import HIP_CONTROL_RANGE, SIDES, TRACK_WIDTH, ground_forces, leg_geometry, make_data
from .state import observe


@dataclass(frozen=True)
class Command:
    """Finite SI commands; baseline targets saturate at 0.3 m/s and 1 rad/s."""

    velocity: float = 0.0
    yaw_rate: float = 0.0
    hip_angle: float = -.2
    jump: bool = False
    left_hip_angle: float | None = None
    right_hip_angle: float | None = None

    def __post_init__(self):
        if not all(math.isfinite(value) for value in (self.velocity, self.yaw_rate, self.hip_angle)):
            raise ValueError("velocity, yaw_rate and hip_angle must be finite")
        if not all(value is None or math.isfinite(value)
                   for value in (self.left_hip_angle, self.right_hip_angle)):
            raise ValueError("left_hip_angle and right_hip_angle must be finite or None")

    @property
    def hip_targets(self) -> tuple[float, float]:
        """Left/right position-servo targets in radians; None inherits hip_angle."""
        return tuple(float(np.clip(self.hip_angle if angle is None else angle,
                                   *HIP_CONTROL_RANGE))
                     for angle in (self.left_hip_angle, self.right_hip_angle))


def balanced_data(model: mujoco.MjModel, hip: float = -.2,
                  pitch_offset: float = 0.0, x: float = 0.0) -> mujoco.MjData:
    """Reset near the CAD mass-derived equilibrium, without fixing the base."""
    design = design_lqr(model, hip)
    return make_data(model, hip, design.pitch + pitch_offset, x)


class BalanceController:
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData):
        self.model, self.data = model, data
        self.dt = float(model.opt.timestep)
        self.design = design_lqr(model)
        self._hip_grid = np.linspace(-.3, 0., 7)
        self._designs = [design_lqr(model, float(q)) for q in self._hip_grid]
        obs = observe(model, data)
        self.x_reference = obs.x
        self.position_reference = np.array([obs.x, obs.y])
        self.yaw_reference = obs.yaw
        self.velocity_reference = 0.0
        self.hip_reference = float(np.mean(obs.hip))
        self.hip_differential_reference = float((obs.hip[0] - obs.hip[1]) / 2)
        self.roll_reference = self._geometric_roll_reference()
        self.phase = "balance"
        self.jump_supervisor = JumpSupervisor()
        self._roll_integral = 0.0
        self._wheel_ids = [model.actuator(s + "_wheel_voltage").id for s in SIDES]
        self._servo_ids = [model.actuator(s + "_servo").id for s in SIDES]

    def _geometric_roll_reference(self) -> float:
        """Flat-support body roll implied by requested left/right leg lengths.

        Left is +Y: a longer left leg raises the left chassis side (+roll).
        Terrain offsets remain disturbances for the residual roll controller.
        Equal legs return exactly zero, preserving the symmetric baseline.
        """
        if self.hip_differential_reference == 0.0:
            return 0.0
        left = leg_geometry(self.hip_reference + self.hip_differential_reference)["height"]
        right = leg_geometry(self.hip_reference - self.hip_differential_reference)["height"]
        return math.atan2(left - right, TRACK_WIDTH)

    def update(self, command: Command = Command()) -> np.ndarray:
        obs = observe(self.model, self.data)
        targets = command.hip_targets
        requested_mean = (targets[0] + targets[1]) / 2
        requested_differential = (targets[0] - targets[1]) / 2
        bounded_velocity = float(np.clip(command.velocity, -.3, .3))
        bounded_yaw_rate = float(np.clip(command.yaw_rate, -1., 1.))
        posture_ready = (abs(obs.pitch - self.design.pitch) < .15
                         and abs(obs.pitch_rate) < 1.0 and abs(obs.roll_rate) < 1.0
                         and abs(obs.speed) < .1 and abs(obs.yaw_rate) < .25
                         and abs(bounded_velocity) < .1 and abs(bounded_yaw_rate) < .1)
        jump = self.jump_supervisor.update(
            obs.time, command.jump, ground_forces(self.data),
            ready=(posture_ready and abs(obs.roll) < .35
                   and abs(obs.roll - self.roll_reference) < .08),
            push_ready=(posture_ready and abs(obs.roll) < .12
                        and abs(obs.hip[0] - obs.hip[1]) < .02
                        and abs(self.hip_differential_reference) < 1e-6),
        )
        self.phase = jump.phase
        # Maximum commanded acceleration is explicit, independent of loop rate.
        self.velocity_reference += np.clip(bounded_velocity - self.velocity_reference,
                                           -.25 * self.dt, .25 * self.dt)
        self.yaw_reference += bounded_yaw_rate * self.dt
        heading_reference = np.array([np.cos(self.yaw_reference), np.sin(self.yaw_reference)])
        self.position_reference += self.velocity_reference * heading_reference * self.dt
        self.x_reference = float(self.position_reference[0])
        hip_target = requested_mean
        if jump.hip_target is not None:
            hip_target = jump.hip_target
        # Firmware jump uses position mode with speed=0/acc=0. The jump target
        # switches directly, but physical motion remains torque/PD limited.
        if jump.direct_target:
            self.hip_reference = hip_target
            self.hip_differential_reference = 0.0
        else:
            differential_target = requested_differential if self.phase == "balance" else 0.0
            if differential_target == self.hip_differential_reference == 0.0:
                # Keep the original symmetric trajectory, including float order.
                self.hip_reference += np.clip(hip_target - self.hip_reference,
                                              -.5 * self.dt, .5 * self.dt)
            else:
                previous = np.array([self.hip_reference + self.hip_differential_reference,
                                     self.hip_reference - self.hip_differential_reference])
                target = np.array([hip_target + differential_target, hip_target - differential_target])
                references = previous + np.clip(target - previous, -.5 * self.dt, .5 * self.dt)
                self.hip_reference = float(np.mean(references))
                self.hip_differential_reference = float((references[0] - references[1]) / 2)
        # Per-leg slew limits imply |mean step| + |half-difference step| <=
        # 0.5*dt, including at either joint limit. Jump push/tuck retain their
        # original direct targets; the posture guard requires symmetry first.
        room = max(0.0, min(self.hip_reference - HIP_CONTROL_RANGE[0],
                            HIP_CONTROL_RANGE[1] - self.hip_reference))
        self.hip_differential_reference = float(np.clip(self.hip_differential_reference, -room, room))
        self.roll_reference = self._geometric_roll_reference()
        actual_hip = float(np.mean(obs.hip))
        pitch_reference = float(np.interp(actual_hip, self._hip_grid,
                                          [d.pitch for d in self._designs]))
        gain = np.array([np.interp(actual_hip, self._hip_grid, [d.gain[i] for d in self._designs])
                         for i in range(4)])
        heading = np.array([np.cos(obs.yaw), np.sin(obs.yaw)])
        position_error = np.array([obs.x, obs.y]) - self.position_reference
        longitudinal_error = float(position_error @ heading)
        state = np.array([longitudinal_error, obs.pitch - pitch_reference,
                          obs.speed - self.velocity_reference, obs.pitch_rate])
        voltage = -float(gain @ state)
        yaw_error = np.arctan2(np.sin(obs.yaw - self.yaw_reference),
                               np.cos(obs.yaw - self.yaw_reference))
        yaw_voltage = -4.0 * yaw_error - .18 * (obs.yaw_rate - bounded_yaw_rate)
        roll_error = obs.roll - self.roll_reference
        integral_candidate = float(np.clip(self._roll_integral + roll_error * self.dt, -.04, .04))
        lower = -room - self.hip_differential_reference
        upper = room - self.hip_differential_reference
        correction_candidate = 2.0 * roll_error + .07 * obs.roll_rate + .6 * integral_candidate
        asymmetric = requested_differential != 0.0 or self.hip_differential_reference != 0.0
        # Preserve the original symmetric-loop integration exactly. New
        # independent-leg commands only integrate when residual authority exists
        # or the error would unwind saturation; that new mode freezes in flight.
        if not asymmetric or (self.phase == "balance" and
                              not ((correction_candidate > upper and roll_error > 0.0)
                                   or (correction_candidate < lower and roll_error < 0.0))):
            self._roll_integral = integral_candidate
        roll_correction = 2.0 * roll_error + .07 * obs.roll_rate + .6 * self._roll_integral
        if self.phase != "balance":
            roll_correction = 0.0
        # Preserve commanded mean leg height at either joint limit. Differential
        # roll control may use only the travel available to both hips together.
        roll_correction = float(np.clip(roll_correction, lower, upper))
        control = np.zeros(self.model.nu)
        # Positive right-wheel voltage turns toward +yaw.
        control[self._wheel_ids] = [voltage - yaw_voltage, voltage + yaw_voltage]
        differential = self.hip_differential_reference + roll_correction
        control[self._servo_ids] = [self.hip_reference + differential,
                                   self.hip_reference - differential]
        return np.clip(control, self.model.actuator_ctrlrange[:, 0],
                       self.model.actuator_ctrlrange[:, 1])
