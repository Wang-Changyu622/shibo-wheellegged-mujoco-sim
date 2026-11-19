"""Voltage wheel feedback and bounded position-servo coordination."""
from dataclasses import dataclass
import math

import mujoco
import numpy as np

from .lqr import design_lqr
from .jump import JumpSupervisor
from .model import SIDES, ground_forces, make_data
from .state import observe


@dataclass(frozen=True)
class Command:
    """Finite SI commands; baseline targets saturate at 0.3 m/s and 1 rad/s."""

    velocity: float = 0.0
    yaw_rate: float = 0.0
    hip_angle: float = -.2
    jump: bool = False

    def __post_init__(self):
        if not all(math.isfinite(value) for value in (self.velocity, self.yaw_rate, self.hip_angle)):
            raise ValueError("velocity, yaw_rate and hip_angle must be finite")


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
        self.phase = "balance"
        self.jump_supervisor = JumpSupervisor()
        self._roll_integral = 0.0
        self._wheel_ids = [model.actuator(s + "_wheel_voltage").id for s in SIDES]
        self._servo_ids = [model.actuator(s + "_servo").id for s in SIDES]

    def update(self, command: Command = Command()) -> np.ndarray:
        obs = observe(self.model, self.data)
        jump = self.jump_supervisor.update(
            obs.time, command.jump, ground_forces(self.data),
            ready=(abs(obs.pitch - self.design.pitch) < .15 and abs(obs.roll) < .12
                   and abs(obs.pitch_rate) < 1.0 and abs(obs.roll_rate) < 1.0
                   and abs(obs.speed) < .1),
        )
        self.phase = jump.phase
        # Maximum commanded acceleration is explicit, independent of loop rate.
        bounded_velocity = float(np.clip(command.velocity, -.3, .3))
        bounded_yaw_rate = float(np.clip(command.yaw_rate, -1., 1.))
        self.velocity_reference += np.clip(bounded_velocity - self.velocity_reference,
                                           -.25 * self.dt, .25 * self.dt)
        self.yaw_reference += bounded_yaw_rate * self.dt
        heading_reference = np.array([np.cos(self.yaw_reference), np.sin(self.yaw_reference)])
        self.position_reference += self.velocity_reference * heading_reference * self.dt
        self.x_reference = float(self.position_reference[0])
        hip_target = float(np.clip(command.hip_angle, -.3, 0.))
        if jump.hip_target is not None:
            hip_target = jump.hip_target
        # Firmware jump uses position mode with speed=0/acc=0. The jump target
        # switches directly, but physical motion remains torque/PD limited.
        if jump.direct_target:
            self.hip_reference = hip_target
        else:
            self.hip_reference += np.clip(hip_target - self.hip_reference,
                                          -.5 * self.dt, .5 * self.dt)
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
        self._roll_integral = float(np.clip(self._roll_integral + obs.roll * self.dt, -.04, .04))
        roll_correction = 2.0 * obs.roll + .07 * obs.roll_rate + .6 * self._roll_integral
        if self.phase != "balance":
            roll_correction = 0.0
        # Preserve commanded mean leg height at either joint limit. Differential
        # roll control may use only the travel available to both hips together.
        room = max(0.0, min(self.hip_reference - (-.3), 0.0 - self.hip_reference))
        roll_correction = float(np.clip(roll_correction, -room, room))
        control = np.zeros(self.model.nu)
        # Positive right-wheel voltage turns toward +yaw.
        control[self._wheel_ids] = [voltage - yaw_voltage, voltage + yaw_voltage]
        control[self._servo_ids] = [self.hip_reference + roll_correction,
                                   self.hip_reference - roll_correction]
        return np.clip(control, self.model.actuator_ctrlrange[:, 0],
                       self.model.actuator_ctrlrange[:, 1])
