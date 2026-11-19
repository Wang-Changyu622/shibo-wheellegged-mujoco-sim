"""Small, explicit observations shared by control, evaluation and ROS."""
from dataclasses import dataclass

import mujoco
import numpy as np


SIDES = ("L", "R")


@dataclass(frozen=True)
class Observation:
    time: float
    x: float
    y: float
    speed: float
    pitch: float
    pitch_rate: float
    roll: float
    roll_rate: float
    yaw: float
    yaw_rate: float
    height: float
    hip: np.ndarray
    wheel_z: np.ndarray


def observe(model: mujoco.MjModel, data: mujoco.MjData) -> Observation:
    rotation = data.body("chassis").xmat.reshape(3, 3)
    pitch = np.arcsin(np.clip(-rotation[2, 0], -1.0, 1.0))
    roll = np.arctan2(rotation[2, 1], rotation[2, 2])
    yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
    # XBODY uses the regular body origin/frame; BODY uses the inertial COM
    # frame. Positions below refer to axle origins, so their velocities must too.
    velocity = np.zeros(6)
    mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_XBODY,
                            model.body("chassis").id, velocity, 0)
    centers = np.array([data.body(s + "_wheel").xpos.copy() for s in SIDES])
    axle_velocity = np.zeros(3)
    for side in SIDES:
        v = np.zeros(6)
        mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_XBODY,
                                model.body(side + "_wheel").id, v, 0)
        axle_velocity += v[3:] / 2
    heading = np.array([np.cos(yaw), np.sin(yaw), 0.0])
    # World angular velocity is not a vector of Euler derivatives at nonzero
    # yaw. Convert to ZYX rates; the controller only operates near upright.
    pitch_rate = -np.sin(yaw) * velocity[0] + np.cos(yaw) * velocity[1]
    cosine_pitch = max(float(np.cos(pitch)), 1e-6)
    roll_rate = (np.cos(yaw) * velocity[0] + np.sin(yaw) * velocity[1]) / cosine_pitch
    yaw_rate = velocity[2] + np.sin(pitch) * roll_rate
    return Observation(float(data.time), float(centers[:, 0].mean()),
                       float(centers[:, 1].mean()), float(axle_velocity @ heading),
                       float(pitch), float(pitch_rate), float(roll),
                       float(roll_rate), float(yaw), float(yaw_rate),
                       float(data.body("chassis").xpos[2]),
                       np.array([data.joint(s + "_hip").qpos[0] for s in SIDES]),
                       centers[:, 2].copy())
