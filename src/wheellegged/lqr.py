"""LQR from a constrained sagittal reduction of the actual MuJoCo model.

The two generalized coordinates are axle translation and chassis pitch. Legs
are held at a given hip angle, both tires roll without slip. This is a local
design model, not a substitute for the floating-base contact simulation.
"""
from dataclasses import dataclass

import mujoco
import numpy as np
from scipy.linalg import expm, solve_discrete_are

from .model import SIDES, WHEEL_RADIUS, make_data, wheel_centers


@dataclass(frozen=True)
class LQRDesign:
    hip: float
    pitch: float
    gain: np.ndarray
    A: np.ndarray
    B: np.ndarray
    poles: np.ndarray
    mass_matrix: np.ndarray


def design_lqr(model: mujoco.MjModel, hip: float = -.2) -> LQRDesign:
    data = make_data(model, hip)
    offset = data.body("chassis").subtree_com - wheel_centers(data).mean(axis=0)
    pitch = float(-np.arctan2(offset[0], offset[2]))
    data = make_data(model, hip, pitch)
    lever = data.body("chassis").xpos - wheel_centers(data).mean(axis=0)
    tangent = np.zeros((model.nv, 2))
    base_dof = int(model.joint("floating_base").dofadr[0])
    tangent[base_dof, 0] = 1
    tangent[base_dof, 1] = lever[2]
    tangent[base_dof + 2, 1] = -lever[0]
    tangent[base_dof + 4, 1] = 1
    damping = np.zeros((2, 2))
    input_vector = np.zeros(2)
    for side in SIDES:
        dof = int(model.joint(side + "_wheel_joint").dofadr[0])
        tangent[dof] = [1 / WHEEL_RADIUS, -1]
        actuator = model.actuator(side + "_wheel_voltage")
        input_vector += tangent[dof] * actuator.gainprm[0]
        damping -= actuator.biasprm[2] * np.outer(tangent[dof], tangent[dof])
    full_mass = np.zeros((model.nv, model.nv))
    mujoco.mj_fullM(model, data, full_mass)
    mass = tangent.T @ full_mass @ tangent
    damping += tangent.T @ (model.dof_damping[:, None] * tangent)
    com_height = float(data.body("chassis").subtree_com[2] - wheel_centers(data)[:, 2].mean())
    stiffness = np.zeros((2, 2))
    stiffness[1, 1] = np.sum(model.body_mass) * abs(model.opt.gravity[2]) * com_height
    continuous_a = np.zeros((4, 4))
    continuous_a[:2, 2:] = np.eye(2)
    continuous_a[2:, :2] = np.linalg.solve(mass, stiffness)
    continuous_a[2:, 2:] = -np.linalg.solve(mass, damping)
    continuous_b = np.zeros((4, 1))
    continuous_b[2:, 0] = np.linalg.solve(mass, input_vector)
    augmented = np.zeros((5, 5))
    augmented[:4, :4], augmented[:4, 4:] = continuous_a, continuous_b
    discrete = expm(augmented * model.opt.timestep)
    A, B = discrete[:4, :4], discrete[:4, 4:]
    Q, R = np.diag([100., 250., 8., 1.]), np.array([[.03]])
    P = solve_discrete_are(A, B, Q, R)
    gain = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
    poles = np.linalg.eigvals(A - B @ gain)
    if max(abs(poles)) >= 1:
        raise RuntimeError("Computed LQR design is not locally stabilizing")
    return LQRDesign(hip, pitch, gain[0], A, B, poles, mass)
