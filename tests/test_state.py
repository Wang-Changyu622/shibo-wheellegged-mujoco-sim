"""Frame checks using MuJoCo quaternion integration as an independent oracle."""
import mujoco
import numpy as np
import pytest

from wheellegged.model import load_model, make_data
from wheellegged.state import observe


@pytest.fixture(scope="module")
def model():
    return load_model()


def set_orientation(model, data, roll, pitch, yaw):
    cx, sx = np.cos(roll), np.sin(roll)
    cy, sy = np.cos(pitch), np.sin(pitch)
    cz, sz = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    quaternion = np.zeros(4)
    mujoco.mju_mat2Quat(quaternion, (rz @ ry @ rx).ravel())
    data.joint("floating_base").qpos[3:7] = quaternion
    mujoco.mj_forward(model, data)


def test_pitch_rate_is_heading_independent(model):
    data = make_data(model)
    set_orientation(model, data, 0, 0, np.pi / 2)
    # Free-joint angular qvel is body-local. Body +Y at yaw=90 degrees is
    # world -X, but it is still a positive pitch motion, not a roll motion.
    data.joint("floating_base").qvel[3:] = [0, 1, 0]
    mujoco.mj_forward(model, data)
    observation = observe(model, data)
    assert observation.pitch_rate == pytest.approx(1.0, abs=1e-12)
    assert observation.roll_rate == pytest.approx(0.0, abs=1e-12)
    assert observation.yaw_rate == pytest.approx(0.0, abs=1e-12)


@pytest.mark.parametrize("angles", [(0.0, 0.0, 0.0), (0.0, 0.0, np.pi / 2),
                                   (0.2, -0.15, 1.2), (-0.25, 0.3, -2.0)])
def test_euler_rates_match_integrated_orientation(model, angles):
    data = make_data(model)
    set_orientation(model, data, *angles)
    data.joint("floating_base").qvel[3:] = [0.3, -0.4, 0.6]
    mujoco.mj_forward(model, data)
    before = observe(model, data)
    eps = 1e-7
    mujoco.mj_integratePos(model, data.qpos, data.qvel, eps)
    mujoco.mj_forward(model, data)
    after = observe(model, data)
    angle_difference = np.array([after.roll - before.roll,
                                 after.pitch - before.pitch,
                                 after.yaw - before.yaw])
    finite_rate = np.arctan2(np.sin(angle_difference), np.cos(angle_difference)) / eps
    reported_rate = [before.roll_rate, before.pitch_rate, before.yaw_rate]
    assert reported_rate == pytest.approx(finite_rate, abs=2e-7)


def test_longitudinal_speed_projects_onto_current_heading(model):
    data = make_data(model)
    set_orientation(model, data, 0, 0, np.pi / 2)
    data.joint("floating_base").qvel[:3] = [0.07, 0.20, 0.0]
    mujoco.mj_forward(model, data)
    assert observe(model, data).speed == pytest.approx(0.20, abs=1e-12)


def test_axle_speed_matches_position_derivative_with_spinning_wheels(model):
    data = make_data(model)
    set_orientation(model, data, .05, -.1, .7)
    data.qvel[:] = np.linspace(-.7, .9, model.nv)
    data.joint("L_wheel_joint").qvel[0] = 12.
    data.joint("R_wheel_joint").qvel[0] = -8.
    mujoco.mj_forward(model, data)
    before = observe(model, data)
    eps = 1e-7
    mujoco.mj_integratePos(model, data.qpos, data.qvel, eps)
    mujoco.mj_forward(model, data)
    after = observe(model, data)
    speed = ((after.x - before.x) * np.cos(before.yaw)
             + (after.y - before.y) * np.sin(before.yaw)) / eps
    assert before.speed == pytest.approx(speed, abs=2e-7)
